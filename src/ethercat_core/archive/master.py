"""EtherCAT master lifecycle and topology configuration."""

from __future__ import annotations

import json
import struct
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, List, Mapping

try:
    import pysoem  # type: ignore
except ImportError:  # pragma: no cover - depends on host environment.
    pysoem = None

from .data_types import EthercatAlStates
from .devices.base import SdoReadSpec, SlaveAdapter, SlaveIdentity
from .devices.beckhoff.el2004.adapter import El2004SlaveAdapter
from .devices.beckhoff.elm3002.adapter import Elm3002SlaveAdapter
from .devices.beckhoff.el5032.adapter import El5032SlaveAdapter
from .devices.motor_drives.Novanta.Everest.adapter import NovantaEverestSlaveAdapter
from .devices.motor_drives.Novanta.Everest.pdo import PdoScaling as EverestPdoScaling
from .devices.motor_drives.Novanta.Volcano.adapter import NovantaVolcanoSlaveAdapter
from .devices.motor_drives.Novanta.Volcano.pdo import PdoScaling as VolcanoPdoScaling


class MasterConfigError(RuntimeError):
    """Raised for invalid topology configuration or startup mismatch."""


def _pysoem_missing_message() -> str:
    repo_root = Path(__file__).resolve().parents[2]
    venv_python = repo_root / ".venv-ecat" / "bin" / "python"
    return (
        "pysoem is not installed for the active interpreter.\n"
        f"interpreter: {sys.executable}\n"
        "Install it with the repo bootstrap script:\n"
        f"  {repo_root}/env_setup_scripts/bootstrap_venv_ecat.sh\n"
        "Then run this tool with the EtherCAT venv interpreter:\n"
        f"  {venv_python} <script> [args]"
    )


def require_pysoem() -> Any:
    """Return the imported pysoem module or raise with setup guidance."""

    if pysoem is None:
        raise RuntimeError(_pysoem_missing_message())
    return pysoem


@dataclass(slots=True)
class SlaveConfig:
    """One configured slave entry from topology config."""

    name: str
    position: int
    kind: str
    vendor_id: int = 0
    product_code: int = 0
    pdo_mapping: List[Dict[str, int]] = field(default_factory=list)
    scaling: Dict[str, float] = field(default_factory=dict)


@dataclass(slots=True)
class MasterConfig:
    """Master startup configuration loaded from JSON topology file."""

    iface: str
    cycle_hz: int = 1000
    strict_pdo_size: bool = False
    slaves: List[SlaveConfig] = field(default_factory=list)


@dataclass(slots=True)
class MasterRuntime:
    """Runtime handles returned by `EthercatMaster.initialize`."""

    master: Any
    adapters: Dict[str, SlaveAdapter[Any, Any]]
    slaves_by_name: Dict[str, Any]
    startup_params: Dict[str, Dict[str, Any]] = field(default_factory=dict)


def load_topology(path: str | Path) -> MasterConfig:
    """Load topology JSON into strongly typed config."""

    with Path(path).open("r", encoding="utf-8") as fh:
        raw = json.load(fh)

    iface = raw.get("iface")
    if not iface:
        raise MasterConfigError("Topology config must include non-empty 'iface'.")

    raw_slaves = raw.get("slaves", [])
    if not isinstance(raw_slaves, list) or not raw_slaves:
        raise MasterConfigError("Topology config 'slaves' must be a non-empty list.")

    slaves: List[SlaveConfig] = []
    for entry in raw_slaves:
        slaves.append(
            SlaveConfig(
                name=entry["name"],
                position=int(entry["position"]),
                kind=entry["kind"],
                vendor_id=int(entry.get("vendor_id", 0)),
                product_code=int(entry.get("product_code", 0)),
                pdo_mapping=list(entry.get("pdo_mapping", [])),
                scaling=dict(entry.get("scaling", {})),
            )
        )

    return MasterConfig(
        iface=iface,
        cycle_hz=int(raw.get("cycle_hz", 1000)),
        strict_pdo_size=bool(raw.get("strict_pdo_size", False)),
        slaves=slaves,
    )


def _build_adapter(cfg: SlaveConfig) -> SlaveAdapter[Any, Any]:
    identity = SlaveIdentity(
        name=cfg.name,
        position=cfg.position,
        vendor_id=cfg.vendor_id,
        product_code=cfg.product_code,
    )

    if cfg.kind == "everest":
        scaling = EverestPdoScaling(
            torque_lsb_per_nm=float(cfg.scaling.get("torque_lsb_per_nm", 10.0)),
            velocity_lsb_per_rad_s=float(cfg.scaling.get("velocity_lsb_per_rad_s", 1000.0)),
            position_lsb_per_rad=float(cfg.scaling.get("position_lsb_per_rad", 10000.0)),
        )
        return NovantaEverestSlaveAdapter(identity=identity, scaling=scaling)
    if cfg.kind == "volcano":
        scaling = VolcanoPdoScaling(
            torque_lsb_per_nm=float(cfg.scaling.get("torque_lsb_per_nm", 10.0)),
            velocity_lsb_per_rad_s=float(cfg.scaling.get("velocity_lsb_per_rad_s", 1000.0)),
            position_lsb_per_rad=float(cfg.scaling.get("position_lsb_per_rad", 10000.0)),
        )
        return NovantaVolcanoSlaveAdapter(identity=identity, scaling=scaling)
    if cfg.kind == "EL2004":
        return El2004SlaveAdapter(identity=identity)
    if cfg.kind in ("ELM3002", "ELM3002"):
        return Elm3002SlaveAdapter(identity=identity)
    if cfg.kind == "EL5032":
        return El5032SlaveAdapter(identity=identity)

    raise MasterConfigError(f"Unsupported slave kind '{cfg.kind}' for '{cfg.name}'.")


class EthercatMaster:
    """Owns pysoem master lifecycle, startup checks, and adapter registry."""

    def __init__(self, config: MasterConfig):
        self.config = config
        self._runtime: MasterRuntime | None = None

    @property
    def runtime(self) -> MasterRuntime:
        if self._runtime is None:
            raise RuntimeError("Master is not initialized.")
        return self._runtime

    def initialize(self) -> MasterRuntime:
        pysoem_mod = require_pysoem()

        master = pysoem_mod.Master()
        master.open(self.config.iface)

        slave_count = master.config_init()
        if slave_count <= 0:
            master.close()
            raise RuntimeError("No EtherCAT slaves detected.")

        # Ensure bus is in PRE-OP before any SDO-based PDO remap writes.
        self._transition_to_preop(master)

        for cfg in self.config.slaves:
            cfg.position = self._resolve_configured_position(master, cfg)

        adapters = {cfg.name: _build_adapter(cfg) for cfg in self.config.slaves}
        slaves_by_name: Dict[str, Any] = {}
        startup_params: Dict[str, Dict[str, Any]] = {}

        for cfg in self.config.slaves:
            slave = master.slaves[cfg.position]
            self._validate_identity(cfg, slave)
            startup_params[cfg.name] = self._read_adapter_startup_params(
                slave=slave,
                cfg=cfg,
                adapter=adapters[cfg.name],
            )
            self._configure_pdo_mapping(slave, cfg)
            slaves_by_name[cfg.name] = slave

        # Build process data mapping before validating PDO buffer sizes.
        master.config_map()

        if self.config.strict_pdo_size:
            for cfg in self.config.slaves:
                slave = master.slaves[cfg.position]
                self._validate_pdo_sizes(cfg, slave, adapters[cfg.name])

        self._transition_to_operational(master)

        self._runtime = MasterRuntime(
            master=master,
            adapters=adapters,
            slaves_by_name=slaves_by_name,
            startup_params=startup_params,
        )
        return self._runtime

    def close(self) -> None:
        if self._runtime is None:
            return

        master = self._runtime.master
        try:
            master.state = pysoem.INIT_STATE
            master.write_state()
            # Let slaves settle to INIT before closing the socket to avoid
            # alternating startup remap failures on rapid reruns.
            try:
                master.state_check(pysoem.INIT_STATE, 50_000)
            except Exception:
                # Keep shutdown best-effort; close is still attempted below.
                pass
        finally:
            master.close()
            self._runtime = None

    def _transition_to_preop(self, master: Any) -> None:
        master.state = pysoem.PREOP_STATE
        master.write_state()
        master.state_check(pysoem.PREOP_STATE, 50_000)

    def _transition_to_operational(self, master: Any) -> None:
        """
        Transition bus through SAFE-OP into OP with process-data pumping.

        Some drives do not enter OP unless process data is exchanged while
        requesting OP.
        """

        safeop_state = getattr(pysoem, "SAFEOP_STATE", None)
        if safeop_state is not None:
            master.state = safeop_state
            master.write_state()
            master.state_check(safeop_state, 50_000)

        # Prime process data before requesting OP.
        for _ in range(5):
            master.send_processdata()
            master.receive_processdata(2_000)

        master.state = pysoem.OP_STATE
        master.write_state()

        # Keep exchanging process data while waiting for OP.
        for _ in range(50):
            master.send_processdata()
            master.receive_processdata(2_000)
            master.read_state()
            if self._all_configured_slaves_in_op(master):
                return
            time.sleep(0.01)

        raise MasterConfigError(self._format_state_error(master))

    @staticmethod
    def _validate_identity(cfg: SlaveConfig, slave: Any) -> None:
        if cfg.vendor_id and int(slave.man) != cfg.vendor_id:
            raise MasterConfigError(
                f"Slave '{cfg.name}' vendor mismatch: expected=0x{cfg.vendor_id:08X} got=0x{int(slave.man):08X}"
            )
        if cfg.product_code and int(slave.id) != cfg.product_code:
            raise MasterConfigError(
                f"Slave '{cfg.name}' product mismatch: expected=0x{cfg.product_code:08X} got=0x{int(slave.id):08X}"
            )

    @staticmethod
    def _matches_identity(cfg: SlaveConfig, slave: Any) -> bool:
        vendor_matches = not cfg.vendor_id or int(slave.man) == cfg.vendor_id
        product_matches = not cfg.product_code or int(slave.id) == cfg.product_code
        return vendor_matches and product_matches

    @staticmethod
    def _resolve_configured_position(master: Any, cfg: SlaveConfig) -> int:
        slave_count = len(master.slaves)
        if cfg.position < slave_count:
            configured_slave = master.slaves[cfg.position]
            if EthercatMaster._matches_identity(cfg, configured_slave):
                return cfg.position

        for position, slave in enumerate(master.slaves):
            if EthercatMaster._matches_identity(cfg, slave):
                return position

        raise MasterConfigError(
            f"No EtherCAT slave matched '{cfg.name}' "
            f"(vendor=0x{cfg.vendor_id:08X}, product=0x{cfg.product_code:08X})."
        )

    @staticmethod
    def _read_adapter_startup_params(
        *,
        slave: Any,
        cfg: SlaveConfig,
        adapter: SlaveAdapter[Any, Any],
    ) -> Dict[str, Any]:
        read_specs_fn = getattr(adapter, "startup_read_specs", None)
        if not callable(read_specs_fn):
            return {}

        specs = read_specs_fn()
        if not isinstance(specs, Mapping):
            raise MasterConfigError(
                f"Adapter startup_read_specs() for '{cfg.name}' must return a mapping."
            )

        values: Dict[str, Any] = {}
        for key, spec in specs.items():
            if not isinstance(spec, SdoReadSpec):
                raise MasterConfigError(
                    f"Invalid SDO read spec '{key}' for '{cfg.name}'; expected SdoReadSpec."
                )
            values[key] = EthercatMaster._read_sdo_with_retry(
                slave=slave,
                cfg=cfg,
                spec=spec,
            )
        return values

    @staticmethod
    def _read_sdo_with_retry(*, slave: Any, cfg: SlaveConfig, spec: SdoReadSpec) -> Any:
        last_exc: Exception | None = None
        for attempt in range(5):
            try:
                raw = slave.sdo_read(int(spec.index), int(spec.subindex))
                return EthercatMaster._decode_sdo_value(raw, spec)
            except Exception as exc:
                last_exc = exc
                if attempt < 4:
                    time.sleep(0.02)

        raise MasterConfigError(
            f"Startup SDO read failed for '{cfg.name}' key='{spec.name}' "
            f"at 0x{int(spec.index):04X}:{int(spec.subindex):02X} type={spec.data_type}: {last_exc}"
        ) from last_exc

    @staticmethod
    def _decode_sdo_value(raw: Any, spec: SdoReadSpec) -> Any:
        if isinstance(raw, int):
            # Some wrappers may return a scalar for narrow values.
            return int(raw)
        if not isinstance(raw, (bytes, bytearray)):
            raise ValueError(f"Unexpected SDO payload type: {type(raw)}")

        data = bytes(raw)
        dtype = spec.data_type
        expected_sizes = {
            "u8": 1,
            "s8": 1,
            "u16": 2,
            "s16": 2,
            "u32": 4,
            "s32": 4,
            "f32": 4,
        }
        if dtype in expected_sizes and len(data) < expected_sizes[dtype]:
            raise ValueError(
                f"SDO payload too short for {spec.name}: got={len(data)} expected>={expected_sizes[dtype]}"
            )

        if dtype == "bytes":
            return data
        if dtype == "u8":
            return int.from_bytes(data[:1], "little", signed=False)
        if dtype == "s8":
            return int.from_bytes(data[:1], "little", signed=True)
        if dtype == "u16":
            return int.from_bytes(data[:2], "little", signed=False)
        if dtype == "s16":
            return int.from_bytes(data[:2], "little", signed=True)
        if dtype == "u32":
            return int.from_bytes(data[:4], "little", signed=False)
        if dtype == "s32":
            return int.from_bytes(data[:4], "little", signed=True)
        if dtype == "f32":
            return float(struct.unpack("<f", data[:4])[0])

        raise ValueError(f"Unsupported SDO data_type '{dtype}'.")

    @staticmethod
    def _configure_pdo_mapping(slave: Any, cfg: SlaveConfig) -> None:
        """
        Optional startup PDO mapping hook.

        Each item in `cfg.pdo_mapping` should include:
        - `index` (int)
        - `subindex` (int)
        - `value` (int)
        - `size` (int, bytes)
        """

        for item in cfg.pdo_mapping:
            index = int(item["index"])
            subindex = int(item["subindex"])
            value = int(item["value"])
            size = int(item["size"])
            payload = value.to_bytes(size, byteorder="little")

            # Some drives reject remap writes briefly after mode/state changes.
            # Retry with short backoff to absorb transient busy/transition states.
            last_exc: Exception | None = None
            for attempt in range(5):
                try:
                    slave.sdo_write(index, subindex, payload)
                    last_exc = None
                    break
                except Exception as exc:
                    last_exc = exc
                    if attempt < 4:
                        time.sleep(0.02)

            if last_exc is not None:
                raise MasterConfigError(
                    f"PDO mapping SDO write failed for '{cfg.name}' at "
                    f"0x{index:04X}:{subindex:02X} value={value} size={size}: {last_exc}"
                ) from last_exc

    @staticmethod
    def _validate_pdo_sizes(
        cfg: SlaveConfig, slave: Any, adapter: SlaveAdapter[Any, Any]
    ) -> None:
        slave_rx_size, slave_tx_size = EthercatMaster._get_slave_pdo_sizes(slave)
        if slave_rx_size != adapter.rx_pdo_size:
            raise MasterConfigError(
                f"RX PDO size mismatch for '{cfg.name}': expected={adapter.rx_pdo_size} got={slave_rx_size}"
            )
        if slave_tx_size != adapter.tx_pdo_size:
            raise MasterConfigError(
                f"TX PDO size mismatch for '{cfg.name}': expected={adapter.tx_pdo_size} got={slave_tx_size}"
            )

    @staticmethod
    def _get_slave_pdo_sizes(slave: Any) -> tuple[int, int]:
        """
        Return (rx_size_bytes, tx_size_bytes) for a slave.

        pysoem versions differ in what attributes they expose, so this probes
        several forms and falls back to mapped process data buffers.
        """

        # Variant 1: explicit byte counts.
        obytes = getattr(slave, "obytes", None)
        ibytes = getattr(slave, "ibytes", None)
        if obytes is not None and ibytes is not None:
            return int(obytes), int(ibytes)

        # Variant 2: explicit bit counts.
        obits = getattr(slave, "obits", None)
        ibits = getattr(slave, "ibits", None)
        if obits is not None and ibits is not None:
            return int(obits) // 8, int(ibits) // 8

        # Variant 3: mapped process data buffers.
        output = getattr(slave, "output", b"")
        input_ = getattr(slave, "input", b"")
        if output is not None and input_ is not None:
            return len(bytes(output)), len(bytes(input_))

        raise MasterConfigError("Unable to determine slave PDO sizes from pysoem object.")

    def _all_configured_slaves_in_op(self, master: Any) -> bool:
        op_state = int(pysoem.OP_STATE)
        for cfg in self.config.slaves:
            state = int(master.slaves[cfg.position].state) & 0x0F
            if state != op_state:
                return False
        return True

    def _format_state_error(self, master: Any) -> str:
        lines = ["Failed to reach OP for all configured slaves."]
        for cfg in self.config.slaves:
            slave = master.slaves[cfg.position]
            state = int(getattr(slave, "state", 0))
            al_status_code = int(
                getattr(slave, "al_status_code", getattr(slave, "al_status", 0))
            )
            lines.append(
                f"  {cfg.name} pos={cfg.position} state=0x{state:02X} al_status=0x{al_status_code:04X}"
            )
        return "\n".join(lines)


def al_state_name(state_code: int) -> str:
    """Best-effort human-readable AL state from raw state code."""

    base = state_code & 0x0F
    has_error = bool(state_code & int(EthercatAlStates.ERROR_FLAG))
    mapping = {
        int(EthercatAlStates.INIT): "INIT",
        int(EthercatAlStates.PRE_OPERATIONAL): "PRE-OP",
        int(EthercatAlStates.BOOTSTRAP): "BOOT",
        int(EthercatAlStates.SAFE_OPERATIONAL): "SAFE-OP",
        int(EthercatAlStates.OPERATIONAL): "OP",
    }
    label = mapping.get(base, f"UNKNOWN(0x{state_code:02X})")
    return f"{label}+ERR" if has_error else label


def resolve_slave_position(config: MasterConfig, slave_name: str) -> int:
    """
    Resolve a configured slave name to a live bus position.

    The configured position is used first. If that position is missing or its
    vendor/product identity does not match the topology entry, the full chain is
    scanned for a matching vendor/product pair.
    """

    pysoem_mod = require_pysoem()

    target_cfg = next((cfg for cfg in config.slaves if cfg.name == slave_name), None)
    if target_cfg is None:
        raise MasterConfigError(
            f"Unknown configured slave '{slave_name}'. Available: {[cfg.name for cfg in config.slaves]}"
        )

    master = pysoem_mod.Master()
    master.open(config.iface)
    try:
        slave_count = master.config_init()
        if slave_count <= 0:
            raise RuntimeError("No EtherCAT slaves detected.")

        return EthercatMaster._resolve_configured_position(master, target_cfg)
    finally:
        master.close()
