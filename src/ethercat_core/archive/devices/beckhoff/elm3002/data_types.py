"""Beckhoff ELM3002 command/data model (AI Oversampling)."""

from __future__ import annotations

from dataclasses import dataclass, field


@dataclass(frozen=True, slots=True)
class Elm3002PdoField:
    """One ELM3002 TX PDO field mapping."""

    name: str
    pdo_index: int
    offset: int
    size: int
    signed: bool = False


# ELM3002 default TX PDO layout:
#   0x1A00  PAI Status Ch.1    (pai_status_1,   4 bytes)
#   0x1A01  PAI Samples Ch.1   (pai_samples_1,  4 bytes)  — subindex001 INT32
#   0x1A10  Timestamp           (timestamp,      8 bytes)
#   0x1A21  PAI Status Ch.2    (pai_status_2,   4 bytes)
#   0x1A22  PAI Samples Ch.2   (pai_samples_2,  4 bytes)  — subindex001 INT32
#
# Reference: ELM3002.java / YoELM3002.java (phantom-hardware repo)
ELM3002_INPUT_FIELD_SIZE = 4
ELM3002_SAMPLES_FIELD_SIZE = 4
ELM3002_TIMESTAMP_FIELD_SIZE = 8
ELM3002_TX_PDO_FIELDS = (
    Elm3002PdoField("pai_status_1", 0x1A00, 0, ELM3002_INPUT_FIELD_SIZE),
    Elm3002PdoField(
        "pai_samples_1",
        0x1A01,
        ELM3002_INPUT_FIELD_SIZE,
        ELM3002_SAMPLES_FIELD_SIZE,
        signed=True,
    ),
    Elm3002PdoField(
        "timestamp",
        0x1A10,
        ELM3002_INPUT_FIELD_SIZE + ELM3002_SAMPLES_FIELD_SIZE,
        ELM3002_TIMESTAMP_FIELD_SIZE,
    ),
    Elm3002PdoField(
        "pai_status_2",
        0x1A21,
        ELM3002_INPUT_FIELD_SIZE
        + ELM3002_SAMPLES_FIELD_SIZE
        + ELM3002_TIMESTAMP_FIELD_SIZE,
        ELM3002_INPUT_FIELD_SIZE,
    ),
    Elm3002PdoField(
        "pai_samples_2",
        0x1A22,
        ELM3002_INPUT_FIELD_SIZE
        + ELM3002_SAMPLES_FIELD_SIZE
        + ELM3002_TIMESTAMP_FIELD_SIZE
        + ELM3002_INPUT_FIELD_SIZE,
        ELM3002_SAMPLES_FIELD_SIZE,
        signed=True,
    ),
)
ELM3002_TX_PDO_SIZE = sum(field.size for field in ELM3002_TX_PDO_FIELDS)


# Bit layout of the 32-bit PAI Status word (0x6000 ch1 / 0x6010 ch2):
#
#   Bits  0- 7 : No of Samples       (60n0:01, UINT8)   — valid sample count this cycle
#   Bit       8 : Error               (60n0:09, BOOLEAN)
#   Bit       9 : Underrange          (60n0:0A, BOOLEAN)
#   Bit      10 : Overrange           (60n0:0B, BOOLEAN)
#   Bit      11 : Diag                (60n0:0D, BOOLEAN)
#   Bit      12 : TxPDO State         (60n0:0E, BOOLEAN) — TRUE: data invalid
#   Bits 13-14  : Input cycle counter (60n0:0F, BIT2)   — incremented when values change
#   Bits 15-31  : reserved
#
# Reference: Beckhoff ELM3xxx documentation, section 4.2.3.3
_MASK_NUM_SAMPLES          = 0x00FF
_SHIFT_NUM_SAMPLES         = 0
_BIT_ERROR                 = 0x0100
_BIT_UNDERRANGE            = 0x0200
_BIT_OVERRANGE             = 0x0400
_BIT_DIAG                  = 0x0800
_BIT_TXPDO_STATE           = 0x1000
_MASK_INPUT_CYCLE_COUNTER  = 0x6000
_SHIFT_INPUT_CYCLE_COUNTER = 13


@dataclass(frozen=True, slots=True)
class Elm3002PaiStatus:
    """Decoded ELM3002 PAI Status word (pai_status_1 or pai_status_2)."""

    num_samples: int          # UINT8: number of valid samples in this PDO cycle
    error: bool
    underrange: bool
    overrange: bool
    diag: bool
    txpdo_state: bool         # TRUE = data invalid
    input_cycle_counter: int  # BIT2: incremented each cycle when values have changed


def decode_pai_status(raw: int) -> Elm3002PaiStatus:
    """Decode the 32-bit ELM3002 PAI Status word into its named fields."""
    return Elm3002PaiStatus(
        num_samples=(raw & _MASK_NUM_SAMPLES) >> _SHIFT_NUM_SAMPLES,
        error=bool(raw & _BIT_ERROR),
        underrange=bool(raw & _BIT_UNDERRANGE),
        overrange=bool(raw & _BIT_OVERRANGE),
        diag=bool(raw & _BIT_DIAG),
        txpdo_state=bool(raw & _BIT_TXPDO_STATE),
        input_cycle_counter=(raw & _MASK_INPUT_CYCLE_COUNTER) >> _SHIFT_INPUT_CYCLE_COUNTER,
    )


@dataclass(slots=True)
class Elm3002Command:
    """Command model for the ELM3002 (input-only terminal)."""


@dataclass(slots=True)
class Elm3002Data:
    """Observed ELM3002 process-data state."""

    pai_status_1: int = 0
    pai_samples_1: int = 0
    timestamp: int = 0
    pai_status_2: int = 0
    pai_samples_2: int = 0
    raw_pdo: bytes = field(default_factory=bytes)
