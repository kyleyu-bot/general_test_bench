#pragma once

#include "ethercat_core/master.hpp"
#include "ethercat_core/devices/beckhoff/el2004/adapter.hpp"
#include "ethercat_core/devices/beckhoff/elm3002/adapter.hpp"
#include "ethercat_core/devices/beckhoff/el5032/adapter.hpp"
#include "ethercat_core/devices/motor_drives/Novanta/Everest/adapter.hpp"
#include "ethercat_core/devices/motor_drives/Novanta/Volcano/adapter.hpp"

#include <memory>

namespace ethercat_core {

// Returns a factory covering all built-in device kinds.
// Use this in functional tests and anywhere that doesn't need a custom set.
inline EthercatMaster::AdapterFactory makeDefaultAdapterFactory() {
    return [](const SlaveConfig& cfg) -> std::unique_ptr<ISlaveAdapter> {
        SlaveIdentity id{cfg.name, cfg.position, cfg.vendor_id, cfg.product_code};

        if (cfg.kind == "everest")
            return std::make_unique<novanta::everest::NovantaEverestAdapter>(id);
        if (cfg.kind == "volcano")
            return std::make_unique<novanta::volcano::NovantaVolcanoAdapter>(id);
        if (cfg.kind == "EL2004")
            return std::make_unique<beckhoff::el2004::El2004Adapter>(id);
        if (cfg.kind == "ELM3002" || cfg.kind == "ELM3002")
            return std::make_unique<beckhoff::elm3002::Elm3002Adapter>(id);
        if (cfg.kind == "EL5032")
            return std::make_unique<beckhoff::el5032::El5032Adapter>(id);

        throw MasterConfigError("Unsupported slave kind '" + cfg.kind + "' for '" + cfg.name + "'.");
    };
}

} // namespace ethercat_core
