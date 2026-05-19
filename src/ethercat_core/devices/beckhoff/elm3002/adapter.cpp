#include "ethercat_core/devices/beckhoff/elm3002/adapter.hpp"
#include <cstring>
#include <stdexcept>
#include <sstream>

namespace ethercat_core::beckhoff::elm3002 {

// ── Free function ─────────────────────────────────────────────────────────────

PaiStatus decodePaiStatus(uint32_t raw) {
    PaiStatus s;
    s.num_samples         = static_cast<uint8_t>(raw & 0x00FFu);
    s.error               = (raw & 0x0100u) != 0;
    s.underrange          = (raw & 0x0200u) != 0;
    s.overrange           = (raw & 0x0400u) != 0;
    s.diag                = (raw & 0x0800u) != 0;
    s.txpdo_state         = (raw & 0x1000u) != 0;
    s.input_cycle_counter = static_cast<uint8_t>((raw & 0x6000u) >> 13u);
    return s;
}

// ── Elm3002Adapter ─────────────────────────────────────────────────────────────

Elm3002Adapter::Elm3002Adapter(SlaveIdentity id, float ch1_scale, float ch2_scale)
    : ISlaveAdapter(std::move(id))
    , ch1_torque_scale_(validateTorqueScale(ch1_scale))
    , ch2_torque_scale_(validateTorqueScale(ch2_scale))
{}

std::vector<uint8_t> Elm3002Adapter::packRxPdo(const std::any& /*command*/) {
    return {};  // input-only terminal
}

std::any Elm3002Adapter::unpackTxPdo(
    const uint8_t* data, int size,
    uint64_t /*seq*/, int64_t /*stamp_ns*/,
    int64_t /*cycle_time_ns*/, int64_t /*dc_error_ns*/)
{
    Data d;
    d.raw_pdo.assign(data, data + size);

    auto readU32 = [&](int offset) -> uint32_t {
        if (size < offset + 4) return 0;
        uint32_t v = 0;
        std::memcpy(&v, data + offset, 4);
        return v;
    };
    auto readI32 = [&](int offset) -> int32_t {
        if (size < offset + 4) return 0;
        int32_t v = 0;
        std::memcpy(&v, data + offset, 4);
        return v;
    };
    auto readU64 = [&](int offset) -> uint64_t {
        if (size < offset + 8) return 0;
        uint64_t v = 0;
        std::memcpy(&v, data + offset, 8);
        return v;
    };

    d.pai_status_1  = readU32(0);
    d.pai_samples_1 = readI32(4);
    d.timestamp     = readU64(8);
    d.pai_status_2  = readU32(16);
    d.pai_samples_2 = readI32(20);

    return d;
}

PaiStatus Elm3002Adapter::getPaiStatus1(const Data& d) const {
    return decodePaiStatus(d.pai_status_1);
}
PaiStatus Elm3002Adapter::getPaiStatus2(const Data& d) const {
    return decodePaiStatus(d.pai_status_2);
}

float Elm3002Adapter::scaleAdc(int32_t sample) {
    return static_cast<float>(sample) / static_cast<float>(1 << 23);
}
float Elm3002Adapter::scaleAdcToVoltage(int32_t sample) {
    return scaleAdc(sample) * 5.0f;
}

float Elm3002Adapter::scaledTorqueCh1(const Data& d) const {
    return (scaleAdc(d.pai_samples_1) - ch1_offset_raw_) * ch1_torque_scale_;
}
float Elm3002Adapter::scaledTorqueCh2(const Data& d) const {
    return (scaleAdc(d.pai_samples_2) - ch2_offset_raw_) * ch2_torque_scale_;
}

void Elm3002Adapter::setCh1TorqueScale(float s) { ch1_torque_scale_ = validateTorqueScale(s); }
void Elm3002Adapter::setCh2TorqueScale(float s) { ch2_torque_scale_ = validateTorqueScale(s); }

void Elm3002Adapter::zeroTorqueCh1(const Data& d) {
    ch1_offset_raw_ = scaleAdc(d.pai_samples_1);
}
void Elm3002Adapter::zeroTorqueCh2(const Data& d) {
    ch2_offset_raw_ = scaleAdc(d.pai_samples_2);
}

float Elm3002Adapter::validateTorqueScale(float s) {
    for (float allowed : ALLOWED_TORQUE_SCALES) {
        if (s == allowed) return s;
    }
    std::ostringstream oss;
    oss << "Unsupported ELM3002 torque scale " << s
        << ". Allowed: 20, 200, 500";
    throw std::invalid_argument(oss.str());
}

} // namespace ethercat_core::beckhoff::elm3002
