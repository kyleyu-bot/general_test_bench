#pragma once

#include "ethercat_core/devices/base.hpp"
#include "ethercat_core/devices/beckhoff/elm3002/data_types.hpp"

namespace ethercat_core::beckhoff::elm3002 {

// Allowed torque scale values for ADC-to-torque conversion.
static constexpr float ALLOWED_TORQUE_SCALES[] = {20.0f, 200.0f, 500.0f};

class Elm3002Adapter : public ISlaveAdapter {
public:
    explicit Elm3002Adapter(
        SlaveIdentity id,
        float         ch1_torque_scale = 200.0f,
        float         ch2_torque_scale = 20.0f
    );

    int rxPdoSize() const override { return 0; }
    int txPdoSize() const override { return TX_PDO_SIZE; }

    std::vector<uint8_t> packRxPdo(const std::any& command) override;
    std::any unpackTxPdo(
        const uint8_t* data, int size,
        uint64_t seq = 0, int64_t stamp_ns = 0,
        int64_t cycle_time_ns = 0, int64_t dc_error_ns = 0
    ) override;

    // Helpers to decode the latest Data object.
    PaiStatus getPaiStatus1(const Data& d) const;
    PaiStatus getPaiStatus2(const Data& d) const;

    // Normalize 24-bit signed ADC value to [-1.0, 1.0].
    static float scaleAdc(int32_t sample);

    // Convert 24-bit signed ADC value to volts (±5 V full scale).
    static float scaleAdcToVoltage(int32_t sample);

    float scaledTorqueCh1(const Data& d) const;
    float scaledTorqueCh2(const Data& d) const;

    void setCh1TorqueScale(float s);
    void setCh2TorqueScale(float s);

    // Capture the current reading as the zero offset (one-shot, mirroring Java YoELM3002).
    void zeroTorqueCh1(const Data& d);
    void zeroTorqueCh2(const Data& d);

private:
    float ch1_torque_scale_;
    float ch2_torque_scale_;
    float ch1_offset_raw_ = 0.0f;   // normalized ADC offset [-1,1]; subtracted before scaling
    float ch2_offset_raw_ = 0.0f;

    static float validateTorqueScale(float s);
};

} // namespace ethercat_core::beckhoff::elm3002
