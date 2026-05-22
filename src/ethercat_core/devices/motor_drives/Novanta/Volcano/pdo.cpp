#include "ethercat_core/devices/motor_drives/Novanta/Volcano/pdo.hpp"
#include "ethercat_core/devices/motor_drives/drive_bases/ds402/pdo.hpp"

#include <cstring>
#include <stdexcept>

namespace ethercat_core::novanta::volcano {

static constexpr uint8_t AL_STATE_OPERATIONAL = 0x08u;

std::vector<uint8_t> packCommand(
    const Command& cmd,
    uint16_t       current_status_word,
    const PdoScaling* /*scaling*/)
{
    using namespace ds402;

    const Cia402State state = decodeCia402State(current_status_word);
    const uint16_t    cw   = controlwordFromCommand(cmd.enable_drive, cmd.clear_fault, state);

    RxPdo pdo{};
    pdo.controlword            = cw;
    pdo.mode_of_operation      = static_cast<int8_t>(cmd.mode_of_operation);
    pdo.target_position        = clampI32(static_cast<int64_t>(cmd.target_position__enc_cnt));
    pdo.target_velocity        = clampI32(static_cast<int64_t>(cmd.target_velocity_mrevs));
    pdo.torque_command_2022    = (cmd.torque_command_2022 != 0.0f)
                                      ? cmd.torque_command_2022
                                      : cmd.target_torque_nm;
    pdo.iq_setpoint            = cmd.iq_setpoint_a;
    pdo.torque_kp              = cmd.torque_kp;
    pdo.torque_loop_max_output = cmd.torque_loop_max_output;
    pdo.torque_loop_min_output = cmd.torque_loop_min_output;
    pdo.velocity_loop_kp       = cmd.velocity_loop_kp;
    pdo.velocity_loop_ki       = cmd.velocity_loop_ki;
    pdo.velocity_loop_kd       = cmd.velocity_loop_kd;
    pdo.position_loop_kp       = cmd.position_loop_kp;
    pdo.position_loop_ki       = cmd.position_loop_ki;
    pdo.position_loop_kd       = cmd.position_loop_kd;

    std::vector<uint8_t> buf(sizeof(RxPdo));
    std::memcpy(buf.data(), &pdo, sizeof(RxPdo));
    return buf;
}

DriveStatus unpackStatus(
    const uint8_t* data, int size,
    uint64_t seq, int64_t stamp_ns,
    int64_t cycle_time_ns, int64_t dc_error_ns,
    const PdoScaling* /*scaling*/)
{
    using namespace ds402;

    if (!data || size < LEGACY_TX_PDO_SIZE) {
        throw std::invalid_argument(
            "TX PDO payload too small for NovantaVolcano"
        );
    }

    uint16_t status_word          = 0;
    int8_t   mode_display         = 0;
    float    measured_torque_raw  = 0.0f;
    int32_t  measured_input_side_velocity_raw = 0;
    int32_t  measured_output_side_position_raw_cnt = 0;
    int32_t  input_encoder_pos     = 0;
    int32_t  position_setpoint     = 0;
    float    received_velocity_raw = 0.0f;
    float    bus_voltage           = 0.0f;
    float    motor_temp            = 0.0f;
    float    iq_actual             = 0.0f;
    float    id_actual             = 0.0f;
    float    idc_actual            = 0.0f;
    float    iq_command            = 0.0f;
    float    id_command            = 0.0f;
    uint16_t error_code            = 0;
    uint8_t  al_state_code         = 0;

    if (size >= TX_PDO_SIZE) {
        TxPdo pdo{};
        std::memcpy(&pdo, data, sizeof(TxPdo));

        status_word            = pdo.statusword;
        mode_display           = pdo.mode_display;
        error_code             = pdo.error_code;
        measured_torque_raw    = static_cast<float>(pdo.estimated_torque);
        measured_input_side_velocity_raw = pdo.motor_velocity;
        measured_output_side_position_raw_cnt = pdo.measured_output_encoder_position_raw;
        input_encoder_pos      = pdo.input_encoder_pos;
        position_setpoint      = pdo.position_setpoint;
        received_velocity_raw  = pdo.velocity_setpoint;
        bus_voltage            = pdo.bus_voltage;
        motor_temp             = pdo.motor_temp;
        iq_actual              = pdo.iq_actual;
        id_actual              = pdo.id_actual;
        idc_actual             = pdo.idc_actual;
        iq_command             = pdo.iq_command;
        id_command             = pdo.id_command;
        al_state_code          = (status_word != 0) ? AL_STATE_OPERATIONAL : 0u;
    } else {
        // Legacy layout: "<HbHhiiB"
        std::memcpy(&status_word,    data + 0,  2);
        std::memcpy(&mode_display,   data + 2,  1);
        std::memcpy(&error_code,     data + 3,  2);
        int16_t torque_raw_i16 = 0;
        std::memcpy(&torque_raw_i16, data + 5,  2);
        int32_t vel_raw_i32 = 0, pos_raw_i32 = 0;
        std::memcpy(&vel_raw_i32,    data + 7,  4);
        std::memcpy(&pos_raw_i32,    data + 11, 4);
        std::memcpy(&al_state_code,  data + 15, 1);
        measured_torque_raw    = static_cast<float>(torque_raw_i16);
        measured_input_side_velocity_raw = vel_raw_i32;
        measured_output_side_position_raw_cnt = pos_raw_i32;
    }

    const Cia402State    cia_state = decodeCia402State(status_word);
    const StatuswordBits bits      = decodeStatuswordBits(status_word);
    const uint8_t        al_base   = al_state_code & 0x0Fu;

    DriveStatus s;
    s.online                    = (al_base != 0);
    s.operational               = (al_base == AL_STATE_OPERATIONAL);
    s.faulted                   = (cia_state == Cia402State::FAULT ||
                                   cia_state == Cia402State::FAULT_REACTION_ACTIVE);
    s.al_state_code             = al_state_code;
    s.cia402_state              = cia_state;
    s.status_word               = status_word;
    s.mode_of_operation_display = mode_display;
    s.error_code                = error_code;
    s.ready_to_switch_on        = bits.ready_to_switch_on;
    s.switched_on               = bits.switched_on;
    s.operation_enabled         = bits.operation_enabled;
    s.fault                     = bits.fault;
    s.voltage_enabled           = bits.voltage_enabled;
    s.quick_stop_active         = bits.quick_stop_active;
    s.switch_on_disabled        = bits.switch_on_disabled;
    s.warning                   = bits.warning;
    s.remote                    = bits.remote;
    s.target_reached            = bits.target_reached;
    s.measured_torque_nm        = measured_torque_raw;
    s.measured_input_side_velocity_raw       = measured_input_side_velocity_raw;
    s.measured_output_side_position_raw_cnt  = measured_output_side_position_raw_cnt;
    s.input_encoder_pos         = input_encoder_pos;
    s.position_setpoint         = position_setpoint;
    s.velocity_command_received = received_velocity_raw;
    s.bus_voltage               = bus_voltage;
    s.motor_temp                = motor_temp;
    s.iq_actual                 = iq_actual;
    s.id_actual                 = id_actual;
    s.idc_actual                = idc_actual;
    s.iq_command                = iq_command;
    s.id_command                = id_command;
    s.dc_time_error_ns          = dc_error_ns;
    s.cycle_time_ns             = cycle_time_ns;
    s.seq                       = seq;
    s.stamp_ns                  = stamp_ns;
    return s;
}

} // namespace ethercat_core::novanta::volcano
