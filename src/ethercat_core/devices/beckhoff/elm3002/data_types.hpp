#pragma once

#include <cstdint>
#include <vector>

namespace ethercat_core::beckhoff::elm3002 {

// ELM3002 default TX PDO layout (24 bytes total):
//   0x1A00  PAI Status Ch.1   — uint32, offset  0
//   0x1A01  PAI Samples Ch.1  — int32,  offset  4
//   0x1A10  Timestamp         — uint64, offset  8
//   0x1A21  PAI Status Ch.2   — uint32, offset 16
//   0x1A22  PAI Samples Ch.2  — int32,  offset 20
static constexpr int TX_PDO_SIZE = 24;

// Bit layout of the 32-bit PAI Status word (0x6000 ch1 / 0x6010 ch2):
//   Bits  0–7  : NumOfSamples       (UINT8) — valid sample count this cycle
//   Bit      8 : Error              (BOOL)
//   Bit      9 : Underrange         (BOOL)
//   Bit     10 : Overrange          (BOOL)
//   Bit     11 : Diag               (BOOL)
//   Bit     12 : TxPDO State        (BOOL)  — TRUE = data invalid
//   Bits 13–14 : InputCycleCounter  (BIT2)  — increments when values change
//   Bits 15–31 : reserved
struct PaiStatus {
    uint8_t num_samples          = 0;
    bool    error                = false;
    bool    underrange           = false;
    bool    overrange            = false;
    bool    diag                 = false;
    bool    txpdo_state          = false;
    uint8_t input_cycle_counter  = 0;
};

PaiStatus decodePaiStatus(uint32_t raw);

struct Command {};  // input-only terminal, no output

struct Data {
    uint32_t             pai_status_1  = 0;
    int32_t              pai_samples_1 = 0;
    uint64_t             timestamp     = 0;
    uint32_t             pai_status_2  = 0;
    int32_t              pai_samples_2 = 0;
    std::vector<uint8_t> raw_pdo;
};

} // namespace ethercat_core::beckhoff::elm3002
