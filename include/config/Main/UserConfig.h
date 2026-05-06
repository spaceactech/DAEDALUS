#ifndef ROCKET_AVIONICS_TEMPLATE_USERCONFIG_H
#define ROCKET_AVIONICS_TEMPLATE_USERCONFIG_H

#include <cstdint>
#include <cstdlib>

// File Name
constexpr const char *RA_FILE_NAME = "OCTAVE_LOGGER_";

// File Extension
constexpr const char *RA_FILE_EXT = "CSV";

// Number of IMU sensors
constexpr size_t RA_NUM_IMU = 1;

// Number of Altimeter sensors
constexpr size_t RA_NUM_ALTIMETER = 2;

// Number of GNSS sensors
constexpr size_t RA_NUM_GNSS = 0;

// LEDs
constexpr bool RA_LED_ENABLED = false;

// USB Debug
constexpr bool RA_USB_DEBUG_ENABLED = true;

constexpr double MAGNETIC_DECLINATION  = 0.0;
constexpr double BNO_MOUNT_OFFSET      = 106.6;   // PCB mounting correction (degrees)

// Spool 1 physical offset from magnetometer reference (degrees)
// constexpr double SPOOL_PHYSICAL_OFFSET = 0.0;  // For test
constexpr double SPOOL_PHYSICAL_OFFSET = 45.0;  // For CanSat

// Stack High Water Mark (define to enable per-thread RAM reporting via serial)
// #define RA_STACK_HWM_ENABLED

// Minimum free stack words before a task is flagged with '!' in HWM output
// (uxTaskGetStackHighWaterMark returns words; 1 word = 4 bytes on ARM Cortex-M)
#define RA_STACK_HWM_MIN_WORDS 100

// Retain Deployment
constexpr bool RA_RETAIN_DEPLOYMENT_ENABLED = true;

// EEPROM feature flag — controls both restore-on-boot and periodic writes.
// When true, EEPROM_Read() restores the last saved state (if magic + CRC valid)
// and CB_EEPROMWrite runs every RA_EEPROM_WRITE_INTERVAL ms.
// Set false to disable all EEPROM activity and always start clean.
constexpr bool RA_EEPROM_ENABLED = true;

// EEPROM write interval (ms)
constexpr uint32_t RA_EEPROM_WRITE_INTERVAL = 10000ul;  // 10 s
  
// Auto-Zero Altitude
constexpr bool RA_AUTO_ZERO_ALT_ENABLED = false;

/* THREAD LOOP INTERVALS */

// u-blox GPS common timeout
constexpr uint32_t UBLOX_CUSTOM_MAX_WAIT = 250ul;

// IMU Reading
constexpr uint32_t RA_INTERVAL_IMU_READING = 5ul;  // ms

// Altimeter Reading
constexpr uint32_t RA_INTERVAL_ALTIMETER_READING = 50ul;  // ms

// GNSS Reading
constexpr uint32_t RA_INTERVAL_GNSS_READING = 100ul;  // ms

// MAG Reading
constexpr uint32_t RA_INTERVAL_MAG_READING = 50ul;  // ms

// TOF Reading
constexpr uint32_t RA_INTERVAL_TOF_READING = 100ul;  // ms

// INA Reading
constexpr uint32_t RA_INTERVAL_INA_READING = 200ul;  // ms

// FSM Evaluation
constexpr uint32_t RA_INTERVAL_FSM_EVAL = 10ul;  // ms

// FSM Evaluation interval maximum jitter tolerance
constexpr uint32_t RA_JITTER_TOLERANCE_FSM_EVAL = 1ul;  // ms

// Data Construct
constexpr uint32_t RA_INTERVAL_CONSTRUCT = 100ul;  // ms

// Control Loop
constexpr uint32_t RA_INTERVAL_Controlling = 50ul;  // ms 20Hz

// Altitude Auto-Zero
constexpr uint32_t RA_INTERVAL_AUTOZERO = 50ul;  // ms

/* BOARD FEATURES */

// Start-up Countdown (for time-based arming)
constexpr bool     RA_STARTUP_COUNTDOWN_ENABLED = true;
constexpr uint32_t RA_STARTUP_COUNTDOWN         = 0. * 1000ul;

/* ACTUATOR SETTINGS */

constexpr int RA_SERVO_A_MIN = 1000;                                // us PWM
constexpr int RA_SERVO_A_MAX = 2000;                               // us PWM
constexpr int RA_SERVO_A_CEN = (RA_SERVO_A_MIN + RA_SERVO_A_MAX) / 2;  // us PWM

constexpr int RA_SERVO_MIN = 500;                                // us PWM
constexpr int RA_SERVO_MAX = 2450;                               // us PWM
constexpr int RA_SERVO_CEN = (RA_SERVO_MIN + RA_SERVO_MAX) / 2;  // us PWM

constexpr float RA_SERVO_A_RELEASE = std::min(45 * 1.8, 180.0);   // deg
constexpr float RA_SERVO_A_LOCK    = std::min(0 * 1.8, 180.0);  // deg

constexpr float RA_SERVO_B_RELEASE = 180;  // deg
constexpr float RA_SERVO_B_LOCK    = 25;   // deg

/* SAMPLER SETTINGS */

// True to false ratio for comparator
constexpr double RA_TRUE_TO_FALSE_RATIO = 1.0;  // (#True / #False), 0.5 = 33.3% 1.0 = 50%, 2.0 = 66.7%

/* LAUNCH CONFIGURATION */

// Enable time-based state-transition guards (state_millis_elapsed) in EvalFSM.
// When false, ASCENT uses velocity detection only; PROBE_REALEASE uses altitude only.
constexpr bool RA_FSM_TIME_GUARD_ENABLED = true;

// Safeguard minimum time to apogee - drogue deployment
constexpr uint32_t RA_TIME_TO_APOGEE_MIN = 10 * 1000ul;  // ms

// Safeguard maximum time to apogee - drogue deployment
constexpr uint32_t RA_TIME_TO_APOGEE_MAX = 13.5 * 1000ul;  // ms

// Launch acceleration: acc. threshold (GT)
constexpr double RA_LAUNCH_ACC = 9.81 * 5.0;  // 9.81 m/s^2 (g)
constexpr double RA_LAUNCH_ALT = 20.0;         // m

// Launch acceleration detection period
constexpr uint32_t RA_LAUNCH_TON     = 200ul;  // ms
constexpr uint32_t RA_LAUNCH_SAMPLES = RA_LAUNCH_TON / RA_INTERVAL_FSM_EVAL;

// Apogee altitude (nominal for safeguard calculation, runtime-adjustable via SET,APOGEE_ALT)
inline double RA_APOGEE_ALT = 850.0;  // m

// Velocity at Apogee: vel. threshold (LT)
constexpr double RA_APOGEE_VEL = 12.5;  // m/s

// Velocity at Apogee detection period
constexpr uint32_t RA_APOGEE_TON     = 500ul;  // ms
constexpr uint32_t RA_APOGEE_SAMPLES = RA_APOGEE_TON / RA_INTERVAL_FSM_EVAL;

// Drogue Descent Theoretical Velocity
constexpr double RA_DROGUE_VEL = 14.0;  // m/s
constexpr double RA_MAIN_VEL   = 5.0;   // m/s

// Main Deployment Event Altitude: altitude threshold (LT)
constexpr double RA_INS_ALT_RAW  = 2.0;                  // m

// INS deployment baro thresholds (runtime-adjustable via SET,INS_TOF / SET,INS_NEAR / SET,INS_CRIT)
inline double RA_INS_TOF_THRESHOLD  = RA_INS_ALT_RAW;  // m — TOF trigger
inline double RA_INS_NEAR_THRESHOLD = 1.5;             // m — baro near-ground trigger
inline double RA_INS_CRIT_THRESHOLD = 2.0;              // m — baro critical trigger

// Safeguard overspeed threshold to main deployment
constexpr double RA_MAIN_OVERSPEED_VEL = RA_DROGUE_VEL * 1.5;

// Main Deployment Event Altitude detection period
constexpr uint32_t RA_MAIN_TON     = 500ul;  // ms
constexpr uint32_t RA_MAIN_SAMPLES = RA_MAIN_TON / RA_INTERVAL_FSM_EVAL;

// Main Deployment Event Altitude detection period
constexpr uint32_t RA_INS_TON     = 50ul;  // ms
constexpr uint32_t RA_INS_SAMPLES = RA_INS_TON / RA_INTERVAL_FSM_EVAL;

// Main Deployment Event Triggering Delay Compensation Multiplier
constexpr double RA_MAIN_COMPENSATION_MULT = 2.0;
constexpr double RA_INS_COMPENSATION_MULT  = 2.0;

// INS Deployment Event Triggering Delay Compensation Value
constexpr double RA_INS_ALT_COMPENSATED = RA_INS_ALT_RAW + RA_INS_COMPENSATION_MULT * RA_MAIN_VEL * (static_cast<double>(RA_MAIN_TON) / 1000.);  // m

// Main deployment altitude default (APOGEE state auto-overrides with apogee_raw * 0.8)
inline double RA_MAIN_ALT_COMPENSATED = 500;  // m — runtime-adjustable via SET,MAIN_ALT

// Main Deployment Event Altitude: altitude threshold (LT)
constexpr double RA_MAIN_ALT_RAW = 760.0;  // m

// Safeguard nominal time to main deployment
inline uint32_t RA_TIME_TO_MAIN_NOM = static_cast<uint32_t>((RA_APOGEE_ALT - RA_MAIN_ALT_RAW) / RA_DROGUE_VEL) * 1000ul;  // ms

// Safeguard minimum time to main deployment
inline uint32_t RA_TIME_TO_MAIN_MIN = RA_TIME_TO_MAIN_NOM * (1.00 - 0.15);  // ms

// Safeguard maximum time to main deployment
inline uint32_t RA_TIME_TO_MAIN_MAX = RA_TIME_TO_MAIN_NOM * (1.00 + 0.05);  // ms

// Velocity at Landed State: vel. threshold (LT)
constexpr double RA_LANDED_ALT = 0.1;  // m/s

// Velocity at Landed State detection period
constexpr uint32_t RA_LANDED_TON     = 10000ul;  // ms
constexpr uint32_t RA_LANDED_SAMPLES = RA_LANDED_TON / RA_INTERVAL_FSM_EVAL;

// Auto Zero Altitude stillness: vel. threshold (LT)
constexpr double RA_AUTOZERO_VEL = 0.30;  // m/s

// Auto Zero Altitude stillness detection period
constexpr uint32_t RA_AUTOZERO_TON     = 5000ul;  // ms
constexpr uint32_t RA_AUTOZERO_SAMPLES = RA_AUTOZERO_TON / RA_INTERVAL_AUTOZERO;

/* SD CARD LOGGER INTERVALS */

constexpr uint32_t RA_SDLOGGER_INTERVAL_IDLE     = 2000ul;  // 0.5 Hz
constexpr uint32_t RA_SDLOGGER_INTERVAL_SLOW     = 500ul;   // 2 Hz
constexpr uint32_t RA_SDLOGGER_INTERVAL_FAST     = 200ul;   // 5 Hz
constexpr uint32_t RA_SDLOGGER_INTERVAL_REALTIME = 100ul;   // 10 Hz

/* TELEMETRY */

inline uint32_t RA_TX_INTERVAL_MS = 1000ul;  // ms — runtime-adjustable via SET,TX_RATE

/* GPS / NAVIGATION */

inline bool RA_USE_KF_GPS = false;  // true=KF-smoothed GPS, false=raw GPS for guidance NEED TO TUNE MUCH ON KF IF USE

// Static assertions validate settings
namespace details::assertions {
  static_assert(RA_TIME_TO_APOGEE_MAX >= RA_TIME_TO_APOGEE_MIN, "Time to apogee is configured incorrectly!");
}  // namespace details::assertions

#endif  //ROCKET_AVIONICS_TEMPLATE_USERCONFIG_H
