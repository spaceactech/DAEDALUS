#ifndef ROCKET_AVIONICS_TEMPLATE_USERCONFIG_H
#define ROCKET_AVIONICS_TEMPLATE_USERCONFIG_H

#include <cstdint>
#include <cstdlib>

// File Name
constexpr const char *RA_FILE_NAME = "DDL_LOGGER_";

// File Extension
constexpr const char *RA_FILE_EXT = "CSV";

// Number of IMU sensors
constexpr size_t RA_NUM_IMU = 1;
  
// Number of Altimeter sensors
constexpr size_t RA_NUM_ALTIMETER = 1;

// Number of GNSS sensors
constexpr size_t RA_NUM_GNSS = 0;

// LEDs
constexpr bool RA_LED_ENABLED = true;

// USB Debug
constexpr bool RA_USB_DEBUG_ENABLED = true;

// Retain Deployment
constexpr bool RA_RETAIN_DEPLOYMENT_ENABLED = true;

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
constexpr uint32_t RA_INTERVAL_GNSS_READING = 56ul;  // ms

// GNSS Reading
constexpr uint32_t RA_INTERVAL_MAG_READING = 30ul;  // ms

// GNSS Reading
constexpr uint32_t RA_INTERVAL_TOF_READING = 20ul;  // ms

// GNSS Reading
constexpr uint32_t RA_INTERVAL_SENSORS_READING = 500ul;  // ms

// FSM Evaluation
constexpr uint32_t RA_INTERVAL_FSM_EVAL = 10ul;  // ms

// FSM Evaluation interval maximum jitter tolerance
constexpr uint32_t RA_JITTER_TOLERANCE_FSM_EVAL = 1ul;  // ms

// Data Construct
constexpr uint32_t RA_INTERVAL_CONSTRUCT = 100ul;  // ms

// Data Construct
constexpr uint32_t RA_INTERVAL_Controlling = 20ul;  // ms

// Altitude Auto-Zero
constexpr uint32_t RA_INTERVAL_AUTOZERO = 50ul;  // ms

/* BOARD FEATURES */

// Start-up Countdown (for time-based arming)
constexpr bool     RA_STARTUP_COUNTDOWN_ENABLED = true;
constexpr uint32_t RA_STARTUP_COUNTDOWN         = 0. * 1000ul;

/* ACTUATOR SETTINGS */

constexpr int RA_SERVO_MIN = 500;                                // us PWM
constexpr int RA_SERVO_MAX = 2450;                               // us PWM
constexpr int RA_SERVO_CEN = (RA_SERVO_MIN + RA_SERVO_MAX) / 2;  // us PWM

constexpr float RA_SERVO_A_RELEASE = 180;    // deg
constexpr float RA_SERVO_A_LOCK    = 115;  // deg

constexpr float RA_SERVO_B_RELEASE = 180;    // deg
constexpr float RA_SERVO_B_LOCK    = 25;  // deg


/* SAMPLER SETTINGS */

// True to false ratio for comparator
constexpr double RA_TRUE_TO_FALSE_RATIO = 1.0;  // (#True / #False), 0.5 = 33.3% 1.0 = 50%, 2.0 = 66.7%

/* LAUNCH CONFIGURATION */

// Safeguard minimum time to motor burnout
constexpr uint32_t RA_TIME_TO_BURNOUT_MIN = 0.4 * 1000ul;  // ms

// Safeguard maximum time to motor burnout
constexpr uint32_t RA_TIME_TO_BURNOUT_MAX = 0.7 * 1000ul;  // ms

// Safeguard minimum time to apogee - drogue deployment
constexpr uint32_t RA_TIME_TO_APOGEE_MIN = 35 * 1000ul;  // ms

// Safeguard maximum time to apogee - drogue deployment
constexpr uint32_t RA_TIME_TO_APOGEE_MAX = 44 * 1000ul;  // ms

// Launch acceleration: acc. threshold (GT)
constexpr double RA_LAUNCH_ACC = 9.81 * 3.0;  // 9.81 m/s^2 (g)
constexpr double RA_LAUNCH_ALT = 50.0;   // m

// Launch acceleration detection period
constexpr uint32_t RA_LAUNCH_TON     = 200ul;  // ms
constexpr uint32_t RA_LAUNCH_SAMPLES = 1200;   // RA_LAUNCH_TON / RA_INTERVAL_FSM_EVAL;

// Motor burnout detection: acc. threshold (LT)
constexpr double RA_BURNOUT_ACC = 0.60 * RA_LAUNCH_ACC;  // 9.81 m/s^2 (g)

// Motor burnout detection period
constexpr uint32_t RA_BURNOUT_TON     = 500ul;  // ms
constexpr uint32_t RA_BURNOUT_SAMPLES = RA_BURNOUT_TON / RA_INTERVAL_FSM_EVAL;

// Apogee altitude (nominal for safeguard calculation)
constexpr double RA_APOGEE_ALT = 668.0;  // m

// Velocity at Apogee: vel. threshold (LT)
constexpr double RA_APOGEE_VEL = 10.0;  // m/s

// Velocity at Apogee detection period
constexpr uint32_t RA_APOGEE_TON     = 500ul;  // ms
constexpr uint32_t RA_APOGEE_SAMPLES = RA_APOGEE_TON / RA_INTERVAL_FSM_EVAL;

// Drogue Descent Theoretical Velocity
constexpr double RA_DROGUE_VEL = 14.0;  // m/s

// Main Deployment Event Altitude: altitude threshold (LT)
constexpr double RA_MAIN_ALT_RAW = 534.4;  // m Apogee * 0.8
constexpr double RA_INS_ALT_RAW = 2.0;  // m

// Safeguard overspeed threshold to main deployment
constexpr double RA_MAIN_OVERSPEED_VEL = RA_DROGUE_VEL * 1.5;

// Main Deployment Event Altitude detection period
constexpr uint32_t RA_MAIN_OVERSPEED_TON     = 500ul;  // ms
constexpr uint32_t RA_MAIN_OVERSPEED_SAMPLES = 1500;   //RA_MAIN_OVERSPEED_TON / RA_INTERVAL_FSM_EVAL;

// Safeguard nominal time to main deployment
constexpr uint32_t RA_TIME_TO_MAIN_NOM = static_cast<uint32_t>((RA_APOGEE_ALT - RA_MAIN_ALT_RAW) / RA_DROGUE_VEL) * 1000ul;  // ms

// Safeguard minimum time to main deployment
constexpr uint32_t RA_TIME_TO_MAIN_MIN = RA_TIME_TO_MAIN_NOM * (1.00 - 0.15);  // ms

// Safeguard maximum time to main deployment
constexpr uint32_t RA_TIME_TO_MAIN_MAX = RA_TIME_TO_MAIN_NOM * (1.00 + 0.05);  // ms

// Main Deployment Event Altitude detection period
constexpr uint32_t RA_MAIN_TON     = 500ul;  // ms
constexpr uint32_t RA_MAIN_SAMPLES = RA_MAIN_TON / RA_INTERVAL_FSM_EVAL;

// Main Deployment Event Altitude detection period
constexpr uint32_t RA_INS_TON     = 500ul;  // ms
constexpr uint32_t RA_INS_SAMPLES = RA_MAIN_TON / RA_INTERVAL_FSM_EVAL;

// Main Deployment Event Triggering Delay Compensation Multiplier
constexpr double RA_MAIN_COMPENSATION_MULT = 2.0;
constexpr double RA_INS_COMPENSATION_MULT = 2.0;

// Main Deployment Event Triggering Delay Compensation Value
constexpr double RA_MAIN_ALT_COMPENSATED = RA_MAIN_ALT_RAW + RA_MAIN_COMPENSATION_MULT * RA_DROGUE_VEL * (static_cast<double>(RA_MAIN_TON) / 1000.);  // m

// INS Deployment Event Triggering Delay Compensation Value
constexpr double RA_INS_ALT_COMPENSATED = RA_INS_ALT_RAW + RA_INS_COMPENSATION_MULT * RA_DROGUE_VEL * (static_cast<double>(RA_MAIN_TON) / 1000.);  // m

// Velocity at Landed State: vel. threshold (LT)
constexpr double RA_LANDED_VEL = 0.5;  // m/s

// Velocity at Landed State detection period
constexpr uint32_t RA_LANDED_TON     = 5000ul;  // ms
constexpr uint32_t RA_LANDED_SAMPLES = RA_LANDED_TON / RA_INTERVAL_FSM_EVAL;

// Auto Zero Altitude stillness: vel. threshold (LT)
constexpr double RA_AUTOZERO_VEL = 0.30;  // m/s

// Auto Zero Altitude stillness detection period
constexpr uint32_t RA_AUTOZERO_TON     = 5000ul;  // ms
constexpr uint32_t RA_AUTOZERO_SAMPLES = RA_AUTOZERO_TON / RA_INTERVAL_AUTOZERO;

/* SD CARD LOGGER INTERVALS */

constexpr uint32_t RA_SDLOGGER_INTERVAL_IDLE     = 1000ul;  // 1 Hz
constexpr uint32_t RA_SDLOGGER_INTERVAL_SLOW     = 200ul;   // 5 Hz
constexpr uint32_t RA_SDLOGGER_INTERVAL_FAST     = 100ul;   // 10 Hz
constexpr uint32_t RA_SDLOGGER_INTERVAL_REALTIME = 50ul;    // 20 Hz

constexpr uint32_t DEPLOY_SHOULD_ACTIVATE = 40 * 60 * 1000ul;

// Static assertions validate settings
namespace details::assertions {
  static_assert(RA_TIME_TO_BURNOUT_MAX >= RA_TIME_TO_BURNOUT_MIN, "Motor burnout is configured incorrectly!");
  static_assert(RA_TIME_TO_APOGEE_MAX >= RA_TIME_TO_APOGEE_MIN, "Time to apogee is configured incorrectly!");
  static_assert(RA_TIME_TO_APOGEE_MIN >= RA_TIME_TO_BURNOUT_MIN, "Time to apogee must be greater than motor burnout!");
  static_assert(RA_TIME_TO_APOGEE_MAX >= RA_TIME_TO_BURNOUT_MAX, "Time to apogee must be greater than motor burnout!");
}  // namespace details::assertions

#endif  //ROCKET_AVIONICS_TEMPLATE_USERCONFIG_H
