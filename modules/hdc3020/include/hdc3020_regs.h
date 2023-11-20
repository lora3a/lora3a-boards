#ifndef HDC3020_REGS_H
#define HDC3020_REGS_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @name    Trigger-On Demand Mode
 *          Single Temperature (T) Measurement
 *          Single Relative Humidity (RH) Measurement
 * @{
 */
#define HDC3020_TRIGGER_ON_DEMAND_MODE_LPM0_MSB 0x24
#define HDC3020_TRIGGER_ON_DEMAND_MODE_LPM0_LSB 0x00
#define HDC3020_TRIGGER_ON_DEMAND_MODE_LPM1_MSB 0x24
#define HDC3020_TRIGGER_ON_DEMAND_MODE_LPM1_LSB 0x0B
#define HDC3020_TRIGGER_ON_DEMAND_MODE_LPM2_MSB 0x24
#define HDC3020_TRIGGER_ON_DEMAND_MODE_LPM2_LSB 0x16
#define HDC3020_TRIGGER_ON_DEMAND_MODE_LPM3_MSB 0x24
#define HDC3020_TRIGGER_ON_DEMAND_MODE_LPM3_LSB 0xFF
/** @} */

/**
 * @name    Auto Measurement Mode
 *          1 measurement per 2 seconds.
 * @{
 */
#define HDC3020_AUTO_MEAS_MODE_LPM0_MPS_0_5_MSB 0x20
#define HDC3020_AUTO_MEAS_MODE_LPM0_MPS_0_5_LSB 0x32
#define HDC3020_AUTO_MEAS_MODE_LPM1_MPS_0_5_MSB 0x20
#define HDC3020_AUTO_MEAS_MODE_LPM1_MPS_0_5_LSB 0x24
#define HDC3020_AUTO_MEAS_MODE_LPM2_MPS_0_5_MSB 0x20
#define HDC3020_AUTO_MEAS_MODE_LPM2_MPS_0_5_LSB 0x2F
#define HDC3020_AUTO_MEAS_MODE_LPM3_MPS_0_5_MSB 0x20
#define HDC3020_AUTO_MEAS_MODE_LPM3_MPS_0_5_LSB 0xFF
/** @} */

/**
 * @name    Auto Measurement Mode
 *          1 measurement per second.
 * @{
 */
#define HDC3020_AUTO_MEAS_MODE_LPM0_MPS_1_MSB 0x21
#define HDC3020_AUTO_MEAS_MODE_LPM0_MPS_1_LSB 0x30
#define HDC3020_AUTO_MEAS_MODE_LPM1_MPS_1_MSB 0x21
#define HDC3020_AUTO_MEAS_MODE_LPM1_MPS_1_LSB 0x26
#define HDC3020_AUTO_MEAS_MODE_LPM2_MPS_1_MSB 0x21
#define HDC3020_AUTO_MEAS_MODE_LPM2_MPS_1_LSB 0x2D
#define HDC3020_AUTO_MEAS_MODE_LPM3_MPS_1_MSB 0x21
#define HDC3020_AUTO_MEAS_MODE_LPM3_MPS_1_LSB 0xFF
/** @} */

/**
 * @name    Auto Measurement Mode
 *          2 measurements per second.
 * @{
 */
#define HDC3020_AUTO_MEAS_MODE_LPM0_MPS_2_MSB 0x22
#define HDC3020_AUTO_MEAS_MODE_LPM0_MPS_2_LSB 0x36
#define HDC3020_AUTO_MEAS_MODE_LPM1_MPS_2_MSB 0x22
#define HDC3020_AUTO_MEAS_MODE_LPM1_MPS_2_LSB 0x20
#define HDC3020_AUTO_MEAS_MODE_LPM2_MPS_2_MSB 0x22
#define HDC3020_AUTO_MEAS_MODE_LPM2_MPS_2_LSB 0x2B
#define HDC3020_AUTO_MEAS_MODE_LPM3_MPS_2_MSB 0x22
#define HDC3020_AUTO_MEAS_MODE_LPM3_MPS_2_LSB 0xFF
/** @} */

/**
 * @name    Auto Measurement Mode
 *          4 measurements per second.
 * @{
 */
#define HDC3020_AUTO_MEAS_MODE_LPM0_MPS_4_MSB 0x23
#define HDC3020_AUTO_MEAS_MODE_LPM0_MPS_4_LSB 0x34
#define HDC3020_AUTO_MEAS_MODE_LPM1_MPS_4_MSB 0x23
#define HDC3020_AUTO_MEAS_MODE_LPM1_MPS_4_LSB 0x22
#define HDC3020_AUTO_MEAS_MODE_LPM2_MPS_4_MSB 0x23
#define HDC3020_AUTO_MEAS_MODE_LPM2_MPS_4_LSB 0x29
#define HDC3020_AUTO_MEAS_MODE_LPM3_MPS_4_MSB 0x23
#define HDC3020_AUTO_MEAS_MODE_LPM3_MPS_4_LSB 0xFF
/** @} */

/**
 * @name    Auto Measurement Mode
 *          10 measurements per second.
 * @{
 */
#define HDC3020_AUTO_MEAS_MODE_LPM0_MPS_10_MSB 0x27
#define HDC3020_AUTO_MEAS_MODE_LPM0_MPS_10_LSB 0x37
#define HDC3020_AUTO_MEAS_MODE_LPM1_MPS_10_MSB 0x27
#define HDC3020_AUTO_MEAS_MODE_LPM1_MPS_10_LSB 0x21
#define HDC3020_AUTO_MEAS_MODE_LPM2_MPS_10_MSB 0x27
#define HDC3020_AUTO_MEAS_MODE_LPM2_MPS_10_LSB 0x2A
#define HDC3020_AUTO_MEAS_MODE_LPM3_MPS_10_MSB 0x27
#define HDC3020_AUTO_MEAS_MODE_LPM3_MPS_10_LSB 0xFF
/** @} */

/**
 * @name    Auto Measurement Mode
 * @{
 */
#define HDC3020_AUTO_MEAS_MODE_EXIT_MSB 0x30
#define HDC3020_AUTO_MEAS_MODE_EXIT_LSB 0x93
#define HDC3020_AUTO_MEAS_MODE_READ_MSB 0xE0
#define HDC3020_AUTO_MEAS_MODE_READ_LSB 0x00
#define HDC3020_AUTO_MEAS_MODE_MIN_T_MSB 0xE0
#define HDC3020_AUTO_MEAS_MODE_MIN_T_LSB 0x02
#define HDC3020_AUTO_MEAS_MODE_MAX_T_MSB 0xE0
#define HDC3020_AUTO_MEAS_MODE_MAX_T_LSB 0x03
#define HDC3020_AUTO_MEAS_MODE_MIN_RH_MSB 0xE0
#define HDC3020_AUTO_MEAS_MODE_MIN_RH_LSB 0x04
#define HDC3020_AUTO_MEAS_MODE_MAX_RH_MSB 0xE0
#define HDC3020_AUTO_MEAS_MODE_MAX_RH_LSB 0x05
/** @} */

/**
 * @name    Configure ALERT Thresholds of T and RH
 * @{
 */
#define HDC3020_PROG_THRESHOLDS_SET_LOW_ALERT_MSB 0x61
#define HDC3020_PROG_THRESHOLDS_SET_LOW_ALERT_LSB 0x00
#define HDC3020_PROG_THRESHOLDS_SET_HIGH_ALERT_MSB 0x61
#define HDC3020_PROG_THRESHOLDS_SET_HIGH_ALERT_LSB 0x1D
#define HDC3020_PROG_THRESHOLDS_CLEAR_LOW_ALERT_MSB 0x61
#define HDC3020_PROG_THRESHOLDS_CLEAR_LOW_ALERT_LSB 0x0B
#define HDC3020_PROG_THRESHOLDS_CLEAR_HIGH_ALERT_MSB 0x61
#define HDC3020_PROG_THRESHOLDS_CLEAR_HIGH_ALERT_LSB 0x16
#define HDC3020_TRANS_ALERT_THRESHOLDS_INTO_NVM_MSB 0x61
#define HDC3020_TRANS_ALERT_THRESHOLDS_INTO_NVM_LSB 0x55
/** @} */

/**
 * @name    Verify ALERT Thresholds of T and RH
 * @{
 */
#define HDC3020_READ_THRESHOLDS_SET_LOW_ALERT_MSB 0xE1
#define HDC3020_READ_THRESHOLDS_SET_LOW_ALERT_LSB 0x02
#define HDC3020_READ_THRESHOLDS_SET_HIGH_ALERT_MSB 0xE1
#define HDC3020_READ_THRESHOLDS_SET_HIGH_ALERT_LSB 0x1F
#define HDC3020_READ_THRESHOLDS_CLEAR_LOW_ALERT_MSB 0xE1
#define HDC3020_READ_THRESHOLDS_CLEAR_LOW_ALERT_LSB 0x09
#define HDC3020_READ_THRESHOLDS_CLEAR_HIGH_ALERT_MSB 0xE1
#define HDC3020_READ_THRESHOLDS_CLEAR_HIGH_ALERT_LSB 0x14
/** @} */

/**
 * @name    Integrated Heater
 * @{
 */
#define HDC3020_INTEGRETED_HEATER_ENABLE_MSB 0x30
#define HDC3020_INTEGRETED_HEATER_ENABLE_LSB 0x6D
#define HDC3020_INTEGRETED_HEATER_DISABLE_MSB 0x30
#define HDC3020_INTEGRETED_HEATER_DISABLE_LSB 0x66
#define HDC3020_INTEGRETED_HEATER_CONFIGURE_MSB 0x30
#define HDC3020_INTEGRETED_HEATER_CONFIGURE_LSB 0x6E
/** @} */

/**
 * @name    Status Register
 * @{
 */
#define HDC3020_STATUS_REGISTER_READ_CONTENT_MSB 0xF3
#define HDC3020_STATUS_REGISTER_READ_CONTENT_LSB 0x2D
#define HDC3020_STATUS_REGISTER_CLEAR_CONTENT_MSB 0x30
#define HDC3020_STATUS_REGISTER_CLEAR_CONTENT_LSB 0x41
/** @} */

/**
 * @name    Program/Read Offset Value of Relative Humidity and
 *          Temperature Results into/from non-volatile memory
 * @{
 */
#define HDC3020_PROG_READ_TEMP_RH_OFFSET_MSB 0xA0
#define HDC3020_PROG_READ_TEMP_RH_OFFSET_LSB 0x04
/** @} */

/**
 * @name    Soft Reset
 * @{
 */
#define HDC3020_SOFT_RESET_MSB 0x30
#define HDC3020_SOFT_RESET_LSB 0xA2
/** @} */

/**
 * @name    Read NIST ID (Serial Number)
 * @{
 */
#define HDC3020_READ_NIST_ID_5_4_BYTES_MSB 0x36
#define HDC3020_READ_NIST_ID_5_4_BYTES_LSB 0x83
#define HDC3020_READ_NIST_ID_3_2_BYTES_MSB 0x36
#define HDC3020_READ_NIST_ID_3_2_BYTES_LSB 0x84
#define HDC3020_READ_NIST_ID_1_0_BYTES_MSB 0x36
#define HDC3020_READ_NIST_ID_1_0_BYTES_LSB 0x85
/** @} */

/**
 * @name    Read Manufacturer ID (Texas Instruments) (0x3000)
 * @{
 */
#define HDC3020_READ_MANUFACTURER_ID_MSB 0x37
#define HDC3020_READ_MANUFACTURER_ID_LSB 0x81
/** @} */

/**
 * @name    Override Default Device Power-On/Reset
 *          Measurement State
 * @{
 */
#define HDC3020_OVERRIDE_DEFAULT_MEASURE_STATE_MSB 0x61
#define HDC3020_OVERRIDE_DEFAULT_MEASURE_STATE_LSB 0xBB
/** @} */

#ifdef __cplusplus
}
#endif

#endif /* HDC3020_REGS_H */
