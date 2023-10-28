/*
 * Copyright 2022 Marcus Behel
 * 
 * This file is part of AUVControlBoard-Firmware.
 * 
 * AUVControlBoard-Firmware is free software: you can redistribute it and/or modify it under the terms of the GNU 
 * General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your 
 * option) any later version.
 * 
 * AUVControlBoard-Firmware is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even 
 * the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for 
 * more details.
 * 
 * You should have received a copy of the GNU General Public License along with AUVControlBoard-Firmware. If not, see 
 * <https://www.gnu.org/licenses/>. 
 * 
 */

#include <cmdctrl.h>
#include <hardware/led.h>
#include <pccomm.h>
#include <util/conversions.h>
#include <motor_control.h>
#include <FreeRTOS.h>
#include <semphr.h>
#include <timers.h>
#include <sensor/bno055.h>
#include <hardware/wdt.h>
#include <hardware/thruster.h>
#include <debug.h>
#include <calibration.h>
#include <metadata.h>

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Macros
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Control modes
#define MODE_RAW        0
#define MODE_LOCAL      1
#define MODE_GLOBAL     2
#define MODE_SASSIST    3
// #define MODE_DHOLD   4
#define MODE_OHOLD      5

// LED colors for different modes
// Different colors on different versions b/c LEDs are different
// Designed to look visually similar
#if defined(CONTROL_BOARD_V1)
#define COLOR_RAW           100, 100, 0
#define COLOR_LOCAL         10, 0, 100
#define COLOR_GLOBAL        150, 50, 0
#define COLOR_SASSIST       0, 100, 100
#define COLOR_DHOLD         10, 50, 0
#define COLOR_OHOLD         100, 7, 7
#elif defined(CONTROL_BOARD_V2)
#define COLOR_RAW           170, 40, 0
#define COLOR_LOCAL         30, 0, 100
#define COLOR_GLOBAL        255, 15, 0
#define COLOR_SASSIST       0, 100, 100
#define COLOR_DHOLD         10, 40, 0
#define COLOR_OHOLD         140, 5, 5
#else
// SimCB. Colors don't show anywhere, so don't matter
#define COLOR_RAW           0, 0, 0
#define COLOR_LOCAL         0, 0, 0
#define COLOR_GLOBAL        0, 0, 0
#define COLOR_SASSIST       0, 0, 0
#define COLOR_DHOLD         0, 0, 0
#define COLOR_OHOLD         0, 0, 0
#endif

// Acknowledge message error codes
#define ACK_ERR_NONE                    0   // No error
#define ACK_ERR_UNKNOWN_MSG             1   // The message is not recognized
#define ACK_ERR_INVALID_ARGS            2   // Arguments are invalid (too many, too few, etc)
#define ACK_ERR_INVALID_CMD             3   // Command is known, but invalid at this time
// Note 255 reserved for timeout error codes



#define SENSOR_DATA_PERIOD              20      // ms
#define SPEED_PERIOD                    20      // ms


// Restrict to range -1.0 to 1.0
#define LIMIT(v) if(v > 1.0f) v = 1.0f; if (v < -1.0f) v = -1.0f;

// Restrict range to 0.0 to 1.0
#define LIMIT_POS(v) if(v > 1.0f) v = 1.0f; if (v < 0.0f) v = 0.0f;

// Get minimum of two values
#define MIN(a, b)    (((a) < (b)) ? (a) : (b))

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Globals
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Used for simulator hijack of the control board
#if defined(CONTROL_BOARD_V1) || defined(CONTROL_BOARD_V2)
bool cmdctrl_sim_hijacked = false;
#elif defined(CONTROL_BOARD_SIM_LINUX) || defined(CONTROL_BOARD_SIM_WINDOWS)
const bool cmdctrl_sim_hijacked = true;
#endif

quaternion_t cmdctrl_sim_quat;
float cmdctrl_sim_depth;
float cmdctrl_sim_speeds[8];

// CMDCTRL state tracking
static unsigned int mode;

// Last used target for each mode
static float raw_target[8];
static mc_local_target_t local_target;
static mc_global_target_t global_target;
static mc_sassist_target_t sassist_target;
static mc_ohold_target_t ohold_target;

// Periodic reading of sensor data timer
static bool periodic_imu;
static bool periodic_depth;
static TimerHandle_t sensor_read_timer;

// Used to periodically re-apply speeds in modes where necessary
static TimerHandle_t periodic_speed_timer;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// CMDCTRL functions / implementation
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static void send_sensor_data(TimerHandle_t timer){
    (void)timer;
    
    // Send the data for sensors as needed
    if(periodic_imu && (imu_get_sensor() != IMU_NONE)){
        // Store current readings
        imu_data_t dat = imu_get_data();

        // Construct message
        uint8_t imu_data_msg[33];
        imu_data_msg[0] = 'I';
        imu_data_msg[1] = 'M';
        imu_data_msg[2] = 'U';
        imu_data_msg[3] = 'D';
        conversions_float_to_data(dat.quat.w, &imu_data_msg[4], true);
        conversions_float_to_data(dat.quat.x, &imu_data_msg[8], true);
        conversions_float_to_data(dat.quat.y, &imu_data_msg[12], true);
        conversions_float_to_data(dat.quat.z, &imu_data_msg[16], true);
        conversions_float_to_data(dat.accum_angles.pitch, &imu_data_msg[20], true);
        conversions_float_to_data(dat.accum_angles.roll, &imu_data_msg[24], true);
        conversions_float_to_data(dat.accum_angles.yaw, &imu_data_msg[28], true);

        // Send message (status message from CB to PC)
        pccomm_write(imu_data_msg, 35);
    }
    if(periodic_depth & (depth_get_sensor() != DEPTH_NONE)){
        // Store current readings
        depth_data_t dat = depth_get_data();

        // Construct message
        uint8_t depth_data[18];
        depth_data[0] = 'D';
        depth_data[1] = 'E';
        depth_data[2] = 'P';
        depth_data[3] = 'T';
        depth_data[4] = 'H';
        depth_data[5] = 'D';
        conversions_float_to_data(dat.depth_m, &depth_data[6], true);
        conversions_float_to_data(dat.pressure_pa, &depth_data[10], true);
        conversions_float_to_data(dat.temperature_c, &depth_data[14], true);

        // Send message (status message from CB to PC)
        pccomm_write(depth_data, 18);
    }

    // Not using auto reload so that any time taken to
    // enforce a duration between sends, not a rate of sending data
    xTimerStart(sensor_read_timer, portMAX_DELAY);
}

static void cmdctrl_apply_speed(void);

static void periodic_reapply_speed(TimerHandle_t timer){
    (void)timer;

    // Modes using sensor data (which may change) need to be periodically reapplied
    if(mode == MODE_GLOBAL || mode == MODE_SASSIST || mode == MODE_OHOLD){
        cmdctrl_apply_speed();
    }

    // Not using auto reload so that any time taken to
    // enforce a duration between sets, not a rate of setting speeds
    xTimerStart(periodic_speed_timer, portMAX_DELAY);
}

void cmdctrl_init(void){
    // Initialize targets for all modes to result in no motion

    // Raw mode (zero all motor speeds)
    // Only need to zero RAW mode since this is the mode the board starts in
    // Other modes will have targets set in handle_message before the mode is enabled
    for(unsigned int i = 0; i < 8; ++i)
        raw_target[i] = 0.0f;

    // Periodic sensor data
    periodic_imu = false;
    periodic_depth = false;
    sensor_read_timer = xTimerCreate(
        "sensor_data",
        pdMS_TO_TICKS(SENSOR_DATA_PERIOD),
        pdFALSE,                                // Don't auto reload
        NULL,
        send_sensor_data
    );
    xTimerStart(sensor_read_timer, portMAX_DELAY);

    // Periodic speed reapply
    periodic_speed_timer = xTimerCreate(
        "periodic_speed",
        pdMS_TO_TICKS(SPEED_PERIOD),
        pdFALSE,
        NULL,
        periodic_reapply_speed
    );
    xTimerStart(periodic_speed_timer, portMAX_DELAY);

    // Default to raw mode
    mode = MODE_RAW;
    led_set(COLOR_RAW);
}

/**
 * Apply speed based on current mode and stored targets
 * @return true if successful. False if mode unknown or sensors are not valid for the selected mode
 */
static void cmdctrl_apply_speed(void){
    quaternion_t m_quat;
    float m_depth;

    switch (mode){
    case MODE_RAW:
        mc_set_raw(raw_target);
        break;
    case MODE_LOCAL:
        mc_set_local(local_target);
        break;
    case MODE_GLOBAL:
        m_quat = imu_get_data().quat;
        if((imu_get_sensor() == IMU_NONE) || (m_quat.w == 0 && m_quat.x == 0 && m_quat.y == 0 && m_quat.z == 0)){
            // Cannot apply real speed b/c sensor data not available or invalid
            // Thus, stop the thrusters
            mc_set_local((mc_local_target_t){.x=0, .y=0, .z=0, .xrot=0, .yrot=0, .zrot=0});
        }else{
            mc_set_global(global_target, m_quat);
        }
        break;
    case MODE_SASSIST:
        m_quat = imu_get_data().quat;
        m_depth = depth_get_data().depth_m;
        if((imu_get_sensor() == IMU_NONE) || 
                (m_quat.w == 0 && m_quat.x == 0 && m_quat.y == 0 && m_quat.z == 0) || 
                (depth_get_sensor() == DEPTH_NONE)){
            // Cannot apply real speed b/c sensor data not available or invalid
            // Thus, stop the thrusters
            mc_set_local((mc_local_target_t){.x=0, .y=0, .z=0, .xrot=0, .yrot=0, .zrot=0});
        }else{
            mc_set_sassist(sassist_target, m_quat, m_depth);
        }
        break;
    case MODE_OHOLD:
        m_quat = imu_get_data().quat;
        if((imu_get_sensor() == IMU_NONE) || 
                (m_quat.w == 0 && m_quat.x == 0 && m_quat.y == 0 && m_quat.z == 0)){
            // Cannot apply real speed b/c sensor data not available or invalid
            // Thus, stop the thrusters
            mc_set_local((mc_local_target_t){.x=0, .y=0, .z=0, .xrot=0, .yrot=0, .zrot=0});
        }else{
            mc_set_ohold(ohold_target, m_quat);
        }
        break;
    }
}

/**
 * Acknowledge receipt of a message
 * @param msg_id The ID of the message being acknowledged
 * @param error_code Error code for the acknowledge operation
 * @param result Data to include as "result" in the ack message
 * @param result_len Length of data
 */
static void cmdctrl_acknowledge(uint16_t msg_id, uint8_t error_code, uint8_t *result, unsigned int result_len){
    // A, C, K, [message_id], [error_code], [response]
    // [message_id] is a 16-bit number big endian
    // [response] is arbitrary data
    uint8_t *data = pvPortMalloc(sizeof(uint8_t) * (6 + result_len));
    data[0] = 'A';
    data[1] = 'C';
    data[2] = 'K';
    conversions_int16_to_data(msg_id, &data[3], false);
    data[5] = error_code;
    for(unsigned int i = 0; i < result_len; ++i)
        data[6 + i] = result[i];
    pccomm_write(data, 6 + result_len);
    vPortFree(data);
}

/**
 * Check if message starts with the given prefix
 */
static bool message_starts_with(const uint8_t *msg, const unsigned int msg_len, const uint8_t *prefix, const unsigned int prefix_len){
    if(msg_len < prefix_len)
        return false;
    for(unsigned int i = 0; i < prefix_len; ++i){
        if(msg[i] != prefix[i])
            return false;
    }
    return true;
}

/**
 * Check if message matches the given data
 */
static bool message_equals(const uint8_t *msg, const unsigned int msg_len, const uint8_t *match, const unsigned int match_len){
    if(msg_len != match_len)
        return false;
    for(unsigned int i = 0; i < msg_len; ++i){
        if(msg[i] != match[i])
            return false;
    }
    return true;
}

#define message_starts_with_str(msg, msg_len, prefix_str)       message_starts_with(msg, msg_len, (uint8_t*)(prefix_str), (sizeof(prefix_str) - 1))
#define message_equals_str(msg, msg_len, match_str)             message_equals(msg, msg_len, (uint8_t*)(match_str), (sizeof(match_str) - 1))



void cmdctrl_handle_message(void){
    // Skip first 2 bytes of pccomm_read_buf (these are message ID)
    // Also skip last 2 bytes (these are CRC)
    uint8_t *msg = &pccomm_read_buf[2];
    unsigned int len = pccomm_read_len - 4;

    // msg_id is first two bytes (unsigned 16-bit int big endian)
    uint16_t msg_id = conversions_data_to_int16(pccomm_read_buf, false);

    // -----------------------------------------------------------------------------------------------------------------
    // Motor motion commands (check these first as they are expected to be most frequently used)
    // -----------------------------------------------------------------------------------------------------------------
    if(message_starts_with_str(msg, len, "RAW")){
        // RAW speed set command
        // R, A, W, [speed_0], [speed_1], [speed_2], [speed_3], [speed_4], [speed_5], [speed_6], [speed_7]
        // [speed_i] is a 32-bit float (little endian)

        if(len != 35){
            // Message is incorrect size
            cmdctrl_acknowledge(msg_id, ACK_ERR_INVALID_ARGS, NULL, 0);
        }else{
            // Message is correct size. Handle it.

            // Get speeds from message
            raw_target[0] = conversions_data_to_float(&msg[3], true);
            raw_target[1] = conversions_data_to_float(&msg[7], true);
            raw_target[2] = conversions_data_to_float(&msg[11], true);
            raw_target[3] = conversions_data_to_float(&msg[15], true);
            raw_target[4] = conversions_data_to_float(&msg[19], true);
            raw_target[5] = conversions_data_to_float(&msg[23], true);
            raw_target[6] = conversions_data_to_float(&msg[27], true);
            raw_target[7] = conversions_data_to_float(&msg[31], true);

            // Ensure speeds are in valid range
            for(unsigned int i = 0; i < 8; ++i){
                LIMIT(raw_target[i]);
            }

            // Reset time until periodic speed set
            xTimerReset(periodic_speed_timer, portMAX_DELAY);

            // Update mode variable and LED color (if needed)
            if(mode != MODE_RAW){
                mode = MODE_RAW;
                led_set(COLOR_RAW);
            }

            // Feed watchdog when speeds are set
            // Important to call before speed set function in case currently killed
            mc_wdog_feed();

            // Update motor speeds
            cmdctrl_apply_speed();

            // Acknowledge message w/ no error.
            cmdctrl_acknowledge(msg_id, ACK_ERR_NONE, NULL, 0);
        }
    }else if(message_starts_with_str(msg, len, "LOCAL")){
        // LOCAL Speed Set
        // L, O, C, A, L, [x], [y], [z], [xrot], [yrot], [zrot]
        // [x], [y], [z], [xrot], [yrot], [zrot]  are 32-bit floats (little endian)

        if(len != 29){
            // Message is incorrect size
            cmdctrl_acknowledge(msg_id, ACK_ERR_INVALID_ARGS, NULL, 0);
        }else{
            // Message is correct size. Handle it.

            // Get speeds from message
            local_target.x = conversions_data_to_float(&msg[5], true);
            local_target.y = conversions_data_to_float(&msg[9], true);
            local_target.z = conversions_data_to_float(&msg[13], true);
            local_target.xrot = conversions_data_to_float(&msg[17], true);
            local_target.yrot = conversions_data_to_float(&msg[21], true);
            local_target.zrot = conversions_data_to_float(&msg[25], true);

            // Ensure speeds are in valid range
            LIMIT(local_target.x);
            LIMIT(local_target.y);
            LIMIT(local_target.z);
            LIMIT(local_target.xrot);
            LIMIT(local_target.yrot);
            LIMIT(local_target.zrot);

            // Reset time until periodic speed set
            xTimerReset(periodic_speed_timer, portMAX_DELAY);

            // Update mode variable and LED color (if needed)
            if(mode != MODE_LOCAL){
                mode = MODE_LOCAL;
                led_set(COLOR_LOCAL);
            }

            // Feed watchdog when speeds are set
            // Important to call before speed set function in case currently killed
            mc_wdog_feed();

            // Update motor speeds
            cmdctrl_apply_speed();

            // Acknowledge message w/ no error.
            cmdctrl_acknowledge(msg_id, ACK_ERR_NONE, NULL, 0);
        }
    }else if(message_starts_with_str(msg, len, "GLOBAL")){
        // GLOBAL speed set
        // G, L, O, B, A, L, [x], [y], [z], [pitch_spd], [roll_spd], [yaw_spd]
        // [x], [y], [z], [pitch_spd], [roll_spd], [yaw_spd]  are 32-bit floats (little endian)
        if(len != 30){
            // Message is incorrect size
            cmdctrl_acknowledge(msg_id, ACK_ERR_INVALID_ARGS, NULL, 0);
        }else{
            // Message is correct size. Handle it.
            quaternion_t m_quat = imu_get_data().quat;

            if((imu_get_sensor() == IMU_NONE) || (m_quat.w == 0 && m_quat.x == 0 && m_quat.y == 0 && m_quat.z == 0)){
                // Need IMU data to use global mode.
                // If not ready, then this command is invalid at this time
                cmdctrl_acknowledge(msg_id, ACK_ERR_INVALID_CMD, NULL, 0);
            }else{
                // Get speeds from message
                global_target.x = conversions_data_to_float(&msg[6], true);
                global_target.y = conversions_data_to_float(&msg[10], true);
                global_target.z = conversions_data_to_float(&msg[14], true);
                global_target.pitch_spd = conversions_data_to_float(&msg[18], true);
                global_target.roll_spd = conversions_data_to_float(&msg[22], true);
                global_target.yaw_spd = conversions_data_to_float(&msg[26], true);

                // Ensure speeds are in valid range
                LIMIT(global_target.x);
                LIMIT(global_target.y);
                LIMIT(global_target.z);
                LIMIT(global_target.pitch_spd);
                LIMIT(global_target.roll_spd);
                LIMIT(global_target.yaw_spd);

                // Reset time until periodic speed set
                xTimerReset(periodic_speed_timer, portMAX_DELAY);

                // Update mode variable and LED color (if needed)
                if(mode != MODE_GLOBAL){
                    mode = MODE_GLOBAL;
                    led_set(COLOR_GLOBAL);
                }

                // Feed watchdog when speeds are set
                // Important to call before speed set function in case currently killed
                mc_wdog_feed();

                // Update motor speeds
                cmdctrl_apply_speed();

                // Acknowledge message w/ no error.
                cmdctrl_acknowledge(msg_id, ACK_ERR_NONE, NULL, 0);
            }
        }
    }else if(message_starts_with_str(msg, len, "SASSIST1")){
        // STABILITY ASSIST speed set variant 1
        // S, A, S, S, I, S, T, 1, [x], [y], [yaw_spd], [target_pitch], [target_roll], [target_depth]
        // [x], [y], [yaw_spd], [target_pitch], [target_roll], [target_depth] are 32-bit floats (little endian)
        if(len != 32){
            // Message is incorrect size
            cmdctrl_acknowledge(msg_id, ACK_ERR_INVALID_ARGS, NULL, 0);
        }else{
            // Message is correct size. Handle it.
            quaternion_t m_quat = imu_get_data().quat;

            if((imu_get_sensor() == IMU_NONE) || 
                    (m_quat.w == 0 && m_quat.x == 0 && m_quat.y == 0 && m_quat.z == 0) || 
                    (depth_get_sensor() == DEPTH_NONE)){
                // Need both IMU and depth sensor for this mode.
                // If not ready, then this command is invalid at this time
                cmdctrl_acknowledge(msg_id, ACK_ERR_INVALID_CMD, NULL, 0);
            }else{
                // Get arguments from message
                sassist_target.x = conversions_data_to_float(&msg[8], true);
                sassist_target.y = conversions_data_to_float(&msg[12], true);
                sassist_target.yaw_spd = conversions_data_to_float(&msg[16], true);
                sassist_target.target_euler.pitch = conversions_data_to_float(&msg[20], true);
                sassist_target.target_euler.roll = conversions_data_to_float(&msg[24], true);
                sassist_target.target_depth = conversions_data_to_float(&msg[28], true);
                sassist_target.use_yaw_pid = false;

                // Ensure speeds are in valid range
                LIMIT(sassist_target.x);
                LIMIT(sassist_target.y);
                LIMIT(sassist_target.yaw_spd);

                // Reset time until periodic speed set
                xTimerReset(periodic_speed_timer, portMAX_DELAY);

                // Update mode variable and LED color (if needed)
                if(mode != MODE_SASSIST){
                    mode = MODE_SASSIST;
                    led_set(COLOR_SASSIST);
                }

                // Feed watchdog when speeds are set
                // Important to call before speed set function in case currently killed
                mc_wdog_feed();

                // Update motor speeds
                cmdctrl_apply_speed();

                // Acknowledge message w/ no error.
                cmdctrl_acknowledge(msg_id, ACK_ERR_NONE, NULL, 0);
            }
        }
    }else if(message_starts_with_str(msg, len, "SASSIST2")){
        // STABILITY ASSIST speed set variant 2
        // S, A, S, S, I, S, T, 2, [x], [y], [target_pitch], [target_roll], [target_yaw], [target_depth]
        // [x], [y], [target_pitch], [target_roll], [target_yaw], [target_depth] are 32-bit floats (little endian)
        if(len != 32){
            // Message is incorrect size
            cmdctrl_acknowledge(msg_id, ACK_ERR_INVALID_ARGS, NULL, 0);
        }else{
            // Message is correct size. Handle it.
            quaternion_t m_quat = imu_get_data().quat;

            if((imu_get_sensor() == IMU_NONE) || 
                    (m_quat.w == 0 && m_quat.x == 0 && m_quat.y == 0 && m_quat.z == 0) || 
                    (depth_get_sensor() == DEPTH_NONE)){
                // Need depth and IMU for this mode
                // If not ready, then this command is invalid at this time
                cmdctrl_acknowledge(msg_id, ACK_ERR_INVALID_CMD, NULL, 0);
            }else{
                // Get arguments from message
                sassist_target.x = conversions_data_to_float(&msg[8], true);
                sassist_target.y = conversions_data_to_float(&msg[12], true);
                sassist_target.target_euler.pitch = conversions_data_to_float(&msg[16], true);
                sassist_target.target_euler.roll = conversions_data_to_float(&msg[20], true);
                sassist_target.target_euler.yaw = conversions_data_to_float(&msg[24], true);
                sassist_target.target_depth = conversions_data_to_float(&msg[28], true);
                sassist_target.use_yaw_pid = true;

                // Ensure speeds are in valid range
                LIMIT(sassist_target.x);
                LIMIT(sassist_target.y);
                LIMIT(sassist_target.yaw_spd);

                // Reset time until periodic speed set
                xTimerReset(periodic_speed_timer, portMAX_DELAY);

                // Update mode variable and LED color (if needed)
                if(mode != MODE_SASSIST){
                    mode = MODE_SASSIST;
                    led_set(COLOR_SASSIST);
                }

                // Feed watchdog when speeds are set
                // Important to call before speed set function in case currently killed
                mc_wdog_feed();

                // Update motor speeds
                cmdctrl_apply_speed();

                // Acknowledge message w/ no error.
                cmdctrl_acknowledge(msg_id, ACK_ERR_NONE, NULL, 0);
            }
        }
    }else if(message_starts_with_str(msg, len, "OHOLD1")){
        // ORIENTATION HOLD speed set variant 1
        // O, H, O, L, D, 1 [x], [y], [z], [yaw_spd], [target_pitch], [target_roll]
        // [x], [y], [z], [yaw_spd], [target_pitch], [target_roll] are 32-bit floats (little endian)
        if(len != 30){
            // Message is incorrect size
            cmdctrl_acknowledge(msg_id, ACK_ERR_INVALID_ARGS, NULL, 0);
        }else{
            // Message is correct size. Handle it.
            quaternion_t m_quat = imu_get_data().quat;

            if((imu_get_sensor() == IMU_NONE) || (m_quat.w == 0 && m_quat.x == 0 && m_quat.y == 0 && m_quat.z == 0)){
                // Need IMU data to use ohold mode.
                // If not ready, then this command is invalid at this time
                cmdctrl_acknowledge(msg_id, ACK_ERR_INVALID_CMD, NULL, 0);
            }else{
                // Get arguments from message
                ohold_target.x = conversions_data_to_float(&msg[6], true);
                ohold_target.y = conversions_data_to_float(&msg[10], true);
                ohold_target.z = conversions_data_to_float(&msg[14], true);
                ohold_target.yaw_spd = conversions_data_to_float(&msg[18], true);
                ohold_target.target_euler.pitch = conversions_data_to_float(&msg[22], true);
                ohold_target.target_euler.roll = conversions_data_to_float(&msg[26], true);
                ohold_target.use_yaw_pid = false;

                // Ensure speeds are in valid range
                LIMIT(ohold_target.x);
                LIMIT(ohold_target.y);
                LIMIT(ohold_target.z);
                LIMIT(ohold_target.yaw_spd);

                // Reset time until periodic speed set
                xTimerReset(periodic_speed_timer, portMAX_DELAY);

                // Update mode variable and LED color (if needed)
                if(mode != MODE_OHOLD){
                    mode = MODE_OHOLD;
                    led_set(COLOR_OHOLD);
                }

                // Feed watchdog when speeds are set
                // Important to call before speed set function in case currently killed
                mc_wdog_feed();

                // Update motor speeds
                cmdctrl_apply_speed();

                // Acknowledge message w/ no error.
                cmdctrl_acknowledge(msg_id, ACK_ERR_NONE, NULL, 0);
            }
        }
    }else if(message_starts_with_str(msg, len, "OHOLD2")){
        // ORIENTATION HOLD speed set variant 2
        // O, H, O, L, D, 2, [x], [y], [z], [target_pitch], [target_roll], [target_yaw]
        // [x], [y], [z], [target_pitch], [target_roll], [target_yaw] are 32-bit floats (little endian)
        if(len != 30){
            // Message is incorrect size
            cmdctrl_acknowledge(msg_id, ACK_ERR_INVALID_ARGS, NULL, 0);
        }else{
            // Message is correct size. Handle it.
            quaternion_t m_quat = imu_get_data().quat;

            if((imu_get_sensor() == IMU_NONE) || (m_quat.w == 0 && m_quat.x == 0 && m_quat.y == 0 && m_quat.z == 0)){
                // Need IMU data to use ohold mode.
                // If not ready, then this command is invalid at this time
                cmdctrl_acknowledge(msg_id, ACK_ERR_INVALID_CMD, NULL, 0);
            }else{
                // Get arguments from message
                ohold_target.x = conversions_data_to_float(&msg[6], true);
                ohold_target.y = conversions_data_to_float(&msg[10], true);
                ohold_target.z = conversions_data_to_float(&msg[14], true);
                ohold_target.target_euler.pitch = conversions_data_to_float(&msg[18], true);
                ohold_target.target_euler.roll = conversions_data_to_float(&msg[22], true);
                ohold_target.target_euler.yaw = conversions_data_to_float(&msg[26], true);

                // Ensure speeds are in valid range
                LIMIT(ohold_target.x);
                LIMIT(ohold_target.y);
                LIMIT(ohold_target.z);

                // Reset time until periodic speed set
                xTimerReset(periodic_speed_timer, portMAX_DELAY);

                // Update mode variable and LED color (if needed)
                if(mode != MODE_OHOLD){
                    mode = MODE_OHOLD;
                    led_set(COLOR_OHOLD);
                }

                // Feed watchdog when speeds are set
                // Important to call before speed set function in case currently killed
                mc_wdog_feed();

                // Update motor speeds
                cmdctrl_apply_speed();

                // Acknowledge message w/ no error.
                cmdctrl_acknowledge(msg_id, ACK_ERR_NONE, NULL, 0);
            }
        }
    }else if(message_equals_str(msg, len, "WDGF")){
        // Feed motor watchdog command
        // W, D, G, F

        // Feed watchdog (as requested)
        bool was_killed = mc_wdog_feed();

        // Restore last set speed if previously killed
        if(was_killed)
            cmdctrl_apply_speed();

        // Acknowledge message w/ no error.
        cmdctrl_acknowledge(msg_id, ACK_ERR_NONE, NULL, 0);
    }
    // -----------------------------------------------------------------------------------------------------------------

    // -----------------------------------------------------------------------------------------------------------------
    // Vehicle configuration commands
    // -----------------------------------------------------------------------------------------------------------------
    else if(message_starts_with_str(msg, len, "TPWM")){
        // Thruster PWM parameter set command
        // T, P, W, M, [pwm_period], [pwm_zero], [pwm_range]
        // All 3 values are 16-bit integers (unsigned, little endian)

        if(len != 10){
            // Incorrect size.
            cmdctrl_acknowledge(msg_id, ACK_ERR_INVALID_ARGS, NULL, 0);
        }else{
            // Correct size. Handle it.
            thr_params_t p;
            p.pwm_period = conversions_data_to_int16(&msg[4], true);
            p.pwm_zero = conversions_data_to_int16(&msg[6], true);
            p.pwm_range = conversions_data_to_int16(&msg[8], true);

            // Apply settings
            thruster_config(p);

            // Apply saved speed properly for newly configured ESCs
            // Note that this is not a speed set, thus does not feed watchdog
            cmdctrl_apply_speed();

            // No error
            cmdctrl_acknowledge(msg_id, ACK_ERR_NONE, NULL, 0);
        }
    }else if(message_starts_with_str(msg, len, "TINV")){
        // Thruster inversion set command
        // T, I, N, V, [inv]
        // [inv] is an 8-bit int where MSB corresponds to thruster 8 and LSB thruster 1
        //       1 = inverted. 0 = not inverted
        
        if(len != 5){
            // Message is incorrect size
            cmdctrl_acknowledge(msg_id, ACK_ERR_INVALID_ARGS, NULL, 0);
        }else{
            // Message is correct size. Handle it.
            uint8_t inv_byte = msg[4];
            bool invert[8];
            for(unsigned int i = 0; i < 8; ++i){
                invert[i] = inv_byte & 1;
                inv_byte >>= 1;
            }
            mc_set_tinv(invert);

            // Reapply saved speed when inversions change
            cmdctrl_apply_speed();

            // Acknowledge message w/ no error.
            cmdctrl_acknowledge(msg_id, ACK_ERR_NONE, NULL, 0);
        }
    }else if(message_starts_with_str(msg, len, "RELDOF")){
        // Relative DoF speed set command
        // R, E, L, D, O, F, [x], [y], [z], [xrot], [yrot], [zrot]
        // Each value is a little endian float (32-bit)

        if(len != 30){
            // Message is incorrect size
            cmdctrl_acknowledge(msg_id, ACK_ERR_INVALID_ARGS, NULL, 0);
        }else{
            // Message is correct size. Handle it.
            float x = conversions_data_to_float(&msg[6], true);
            float y = conversions_data_to_float(&msg[10], true);
            float z = conversions_data_to_float(&msg[14], true);
            float xrot = conversions_data_to_float(&msg[18], true);
            float yrot = conversions_data_to_float(&msg[22], true);
            float zrot = conversions_data_to_float(&msg[26], true);

            // Restrict to 0.0 - 1.0
            LIMIT_POS(x);
            LIMIT_POS(y);
            LIMIT_POS(z);
            LIMIT_POS(xrot);
            LIMIT_POS(yrot);
            LIMIT_POS(zrot);

            float mc_relscale[6];

            // Linear scale DOWN factors
            //  x = min(x, y, z) / x
            //  y = min(x, y, z) / y
            //  z = min(x, y, z) / z
            mc_relscale[0] = x == 0.0f ? 1.0f : MIN(x, MIN(y, z)) / x;
            mc_relscale[1] = y == 0.0f ? 1.0f : MIN(x, MIN(y, z)) / y;
            mc_relscale[2] = z == 0.0f ? 1.0f : MIN(x, MIN(y, z)) / z;

            // Angular scale DOWN factors
            //  xrot = min(xrot, yrot, zrot) / xrot
            //  yrot = min(xrot, yrot, zrot) / yrot
            //  zrot = min(xrot, yrot, zrot) / zrot
            mc_relscale[3] = xrot == 0.0f ? 1.0f : MIN(xrot, MIN(yrot, zrot)) / xrot;
            mc_relscale[4] = yrot == 0.0f ? 1.0f : MIN(xrot, MIN(yrot, zrot)) / yrot;
            mc_relscale[5] = zrot == 0.0f ? 1.0f : MIN(xrot, MIN(yrot, zrot)) / zrot;

            mc_set_relscale(mc_relscale);

            // Reapply saved speed when scale factors change
            cmdctrl_apply_speed();

            // Acknowledge message w/ no error
            cmdctrl_acknowledge(msg_id, ACK_ERR_NONE, NULL, 0);
        }

    }else if(message_starts_with_str(msg, len, "MMATS")){
        // Set Motor Matrix Row command
        // M, M, A, T, S, [thruster_num], [data]
        // [thruster_num] is an 8-bit int from 1-8 (inclusive on both ends)
        // [data] is a set of 6 32-bit floats (24 bytes)
        //        Lowest indices are lowest thruster number (little endian floats)

        if(len != 30){
            // Message is incorrect size
            cmdctrl_acknowledge(msg_id, ACK_ERR_INVALID_ARGS, NULL, 0);
        }else{
            // Message is correct size
            
            if(msg[5] > 8 || msg[5] < 1){
                // Invalid thruster number
                cmdctrl_acknowledge(msg_id, ACK_ERR_INVALID_ARGS, NULL, 0);
            }else{
                // Construct data array
                // Note: there is no validation of motor matrix data
                float data[6];
                data[0] = conversions_data_to_float(&msg[6], true);
                data[1] = conversions_data_to_float(&msg[10], true);
                data[2] = conversions_data_to_float(&msg[14], true);
                data[3] = conversions_data_to_float(&msg[18], true);
                data[4] = conversions_data_to_float(&msg[22], true);
                data[5] = conversions_data_to_float(&msg[26], true);

                // Set the data
                mc_set_dof_matrix(msg[5], data);

                // Acknowledge message w/ no error.
                cmdctrl_acknowledge(msg_id, ACK_ERR_NONE, NULL, 0);
            }
        }
    }else if(message_starts_with_str(msg, len, "MMATU")){
        // Motor Matrix Update command (call after all rows written)
        // M, M, A, T, U

        // Recalc things after motor matrix is fully updated
        mc_recalc();

        // Need to re-apply speeds if motor matrix changes
        cmdctrl_apply_speed();

        // Acknowledge message w/ no error.
        cmdctrl_acknowledge(msg_id, ACK_ERR_NONE, NULL, 0);
    }else if(message_starts_with_str(msg, len, "PIDTN")){
        // Tune PID
        // P, I, D, T, N, [which], [kp], [ki], [kd], [limit], [invert]
        // Each gain value (kp, ki, kd) is a 32-bit little endian float
        // which = what PID to tune X (xrot), Y (yrot), Z (zrot), D (depth) (one byte, ASCII char)
        // kp, ki, kd are gains. limit is max output of PID (magnitude, must be positive)
        // invert == 1 negates the default PID output (single byte 1 or 0)
        if(len != 23){
            cmdctrl_acknowledge(msg_id, ACK_ERR_INVALID_ARGS, NULL, 0);
        }else{
            float kp = conversions_data_to_float(&msg[6], true);
            float ki = conversions_data_to_float(&msg[10], true);
            float kd = conversions_data_to_float(&msg[14], true);
            float limit = conversions_data_to_float(&msg[18], true);
            bool invert = msg[22];
            switch(msg[5]){
            case 'X':
                mc_sassist_tune_xrot(kp, ki, kd, limit, invert);
                break;
            case 'Y':
                mc_sassist_tune_yrot(kp, ki, kd, limit, invert);
                break;
            case 'Z':
                mc_sassist_tune_zrot(kp, ki, kd, limit, invert);
                break;
            case 'D':
                mc_sassist_tune_depth(kp, ki, kd, limit, invert);
                break;
            }
            cmdctrl_acknowledge(msg_id, ACK_ERR_NONE, NULL, 0);
        }
    }
    // -----------------------------------------------------------------------------------------------------------------

    // -----------------------------------------------------------------------------------------------------------------
    // Sensor data commands / queries
    // -----------------------------------------------------------------------------------------------------------------
    else if(message_equals_str(msg, len, "SSTAT")){
        // Sensor status query
        // S, S, T, A, T
        // ACK contains data in the following format [imu_status],[depth_status]
        // each [sensor_status] is an 8-bit int where each bit indicates if a sensor in use. 
        // A value of 0 indicates no sensor. Any non-zero value indicates a specific IMU or depth sensor is available

        uint8_t response[2];
        response[0] = imu_get_sensor();
        response[1] = depth_get_sensor();

        cmdctrl_acknowledge(msg_id, ACK_ERR_NONE, response, 2);
    }else if(message_equals_str(msg, len, "IMUR")){
        // One-shot read of IMU data
        // I, M, U, R
        // Response contains [quat_w], [quat_x], [quat_y], [quat_z], [accum_pitch], [accum_roll], [accum_yaw]
        // where each value is a 32-bit float little endian

        if(imu_get_sensor() == IMU_NONE){
            // Sensor not ready. This command is not valid right now.
            cmdctrl_acknowledge(msg_id, ACK_ERR_INVALID_CMD, NULL, 0);
        }else{
            // Store current readings
            imu_data_t dat = imu_get_data();

            // Construct response data
            uint8_t response_data[28];
            conversions_float_to_data(dat.quat.w, &response_data[0], true);
            conversions_float_to_data(dat.quat.x, &response_data[4], true);
            conversions_float_to_data(dat.quat.y, &response_data[8], true);
            conversions_float_to_data(dat.quat.z, &response_data[12], true);
            conversions_float_to_data(dat.accum_angles.pitch, &response_data[16], true);
            conversions_float_to_data(dat.accum_angles.roll, &response_data[20], true);
            conversions_float_to_data(dat.accum_angles.yaw, &response_data[24], true);

            // Send ack with response data
            cmdctrl_acknowledge(msg_id, ACK_ERR_NONE, response_data, 28);
        }
    }else if(message_equals_str(msg, len, "IMUW")){
        // Read IMU RAW data
        // I, M, U, W
        // Response contains [accel_x], [accel_y], [accel_z], [gyro_x], [gyro_y], [gyro_z]
        // where each value is a 32-bit float little endian

        if(imu_get_sensor() == IMU_NONE){
            // Sensor not ready. This command is not valid right now.
            cmdctrl_acknowledge(msg_id, ACK_ERR_INVALID_CMD, NULL, 0);
        }else{
            // Store current readings
            imu_data_t data = imu_get_data();

            // Construct response data
            uint8_t response_data[24];
            conversions_float_to_data(data.raw_accel.x, &response_data[0], true);
            conversions_float_to_data(data.raw_accel.y, &response_data[4], true);
            conversions_float_to_data(data.raw_accel.z, &response_data[8], true);
            conversions_float_to_data(data.raw_gyro.x, &response_data[12], true);
            conversions_float_to_data(data.raw_gyro.y, &response_data[16], true);
            conversions_float_to_data(data.raw_gyro.z, &response_data[20], true);

            // Send ack with response data
            cmdctrl_acknowledge(msg_id, ACK_ERR_NONE, response_data, 24);
        }
    }else if(message_starts_with_str(msg, len, "IMUP")){
        // IMU periodic read configure
        // I, M, U, P, [enable]
        // [enable] is 1 or 0 (8-bit int) 1 = true (periodic read enabled). 0 = false (not enabled)
        
        if(len != 5){
            cmdctrl_acknowledge(msg_id, ACK_ERR_INVALID_ARGS, NULL, 0);
        }else{
            periodic_imu = msg[4];
            cmdctrl_acknowledge(msg_id, ACK_ERR_NONE, NULL, 0);
        }
    }else if(message_equals_str(msg, len, "DEPTHR")){
        // One-shot read of BNO055 data (all data)
        // D, E, P, T, H, R
        // Response contains [depth_m], [pressure], [temp]
        // where each value is a 32-bit float little endian

        if(depth_get_sensor() == DEPTH_NONE){
            // Sensor not ready. This command is not valid right now.
            cmdctrl_acknowledge(msg_id, ACK_ERR_INVALID_CMD, NULL, 0);
        }else{
            // Store current readings
            depth_data_t dat = depth_get_data();

            // Construct response data
            uint8_t response_data[12];
            conversions_float_to_data(dat.depth_m, &response_data[0], true);
            conversions_float_to_data(dat.pressure_pa, &response_data[4], true);
            conversions_float_to_data(dat.temperature_c, &response_data[8], true);

            // Send ack with response data
            cmdctrl_acknowledge(msg_id, ACK_ERR_NONE, response_data, 12);
        }
    }else if(message_starts_with_str(msg, len, "DEPTHP")){
        // MS5837 periodic read configure
        // D, E, P, T, H, P, [enable]
        // [enable] is 1 or 0 (8-bit int) 1 = true (periodic read enabled). 0 = false (not enabled)
        
        if(len != 7){
            cmdctrl_acknowledge(msg_id, ACK_ERR_INVALID_ARGS, NULL, 0);
        }else{
            periodic_depth = msg[6];
            cmdctrl_acknowledge(msg_id, ACK_ERR_NONE, NULL, 0);
        }
    }
    // -----------------------------------------------------------------------------------------------------------------


    // -----------------------------------------------------------------------------------------------------------------
    // BNO055 Configuration Commands & Queries
    // -----------------------------------------------------------------------------------------------------------------
    else if(message_starts_with_str(msg, len, "BNO055A")){
        // BNO055 Axis config command: sets axis remap for BNO055 IMU
        // B, N, O, 0, 5, 5, A, [mode]
        // [mode] is a single byte with value 0-7 for BNO055 axis config P0 to P7
        // Modes are described in sensor's datasheet
        if(len != 8){
            // Wrong length
            cmdctrl_acknowledge(msg_id, ACK_ERR_INVALID_ARGS, NULL, 0);
        }else{
            // Message is correct size. Handle it.
            if(msg[7] > 7){
                // Invalid mode
                cmdctrl_acknowledge(msg_id, ACK_ERR_INVALID_ARGS, NULL, 0);
            }else{
                // Valid mode. Set it.
                bno055_set_axis(msg[7]);
                cmdctrl_acknowledge(msg_id, ACK_ERR_NONE, NULL, 0);
            }
        }
    }else if(message_equals_str(msg, len, "SCBNO055R")){
        // S, C, B, N, O, 0, 5, 5, R
        // Read stored calibration constants for BNO055
        // ACK contains the following data
        // [valid], [accel_offset_x], [accel_offset_y], [accel_offset_z], [accel_radius], [gyro_offset_x], [gyro_offset_y], [gyro_offset_z]
        // Valid is a single byte. 0 = invalid, 1 = valid data
        // All other values are 16-bit little endian integers. If data is valid these will be calibration constants.
        
        // Calibration constants are loaded on program startup (and will not change)
        // so no reason to call calibration_load_bno055 again
        uint8_t buf[15];
        buf[0] = calibration_bno055.valid ? 1 : 0;
        conversions_int16_to_data(calibration_bno055.accel_offset_x, &buf[1], true);
        conversions_int16_to_data(calibration_bno055.accel_offset_y, &buf[3], true);
        conversions_int16_to_data(calibration_bno055.accel_offset_z, &buf[5], true);
        conversions_int16_to_data(calibration_bno055.accel_radius, &buf[7], true);
        conversions_int16_to_data(calibration_bno055.gyro_offset_x, &buf[9], true);
        conversions_int16_to_data(calibration_bno055.gyro_offset_y, &buf[11], true);
        conversions_int16_to_data(calibration_bno055.gyro_offset_z, &buf[13], true);
        cmdctrl_acknowledge(msg_id, ACK_ERR_NONE, buf, 15);
    }else if(message_equals_str(msg, len, "SCBNO055E")){
        // S, C, B, N, O, 0, 5, 5, E
        // Erase stored calibration constants for BNO055
        // Then reset the sensor (so it loosed the ones programmed earlier)
        calibration_erase_bno055();
        if(imu_get_sensor() == IMU_BNO055)
            bno055_configure();   // This will reset the sensor as a part of configuration process
        cmdctrl_acknowledge(msg_id, ACK_ERR_NONE, NULL, 0);
    }else if(message_starts_with_str(msg, len, "SCBNO055S")){
        // S, C, B, N, O, 0, 5, 5, S, [accel_offset_x], [accel_offset_y], [accel_offset_z], [accel_radius], [gyro_offset_x], [gyro_offset_y], [gyro_offset_z]
        // Write stored calibration constants for BNO055
        // All values are 16-bit little endian integers (calibration values)
        if(len != 23){
            cmdctrl_acknowledge(msg_id, ACK_ERR_INVALID_ARGS, NULL, 0);
        }else{
            bno055_cal_t new_cal;
            new_cal.accel_offset_x = conversions_data_to_int16(&msg[9], true);
            new_cal.accel_offset_y = conversions_data_to_int16(&msg[11], true);
            new_cal.accel_offset_z = conversions_data_to_int16(&msg[13], true);
            new_cal.accel_radius = conversions_data_to_int16(&msg[15], true);
            new_cal.gyro_offset_x = conversions_data_to_int16(&msg[17], true);
            new_cal.gyro_offset_y = conversions_data_to_int16(&msg[19], true);
            new_cal.gyro_offset_z = conversions_data_to_int16(&msg[21], true);
            calibration_store_bno055(new_cal);
            if(imu_get_sensor() == IMU_BNO055)
                bno055_configure();     // Reconfigure bno055 will reset the sensor and apply the stored calibration
            cmdctrl_acknowledge(msg_id, ACK_ERR_NONE, NULL, 0);
        }
    }else if(message_equals_str(msg, len, "BNO055CS")){
        // Get current BNO055 calibration status (from the sensor itself)
        // ACK will contain the following
        // [status] an 8-bit integer with the value of the sensor's CALIB_STAT register
        if(imu_get_sensor() != IMU_BNO055){
            // Cannot read status if sensor not ready
            cmdctrl_acknowledge(msg_id, ACK_ERR_INVALID_CMD, NULL, 0);
        }else{
            uint8_t status;
            bool res = bno055_read_calibration_status(&status);
            if(!res){
                // If this fails, sensor is probably not connected anymore
                cmdctrl_acknowledge(msg_id, ACK_ERR_INVALID_CMD, NULL, 0);
            }else{
                cmdctrl_acknowledge(msg_id, ACK_ERR_NONE, &status, 1);
            }
        }
    }else if(message_equals_str(msg, len, "BNO055CV")){
        // Get current BNO055 calibration values (from the sensor itself)
        // ACK will contain the following
        // [accel_offset_x], [accel_offset_y], [accel_offset_z], [accel_radius], [gyro_offset_x], [gyro_offset_y], [gyro_offset_z]
        //All are little endian 16-bit integers (signed)
        
        if(imu_get_sensor() != IMU_BNO055){
            // Cannot read status if sensor not ready
            cmdctrl_acknowledge(msg_id, ACK_ERR_INVALID_CMD, NULL, 0);
        }else{
            int16_t acc_offset_x, acc_offset_y, acc_offset_z, acc_radius;
            int16_t gyr_offset_x, gyr_offset_y, gyr_offset_z;
            bool res = bno055_read_calibration(&acc_offset_x, &acc_offset_y, &acc_offset_z, &acc_radius,
                    &gyr_offset_x, &gyr_offset_y, &gyr_offset_z);
            if(!res){
                // If this fails, sensor is probably not connected anymore
                cmdctrl_acknowledge(msg_id, ACK_ERR_INVALID_CMD, NULL, 0);
            }else{
                uint8_t buf[14];
                conversions_int16_to_data(acc_offset_x, &buf[0], true);
                conversions_int16_to_data(acc_offset_y, &buf[2], true);
                conversions_int16_to_data(acc_offset_z, &buf[4], true);
                conversions_int16_to_data(acc_radius, &buf[6], true);
                conversions_int16_to_data(gyr_offset_x, &buf[8], true);
                conversions_int16_to_data(gyr_offset_y, &buf[10], true);
                conversions_int16_to_data(gyr_offset_z, &buf[12], true);
                cmdctrl_acknowledge(msg_id, ACK_ERR_NONE, buf, 14);
            }
        }
    }else if(message_equals_str(msg, len, "BNO055RST")){
        // B, N, O, 0, 5, 5, R, S, T
        // BNO055 reset / reconfigure
        // This is typically used to clear auto generated calibration constants when
        // no calibration is stored.
        bno055_configure();
        cmdctrl_acknowledge(msg_id, ACK_ERR_NONE, NULL, 0);
    }// -----------------------------------------------------------------------------------------------------------------
    // -----------------------------------------------------------------------------------------------------------------

    // -----------------------------------------------------------------------------------------------------------------
    // MS5837 Configuration Commands & Queries
    // -----------------------------------------------------------------------------------------------------------------
    else if(message_equals_str(msg, len, "MS5837CALG")){
        // M, S, 5, 8, 3, 7, C, A, L, G
        // Read MS5837 calibration
        // ACK contains [atm_pressure], [fluid_density]
        // Each is a little endian 32-bit float
        if(depth_get_sensor() != DEPTH_MS5837){
            cmdctrl_acknowledge(msg_id, ACK_ERR_INVALID_CMD, NULL, 0);
        }else{
            uint8_t buf[8];
            conversions_float_to_data(calibration_ms5837.atm_pressure, &buf[0], true);
            conversions_float_to_data(calibration_ms5837.fluid_density, &buf[4], true);
            cmdctrl_acknowledge(msg_id, ACK_ERR_NONE, buf, 8);
        }
    }else if(message_starts_with_str(msg, len, "MS5837CALS")){
        // M, S, 5, 8, 3, 7, C, A, L, S, [atm_pressure], [fluid_density]
        // Set MS5837 calibration
        // Both values are little endian 32-bit floats
        if(len != 18){
            cmdctrl_acknowledge(msg_id, ACK_ERR_INVALID_ARGS, NULL, 0);
        }else{
            if(depth_get_sensor() != DEPTH_MS5837){
                cmdctrl_acknowledge(msg_id, ACK_ERR_INVALID_CMD, NULL, 0);
            }else{
                calibration_ms5837.atm_pressure = conversions_data_to_float(&msg[10], true);
                calibration_ms5837.fluid_density = conversions_data_to_float(&msg[14], true);
                cmdctrl_acknowledge(msg_id, ACK_ERR_NONE, NULL, 0);
            }
        }
    }
    // -----------------------------------------------------------------------------------------------------------------


    // -----------------------------------------------------------------------------------------------------------------
    // Misc commands & queries
    // -----------------------------------------------------------------------------------------------------------------
    else if(message_equals(msg, len, (uint8_t[]){'R', 'E', 'S', 'E', 'T', 0x0D, 0x1E}, 7)){
        // Reset control board command
        // R, E, S, E, T, 0x0D, 0x1E
#if defined(CONTROL_BOARD_V1) || defined(CONTROL_BOARD_V2)
        NVIC_SystemReset();
        while(1);
#endif
        // Not acknowledged. Board resets!
    }else if(message_equals_str(msg, len, "RSTWHY")){
        // R, S, T, W, H, Y
        // ACK contains a 32-bit integer (signed, little endian)
        // indicating one of the HALT_EC codes in debug.h
        uint8_t response[4];
        conversions_int32_to_data(reset_cause, response, true);
        cmdctrl_acknowledge(msg_id, ACK_ERR_NONE, response, 4);
    }else if(message_starts_with_str(msg, len, "SIMHIJACK")){
        // S, I, M, H, I, J, A, C, K, [hijack]
        // [hijack] is an 8-bit int (unsigned) 0 = release, 1 = hijack
        if(len != 10){
            // Message is incorrect size
            cmdctrl_acknowledge(msg_id, ACK_ERR_INVALID_ARGS, NULL, 0);
        }else{
            cmdctrl_simhijack(msg[9]);
            cmdctrl_acknowledge(msg_id, ACK_ERR_NONE, NULL, 0);
        }
    }else if(message_starts_with_str(msg, len, "SIMDAT")){
        // S, I, M, D, A, T, [w], [x], [y], [z], [depth]
        // All values are little endian floats (32-bit)
        // x, y, z, w are current quaternion (BNO055 data)
        // depth is current depth (MS5837 data)
        if(len != 26){
            // Message is incorrect size
            cmdctrl_acknowledge(msg_id, ACK_ERR_INVALID_ARGS, NULL, 0);
        }else{
            // Parse received data
            cmdctrl_sim_quat.w = conversions_data_to_float(&msg[6], true);
            cmdctrl_sim_quat.x = conversions_data_to_float(&msg[10], true);
            cmdctrl_sim_quat.y = conversions_data_to_float(&msg[14], true);
            cmdctrl_sim_quat.z = conversions_data_to_float(&msg[18], true);
            cmdctrl_sim_depth = conversions_data_to_float(&msg[22], true);

            // Message handled successfully
            cmdctrl_acknowledge(msg_id, ACK_ERR_NONE, NULL, 0);
        }
    }else if(message_equals_str(msg, len, "CBVER")){
        // Control board version info query
        // C, B, V, E, R
        // Responds with
        // [CB_VER], [FW_MAJOR], [FW_MINOR], [FW_REV], [FW_TYPE], [FW_BUILD]
        // All values are unsigned 8-bit integers
        // CB_VER = control board version (v1 or v2 hardware)
        // FW_MAJOR = Firmware major version
        // FW_MINOR = Firmware minor version
        // FW_REV = Firmware revision version
        // FW_TYPE = Firmware type (ASCII: a = alpha, b = beta, c = release candidate, ' ' = full release)
        // FW_BUILD = Firmware build version (if type is not ' '). If type is ' ' this will be 0.
        uint8_t response[6];
        #if defined(CONTROL_BOARD_V1)
            response[0] = 1;
        #elif defined(CONTROL_BOARD_V2)
            response[0] = 2;
        #else
            response[0] = 255;
        #endif
        response[1] = FW_VER_MAJOR;
        response[2] = FW_VER_MINOR;
        response[3] = FW_VER_REVISION;
        response[4] = FW_VER_TYPE;
        response[5] = (FW_VER_TYPE == ' ') ? 0 : FW_VER_BUILD;
        cmdctrl_acknowledge(msg_id, ACK_ERR_NONE, response, 6);
    }
    // -----------------------------------------------------------------------------------------------------------------


    else{
        // This is an unrecognized message
        cmdctrl_acknowledge(msg_id, ACK_ERR_UNKNOWN_MSG, NULL, 0);
    }
}

void cmdctrl_send_mwodg_status(bool me){
    pccomm_write((uint8_t[]){'W', 'D', 'G', 'S', me}, 5);
}

void cmdctrl_send_simstat(void){
    uint8_t simstat[41];
    simstat[0] = 'S';
    simstat[1] = 'I';
    simstat[2] = 'M';
    simstat[3] = 'S';
    simstat[4] = 'T';
    simstat[5] = 'A';
    simstat[6] = 'T';
    conversions_float_to_data(cmdctrl_sim_speeds[0], &simstat[7], true);
    conversions_float_to_data(cmdctrl_sim_speeds[1], &simstat[11], true);
    conversions_float_to_data(cmdctrl_sim_speeds[2], &simstat[15], true);
    conversions_float_to_data(cmdctrl_sim_speeds[3], &simstat[19], true);
    conversions_float_to_data(cmdctrl_sim_speeds[4], &simstat[23], true);
    conversions_float_to_data(cmdctrl_sim_speeds[5], &simstat[27], true);
    conversions_float_to_data(cmdctrl_sim_speeds[6], &simstat[31], true);
    conversions_float_to_data(cmdctrl_sim_speeds[7], &simstat[35], true);
    simstat[39] = mode & 0xFF;
    simstat[40] = mc_wdog_is_killed() ? 1 : 0;
    pccomm_write(simstat, 41);
}

void cmdctrl_simhijack(bool hijack){
#if defined(CONTROL_BOARD_SIM_LINUX) || defined(CONTROL_BOARD_SIM_WINDOWS)
    hijack = true; // SIMCB only supports SIMHIJACK mode
#endif

    if(hijack){
        // Reset data received from simulator
        cmdctrl_sim_quat.w = 0.0f;
        cmdctrl_sim_quat.x = 0.0f;
        cmdctrl_sim_quat.y = 0.0f;
        cmdctrl_sim_quat.z = 0.0f;
        cmdctrl_sim_depth = 0.0f;

        // Reset data output to simulator
        cmdctrl_sim_speeds[0] = 0;
        cmdctrl_sim_speeds[1] = 0;
        cmdctrl_sim_speeds[2] = 0;
        cmdctrl_sim_speeds[3] = 0;
        cmdctrl_sim_speeds[4] = 0;
        cmdctrl_sim_speeds[5] = 0;
        cmdctrl_sim_speeds[6] = 0;
        cmdctrl_sim_speeds[7] = 0;

        // Revert to a stoped state
        mode = MODE_RAW;
        led_set(COLOR_RAW);
        raw_target[0] = 0.0f;
        raw_target[1] = 0.0f;
        raw_target[2] = 0.0f;
        raw_target[3] = 0.0f;
        raw_target[4] = 0.0f;
        raw_target[5] = 0.0f;
        raw_target[6] = 0.0f;
        raw_target[7] = 0.0f;
        mc_set_raw(raw_target);

        // Do this last so set_local (above) uses real thrusters
#if !(defined(CONTROL_BOARD_SIM_LINUX) || defined(CONTROL_BOARD_SIM_WINDOWS))
        cmdctrl_sim_hijacked = true;
#endif
    }else{
#if !(defined(CONTROL_BOARD_SIM_LINUX) || defined(CONTROL_BOARD_SIM_WINDOWS))
        // Do this first so set_local (below) uses real thrusters
        cmdctrl_sim_hijacked = false;
#endif

        // Revert to a stoped state
        mode = MODE_RAW;
        led_set(COLOR_RAW);
        raw_target[0] = 0.0f;
        raw_target[1] = 0.0f;
        raw_target[2] = 0.0f;
        raw_target[3] = 0.0f;
        raw_target[4] = 0.0f;
        raw_target[5] = 0.0f;
        raw_target[6] = 0.0f;
        raw_target[7] = 0.0f;
        mc_set_raw(raw_target);
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

