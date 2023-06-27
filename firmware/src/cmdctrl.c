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
#include <led.h>
#include <pccomm.h>
#include <conversions.h>
#include <thruster.h>
#include <motor_control.h>
#include <FreeRTOS.h>
#include <semphr.h>
#include <timers.h>
#include <bno055.h>
#include <wdt.h>
#include <debug.h>
#include <simulator.h>
#include <calibration.h>

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Macros
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Control modes
#define MODE_RAW        0
#define MODE_LOCAL      1
#define MODE_GLOBAL     2
#define MODE_SASSIST    3
#define MODE_DHOLD      4

// LED colors for different modes
// Different colors on different versions b/c LEDs are different
// Designed to look visually similar
#if defined(CONTROL_BOARD_V1)
#define COLOR_RAW           100, 100, 0
#define COLOR_LOCAL         10, 0, 100
#define COLOR_GLOBAL        150, 50, 0
#define COLOR_SASSIST       0, 100, 100
#define COLOR_DHOLD         10, 50, 0
#elif defined(CONTROL_BOARD_V2)
#define COLOR_RAW           170, 40, 0
#define COLOR_LOCAL         30, 0, 100
#define COLOR_GLOBAL        255, 15, 0
#define COLOR_SASSIST       0, 100, 100
#define COLOR_DHOLD         10, 40, 0
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

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Globals
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static bool motors_enabled;

static unsigned int mode;

// Last used raw mode target
static float raw_target[8];

// Last used local mode target
static float local_x;
static float local_y;
static float local_z;
static float local_xrot;
static float local_yrot;
static float local_zrot;

// Last used global mode target
static float global_x;
static float global_y;
static float global_z;
static float global_pitch_spd;
static float global_roll_spd;
static float global_yaw_spd;

// Last used sassist mode target
static bool sassist_valid;
static unsigned int sassist_variant;
static float sassist_x;
static float sassist_y;
static float sassist_yaw_spd;
static euler_t sassist_target_euler;
static float sassist_depth_target;

// Last used depth hold target
static float dhold_x;
static float dhold_y;
static float dhold_pitch_spd;
static float dhold_roll_spd;
static float dhold_yaw_spd;
static float dhold_depth;

// Current sensor data
// Need mutex b/c don't want to read one value (eg x) then have others (y, z) changed before reading them
static SemaphoreHandle_t sensor_data_mutex;
static bno055_data curr_bno055_data;
static ms5837_data curr_ms5837_data;

// Sensor status flags
static bool _bno055_ready;
static bool _ms5837_ready;

// Periodic reading of sensor data timer
static bool periodic_bno055;
static bool periodic_ms5837;
static TimerHandle_t sensor_read_timer;

// Used to periodically re-apply speeds in modes where 
static TimerHandle_t periodic_speed_timer;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// CMDCTRL functions / implementation
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static inline bool __attribute__((always_inline)) bno055_ready(void){
    return sim_hijacked ? true : _bno055_ready;
}

static inline bool __attribute__((always_inline)) ms5837_ready(void){
    return sim_hijacked ? true : _ms5837_ready;
}

static void send_sensor_data(TimerHandle_t timer){
    (void)timer;
    
    // Send the data for sensors as needed
    if(periodic_bno055 && bno055_ready()){
        // Store current readings
        xSemaphoreTake(sensor_data_mutex, portMAX_DELAY);
        bno055_data dat = curr_bno055_data;
        xSemaphoreGive(sensor_data_mutex);

        // Construct message
        uint8_t bno055_data[35];
        bno055_data[0] = 'B';
        bno055_data[1] = 'N';
        bno055_data[2] = 'O';
        bno055_data[3] = '0';
        bno055_data[4] = '5';
        bno055_data[5] = '5';
        bno055_data[6] = 'D';
        conversions_float_to_data(dat.curr_quat.w, &bno055_data[7], true);
        conversions_float_to_data(dat.curr_quat.x, &bno055_data[11], true);
        conversions_float_to_data(dat.curr_quat.y, &bno055_data[15], true);
        conversions_float_to_data(dat.curr_quat.z, &bno055_data[19], true);
        conversions_float_to_data(dat.accum_pitch, &bno055_data[23], true);
        conversions_float_to_data(dat.accum_roll, &bno055_data[27], true);
        conversions_float_to_data(dat.accum_yaw, &bno055_data[31], true);

        // Send message (status message from CB to PC)
        pccomm_write(bno055_data, 35);
    }
    if(periodic_ms5837 & ms5837_ready()){
        // Store current readings
        xSemaphoreTake(sensor_data_mutex, portMAX_DELAY);
        float m_depth_m = curr_ms5837_data.depth_m;
        xSemaphoreGive(sensor_data_mutex);

        // Construct message
        uint8_t ms5837_data[11];
        ms5837_data[0] = 'M';
        ms5837_data[1] = 'S';
        ms5837_data[2] = '5';
        ms5837_data[3] = '8';
        ms5837_data[4] = '3';
        ms5837_data[5] = '7';
        ms5837_data[6] = 'D';
        conversions_float_to_data(m_depth_m, &ms5837_data[7], true);

        // Send message (status message from CB to PC)
        pccomm_write(ms5837_data, 11);
    }

    // Not using auto reload so that any time taken to
    // enforce a duration between sends, not a rate of sending data
    xTimerStart(sensor_read_timer, portMAX_DELAY);
}

static void periodic_reapply_speed(TimerHandle_t timer){
    (void)timer;

    // Modes using sensor data (which may change) need to be periodically reapplied
    if(mode == MODE_GLOBAL || mode == MODE_SASSIST || mode == MODE_DHOLD){
        cmdctrl_apply_saved_speed();
    }

    // Not using auto reload so that any time taken to
    // enforce a duration between sets, not a rate of setting speeds
    xTimerStart(periodic_speed_timer, portMAX_DELAY);
}

void cmdctrl_init(void){
    // Initialize targets for all modes to result in no motion

    // Raw mode (zero all motor speeds)
    for(unsigned int i = 0; i < 8; ++i)
        raw_target[i] = 0.0f;

    // Local mode (zero all)
    local_x = 0.0f;
    local_y = 0.0f;
    local_z = 0.0f;
    local_xrot = 0.0f;
    local_yrot = 0.0f;
    local_zrot = 0.0f;

    // Global mode (zero all)
    global_x = 0.0f;
    global_y = 0.0f;
    global_z = 0.0f;
    global_pitch_spd = 0.0f;
    global_roll_spd = 0.0f;
    global_yaw_spd = 0.0f;

    // Sassist mode (set valid bool to false)
    sassist_valid = false;
    sassist_variant = 1;
    sassist_x = 0.0f;
    sassist_y = 0.0f;
    sassist_yaw_spd = 0.0f;
    sassist_target_euler.pitch = 0.0f;
    sassist_target_euler.roll = 0.0f;
    sassist_target_euler.yaw = 0.0f;
    sassist_target_euler.is_deg = true;
    sassist_depth_target = 0.0f;

    // Zero depth hold target
    dhold_x = 0.0;
    dhold_y = 0.0;
    dhold_pitch_spd = 0.0;
    dhold_roll_spd = 0.0;
    dhold_yaw_spd = 0.0;
    dhold_depth = 0.0;

    // Initial sensor status
    _bno055_ready = false;
    _ms5837_ready = false;

    // Initial sensor data
    curr_bno055_data.curr_quat.w = 0;
    curr_bno055_data.curr_quat.x = 0;
    curr_bno055_data.curr_quat.y = 0;
    curr_bno055_data.curr_quat.z = 0;
    curr_bno055_data.accum_pitch = 0;
    curr_bno055_data.accum_roll = 0;
    curr_bno055_data.accum_yaw = 0;
    sensor_data_mutex = xSemaphoreCreateMutex();

    // Periodic sensor data
    periodic_bno055 = false;
    periodic_ms5837 = false;
    sensor_read_timer = xTimerCreate(
        "sensor_read",
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
 * Check if two byte arrays are identical
 * @param a First byte array
 * @param len_a Length of first array
 * @param b Second byte array
 * @param len_b Length of second array
 * @return true If arrays match
 * @return false If arrays do not match
 */
static bool data_matches(const uint8_t *a, uint32_t len_a, const uint8_t *b, uint32_t len_b){
    if(len_a != len_b)
        return false;
    for(uint32_t i = 0; i < len_a; ++i){
        if(a[i] != b[i])
            return false;
    }
    return true;
}

/**
 * Check if one array starts with the data in another array
 * @param a The array to search in ("full" data)
 * @param len_a Length of array a
 * @param b The array to search for ("sub" / "prefix" data)
 * @param len_b Length of array b
 * @return true If array a starts with array b
 * @return false If array a does not start with array b
 */
static bool data_startswith(const uint8_t *a, uint32_t len_a, const uint8_t *b, uint32_t len_b){
    if(len_a < len_b)
        return false;
    for(uint32_t i = 0; i < len_b; ++i){
        if(a[i] != b[i])
            return false;
    }
    return true;
}

/**
 * Apply the last saved speed for the current mode
 */
void cmdctrl_apply_saved_speed(void){
    quaternion_t m_quat;
    float m_depth;

    switch (mode){
    case MODE_RAW:
        mc_set_raw(raw_target);
        break;
    case MODE_LOCAL:
        mc_set_local(local_x, local_y, local_z, local_xrot, local_yrot, local_zrot);
        break;
    case MODE_GLOBAL:
        xSemaphoreTake(sensor_data_mutex, portMAX_DELAY);
        m_quat = curr_bno055_data.curr_quat;
        xSemaphoreGive(sensor_data_mutex);

        // Make sure sensors still working before applying in global mode
        if(!bno055_ready() || (m_quat.w == 0 && m_quat.x == 0 && m_quat.y == 0 && m_quat.z == 0)){
            // Cannot apply real speed b/c sensor data not available or invalid
            // Thus, stop the thrusters
            mc_set_local(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
        }else{
            mc_set_global(global_x, global_y, global_z, global_pitch_spd, global_roll_spd, global_yaw_spd, m_quat);
        }
        break;
    case MODE_SASSIST:
        if(sassist_valid){
            xSemaphoreTake(sensor_data_mutex, portMAX_DELAY);
            m_quat = curr_bno055_data.curr_quat;
            m_depth = curr_ms5837_data.depth_m;
            xSemaphoreGive(sensor_data_mutex);

            if(!bno055_ready() || (m_quat.w == 0 && m_quat.x == 0 && m_quat.y == 0 && m_quat.z == 0) || !ms5837_ready()){
                // Cannot apply real speed b/c sensor data not available or invalid
                // Thus, stop the thrusters
                mc_set_local(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
            }else if(sassist_variant == 1){
                mc_set_sassist(sassist_x, 
                    sassist_y, 
                    sassist_yaw_spd, 
                    sassist_target_euler, 
                    sassist_depth_target, 
                    m_quat,
                    m_depth,
                    false);
            }else if(sassist_variant == 2){
                mc_set_sassist(sassist_x, 
                    sassist_y, 
                    0.0f,
                    sassist_target_euler, 
                    sassist_depth_target, 
                    m_quat,
                    m_depth,
                    true);
            }
        }
        break;
    case MODE_DHOLD:
        xSemaphoreTake(sensor_data_mutex, portMAX_DELAY);
        m_quat = curr_bno055_data.curr_quat;
        m_depth = curr_ms5837_data.depth_m;
        xSemaphoreGive(sensor_data_mutex);
        if(!bno055_ready() || (m_quat.w == 0 && m_quat.x == 0 && m_quat.y == 0 && m_quat.z == 0) || !ms5837_ready()){
            // Cannot apply real speed b/c sensor data not available or invalid
            // Thus, stop the thrusters
            mc_set_local(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
        }else{
            mc_set_dhold(dhold_x, dhold_y, dhold_pitch_spd, dhold_roll_spd, dhold_yaw_spd, dhold_depth, m_quat, m_depth);
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

void cmdctrl_handle_message(void){
    // Helper macros
    // First two bytes of message are message ID (so skip them for actual message content)
    // Also, skip last two bytes (these are message CRC)
    #define msg     (&pccomm_read_buf[2])
    #define len     (pccomm_read_len - 4)
    
    // Compare message to byte arrays
    #define MSG_STARTS_WITH(x)      data_startswith(msg, len, (x), sizeof((x)))
    #define MSG_EQUALS(x)           data_matches(msg, len, (x), sizeof(x))

    #define MIN(a, b)    (((a) < (b)) ? (a) : (b))

    // msg_id is first two bytes (unsigned 16-bit int big endian)
    uint16_t msg_id = conversions_data_to_int16(pccomm_read_buf, false);

    if(MSG_STARTS_WITH(((uint8_t[]){'R', 'A', 'W'}))){
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
                if(raw_target[i] < -1.0f) raw_target[i] = -1.0f;
                else if(raw_target[i] > 1.0f) raw_target[i] = 1.0f;
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
            mc_set_raw(raw_target);

            // Acknowledge message w/ no error.
            cmdctrl_acknowledge(msg_id, ACK_ERR_NONE, NULL, 0);
        }
    }else if(MSG_STARTS_WITH(((uint8_t[]){'T', 'I', 'N', 'V'}))){
        // Thruster inversion set command
        // T, I, N, V, [inv]
        // [inv] is an 8-bit int where MSB corresponds to thruster 8 and LSB thruster 1
        //       1 = inverted. 0 = not inverted
        
        if(len != 5){
            // Message is incorrect size
            cmdctrl_acknowledge(msg_id, ACK_ERR_INVALID_ARGS, NULL, 0);
        }else{
            // Message is correct size. Handle it.
            uint8_t inv = msg[4];
            for(unsigned int i = 0; i < 8; ++i){
                mc_invert[i] = inv & 1;
                inv >>= 1;
            }

            // Reapply saved speed when inversions change
            cmdctrl_apply_saved_speed();

            // Acknowledge message w/ no error.
            cmdctrl_acknowledge(msg_id, ACK_ERR_NONE, NULL, 0);
        }
    }else if(MSG_STARTS_WITH(((uint8_t[]){'R', 'E', 'L', 'D', 'O', 'F'}))){
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

            // Reapply saved speed when scale factors change
            cmdctrl_apply_saved_speed();

            // Acknowledge message w/ no error
            cmdctrl_acknowledge(msg_id, ACK_ERR_NONE, NULL, 0);
        }

    }else if(MSG_EQUALS(((uint8_t[]){'W', 'D', 'G', 'F'}))){
        // Feed motor watchdog command
        // W, D, G, F

        // Feed watchdog (as requested)
        bool was_killed = mc_wdog_feed();

        // Restore last set speed if previously killed
        if(was_killed)
            cmdctrl_apply_saved_speed();

        // Acknowledge message w/ no error.
        cmdctrl_acknowledge(msg_id, ACK_ERR_NONE, NULL, 0);
    }else if(MSG_STARTS_WITH(((uint8_t[]){'M', 'M', 'A', 'T', 'S'}))){
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
    }else if(MSG_EQUALS(((uint8_t[]){'M', 'M', 'A', 'T', 'U'}))){
        // Motor Matrix Update command (call after all rows written)
        // M, M, A, T, U

        // Recalc things after motor matrix is fully updated
        mc_recalc();

        // Acknowledge message w/ no error.
        cmdctrl_acknowledge(msg_id, ACK_ERR_NONE, NULL, 0);
    }else if(MSG_STARTS_WITH(((uint8_t[]){'L', 'O', 'C', 'A', 'L'}))){
        // LOCAL Speed Set
        // L, O, C, A, L, [x], [y], [z], [xrot], [yrot], [zrot]
        // [x], [y], [z], [xrot], [yrot], [zrot]  are 32-bit floats (little endian)

        if(len != 29){
            // Message is incorrect size
            cmdctrl_acknowledge(msg_id, ACK_ERR_INVALID_ARGS, NULL, 0);
        }else{
            // Message is correct size. Handle it.

            // Get speeds from message
            local_x = conversions_data_to_float(&msg[5], true);
            local_y = conversions_data_to_float(&msg[9], true);
            local_z = conversions_data_to_float(&msg[13], true);
            local_xrot = conversions_data_to_float(&msg[17], true);
            local_yrot = conversions_data_to_float(&msg[21], true);
            local_zrot = conversions_data_to_float(&msg[25], true);

            // Ensure speeds are in valid range
            LIMIT(local_x);
            LIMIT(local_y);
            LIMIT(local_z);
            LIMIT(local_xrot);
            LIMIT(local_yrot);
            LIMIT(local_zrot);

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
            mc_set_local(local_x, local_y, local_z, local_xrot, local_yrot, local_zrot);

            // Acknowledge message w/ no error.
            cmdctrl_acknowledge(msg_id, ACK_ERR_NONE, NULL, 0);
        }
    }else if(MSG_EQUALS(((uint8_t[]){'S', 'S', 'T', 'A', 'T'}))){
        // Sensor status query
        // S, S, T, A, T
        // ACK contains data in the following format [sensor_status]
        // [sensor_status] is an 8-bit int where each bit indicates if a sensor is ready
        // bit 0 (LSB) is BNO055 status (1 = ready, 0 = not ready)
        // bit 1 is MS5837 status (1 = ready, 0 = not ready)

        uint8_t response[1];
        response[0] = 0x00;
        response[0] |= bno055_ready();
        response[0] |= (ms5837_ready() << 1);

        cmdctrl_acknowledge(msg_id, ACK_ERR_NONE, response, 1);
    }else if(MSG_STARTS_WITH(((uint8_t[]){'B', 'N', 'O', '0', '5', '5', 'A'}))){
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
    }else if(MSG_STARTS_WITH(((uint8_t[]){'G', 'L', 'O', 'B', 'A', 'L'}))){
        // GLOBAL speed set
        // G, L, O, B, A, L, [x], [y], [z], [pitch_spd], [roll_spd], [yaw_spd]
        // [x], [y], [z], [pitch_spd], [roll_spd], [yaw_spd]  are 32-bit floats (little endian)
        if(len != 30){
            // Message is incorrect size
            cmdctrl_acknowledge(msg_id, ACK_ERR_INVALID_ARGS, NULL, 0);
        }else{
            // Message is correct size. Handle it.

            xSemaphoreTake(sensor_data_mutex, portMAX_DELAY);
            quaternion_t m_quat = curr_bno055_data.curr_quat;
            xSemaphoreGive(sensor_data_mutex);

            if(!bno055_ready() || (m_quat.w == 0 && m_quat.x == 0 && m_quat.y == 0 && m_quat.z == 0)){
                // Need BNO055 IMU data to use global mode.
                // If not ready, then this command is invalid at this time
                cmdctrl_acknowledge(msg_id, ACK_ERR_INVALID_CMD, NULL, 0);
            }else{
                // Get speeds from message
                global_x = conversions_data_to_float(&msg[6], true);
                global_y = conversions_data_to_float(&msg[10], true);
                global_z = conversions_data_to_float(&msg[14], true);
                global_pitch_spd = conversions_data_to_float(&msg[18], true);
                global_roll_spd = conversions_data_to_float(&msg[22], true);
                global_yaw_spd = conversions_data_to_float(&msg[26], true);

                // Ensure speeds are in valid range
                LIMIT(global_x);
                LIMIT(global_y);
                LIMIT(global_z);
                LIMIT(global_pitch_spd);
                LIMIT(global_roll_spd);
                LIMIT(global_yaw_spd);

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
                mc_set_global(global_x, global_y, global_z, global_pitch_spd, global_roll_spd, global_yaw_spd, m_quat);

                // Acknowledge message w/ no error.
                cmdctrl_acknowledge(msg_id, ACK_ERR_NONE, NULL, 0);
            }
        }
    }else if(MSG_EQUALS(((uint8_t[]){'B', 'N', 'O', '0', '5', '5', 'R'}))){
        // One-shot read of BNO055 data (all data)
        // B, N, O, 0, 5, 5, R
        // Response contains [grav_x], [grav_y], [grav_z], [euler_pitch], [euler_roll], [euler_yaw]
        // where each value is a 32-bit float little endian

        if(!bno055_ready()){
            // Sensor not ready. This command is not valid right now.
            cmdctrl_acknowledge(msg_id, ACK_ERR_INVALID_CMD, NULL, 0);
        }else{
            // Store current readings
            xSemaphoreTake(sensor_data_mutex, portMAX_DELAY);
            bno055_data dat = curr_bno055_data;
            xSemaphoreGive(sensor_data_mutex);

            // Construct response data
            uint8_t response_data[28];
            conversions_float_to_data(dat.curr_quat.w, &response_data[0], true);
            conversions_float_to_data(dat.curr_quat.x, &response_data[4], true);
            conversions_float_to_data(dat.curr_quat.y, &response_data[8], true);
            conversions_float_to_data(dat.curr_quat.z, &response_data[12], true);
            conversions_float_to_data(dat.accum_pitch, &response_data[16], true);
            conversions_float_to_data(dat.accum_roll, &response_data[20], true);
            conversions_float_to_data(dat.accum_yaw, &response_data[24], true);

            // Send ack with response data
            cmdctrl_acknowledge(msg_id, ACK_ERR_NONE, response_data, 28);
        }
    }else if(MSG_STARTS_WITH(((uint8_t[]){'B', 'N', 'O', '0', '5', '5', 'P'}))){
        // BNO055 periodic read configure
        // B, N, O, 0, 5, 5, P, [enable]
        // [enable] is 1 or 0 (8-bit int) 1 = true (periodic read enabled). 0 = false (not enabled)
        
        if(len != 8){
            cmdctrl_acknowledge(msg_id, ACK_ERR_INVALID_ARGS, NULL, 0);
        }else{
            periodic_bno055 = msg[7];
            cmdctrl_acknowledge(msg_id, ACK_ERR_NONE, NULL, 0);
        }
    }else if(MSG_EQUALS(((uint8_t[]){'M', 'S', '5', '8', '3', '7', 'R'}))){
        // One-shot read of BNO055 data (all data)
        // M, S, 5, 8, 3, 7, R
        // Response contains [depth_m]
        // where each value is a 32-bit float little endian

        if(!ms5837_ready()){
            // Sensor not ready. This command is not valid right now.
            cmdctrl_acknowledge(msg_id, ACK_ERR_INVALID_CMD, NULL, 0);
        }else{
            // Store current readings
            xSemaphoreTake(sensor_data_mutex, portMAX_DELAY);
            float m_depth_m = curr_ms5837_data.depth_m;
            xSemaphoreGive(sensor_data_mutex);

            // Construct response data
            uint8_t response_data[4];
            conversions_float_to_data(m_depth_m, &response_data[0], true);

            // Send ack with response data
            cmdctrl_acknowledge(msg_id, ACK_ERR_NONE, response_data, 4);
        }
    }else if(MSG_STARTS_WITH(((uint8_t[]){'M', 'S', '5', '8', '3', '7', 'P'}))){
        // MS5837 periodic read configure
        // M, S, 5, 8, 3, 7, P, [enable]
        // [enable] is 1 or 0 (8-bit int) 1 = true (periodic read enabled). 0 = false (not enabled)
        
        if(len != 8){
            cmdctrl_acknowledge(msg_id, ACK_ERR_INVALID_ARGS, NULL, 0);
        }else{
            periodic_ms5837 = msg[7];
            cmdctrl_acknowledge(msg_id, ACK_ERR_NONE, NULL, 0);
        }
    }else if(MSG_STARTS_WITH(((uint8_t[]){'S', 'A', 'S', 'S', 'I', 'S', 'T', 'T', 'N'}))){
        // Tune SASSIST PID
        // S, A, S, S, I, S, T, T, N, [which], [kp], [ki], [kd], [limit], [invert]
        // Each value is a 32-bit little endian float
        // which = what PID to tune X (xrot), Y (yrot), Z (zrot), D (depth)
        // kp, ki, kd are gains. limit is max output of PID (magnitude, must be positive)
        // invert == 1 negates the default PID output
        if(len != 27){
            cmdctrl_acknowledge(msg_id, ACK_ERR_INVALID_ARGS, NULL, 0);
        }else{
            float kp = conversions_data_to_float(&msg[10], true);
            float ki = conversions_data_to_float(&msg[14], true);
            float kd = conversions_data_to_float(&msg[18], true);
            float limit = conversions_data_to_float(&msg[22], true);
            bool invert = msg[26];
            switch(msg[9]){
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
    }else if(MSG_STARTS_WITH(((uint8_t[]){'S', 'A', 'S', 'S', 'I', 'S', 'T', '1'}))){
        // STABILITY ASSIST speed set variant 1
        // S, A, S, S, I, S, T, 1, [x], [y], [yaw_spd], [target_pitch], [target_roll], [target_depth]
        // [x], [y], [yaw_spd], [target_pitch], [target_roll], [target_depth] are 32-bit floats (little endian)
        if(len != 32){
            // Message is incorrect size
            cmdctrl_acknowledge(msg_id, ACK_ERR_INVALID_ARGS, NULL, 0);
        }else{
            // Message is correct size. Handle it.

            xSemaphoreTake(sensor_data_mutex, portMAX_DELAY);
            quaternion_t m_quat = curr_bno055_data.curr_quat;
            float m_depth = curr_ms5837_data.depth_m;
            xSemaphoreGive(sensor_data_mutex);

            if(!bno055_ready() || (m_quat.w == 0 && m_quat.x == 0 && m_quat.y == 0 && m_quat.z == 0) || !ms5837_ready()){
                // Need BNO055 IMU data to use sassist mode.
                // Also need MS5837 data to use sassist mode
                // If not ready, then this command is invalid at this time
                cmdctrl_acknowledge(msg_id, ACK_ERR_INVALID_CMD, NULL, 0);
            }else{
                // Get arguments from message
                sassist_valid = true;
                sassist_variant = 1;
                sassist_x = conversions_data_to_float(&msg[8], true);
                sassist_y = conversions_data_to_float(&msg[12], true);
                sassist_yaw_spd = conversions_data_to_float(&msg[16], true);
                sassist_target_euler.pitch = conversions_data_to_float(&msg[20], true);
                sassist_target_euler.roll = conversions_data_to_float(&msg[24], true);
                sassist_depth_target = conversions_data_to_float(&msg[28], true);

                // Ensure speeds are in valid range
                LIMIT(sassist_x);
                LIMIT(sassist_y);
                LIMIT(sassist_yaw_spd);

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
                mc_set_sassist(sassist_x, 
                    sassist_y, 
                    sassist_yaw_spd, 
                    sassist_target_euler, 
                    sassist_depth_target, 
                    m_quat,
                    m_depth,
                    false);

                // Acknowledge message w/ no error.
                cmdctrl_acknowledge(msg_id, ACK_ERR_NONE, NULL, 0);
            }
        }
    }else if(MSG_STARTS_WITH(((uint8_t[]){'S', 'A', 'S', 'S', 'I', 'S', 'T', '2'}))){
        // STABILITY ASSIST speed set variant 2
        // S, A, S, S, I, S, T, 2, [x], [y], [target_pitch], [target_roll], [target_yaw], [target_depth]
        // [x], [y], [target_pitch], [target_roll], [target_yaw], [target_depth] are 32-bit floats (little endian)
        if(len != 32){
            // Message is incorrect size
            cmdctrl_acknowledge(msg_id, ACK_ERR_INVALID_ARGS, NULL, 0);
        }else{
            // Message is correct size. Handle it.

            xSemaphoreTake(sensor_data_mutex, portMAX_DELAY);
            quaternion_t m_quat = curr_bno055_data.curr_quat;
            float m_depth = curr_ms5837_data.depth_m;
            xSemaphoreGive(sensor_data_mutex);

            if(!bno055_ready() || (m_quat.w == 0 && m_quat.x == 0 && m_quat.y == 0 && m_quat.z == 0) || !ms5837_ready()){
                // Need BNO055 IMU data to use sassist mode.
                // Also need MS5837 data to use sassist mode
                // If not ready, then this command is invalid at this time
                cmdctrl_acknowledge(msg_id, ACK_ERR_INVALID_CMD, NULL, 0);
            }else{
                // Get arguments from message
                sassist_valid = true;
                sassist_variant = 2;
                sassist_x = conversions_data_to_float(&msg[8], true);
                sassist_y = conversions_data_to_float(&msg[12], true);
                sassist_target_euler.pitch = conversions_data_to_float(&msg[16], true);
                sassist_target_euler.roll = conversions_data_to_float(&msg[20], true);
                sassist_target_euler.yaw = conversions_data_to_float(&msg[24], true);
                sassist_depth_target = conversions_data_to_float(&msg[28], true);

                // Ensure speeds are in valid range
                LIMIT(sassist_x);
                LIMIT(sassist_y);
                LIMIT(sassist_yaw_spd);

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
                mc_set_sassist(sassist_x, 
                    sassist_y, 
                    0.0f,
                    sassist_target_euler, 
                    sassist_depth_target, 
                    m_quat,
                    m_depth,
                    true);

                // Acknowledge message w/ no error.
                cmdctrl_acknowledge(msg_id, ACK_ERR_NONE, NULL, 0);
            }
        }
    }else if(MSG_EQUALS(((uint8_t[]){'R', 'E', 'S', 'E', 'T', 0x0D, 0x1E}))){
        wdt_reset_now();
    }else if(MSG_STARTS_WITH(((uint8_t[]){'D', 'H', 'O', 'L', 'D'}))){
        // DEPTH_HOLD speed set
        // D, H, O, L, D, [x], [y], [pitch_spd], [roll_spd], [yaw_spd], [target_depth]
        // [x], [y], [pitch_spd], [roll_spd], [yaw_spd], [target_depth] are 32-bit floats (little endian)
        if(len != 29){
            // Message is incorrect size
            cmdctrl_acknowledge(msg_id, ACK_ERR_INVALID_ARGS, NULL, 0);
        }else{
            // Message is correct size. Handle it.

            xSemaphoreTake(sensor_data_mutex, portMAX_DELAY);
            quaternion_t m_quat = curr_bno055_data.curr_quat;
            float m_depth = curr_ms5837_data.depth_m;
            xSemaphoreGive(sensor_data_mutex);

            if(!bno055_ready() || (m_quat.w == 0 && m_quat.x == 0 && m_quat.y == 0 && m_quat.z == 0) || !ms5837_ready()){
                // Need BNO055 IMU data to use depth hold mode.
                // Also need MS5837 data to use depth hold mode
                // If not ready, then this command is invalid at this time
                cmdctrl_acknowledge(msg_id, ACK_ERR_INVALID_CMD, NULL, 0);
            }else{
                // Get speeds from message
                dhold_x = conversions_data_to_float(&msg[5], true);
                dhold_y = conversions_data_to_float(&msg[9], true);
                dhold_pitch_spd = conversions_data_to_float(&msg[13], true);
                dhold_roll_spd = conversions_data_to_float(&msg[17], true);
                dhold_yaw_spd = conversions_data_to_float(&msg[21], true);
                dhold_depth = conversions_data_to_float(&msg[25], true);

                // Ensure speeds are in valid range
                LIMIT(dhold_x);
                LIMIT(dhold_y);
                LIMIT(dhold_pitch_spd);
                LIMIT(dhold_roll_spd);
                LIMIT(dhold_yaw_spd);
                
                // Reset time until periodic speed set
                xTimerReset(periodic_speed_timer, portMAX_DELAY);

                // Update mode variable and LED color (if needed)
                if(mode != MODE_DHOLD){
                    mode = MODE_DHOLD;
                    led_set(COLOR_DHOLD);
                }

                // Feed watchdog when speeds are set
                // Important to call before speed set function in case currently killed
                mc_wdog_feed();

                // Update motor speeds
                mc_set_dhold(dhold_x, dhold_y, dhold_pitch_spd, dhold_roll_spd, dhold_yaw_spd, dhold_depth, m_quat, m_depth);

                // Acknowledge message w/ no error.
                cmdctrl_acknowledge(msg_id, ACK_ERR_NONE, NULL, 0);
            }
        }
    }else if(MSG_EQUALS(((uint8_t[]){'R', 'S', 'T', 'W', 'H', 'Y'}))){
        // R, S, T, W, H, Y
        // ACK contains a 32-bit integer (signed, little endian)
        // indicating one of the HALT_EC codes in debug.h
        uint8_t response[4];
        conversions_int32_to_data(reset_cause, response, true);
        cmdctrl_acknowledge(msg_id, ACK_ERR_NONE, response, 4);
    }else if(MSG_STARTS_WITH(((uint8_t[]){'S', 'I', 'M', 'H', 'I', 'J', 'A', 'C', 'K'}))){
        // S, I, M, H, I, J, A, C, K, [hijack]
        // [hijack] is an 8-bit int (unsigned) 0 = release, 1 = hijack
        if(len != 10){
            // Message is incorrect size
            cmdctrl_acknowledge(msg_id, ACK_ERR_INVALID_ARGS, NULL, 0);
        }else{
            cmdctrl_simhijack(msg[9]);
            cmdctrl_acknowledge(msg_id, ACK_ERR_NONE, NULL, 0);
        }
    }else if(MSG_STARTS_WITH(((uint8_t[]){'S', 'I', 'M', 'D', 'A', 'T'}))){
        // S, I, M, D, A, T, [w], [x], [y], [z], [depth]
        // All values are little endian floats (32-bit)
        // x, y, z, w are current quaternion (BNO055 data)
        // depth is current depth (MS5837 data)
        if(len != 26){
            // Message is incorrect size
            cmdctrl_acknowledge(msg_id, ACK_ERR_INVALID_ARGS, NULL, 0);
        }else{
            // Parse received data
            sim_quat.w = conversions_data_to_float(&msg[6], true);
            sim_quat.x = conversions_data_to_float(&msg[10], true);
            sim_quat.y = conversions_data_to_float(&msg[14], true);
            sim_quat.z = conversions_data_to_float(&msg[18], true);
            sim_depth = conversions_data_to_float(&msg[22], true);

            // Message handled successfully
            cmdctrl_acknowledge(msg_id, ACK_ERR_NONE, NULL, 0);
        }
    }else if(MSG_EQUALS(((uint8_t[]){'S', 'E', 'N', 'C', 'A', 'L', 'R'}))){
        // S, E, N, C, A, L, 'R'
        // Sensor calibration status query
        // ACK contains the following data
        // [valid], [accel_offset_x], [accel_offset_y], [accel_offset_z], [accel_radius], [gyro_offset_x], [gyro_offset_y], [gyro_offset_z]
        // Valid is a single byte. 0 = invalid, 1 = valid data
        // All other values are 16-bit little endian integers. If data is valid these will be calibration constants. If not, these will be zero
        // TODO
    }else if(MSG_EQUALS(((uint8_t[]){'S', 'E', 'N', 'C', 'A', 'L', 'E'}))){
        // S, E, N, C, A, L, E
        // Sensor calibration erase command
        calibration_erase();
    }else if(MSG_STARTS_WITH(((uint8_t[]){'S', 'E', 'N', 'C', 'A', 'L', 'S'}))){
        // S, E, N, C, A, L, S, [accel_offset_x], [accel_offset_y], [accel_offset_z], [accel_radius], [gyro_offset_x], [gyro_offset_y], [gyro_offset_z]
        // Sensor calibration set command
        // All values are 16-bit little endian integers (calibration values)
        // TODO: Store calibration
    }else if(MSG_EQUALS(((uint8_t[]){'B', 'N', 'O', '0', '5', '5', 'C'}))){
        // Get current BNO055 calibration status
        // Note that this will only be "generated" if the stored calibration (calibration.h/c) is erased (using SENCAL) commands
        // ACK will contain the following
        // [status], [accel_offset_x], [accel_offset_y], [accel_offset_z], [accel_radius], [gyro_offset_x], [gyro_offset_y], [gyro_offset_z]
        // status is an 8-bit integer (CALIB_STAT register value). Others are little endian 16-bit integers
        
        if(!bno055_ready()){
            // Cannot read status if sensor not ready
            cmdctrl_acknowledge(msg_id, ACK_ERR_INVALID_CMD, NULL, 0);
        }else{
            uint8_t status;
            if(!bno055_read_calibration(&status)){
                // If this fails, sensor is probably not connected anymore
                cmdctrl_acknowledge(msg_id, ACK_ERR_INVALID_CMD, NULL, 0);
            }else{
                uint8_t buf[15];
                buf[0] = status;
                // TODO: Other values later
                cmdctrl_acknowledge(msg_id, ACK_ERR_NONE, buf, 15);
            }
        }
        
    }else{
        // This is an unrecognized message
        cmdctrl_acknowledge(msg_id, ACK_ERR_UNKNOWN_MSG, NULL, 0);
    }
}

void cmdctrl_mwdog_change(bool me){
    motors_enabled = me;
    pccomm_write((uint8_t[]){'W', 'D', 'G', 'S', me}, 5);
}

void cmdctrl_bno055_status(bool status){
    _bno055_ready = status;
}

void cmdctrl_bno055_data(bno055_data data){
    xSemaphoreTake(sensor_data_mutex, portMAX_DELAY);
    curr_bno055_data = data;
    xSemaphoreGive(sensor_data_mutex);
}

void cmdctrl_ms5837_status(bool status){
    _ms5837_ready = status;
}

void cmdctrl_ms5837_data(ms5837_data data){
    xSemaphoreTake(sensor_data_mutex, portMAX_DELAY);
    curr_ms5837_data = data;
    xSemaphoreGive(sensor_data_mutex);
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
    conversions_float_to_data(sim_speeds[0], &simstat[7], true);
    conversions_float_to_data(sim_speeds[1], &simstat[11], true);
    conversions_float_to_data(sim_speeds[2], &simstat[15], true);
    conversions_float_to_data(sim_speeds[3], &simstat[19], true);
    conversions_float_to_data(sim_speeds[4], &simstat[23], true);
    conversions_float_to_data(sim_speeds[5], &simstat[27], true);
    conversions_float_to_data(sim_speeds[6], &simstat[31], true);
    conversions_float_to_data(sim_speeds[7], &simstat[35], true);
    simstat[39] = mode & 0xFF;
    simstat[40] = motors_enabled ? 0 : 1;
    pccomm_write(simstat, 41);
}

void cmdctrl_simhijack(bool hijack){
    if(hijack){
        // Reset data received from simulator
        sim_quat.w = 0.0f;
        sim_quat.x = 0.0f;
        sim_quat.y = 0.0f;
        sim_quat.z = 0.0f;
        sim_depth = 0.0f;

        // Reset data output to simulator
        sim_speeds[0] = 0;
        sim_speeds[1] = 0;
        sim_speeds[2] = 0;
        sim_speeds[3] = 0;
        sim_speeds[4] = 0;
        sim_speeds[5] = 0;
        sim_speeds[6] = 0;
        sim_speeds[7] = 0;

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

        sim_hijacked = true;    // Do this last so set_local (above) uses real thrusters

        // Reset accumulated angles (after sim_hijacked change!)
        bno055_reset_accum_euler();

    }else{
        sim_hijacked = false;   // Do this first so set_local (below) uses real thrusters

        // Reset accumulated angles (after sim_hijacked change!)
        bno055_reset_accum_euler();

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

