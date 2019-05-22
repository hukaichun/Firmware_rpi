#pragma once
// MESSAGE CONTROL_INFO PACKING

#define MAVLINK_MSG_ID_CONTROL_INFO 1

MAVPACKED(
typedef struct __mavlink_control_info_t {
 uint64_t timestamp; /*<  timestamp*/
 float quaternion[4]; /*<  quaternion*/
 float position[3]; /*<  position*/
 float angular_velocity[3]; /*<  angular velocity*/
 float velocity[3]; /*<  velocity*/
 float control_h[3]; /*<  high level control*/
 float control_l[4]; /*<  low level control*/
 float local_position[3]; /*<  local position*/
 float position_sp[3]; /*<  position set point*/
 float voltage[1]; /*<  voltage*/
}) mavlink_control_info_t;

#define MAVLINK_MSG_ID_CONTROL_INFO_LEN 116
#define MAVLINK_MSG_ID_CONTROL_INFO_MIN_LEN 116
#define MAVLINK_MSG_ID_1_LEN 116
#define MAVLINK_MSG_ID_1_MIN_LEN 116

#define MAVLINK_MSG_ID_CONTROL_INFO_CRC 35
#define MAVLINK_MSG_ID_1_CRC 35

#define MAVLINK_MSG_CONTROL_INFO_FIELD_QUATERNION_LEN 4
#define MAVLINK_MSG_CONTROL_INFO_FIELD_POSITION_LEN 3
#define MAVLINK_MSG_CONTROL_INFO_FIELD_ANGULAR_VELOCITY_LEN 3
#define MAVLINK_MSG_CONTROL_INFO_FIELD_VELOCITY_LEN 3
#define MAVLINK_MSG_CONTROL_INFO_FIELD_CONTROL_H_LEN 3
#define MAVLINK_MSG_CONTROL_INFO_FIELD_CONTROL_L_LEN 4
#define MAVLINK_MSG_CONTROL_INFO_FIELD_LOCAL_POSITION_LEN 3
#define MAVLINK_MSG_CONTROL_INFO_FIELD_POSITION_SP_LEN 3
#define MAVLINK_MSG_CONTROL_INFO_FIELD_VOLTAGE_LEN 1

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_CONTROL_INFO { \
    1, \
    "CONTROL_INFO", \
    10, \
    {  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_control_info_t, timestamp) }, \
         { "quaternion", NULL, MAVLINK_TYPE_FLOAT, 4, 8, offsetof(mavlink_control_info_t, quaternion) }, \
         { "position", NULL, MAVLINK_TYPE_FLOAT, 3, 24, offsetof(mavlink_control_info_t, position) }, \
         { "angular_velocity", NULL, MAVLINK_TYPE_FLOAT, 3, 36, offsetof(mavlink_control_info_t, angular_velocity) }, \
         { "velocity", NULL, MAVLINK_TYPE_FLOAT, 3, 48, offsetof(mavlink_control_info_t, velocity) }, \
         { "control_h", NULL, MAVLINK_TYPE_FLOAT, 3, 60, offsetof(mavlink_control_info_t, control_h) }, \
         { "control_l", NULL, MAVLINK_TYPE_FLOAT, 4, 72, offsetof(mavlink_control_info_t, control_l) }, \
         { "local_position", NULL, MAVLINK_TYPE_FLOAT, 3, 88, offsetof(mavlink_control_info_t, local_position) }, \
         { "position_sp", NULL, MAVLINK_TYPE_FLOAT, 3, 100, offsetof(mavlink_control_info_t, position_sp) }, \
         { "voltage", NULL, MAVLINK_TYPE_FLOAT, 1, 112, offsetof(mavlink_control_info_t, voltage) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_CONTROL_INFO { \
    "CONTROL_INFO", \
    10, \
    {  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_control_info_t, timestamp) }, \
         { "quaternion", NULL, MAVLINK_TYPE_FLOAT, 4, 8, offsetof(mavlink_control_info_t, quaternion) }, \
         { "position", NULL, MAVLINK_TYPE_FLOAT, 3, 24, offsetof(mavlink_control_info_t, position) }, \
         { "angular_velocity", NULL, MAVLINK_TYPE_FLOAT, 3, 36, offsetof(mavlink_control_info_t, angular_velocity) }, \
         { "velocity", NULL, MAVLINK_TYPE_FLOAT, 3, 48, offsetof(mavlink_control_info_t, velocity) }, \
         { "control_h", NULL, MAVLINK_TYPE_FLOAT, 3, 60, offsetof(mavlink_control_info_t, control_h) }, \
         { "control_l", NULL, MAVLINK_TYPE_FLOAT, 4, 72, offsetof(mavlink_control_info_t, control_l) }, \
         { "local_position", NULL, MAVLINK_TYPE_FLOAT, 3, 88, offsetof(mavlink_control_info_t, local_position) }, \
         { "position_sp", NULL, MAVLINK_TYPE_FLOAT, 3, 100, offsetof(mavlink_control_info_t, position_sp) }, \
         { "voltage", NULL, MAVLINK_TYPE_FLOAT, 1, 112, offsetof(mavlink_control_info_t, voltage) }, \
         } \
}
#endif

/**
 * @brief Pack a control_info message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param timestamp  timestamp
 * @param quaternion  quaternion
 * @param position  position
 * @param angular_velocity  angular velocity
 * @param velocity  velocity
 * @param control_h  high level control
 * @param control_l  low level control
 * @param local_position  local position
 * @param position_sp  position set point
 * @param voltage  voltage
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_control_info_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint64_t timestamp, const float *quaternion, const float *position, const float *angular_velocity, const float *velocity, const float *control_h, const float *control_l, const float *local_position, const float *position_sp, const float *voltage)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_CONTROL_INFO_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float_array(buf, 8, quaternion, 4);
    _mav_put_float_array(buf, 24, position, 3);
    _mav_put_float_array(buf, 36, angular_velocity, 3);
    _mav_put_float_array(buf, 48, velocity, 3);
    _mav_put_float_array(buf, 60, control_h, 3);
    _mav_put_float_array(buf, 72, control_l, 4);
    _mav_put_float_array(buf, 88, local_position, 3);
    _mav_put_float_array(buf, 100, position_sp, 3);
    _mav_put_float_array(buf, 112, voltage, 1);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_CONTROL_INFO_LEN);
#else
    mavlink_control_info_t packet;
    packet.timestamp = timestamp;
    mav_array_memcpy(packet.quaternion, quaternion, sizeof(float)*4);
    mav_array_memcpy(packet.position, position, sizeof(float)*3);
    mav_array_memcpy(packet.angular_velocity, angular_velocity, sizeof(float)*3);
    mav_array_memcpy(packet.velocity, velocity, sizeof(float)*3);
    mav_array_memcpy(packet.control_h, control_h, sizeof(float)*3);
    mav_array_memcpy(packet.control_l, control_l, sizeof(float)*4);
    mav_array_memcpy(packet.local_position, local_position, sizeof(float)*3);
    mav_array_memcpy(packet.position_sp, position_sp, sizeof(float)*3);
    mav_array_memcpy(packet.voltage, voltage, sizeof(float)*1);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_CONTROL_INFO_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_CONTROL_INFO;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_CONTROL_INFO_MIN_LEN, MAVLINK_MSG_ID_CONTROL_INFO_LEN, MAVLINK_MSG_ID_CONTROL_INFO_CRC);
}

/**
 * @brief Pack a control_info message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param timestamp  timestamp
 * @param quaternion  quaternion
 * @param position  position
 * @param angular_velocity  angular velocity
 * @param velocity  velocity
 * @param control_h  high level control
 * @param control_l  low level control
 * @param local_position  local position
 * @param position_sp  position set point
 * @param voltage  voltage
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_control_info_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint64_t timestamp,const float *quaternion,const float *position,const float *angular_velocity,const float *velocity,const float *control_h,const float *control_l,const float *local_position,const float *position_sp,const float *voltage)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_CONTROL_INFO_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float_array(buf, 8, quaternion, 4);
    _mav_put_float_array(buf, 24, position, 3);
    _mav_put_float_array(buf, 36, angular_velocity, 3);
    _mav_put_float_array(buf, 48, velocity, 3);
    _mav_put_float_array(buf, 60, control_h, 3);
    _mav_put_float_array(buf, 72, control_l, 4);
    _mav_put_float_array(buf, 88, local_position, 3);
    _mav_put_float_array(buf, 100, position_sp, 3);
    _mav_put_float_array(buf, 112, voltage, 1);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_CONTROL_INFO_LEN);
#else
    mavlink_control_info_t packet;
    packet.timestamp = timestamp;
    mav_array_memcpy(packet.quaternion, quaternion, sizeof(float)*4);
    mav_array_memcpy(packet.position, position, sizeof(float)*3);
    mav_array_memcpy(packet.angular_velocity, angular_velocity, sizeof(float)*3);
    mav_array_memcpy(packet.velocity, velocity, sizeof(float)*3);
    mav_array_memcpy(packet.control_h, control_h, sizeof(float)*3);
    mav_array_memcpy(packet.control_l, control_l, sizeof(float)*4);
    mav_array_memcpy(packet.local_position, local_position, sizeof(float)*3);
    mav_array_memcpy(packet.position_sp, position_sp, sizeof(float)*3);
    mav_array_memcpy(packet.voltage, voltage, sizeof(float)*1);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_CONTROL_INFO_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_CONTROL_INFO;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_CONTROL_INFO_MIN_LEN, MAVLINK_MSG_ID_CONTROL_INFO_LEN, MAVLINK_MSG_ID_CONTROL_INFO_CRC);
}

/**
 * @brief Encode a control_info struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param control_info C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_control_info_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_control_info_t* control_info)
{
    return mavlink_msg_control_info_pack(system_id, component_id, msg, control_info->timestamp, control_info->quaternion, control_info->position, control_info->angular_velocity, control_info->velocity, control_info->control_h, control_info->control_l, control_info->local_position, control_info->position_sp, control_info->voltage);
}

/**
 * @brief Encode a control_info struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param control_info C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_control_info_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_control_info_t* control_info)
{
    return mavlink_msg_control_info_pack_chan(system_id, component_id, chan, msg, control_info->timestamp, control_info->quaternion, control_info->position, control_info->angular_velocity, control_info->velocity, control_info->control_h, control_info->control_l, control_info->local_position, control_info->position_sp, control_info->voltage);
}

/**
 * @brief Send a control_info message
 * @param chan MAVLink channel to send the message
 *
 * @param timestamp  timestamp
 * @param quaternion  quaternion
 * @param position  position
 * @param angular_velocity  angular velocity
 * @param velocity  velocity
 * @param control_h  high level control
 * @param control_l  low level control
 * @param local_position  local position
 * @param position_sp  position set point
 * @param voltage  voltage
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_control_info_send(mavlink_channel_t chan, uint64_t timestamp, const float *quaternion, const float *position, const float *angular_velocity, const float *velocity, const float *control_h, const float *control_l, const float *local_position, const float *position_sp, const float *voltage)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_CONTROL_INFO_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float_array(buf, 8, quaternion, 4);
    _mav_put_float_array(buf, 24, position, 3);
    _mav_put_float_array(buf, 36, angular_velocity, 3);
    _mav_put_float_array(buf, 48, velocity, 3);
    _mav_put_float_array(buf, 60, control_h, 3);
    _mav_put_float_array(buf, 72, control_l, 4);
    _mav_put_float_array(buf, 88, local_position, 3);
    _mav_put_float_array(buf, 100, position_sp, 3);
    _mav_put_float_array(buf, 112, voltage, 1);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CONTROL_INFO, buf, MAVLINK_MSG_ID_CONTROL_INFO_MIN_LEN, MAVLINK_MSG_ID_CONTROL_INFO_LEN, MAVLINK_MSG_ID_CONTROL_INFO_CRC);
#else
    mavlink_control_info_t packet;
    packet.timestamp = timestamp;
    mav_array_memcpy(packet.quaternion, quaternion, sizeof(float)*4);
    mav_array_memcpy(packet.position, position, sizeof(float)*3);
    mav_array_memcpy(packet.angular_velocity, angular_velocity, sizeof(float)*3);
    mav_array_memcpy(packet.velocity, velocity, sizeof(float)*3);
    mav_array_memcpy(packet.control_h, control_h, sizeof(float)*3);
    mav_array_memcpy(packet.control_l, control_l, sizeof(float)*4);
    mav_array_memcpy(packet.local_position, local_position, sizeof(float)*3);
    mav_array_memcpy(packet.position_sp, position_sp, sizeof(float)*3);
    mav_array_memcpy(packet.voltage, voltage, sizeof(float)*1);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CONTROL_INFO, (const char *)&packet, MAVLINK_MSG_ID_CONTROL_INFO_MIN_LEN, MAVLINK_MSG_ID_CONTROL_INFO_LEN, MAVLINK_MSG_ID_CONTROL_INFO_CRC);
#endif
}

/**
 * @brief Send a control_info message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_control_info_send_struct(mavlink_channel_t chan, const mavlink_control_info_t* control_info)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_control_info_send(chan, control_info->timestamp, control_info->quaternion, control_info->position, control_info->angular_velocity, control_info->velocity, control_info->control_h, control_info->control_l, control_info->local_position, control_info->position_sp, control_info->voltage);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CONTROL_INFO, (const char *)control_info, MAVLINK_MSG_ID_CONTROL_INFO_MIN_LEN, MAVLINK_MSG_ID_CONTROL_INFO_LEN, MAVLINK_MSG_ID_CONTROL_INFO_CRC);
#endif
}

#if MAVLINK_MSG_ID_CONTROL_INFO_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_control_info_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t timestamp, const float *quaternion, const float *position, const float *angular_velocity, const float *velocity, const float *control_h, const float *control_l, const float *local_position, const float *position_sp, const float *voltage)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float_array(buf, 8, quaternion, 4);
    _mav_put_float_array(buf, 24, position, 3);
    _mav_put_float_array(buf, 36, angular_velocity, 3);
    _mav_put_float_array(buf, 48, velocity, 3);
    _mav_put_float_array(buf, 60, control_h, 3);
    _mav_put_float_array(buf, 72, control_l, 4);
    _mav_put_float_array(buf, 88, local_position, 3);
    _mav_put_float_array(buf, 100, position_sp, 3);
    _mav_put_float_array(buf, 112, voltage, 1);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CONTROL_INFO, buf, MAVLINK_MSG_ID_CONTROL_INFO_MIN_LEN, MAVLINK_MSG_ID_CONTROL_INFO_LEN, MAVLINK_MSG_ID_CONTROL_INFO_CRC);
#else
    mavlink_control_info_t *packet = (mavlink_control_info_t *)msgbuf;
    packet->timestamp = timestamp;
    mav_array_memcpy(packet->quaternion, quaternion, sizeof(float)*4);
    mav_array_memcpy(packet->position, position, sizeof(float)*3);
    mav_array_memcpy(packet->angular_velocity, angular_velocity, sizeof(float)*3);
    mav_array_memcpy(packet->velocity, velocity, sizeof(float)*3);
    mav_array_memcpy(packet->control_h, control_h, sizeof(float)*3);
    mav_array_memcpy(packet->control_l, control_l, sizeof(float)*4);
    mav_array_memcpy(packet->local_position, local_position, sizeof(float)*3);
    mav_array_memcpy(packet->position_sp, position_sp, sizeof(float)*3);
    mav_array_memcpy(packet->voltage, voltage, sizeof(float)*1);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CONTROL_INFO, (const char *)packet, MAVLINK_MSG_ID_CONTROL_INFO_MIN_LEN, MAVLINK_MSG_ID_CONTROL_INFO_LEN, MAVLINK_MSG_ID_CONTROL_INFO_CRC);
#endif
}
#endif

#endif

// MESSAGE CONTROL_INFO UNPACKING


/**
 * @brief Get field timestamp from control_info message
 *
 * @return  timestamp
 */
static inline uint64_t mavlink_msg_control_info_get_timestamp(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field quaternion from control_info message
 *
 * @return  quaternion
 */
static inline uint16_t mavlink_msg_control_info_get_quaternion(const mavlink_message_t* msg, float *quaternion)
{
    return _MAV_RETURN_float_array(msg, quaternion, 4,  8);
}

/**
 * @brief Get field position from control_info message
 *
 * @return  position
 */
static inline uint16_t mavlink_msg_control_info_get_position(const mavlink_message_t* msg, float *position)
{
    return _MAV_RETURN_float_array(msg, position, 3,  24);
}

/**
 * @brief Get field angular_velocity from control_info message
 *
 * @return  angular velocity
 */
static inline uint16_t mavlink_msg_control_info_get_angular_velocity(const mavlink_message_t* msg, float *angular_velocity)
{
    return _MAV_RETURN_float_array(msg, angular_velocity, 3,  36);
}

/**
 * @brief Get field velocity from control_info message
 *
 * @return  velocity
 */
static inline uint16_t mavlink_msg_control_info_get_velocity(const mavlink_message_t* msg, float *velocity)
{
    return _MAV_RETURN_float_array(msg, velocity, 3,  48);
}

/**
 * @brief Get field control_h from control_info message
 *
 * @return  high level control
 */
static inline uint16_t mavlink_msg_control_info_get_control_h(const mavlink_message_t* msg, float *control_h)
{
    return _MAV_RETURN_float_array(msg, control_h, 3,  60);
}

/**
 * @brief Get field control_l from control_info message
 *
 * @return  low level control
 */
static inline uint16_t mavlink_msg_control_info_get_control_l(const mavlink_message_t* msg, float *control_l)
{
    return _MAV_RETURN_float_array(msg, control_l, 4,  72);
}

/**
 * @brief Get field local_position from control_info message
 *
 * @return  local position
 */
static inline uint16_t mavlink_msg_control_info_get_local_position(const mavlink_message_t* msg, float *local_position)
{
    return _MAV_RETURN_float_array(msg, local_position, 3,  88);
}

/**
 * @brief Get field position_sp from control_info message
 *
 * @return  position set point
 */
static inline uint16_t mavlink_msg_control_info_get_position_sp(const mavlink_message_t* msg, float *position_sp)
{
    return _MAV_RETURN_float_array(msg, position_sp, 3,  100);
}

/**
 * @brief Get field voltage from control_info message
 *
 * @return  voltage
 */
static inline uint16_t mavlink_msg_control_info_get_voltage(const mavlink_message_t* msg, float *voltage)
{
    return _MAV_RETURN_float_array(msg, voltage, 1,  112);
}

/**
 * @brief Decode a control_info message into a struct
 *
 * @param msg The message to decode
 * @param control_info C-struct to decode the message contents into
 */
static inline void mavlink_msg_control_info_decode(const mavlink_message_t* msg, mavlink_control_info_t* control_info)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    control_info->timestamp = mavlink_msg_control_info_get_timestamp(msg);
    mavlink_msg_control_info_get_quaternion(msg, control_info->quaternion);
    mavlink_msg_control_info_get_position(msg, control_info->position);
    mavlink_msg_control_info_get_angular_velocity(msg, control_info->angular_velocity);
    mavlink_msg_control_info_get_velocity(msg, control_info->velocity);
    mavlink_msg_control_info_get_control_h(msg, control_info->control_h);
    mavlink_msg_control_info_get_control_l(msg, control_info->control_l);
    mavlink_msg_control_info_get_local_position(msg, control_info->local_position);
    mavlink_msg_control_info_get_position_sp(msg, control_info->position_sp);
    mavlink_msg_control_info_get_voltage(msg, control_info->voltage);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_CONTROL_INFO_LEN? msg->len : MAVLINK_MSG_ID_CONTROL_INFO_LEN;
        memset(control_info, 0, MAVLINK_MSG_ID_CONTROL_INFO_LEN);
    memcpy(control_info, _MAV_PAYLOAD(msg), len);
#endif
}
