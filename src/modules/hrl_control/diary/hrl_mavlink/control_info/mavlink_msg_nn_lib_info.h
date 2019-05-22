#pragma once
// MESSAGE NN_LIB_INFO PACKING

#define MAVLINK_MSG_ID_NN_LIB_INFO 0

MAVPACKED(
typedef struct __mavlink_nn_lib_info_t {
 uint64_t  D_timestamp; /*<  delta t*/
 char dir[128]; /*<  new directory of nn shared obj*/
}) mavlink_nn_lib_info_t;

#define MAVLINK_MSG_ID_NN_LIB_INFO_LEN 136
#define MAVLINK_MSG_ID_NN_LIB_INFO_MIN_LEN 136
#define MAVLINK_MSG_ID_0_LEN 136
#define MAVLINK_MSG_ID_0_MIN_LEN 136

#define MAVLINK_MSG_ID_NN_LIB_INFO_CRC 82
#define MAVLINK_MSG_ID_0_CRC 82

#define MAVLINK_MSG_NN_LIB_INFO_FIELD_DIR_LEN 128

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_NN_LIB_INFO { \
    0, \
    "NN_LIB_INFO", \
    2, \
    {  { " D_timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_nn_lib_info_t,  D_timestamp) }, \
         { "dir", NULL, MAVLINK_TYPE_CHAR, 128, 8, offsetof(mavlink_nn_lib_info_t, dir) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_NN_LIB_INFO { \
    "NN_LIB_INFO", \
    2, \
    {  { " D_timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_nn_lib_info_t,  D_timestamp) }, \
         { "dir", NULL, MAVLINK_TYPE_CHAR, 128, 8, offsetof(mavlink_nn_lib_info_t, dir) }, \
         } \
}
#endif

/**
 * @brief Pack a nn_lib_info message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param  D_timestamp  delta t
 * @param dir  new directory of nn shared obj
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_nn_lib_info_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint64_t  D_timestamp, const char *dir)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_NN_LIB_INFO_LEN];
    _mav_put_uint64_t(buf, 0,  D_timestamp);
    _mav_put_char_array(buf, 8, dir, 128);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_NN_LIB_INFO_LEN);
#else
    mavlink_nn_lib_info_t packet;
    packet. D_timestamp =  D_timestamp;
    mav_array_memcpy(packet.dir, dir, sizeof(char)*128);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_NN_LIB_INFO_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_NN_LIB_INFO;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_NN_LIB_INFO_MIN_LEN, MAVLINK_MSG_ID_NN_LIB_INFO_LEN, MAVLINK_MSG_ID_NN_LIB_INFO_CRC);
}

/**
 * @brief Pack a nn_lib_info message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param  D_timestamp  delta t
 * @param dir  new directory of nn shared obj
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_nn_lib_info_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint64_t  D_timestamp,const char *dir)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_NN_LIB_INFO_LEN];
    _mav_put_uint64_t(buf, 0,  D_timestamp);
    _mav_put_char_array(buf, 8, dir, 128);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_NN_LIB_INFO_LEN);
#else
    mavlink_nn_lib_info_t packet;
    packet. D_timestamp =  D_timestamp;
    mav_array_memcpy(packet.dir, dir, sizeof(char)*128);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_NN_LIB_INFO_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_NN_LIB_INFO;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_NN_LIB_INFO_MIN_LEN, MAVLINK_MSG_ID_NN_LIB_INFO_LEN, MAVLINK_MSG_ID_NN_LIB_INFO_CRC);
}

/**
 * @brief Encode a nn_lib_info struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param nn_lib_info C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_nn_lib_info_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_nn_lib_info_t* nn_lib_info)
{
    return mavlink_msg_nn_lib_info_pack(system_id, component_id, msg, nn_lib_info-> D_timestamp, nn_lib_info->dir);
}

/**
 * @brief Encode a nn_lib_info struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param nn_lib_info C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_nn_lib_info_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_nn_lib_info_t* nn_lib_info)
{
    return mavlink_msg_nn_lib_info_pack_chan(system_id, component_id, chan, msg, nn_lib_info-> D_timestamp, nn_lib_info->dir);
}

/**
 * @brief Send a nn_lib_info message
 * @param chan MAVLink channel to send the message
 *
 * @param  D_timestamp  delta t
 * @param dir  new directory of nn shared obj
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_nn_lib_info_send(mavlink_channel_t chan, uint64_t  D_timestamp, const char *dir)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_NN_LIB_INFO_LEN];
    _mav_put_uint64_t(buf, 0,  D_timestamp);
    _mav_put_char_array(buf, 8, dir, 128);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_NN_LIB_INFO, buf, MAVLINK_MSG_ID_NN_LIB_INFO_MIN_LEN, MAVLINK_MSG_ID_NN_LIB_INFO_LEN, MAVLINK_MSG_ID_NN_LIB_INFO_CRC);
#else
    mavlink_nn_lib_info_t packet;
    packet. D_timestamp =  D_timestamp;
    mav_array_memcpy(packet.dir, dir, sizeof(char)*128);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_NN_LIB_INFO, (const char *)&packet, MAVLINK_MSG_ID_NN_LIB_INFO_MIN_LEN, MAVLINK_MSG_ID_NN_LIB_INFO_LEN, MAVLINK_MSG_ID_NN_LIB_INFO_CRC);
#endif
}

/**
 * @brief Send a nn_lib_info message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_nn_lib_info_send_struct(mavlink_channel_t chan, const mavlink_nn_lib_info_t* nn_lib_info)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_nn_lib_info_send(chan, nn_lib_info-> D_timestamp, nn_lib_info->dir);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_NN_LIB_INFO, (const char *)nn_lib_info, MAVLINK_MSG_ID_NN_LIB_INFO_MIN_LEN, MAVLINK_MSG_ID_NN_LIB_INFO_LEN, MAVLINK_MSG_ID_NN_LIB_INFO_CRC);
#endif
}

#if MAVLINK_MSG_ID_NN_LIB_INFO_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_nn_lib_info_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t  D_timestamp, const char *dir)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0,  D_timestamp);
    _mav_put_char_array(buf, 8, dir, 128);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_NN_LIB_INFO, buf, MAVLINK_MSG_ID_NN_LIB_INFO_MIN_LEN, MAVLINK_MSG_ID_NN_LIB_INFO_LEN, MAVLINK_MSG_ID_NN_LIB_INFO_CRC);
#else
    mavlink_nn_lib_info_t *packet = (mavlink_nn_lib_info_t *)msgbuf;
    packet-> D_timestamp =  D_timestamp;
    mav_array_memcpy(packet->dir, dir, sizeof(char)*128);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_NN_LIB_INFO, (const char *)packet, MAVLINK_MSG_ID_NN_LIB_INFO_MIN_LEN, MAVLINK_MSG_ID_NN_LIB_INFO_LEN, MAVLINK_MSG_ID_NN_LIB_INFO_CRC);
#endif
}
#endif

#endif

// MESSAGE NN_LIB_INFO UNPACKING


/**
 * @brief Get field  D_timestamp from nn_lib_info message
 *
 * @return  delta t
 */
static inline uint64_t mavlink_msg_nn_lib_info_get_ D_timestamp(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field dir from nn_lib_info message
 *
 * @return  new directory of nn shared obj
 */
static inline uint16_t mavlink_msg_nn_lib_info_get_dir(const mavlink_message_t* msg, char *dir)
{
    return _MAV_RETURN_char_array(msg, dir, 128,  8);
}

/**
 * @brief Decode a nn_lib_info message into a struct
 *
 * @param msg The message to decode
 * @param nn_lib_info C-struct to decode the message contents into
 */
static inline void mavlink_msg_nn_lib_info_decode(const mavlink_message_t* msg, mavlink_nn_lib_info_t* nn_lib_info)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    nn_lib_info-> D_timestamp = mavlink_msg_nn_lib_info_get_ D_timestamp(msg);
    mavlink_msg_nn_lib_info_get_dir(msg, nn_lib_info->dir);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_NN_LIB_INFO_LEN? msg->len : MAVLINK_MSG_ID_NN_LIB_INFO_LEN;
        memset(nn_lib_info, 0, MAVLINK_MSG_ID_NN_LIB_INFO_LEN);
    memcpy(nn_lib_info, _MAV_PAYLOAD(msg), len);
#endif
}
