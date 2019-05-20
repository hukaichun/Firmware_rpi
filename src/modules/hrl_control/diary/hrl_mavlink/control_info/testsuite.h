/** @file
 *    @brief MAVLink comm protocol testsuite generated from control_info.xml
 *    @see http://qgroundcontrol.org/mavlink/
 */
#pragma once
#ifndef CONTROL_INFO_TESTSUITE_H
#define CONTROL_INFO_TESTSUITE_H

#ifdef __cplusplus
extern "C" {
#endif

#ifndef MAVLINK_TEST_ALL
#define MAVLINK_TEST_ALL

static void mavlink_test_control_info(uint8_t, uint8_t, mavlink_message_t *last_msg);

static void mavlink_test_all(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{

    mavlink_test_control_info(system_id, component_id, last_msg);
}
#endif




static void mavlink_test_control_info(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_CONTROL_INFO >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_control_info_t packet_in = {
        93372036854775807ULL,{ 73.0, 74.0, 75.0, 76.0 },{ 185.0, 186.0, 187.0 },{ 269.0, 270.0, 271.0 },{ 353.0, 354.0, 355.0 },{ 437.0, 438.0, 439.0 },{ 521.0, 522.0, 523.0, 524.0 }
    };
    mavlink_control_info_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.timestamp = packet_in.timestamp;
        
        mav_array_memcpy(packet1.quaternion, packet_in.quaternion, sizeof(float)*4);
        mav_array_memcpy(packet1.position, packet_in.position, sizeof(float)*3);
        mav_array_memcpy(packet1.angular_velocity, packet_in.angular_velocity, sizeof(float)*3);
        mav_array_memcpy(packet1.velocity, packet_in.velocity, sizeof(float)*3);
        mav_array_memcpy(packet1.control_h, packet_in.control_h, sizeof(float)*3);
        mav_array_memcpy(packet1.control_l, packet_in.control_l, sizeof(float)*4);
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_CONTROL_INFO_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_CONTROL_INFO_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_control_info_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_control_info_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_control_info_pack(system_id, component_id, &msg , packet1.timestamp , packet1.quaternion , packet1.position , packet1.angular_velocity , packet1.velocity , packet1.control_h , packet1.control_l );
    mavlink_msg_control_info_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_control_info_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.timestamp , packet1.quaternion , packet1.position , packet1.angular_velocity , packet1.velocity , packet1.control_h , packet1.control_l );
    mavlink_msg_control_info_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_control_info_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_control_info_send(MAVLINK_COMM_1 , packet1.timestamp , packet1.quaternion , packet1.position , packet1.angular_velocity , packet1.velocity , packet1.control_h , packet1.control_l );
    mavlink_msg_control_info_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_control_info(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
    mavlink_test_control_info(system_id, component_id, last_msg);
}

#ifdef __cplusplus
}
#endif // __cplusplus
#endif // CONTROL_INFO_TESTSUITE_H
