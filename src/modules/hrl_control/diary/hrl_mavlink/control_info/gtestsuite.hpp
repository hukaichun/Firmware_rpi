/** @file
 *	@brief MAVLink comm testsuite protocol generated from control_info.xml
 *	@see http://mavlink.org
 */

#pragma once

#include <gtest/gtest.h>
#include "control_info.hpp"

#ifdef TEST_INTEROP
using namespace mavlink;
#undef MAVLINK_HELPER
#include "mavlink.h"
#endif


TEST(control_info, CONTROL_INFO)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::control_info::msg::CONTROL_INFO packet_in{};
    packet_in.timestamp = 93372036854775807ULL;
    packet_in.quaternion = {{ 73.0, 74.0, 75.0, 76.0 }};
    packet_in.position = {{ 185.0, 186.0, 187.0 }};
    packet_in.angular_velocity = {{ 269.0, 270.0, 271.0 }};
    packet_in.velocity = {{ 353.0, 354.0, 355.0 }};
    packet_in.control_h = {{ 437.0, 438.0, 439.0 }};
    packet_in.control_l = {{ 521.0, 522.0, 523.0, 524.0 }};

    mavlink::control_info::msg::CONTROL_INFO packet1{};
    mavlink::control_info::msg::CONTROL_INFO packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.timestamp, packet2.timestamp);
    EXPECT_EQ(packet1.quaternion, packet2.quaternion);
    EXPECT_EQ(packet1.position, packet2.position);
    EXPECT_EQ(packet1.angular_velocity, packet2.angular_velocity);
    EXPECT_EQ(packet1.velocity, packet2.velocity);
    EXPECT_EQ(packet1.control_h, packet2.control_h);
    EXPECT_EQ(packet1.control_l, packet2.control_l);
}

#ifdef TEST_INTEROP
TEST(control_info_interop, CONTROL_INFO)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_control_info_t packet_c {
         93372036854775807ULL, { 73.0, 74.0, 75.0, 76.0 }, { 185.0, 186.0, 187.0 }, { 269.0, 270.0, 271.0 }, { 353.0, 354.0, 355.0 }, { 437.0, 438.0, 439.0 }, { 521.0, 522.0, 523.0, 524.0 }
    };

    mavlink::control_info::msg::CONTROL_INFO packet_in{};
    packet_in.timestamp = 93372036854775807ULL;
    packet_in.quaternion = {{ 73.0, 74.0, 75.0, 76.0 }};
    packet_in.position = {{ 185.0, 186.0, 187.0 }};
    packet_in.angular_velocity = {{ 269.0, 270.0, 271.0 }};
    packet_in.velocity = {{ 353.0, 354.0, 355.0 }};
    packet_in.control_h = {{ 437.0, 438.0, 439.0 }};
    packet_in.control_l = {{ 521.0, 522.0, 523.0, 524.0 }};

    mavlink::control_info::msg::CONTROL_INFO packet2{};

    mavlink_msg_control_info_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.timestamp, packet2.timestamp);
    EXPECT_EQ(packet_in.quaternion, packet2.quaternion);
    EXPECT_EQ(packet_in.position, packet2.position);
    EXPECT_EQ(packet_in.angular_velocity, packet2.angular_velocity);
    EXPECT_EQ(packet_in.velocity, packet2.velocity);
    EXPECT_EQ(packet_in.control_h, packet2.control_h);
    EXPECT_EQ(packet_in.control_l, packet2.control_l);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(control_info, NN_LIB_INFO)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::control_info::msg::NN_LIB_INFO packet_in{};
    packet_in. D_timestamp = 93372036854775807ULL;
    packet_in.dir = to_char_array("IJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDE");

    mavlink::control_info::msg::NN_LIB_INFO packet1{};
    mavlink::control_info::msg::NN_LIB_INFO packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1. D_timestamp, packet2. D_timestamp);
    EXPECT_EQ(packet1.dir, packet2.dir);
}

#ifdef TEST_INTEROP
TEST(control_info_interop, NN_LIB_INFO)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_nn_lib_info_t packet_c {
         93372036854775807ULL, "IJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDE"
    };

    mavlink::control_info::msg::NN_LIB_INFO packet_in{};
    packet_in. D_timestamp = 93372036854775807ULL;
    packet_in.dir = to_char_array("IJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDE");

    mavlink::control_info::msg::NN_LIB_INFO packet2{};

    mavlink_msg_nn_lib_info_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in. D_timestamp, packet2. D_timestamp);
    EXPECT_EQ(packet_in.dir, packet2.dir);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif
