/** @file
 *  @brief MAVLink comm protocol generated from control_info.xml
 *  @see http://mavlink.org
 */
#pragma once
#ifndef MAVLINK_CONTROL_INFO_H
#define MAVLINK_CONTROL_INFO_H

#ifndef MAVLINK_H
    #error Wrong include order: MAVLINK_CONTROL_INFO.H MUST NOT BE DIRECTLY USED. Include mavlink.h from the same directory instead or set ALL AND EVERY defines from MAVLINK.H manually accordingly, including the #define MAVLINK_H call.
#endif

#undef MAVLINK_THIS_XML_IDX
#define MAVLINK_THIS_XML_IDX 0

#ifdef __cplusplus
extern "C" {
#endif

// MESSAGE LENGTHS AND CRCS

#ifndef MAVLINK_MESSAGE_LENGTHS
#define MAVLINK_MESSAGE_LENGTHS {}
#endif

#ifndef MAVLINK_MESSAGE_CRCS
#define MAVLINK_MESSAGE_CRCS {{0, 82, 136, 136, 0, 0, 0}, {1, 35, 116, 116, 0, 0, 0}}
#endif

#include "../protocol.h"

#define MAVLINK_ENABLED_CONTROL_INFO

// ENUM DEFINITIONS



// MAVLINK VERSION

#ifndef MAVLINK_VERSION
#define MAVLINK_VERSION 1
#endif

#if (MAVLINK_VERSION == 0)
#undef MAVLINK_VERSION
#define MAVLINK_VERSION 1
#endif

// MESSAGE DEFINITIONS
#include "./mavlink_msg_nn_lib_info.h"
#include "./mavlink_msg_control_info.h"

// base include


#undef MAVLINK_THIS_XML_IDX
#define MAVLINK_THIS_XML_IDX 0

#if MAVLINK_THIS_XML_IDX == MAVLINK_PRIMARY_XML_IDX
# define MAVLINK_MESSAGE_INFO {MAVLINK_MESSAGE_INFO_NN_LIB_INFO, MAVLINK_MESSAGE_INFO_CONTROL_INFO}
# define MAVLINK_MESSAGE_NAMES {{ "CONTROL_INFO", 1 }, { "NN_LIB_INFO", 0 }}
# if MAVLINK_COMMAND_24BIT
#  include "../mavlink_get_info.h"
# endif
#endif

#ifdef __cplusplus
}
#endif // __cplusplus
#endif // MAVLINK_CONTROL_INFO_H
