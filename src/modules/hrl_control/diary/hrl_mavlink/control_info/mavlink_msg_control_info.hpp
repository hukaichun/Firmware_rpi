// MESSAGE CONTROL_INFO support class

#pragma once

namespace mavlink {
namespace control_info {
namespace msg {

/**
 * @brief CONTROL_INFO message
 *
 * state action pair
 */
struct CONTROL_INFO : mavlink::Message {
    static constexpr msgid_t MSG_ID = 1;
    static constexpr size_t LENGTH = 116;
    static constexpr size_t MIN_LENGTH = 116;
    static constexpr uint8_t CRC_EXTRA = 35;
    static constexpr auto NAME = "CONTROL_INFO";


    uint64_t timestamp; /*<  timestamp */
    std::array<float, 4> quaternion; /*<  quaternion */
    std::array<float, 3> position; /*<  position */
    std::array<float, 3> angular_velocity; /*<  angular velocity */
    std::array<float, 3> velocity; /*<  velocity */
    std::array<float, 3> control_h; /*<  high level control */
    std::array<float, 4> control_l; /*<  low level control */
    std::array<float, 3> local_position; /*<  local position */
    std::array<float, 3> position_sp; /*<  position set point */
    std::array<float, 1> voltage; /*<  voltage */


    inline std::string get_name(void) const override
    {
            return NAME;
    }

    inline Info get_message_info(void) const override
    {
            return { MSG_ID, LENGTH, MIN_LENGTH, CRC_EXTRA };
    }

    inline std::string to_yaml(void) const override
    {
        std::stringstream ss;

        ss << NAME << ":" << std::endl;
        ss << "  timestamp: " << timestamp << std::endl;
        ss << "  quaternion: [" << to_string(quaternion) << "]" << std::endl;
        ss << "  position: [" << to_string(position) << "]" << std::endl;
        ss << "  angular_velocity: [" << to_string(angular_velocity) << "]" << std::endl;
        ss << "  velocity: [" << to_string(velocity) << "]" << std::endl;
        ss << "  control_h: [" << to_string(control_h) << "]" << std::endl;
        ss << "  control_l: [" << to_string(control_l) << "]" << std::endl;
        ss << "  local_position: [" << to_string(local_position) << "]" << std::endl;
        ss << "  position_sp: [" << to_string(position_sp) << "]" << std::endl;
        ss << "  voltage: [" << to_string(voltage) << "]" << std::endl;

        return ss.str();
    }

    inline void serialize(mavlink::MsgMap &map) const override
    {
        map.reset(MSG_ID, LENGTH);

        map << timestamp;                     // offset: 0
        map << quaternion;                    // offset: 8
        map << position;                      // offset: 24
        map << angular_velocity;              // offset: 36
        map << velocity;                      // offset: 48
        map << control_h;                     // offset: 60
        map << control_l;                     // offset: 72
        map << local_position;                // offset: 88
        map << position_sp;                   // offset: 100
        map << voltage;                       // offset: 112
    }

    inline void deserialize(mavlink::MsgMap &map) override
    {
        map >> timestamp;                     // offset: 0
        map >> quaternion;                    // offset: 8
        map >> position;                      // offset: 24
        map >> angular_velocity;              // offset: 36
        map >> velocity;                      // offset: 48
        map >> control_h;                     // offset: 60
        map >> control_l;                     // offset: 72
        map >> local_position;                // offset: 88
        map >> position_sp;                   // offset: 100
        map >> voltage;                       // offset: 112
    }
};

} // namespace msg
} // namespace control_info
} // namespace mavlink
