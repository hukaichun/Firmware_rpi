// MESSAGE NN_LIB_INFO support class

#pragma once

namespace mavlink {
namespace control_info {
namespace msg {

/**
 * @brief NN_LIB_INFO message
 *
 * update nn shared obj
 */
struct NN_LIB_INFO : mavlink::Message {
    static constexpr msgid_t MSG_ID = 0;
    static constexpr size_t LENGTH = 136;
    static constexpr size_t MIN_LENGTH = 136;
    static constexpr uint8_t CRC_EXTRA = 82;
    static constexpr auto NAME = "NN_LIB_INFO";


    uint64_t  D_timestamp; /*<  delta t */
    std::array<char, 128> dir; /*<  new directory of nn shared obj */


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
        ss << "   D_timestamp: " <<  D_timestamp << std::endl;
        ss << "  dir: \"" << to_string(dir) << "\"" << std::endl;

        return ss.str();
    }

    inline void serialize(mavlink::MsgMap &map) const override
    {
        map.reset(MSG_ID, LENGTH);

        map <<  D_timestamp;                  // offset: 0
        map << dir;                           // offset: 8
    }

    inline void deserialize(mavlink::MsgMap &map) override
    {
        map >>  D_timestamp;                  // offset: 0
        map >> dir;                           // offset: 8
    }
};

} // namespace msg
} // namespace control_info
} // namespace mavlink
