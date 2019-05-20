#pragma once

#include <array>
#include <cstdint>
#include <vector>
#include <map>

#include "hrl_mavlink/control_info/control_info.hpp"
#include "hrl_socket/server.hpp"




class Diary : public SocketServer {

private:
	mavlink::control_info::msg::CONTROL_INFO   _control_info;
	mavlink::mavlink_message_t                 _msg_t;
	mavlink::MsgMap                            _mag_map;

	uint8_t _system_id;


public:
	
	

	size_t store(uint64_t t,
		     const std::array<float,4>& q, 
		     const std::array<float,3>& p, 
		     const std::array<float,3>& w, 
		     const std::array<float,3>& v,
		     const std::array<float,3>& hc,
		     const std::array<float,4>& lc);

	Diary() = delete;
	Diary(int port, uint8_t sys_id);

};






