#pragma once

#include <array>
#include <cstdint>
#include <vector>

#include "mavlink/control_info/control_info.hpp"
#include "hrl_socket/server.hpp"




class Diary : public SocketServer {

private:
	mavlink::control_info::msg::CONTROL_INFO   _control_info;
	mavlink::mavlink_message_t                 _msg_t;
	mavlink::MsgMap                            _mag_map;

	uint8_t _system_id;

	std::vector<unsigned char> msg2buff();


public:
	
	

	size_t store(uint64_t t,
		     std::array<float,4>& q, 
		     std::array<float,3>& p, 
		     std::array<float,3>& w, 
		     std::array<float,3>& v,
		     std::array<float,3>& hc,
		     std::array<float,4>& lc);

	Diary() = delete;
	Diary(int port, uint8_t sys_id);

};






