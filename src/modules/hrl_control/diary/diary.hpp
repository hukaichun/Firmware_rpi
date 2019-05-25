#pragma once

#include <array>
#include <cstdint>
#include <vector>
#include <map>

#include "hrl_mavlink/control_info/control_info.hpp"
#include "hrl_socket/Socket.hpp"




class Diary : public Socket_TCP {

private:

	mavlink::control_info::msg::CONTROL_INFO   _control_info;
	mavlink::mavlink_message_t                 _msg_t;
	mavlink::MsgMap                            _mag_map;

	uint8_t _system_id;


public:

	size_t take_note(uint64_t t,
	        	const std::array<float,4>& q, 
		     	const std::array<float,3>& p, 
		     	const std::array<float,3>& w, 
		     	const std::array<float,3>& v,
		     	const std::array<float,3>& hc,
		     	const std::array<float,4>& lc,
		     	const std::array<float,3>& lp,
		     	const std::array<float,3>& sp,
		     	const std::array<float,1>& vo);

	Diary() = delete;
	Diary(int port, uint8_t sys_id);

};



class Blabbermouth : public Socket_UDP {
private:

	mavlink::control_info::msg::CONTROL_INFO   _control_info;
	mavlink::mavlink_message_t                 _msg_t;
	mavlink::MsgMap                            _mag_map;

	uint8_t _system_id;

public:

	size_t take_note(uint64_t t,
	        	const std::array<float,4>& q, 
		     	const std::array<float,3>& p, 
		     	const std::array<float,3>& w, 
		     	const std::array<float,3>& v,
		     	const std::array<float,3>& hc,
		     	const std::array<float,4>& lc,
		     	const std::array<float,3>& lp,
		     	const std::array<float,3>& sp,
		     	const std::array<float,1>& vo);


	Blabbermouth() = delete;
	Blabbermouth(int recv_port, int sys_id);


};



