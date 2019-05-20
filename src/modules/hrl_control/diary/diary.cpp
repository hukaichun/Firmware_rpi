#include "diary.hpp"


#include <vector>
#include <cstring>
#include <utility>

//assume mavlink 2.0
std::vector<unsigned char> msg2buff(
	mavlink::mavlink_message_t& msg, 
	uint8_t system_id, 
	uint8_t component_id,
	const uint8_t crc_extra){

	static uint8_t count = 0;

	msg.magic          = MAVLINK_STX;
	msg.len            = mavlink::_mav_trim_payload(_MAV_PAYLOAD(&msg), 0xff);
	msg.sysid          = system_id;
	msg.compid         = component_id;
	msg.incompat_flags = 0;
	msg.seq            = count++;

	uint8_t head_len = MAVLINK_CORE_HEADER_LEN + 1;
	uint16_t msg_len = msg.len + head_len + 2;
	std::vector<unsigned char> buf(msg_len);
	buf[0] = msg.magic;
	buf[1] = msg.len;
	buf[2] = msg.incompat_flags;
	buf[3] = msg.compat_flags;
	buf[4] = msg.seq;
	buf[5] = msg.sysid;
	buf[6] = msg.compid;
	buf[7] = msg.msgid & 0xFF;
	buf[8] = (msg.msgid >> 8) & 0xFF;
	buf[9] = (msg.msgid >> 16) & 0xFF; // +1
	memcpy(buf.data()+10, _MAV_PAYLOAD(&msg), msg.len);
	mavlink::crc_accumulate_buffer(&msg.checksum, (char*)&buf[1], msg.len+head_len);
	mavlink::crc_accumulate(crc_extra, &msg.checksum);
	buf[msg.len+head_len] = (uint8_t)(msg.checksum & 0xFF); //+1
	buf[msg.len+head_len+1] = (uint8_t)(msg.checksum >> 8); //+1
	
	return buf;
}




Diary::Diary(int port, uint8_t sys_id):
	SocketServer(port),
	_msg_t(),
	_mag_map(_msg_t),
	_system_id(sys_id){

	memset(&_msg_t, 0, sizeof(_msg_t));
}


size_t Diary::store(uint64_t t,
		     std::array<float,4>& q, 
		     std::array<float,3>& p, 
		     std::array<float,3>& w, 
		     std::array<float,3>& v,
		     std::array<float,3>& hc,
		     std::array<float,4>& lc){

	
	_control_info.timestamp = t;
	memcpy(_control_info.quaternion.data(),       q.data(),  sizeof(float)*q.size());
	memcpy(_control_info.position.data(),         p.data(),  sizeof(float)*p.size());
	memcpy(_control_info.angular_velocity.data(), w.data(),  sizeof(float)*w.size());
	memcpy(_control_info.velocity.data(),         v.data(),  sizeof(float)*v.size());
	memcpy(_control_info.control_h.data(),        hc.data(), sizeof(float)*hc.size());
	memcpy(_control_info.control_l.data(),        lc.data(), sizeof(float)*lc.size());
	_control_info.serialize(_mag_map);
	
	//finialize
	auto buf = ::msg2buff(_msg_t, _system_id, 0, _control_info.CRC_EXTRA);
	return SocketServer::store(buf);
	
}



