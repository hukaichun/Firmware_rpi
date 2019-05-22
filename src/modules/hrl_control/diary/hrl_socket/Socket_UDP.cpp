#include "Socket.hpp"

#include <sstream>


void Socket_UDP::init_server(int send_port_, int recv_port_, int timeout) {

	std::stringstream ss;
    	std::string error_message;

	_self_info.sin_family = AF_INET;
	inet_aton("0.0.0.0", &_self_info.sin_addr);
	memcpy(&_recv_info, &_self_info, sizeof(_self_info));
	_self_info.sin_port = htons(send_port_);
	_recv_info.sin_port = htons(recv_port_);


	_self_fd = socket(AF_INET, SOCK_DGRAM, 0);
	_recv_fd = socket(AF_INET, SOCK_DGRAM, 0);


	if(_self_fd<0 or _recv_fd<0) {
		ss << "\t[" << __func__ << "]"
               	   << "_self_fd:"  << _self_fd 
               	   << " _recv_fd:" << _recv_fd;
            	ss >> error_message;
            	throw std::runtime_error(error_message.c_str()); 
	}


	int bind_send = bind(_self_fd, (sockaddr*)&_self_info, sizeof(_self_info));
	int bind_recv = bind(_recv_fd, (sockaddr*)&_recv_info, sizeof(_recv_info));


	if(bind_send<0 or bind_recv<0) {
		ss << "\t[" << __func__ << "]"
               	   << "bind_send:"  << bind_send 
               	   << " bind_recv:" << bind_recv;
            	ss >> error_message;
            	throw std::runtime_error(error_message.c_str()); 
	}

	timeval timeout_s={timeout, 0};
	setsockopt(_recv_fd, SOL_SOCKET, SO_RCVTIMEO, (const char*)&timeout_s, sizeof(timeout_s));

	return;
}





void Socket_UDP::register_partner(int id, const char* ip, int port) {
	Parnet_Info new_partner;
	memset(&new_partner.info, 0, sizeof(new_partner.info));
	inet_aton(ip, &new_partner.info.sin_addr);

	new_partner.info.sin_port = htons(port);
	_partner[id] = new_partner;
	return;
}





void Socket_UDP::send(std::vector<unsigned char> v) {
	for(auto &partner :_partner) {
		sockaddr_in& partner_info = partner.second.info;
		socklen_t partner_addr_len = sizeof(partner_info);

		sendto(_self_fd, v.data(), v.size(), 0, (sockaddr*)&partner_info, partner_addr_len);
	}
	return;
}




ssize_t Socket_UDP::recv(void *buf, size_t len) {
	return recvfrom(_recv_fd, buf, len, 0, (sockaddr*)&_partner_info, &_partner_addr_len);
}




void Socket_UDP::flush() {return;}




Socket_UDP::Socket_UDP(int send_port_, int recv_port_, int timeout)
:Socket(),_recv_fd(0) {
	
	init_server(send_port_,recv_port_, timeout);

}

Socket_UDP::~Socket_UDP() {
	close(_self_fd);
	close(_recv_fd);
}