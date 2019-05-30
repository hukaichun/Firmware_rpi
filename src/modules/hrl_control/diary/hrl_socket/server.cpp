#include "server.hpp"







int accept(
	int sockfd, 
	sockaddr*, 
	socklen_t*);




ssize_t write(
	int fd, 
	const void *buf, 
	size_t count);





void SocketServer::init_server(int port, int client_num=1) {

#ifdef __THROW_ERROR_MESSAGE__
	std::stringstream ss;
	std::string error_message;
#endif //__THROW_ERROR_MESSAGE__

	// init client info
	memset(&_tcp_client, 0, sizeof(_tcp_client));

	// init _tcp_server_info
	memset(&_tcp_server_info, 0, sizeof(_tcp_server_info));
	inet_aton("0.0.0.0", &_tcp_server_info.sin_addr);
	_tcp_server_info.sin_family = PF_INET;
	_tcp_server_info.sin_port = htons(port);


	// init _server_fd
	_tcp_server_fd = socket(AF_INET, SOCK_STREAM, 0);


	if(_tcp_server_fd<0) {
		errx(-1, "[%s] ask tcp socket file discription fail", __func__);
	}




	// binding tcp port
	int binding_result = bind(_tcp_server_fd, (sockaddr*)&_tcp_server_info, sizeof(_tcp_server_info));
	// handle error
	if(binding_result < 0) {
		exit(-1);
	}


	listen(_tcp_server_fd, client_num);
	

	_udp_fd = socket(AF_INET, SOCK_DGRAM, 0);
	//handle error
	if(_udp_fd<0) {
		exit(-1);
	}

	return;
}





void SocketServer::accept() {
	_tcp_client_fd = ::accept(_tcp_server_fd, (sockaddr*)&_tcp_client.info, &_tcp_client.addr_len);
	return;
}





void SocketServer::send_udp(std::vector<unsigned char> message) {
	for(auto &partner: _udp_clients) {
		sockaddr_in& 	partner_info = partner.second.info;

		partner.second.addr_len = sizeof(partner_info);
		sendto(_udp_fd, message.data(), 
			message.size(), 0, 
			(sockaddr*)&partner_info, 
			partner.second.addr_len);
	}
}


size_t SocketServer::store(std::vector<unsigned char> message) {
	_message_queue.push(std::move(message));
	return _message_queue.size();
}


void SocketServer::send_tcp(std::queue<std::vector<unsigned char>> msg_queue) {

	if(_tcp_client.addr_len>0) {
		while(msg_queue.size()>0) {
			auto& msg = msg_queue.front();
			::write(_tcp_client_fd, msg.data(), msg.size());
			msg_queue.pop();
		}
	} else {
		clean_message_queue();
	}
		
}





void SocketServer::clean_message_queue()
{
	std::queue<std::vector<unsigned char>> empty;
	std::swap(empty, _message_queue);
}





void SocketServer::register_partner(int id, const char* ip, int port) {
	ClientInfo new_partner;
	memset(&new_partner.info, 0, sizeof(new_partner.info));
	inet_aton(ip, &new_partner.info.sin_addr);

	new_partner.info.sin_port = htons(port);
	_udp_clients[id] = new_partner;
	return;
}





SocketServer::SocketServer(int tcp_port_)
: _tcp_server_fd(-1), _tcp_client_fd(-1), _tcp_port(tcp_port_), _udp_fd(-1) {
	init_server(tcp_port_);
}





SocketServer::~SocketServer() {
	if(_tcp_server_fd>0) close(_tcp_server_fd);

	if(_tcp_client_fd>0) close(_tcp_client_fd);

	if(_udp_fd>0) close(_udp_fd);
}

