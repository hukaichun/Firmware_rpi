#pragma once

//STL
#include <queue>
#include <cstring>
#include <vector>
#include <map>

#include <cstdlib>

#include <sys/socket.h> 
#include <arpa/inet.h>  //sockaddr_in
#include <unistd.h>     //open close write







class SocketServer{

	struct ClientInfo {
	sockaddr_in	info;
	socklen_t	addr_len;
	};

private:
	// tcp obj
	int _tcp_server_fd, _tcp_client_fd,
	    _tcp_port;

	sockaddr_in 	_tcp_server_info;
	ClientInfo	_tcp_client;

	std::queue<std::vector<unsigned char>> _message_queue;

	
	// udp obj
	int _udp_fd;

	std::map<int, ClientInfo> _udp_clients;




private:
	void init_server(int prot, int client_num);

	

public:

	SocketServer() = delete;
	SocketServer(int _tcp_port);
	virtual ~SocketServer();

	void accept();

	virtual size_t send(std::vector<unsigned char>);

	size_t send_tcp();

	void clean_message_queue();

	//TODO: 
	//std::vector<unsigned char> recv_tcp();

	void register_partner(int id, const char* ip, int port);
};


