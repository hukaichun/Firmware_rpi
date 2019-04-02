#include <px4_log.h>
#include <systemlib/err.h>

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <cstring>
#include <unistd.h>

#include <iostream>
#include <thread>

#include <matrix/math.hpp>
#include <matrix/matrix/math.hpp>

#define TRANSDOMAIN 	AF_INET
#define TYPE 		SOCK_STREAM
#define PORT 		8787
#define IP		"192.168.1.159"

#define BUFFER_SIZE 	10000
#define DATA_LEN 	23

using namespace std;
using namespace matrix;


class Log_socket
{
public:
	Log_socket();
	~Log_socket();

	int 	socket_init();
	void 	trans_data();
	void	store(const Vector<float, 18> &states, const Vector<float, 4> &nn_output);
	void 	reconnect();

private:
	int on = 1;
	int server_socket = -1;
	sockaddr_in serv_addr;
	sockaddr_in clin_addr;
	socklen_t client_addr_size = sizeof(clin_addr);
	int client_socket = -1;
	struct timeval timeout{.tv_sec = 1, .tv_usec = 0};

	float 	buffer[DATA_LEN*BUFFER_SIZE + 1];
	float 	socket_buffer[DATA_LEN*BUFFER_SIZE + 1];
	int 	counter = 0;
	thread 	socket_thread;
	bool 	should_send = false;
};

Log_socket::Log_socket()
{
	if(socket_init()<0){
		warn("create socket fail");
	}
}

Log_socket::~Log_socket()
{
	close(server_socket);
	close(client_socket);
	if (socket_thread.joinable()) 
		socket_thread.join();
}

int
Log_socket::socket_init()
{
	server_socket = socket(TRANSDOMAIN, TYPE, 0);
	if (server_socket == -1)
	{
		warn("failed socket server");
		return -1;
	}else
	{
		PX4_INFO("server_socket : %d", server_socket); 
	}


	memset(&serv_addr, 0, sizeof(serv_addr));
	serv_addr.sin_family = TRANSDOMAIN;
	serv_addr.sin_addr.s_addr = inet_addr(IP);
	serv_addr.sin_port = htons(PORT);

    if (setsockopt(server_socket, SOL_SOCKET, SO_REUSEADDR, &on, sizeof(on)) < 0)
        warn("setsockopt REUSEADDR failed");

    if (setsockopt(server_socket, SOL_SOCKET, SO_SNDTIMEO, (char *)&timeout, sizeof(timeout)) < 0)
        warn("setsockopt SO_SNDTIMEO failed");

	if (bind(server_socket, (sockaddr*)& serv_addr, sizeof(serv_addr)) != 0)
	{
		warn("binding fail");
		return -1;
	}

	listen(server_socket, 20);

	thread reconnect_thread = thread(&Log_socket::reconnect, this);
	reconnect_thread.detach();	

	return 1;
}

void
Log_socket::reconnect()
{
	client_socket = accept(server_socket, (struct sockaddr*)&clin_addr, &client_addr_size);
	PX4_INFO("client connected");
	thread reconnect_thread = thread(&Log_socket::reconnect, this);
	reconnect_thread.detach();	
}

void
Log_socket::trans_data()
{
	for (int i = 0; i < DATA_LEN*BUFFER_SIZE+1; ++i)
	{
		socket_buffer[i] = buffer[i];
	}
	write(client_socket, socket_buffer, sizeof(socket_buffer));
#define DEGUB
#ifdef DEGUB
	PX4_INFO("states send");
#endif
}

void
Log_socket::store(const Vector<float, 18> &states, 
					const Vector<float, 4> &nn_output)
{
	
	buffer[counter*DATA_LEN] = 9487;	// Seperate each step by 9487

	for (int i = 0; i < 18; ++i)
	{
		buffer[counter*DATA_LEN + 1 + i] = states(i);
	}

	for (int i = 0; i < 4; ++i)
	{
		buffer[counter*DATA_LEN + 19 + i] = nn_output(i);
	}

	if (counter == BUFFER_SIZE-1)
	{
		buffer[DATA_LEN*BUFFER_SIZE] = 666;

		socket_thread = thread(&Log_socket::trans_data, this);
		socket_thread.detach();
		counter = 0;
	} else {
		counter++;
	}
}