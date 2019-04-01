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

#define BUFFER_SIZE 	1000
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

private:
	int on = 1;
	int socket_state = -1;
	int server_socket = -1;
	sockaddr_in serv_addr;
	sockaddr_in clin_addr;
	int client_socket = -1;

	float 	buffer[DATA_LEN*BUFFER_SIZE + 1];
	int 	counter = 0;
	thread 	socket_thread;
	bool 	should_send = false;
};

Log_socket::Log_socket()
{
	if(socket_init()){
		// socket_thread = thread(&Log_socket::trans_data, this);
		// socket_thread.detach();
	} else {
		cout << "create socket fail" << endl;
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
		cout << "failed socket server" << endl;
		return -1;
	}else
	{
		cout << "server_socket : " << server_socket << endl; 
	}


	memset(&serv_addr, 0, sizeof(serv_addr));
	serv_addr.sin_family = TRANSDOMAIN;
	serv_addr.sin_addr.s_addr = inet_addr(IP);
	serv_addr.sin_port = htons(PORT);

	socket_state = setsockopt(server_socket, SOL_SOCKET, SO_REUSEADDR, &on, sizeof(on));
	socket_state = bind(server_socket, (sockaddr*)& serv_addr, sizeof(serv_addr));
	if(socket_state != 0)
	{
		cout << "binding fail" << endl;
		return -1;
	}

	listen(server_socket, 20);

	socklen_t client_addr_size = sizeof(clin_addr);
	cout << "Waiting for client connect" << endl;
	client_socket = accept(server_socket, (struct sockaddr*)&clin_addr, &client_addr_size);

	return 1;
}

void
Log_socket::trans_data()
{
	write(client_socket, buffer, sizeof(buffer));
	cout << "states send" << endl;
	// float test[3] = {1.1, 2.2, 3.3};
	// write(client_socket, test, sizeof(buffer));

}

void
Log_socket::store(const Vector<float, 18> &states, 
					const Vector<float, 4> &nn_output)
{
	char heart_beat[1] = {};

	read(client_socket, heart_beat, sizeof(heart_beat));
	if (heart_beat[0] == 'b')
		cout << "alive" << endl;
			
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