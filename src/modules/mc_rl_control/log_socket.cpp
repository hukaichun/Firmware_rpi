#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <cstring>
#include <unistd.h>

#include <iostream>

#include <matrix/math.hpp>
#include <matrix/matrix/math.hpp>

using namespace std;
using namespace matrix;

class Log_socket
{
public:
	Log_socket();
	~Log_socket();
	
};