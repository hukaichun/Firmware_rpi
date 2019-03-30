#include <thread>
#include <matrix/math.hpp>
#include <matrix/matrix/math.hpp>
#include <sys/socket.h>

#define BUFFER_SIZE 	1000
#define FILE_DIR 	"/home/pi/log_file"

using namespace matrix;
using namespace std;

class Log_buffer
{
public:
	Log_buffer();
	~Log_buffer(){};
	void	store(Vector<float, 18> &states, 
				  Vector<float, 4> &nn_output);
	void 	write2file();
	float 	buffer[22*BUFFER_SIZE];
	int 	counter = 0;

	FILE		*fp;


};

Log_buffer::Log_buffer()
{
	//TODO: change file name to policy name
	fp = fopen("/home/pi/log_flie", "wb");
	thread 	write_thread([this] { write2file(); });
	write_thread.detach();
}

void
Log_buffer::store(Vector<float, 18> &states, 
				  Vector<float, 4> &nn_output)
{
	for (int i = 0; i < 18; ++i)
	{
		buffer[counter*22 + i] = states(i);
	}

	for (int i = 0; i < 4; ++i)
	{
		buffer[counter*22 + 18 + i] = nn_output(i);
	}

	if (counter == BUFFER_SIZE)
	{
		write2file();
	}
}

void 
Log_buffer::write2file()
{
	fwrite(buffer, sizeof(float), 22 * BUFFER_SIZE, fp);
}