#include <thread>
#include <iostream>
#include <matrix/math.hpp>
#include <matrix/matrix/math.hpp>
#include <sys/socket.h>

#define BUFFER_SIZE 	10000
#define FILE_DIR 	"/home/pi/log_file"
#define DATA_LEN 	23

using namespace matrix;
using namespace std;

class Log_buffer
{
public:
	Log_buffer();
	~Log_buffer();
	void	store(const Vector<float, 18> &states, const Vector<float, 4> &nn_output);
	void 	write2file();

	float 	buffer[DATA_LEN*BUFFER_SIZE];
	int 	counter = 0;
	thread 	write_thread;
	FILE	*fp;
};

Log_buffer::Log_buffer()
{
	//TODO: change file name to policy name
	fp = fopen("/home/pi/log_file", "wb");
}

Log_buffer::~Log_buffer()
{
	if (write_thread.joinable()) 
		write_thread.join();
}

void
Log_buffer::store(const Vector<float, 18> &states, 
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
		write_thread = thread(&Log_buffer::write2file, this);
		write_thread.detach();
		counter = 0;
	} else {
		counter++;
	}
}

void 
Log_buffer::write2file()
{
	fwrite(buffer, sizeof(float), DATA_LEN * BUFFER_SIZE, fp);
	cout << "finish write to file" << endl;
}