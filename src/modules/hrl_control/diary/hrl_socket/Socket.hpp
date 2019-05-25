#include <queue>
#include <vector>
#include <map>
#include <string>

#include <sys/socket.h> 
#include <arpa/inet.h>  //sockaddr_in
#include <unistd.h>     //open close write

#include <stdexcept>
#include <cstring>
#include <cstdint>



class Socket {
protected:
    	int _self_fd,
            _adjoint_fd;

    	sockaddr_in _self_info,
                    _adjoint_info;

    	socklen_t _adjoint_addr_len;


    	//default init
    	Socket() {
    		_self_fd = -1;
    		_adjoint_fd = -1;
    		_adjoint_addr_len=0;

    		memset(&_self_info, 0, sizeof(_self_info));
    		memset(&_adjoint_info, 0, sizeof(_adjoint_info));
    	}

public:
	virtual ~Socket(){}

	virtual void    send(std::vector<unsigned char>) = 0;
	virtual ssize_t recv(void *buf, size_t len)      = 0;
	virtual void    flush()                          = 0;

};



class Socket_UDP: public Socket {
protected:
	struct Parnet_Info {
		sockaddr_in info;
		uint64_t    timestamp;
	};

	std::map<int, Parnet_Info> _partner;
	


protected:
	void init_server(int recv_port_, int timeout);

public:
	Socket_UDP()=delete;
	Socket_UDP(int recv_port, int timeout_sec=3);
	virtual ~Socket_UDP();


	void register_partner(int id, const char* ip, int port);
	

	virtual void send(std::vector<unsigned char> v) override;
	virtual ssize_t recv(void *buf, size_t len)     override;
	virtual void flush()                            override;
};






class Socket_TCP: public Socket {
protected:
	std::queue<std::vector<unsigned char>> _message_queue;
	bool _attach_client;


private:
	void init_server(int port, int client_num);

public:
	Socket_TCP() = delete;
    	Socket_TCP(int port);
    	virtual ~Socket_TCP();

    	void accept();
    	void clear_all();

    	virtual void send(std::vector<unsigned char> v) override;
	virtual ssize_t recv(void *buf, size_t len)     override;
	virtual void flush()                            override;
};