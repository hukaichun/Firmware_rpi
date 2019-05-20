
//STL
#include <queue>
#include <cstring>
#include <vector>

#include <sys/socket.h> 
#include <arpa/inet.h>  //sockaddr_in
#include <unistd.h>     //open close write

#include <stdexcept>






class SocketServer{

private:
    int _server_fd,
        _client_fd;

    sockaddr_in _server_info,
                _client_info;

    socklen_t _client_addr_len;

    std::queue<std::vector<unsigned char>> _message_queue;

    void init_server(int prot, int client_num);

    bool _attach_client;

public:


    SocketServer() = delete;
    SocketServer(int port);
    virtual ~SocketServer();

    void accept();

    virtual size_t store(std::vector<unsigned char>);

    void send_all();

    void clear_all();
};


