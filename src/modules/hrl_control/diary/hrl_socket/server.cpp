#include "server.hpp"


#include <sstream>
#include <utility>


int accept(
    int sockfd, 
    sockaddr*, 
    socklen_t*);

ssize_t write(
    int fd, 
    const void *buf, 
    size_t count);


void SocketServer::init_server(int port, int client_num=1) {

    std::stringstream ss;
    std::string error_message;

    // init _server_info
    memset(&_server_info, 0, sizeof(_server_info));
    memset(&_client_info, 0, sizeof(_client_info));
    inet_aton("0.0.0.0", &_server_info.sin_addr);
    _server_info.sin_family = PF_INET;
    _server_info.sin_port = htons(port);


    // init _server_fd
    _server_fd = socket(AF_INET, SOCK_STREAM, 0);
    if(_server_fd<0) {
            ss << "\t[" << __func__ << "]"
               << "_server_fd:" << _server_fd ;
            ss >> error_message;
            throw std::runtime_error(error_message.c_str()); 
    }

    int binding_result = bind(_server_fd, (sockaddr*)&_server_info, sizeof(_server_info));
    if(binding_result < 0) {
            ss << "\t[" << __func__ << "]"
               << "binding_result:" << binding_result ;
            ss >> error_message;
            throw std::runtime_error(error_message.c_str()); 
    }

    listen(_server_fd, client_num);
    return;
}


void SocketServer::accept() {
    _client_fd = ::accept(_server_fd, (sockaddr*)&_client_info, &_client_addr_len);
    return;
}


size_t SocketServer::store(std::vector<unsigned char> message) {
    _message_queue.push(std::move(message));
    return _message_queue.size();
}


void SocketServer::send_all() {
    while(_message_queue.size()>0) {
            auto& msg = _message_queue.front();
            ::write(_client_fd, msg.data(), msg.size());
            _message_queue.pop();
    }
    return;
}


SocketServer::SocketServer(int port):
    _server_fd(0), _client_fd(0), _client_addr_len(0) {
    init_server(port);
}


SocketServer::~SocketServer() {
    if(_server_fd>0) close(_server_fd);

    if(_client_fd>0) close(_client_fd);
}

