#include "Socket.hpp"

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

ssize_t recv(int sockfd, void *buf, size_t len, int flags);



void Socket_TCP::init_server(int port, int client_num=1) {

    std::stringstream ss;
    std::string error_message;

    // init _server_info
    inet_aton("0.0.0.0", &_self_info.sin_addr);
    _self_info.sin_family = PF_INET;
    _self_info.sin_port = htons(port);


    // init _server_fd
    _self_fd = socket(AF_INET, SOCK_STREAM, 0);
    if(_self_fd<0) {
            ss << "\t[" << __func__ << "]"
               << "_server_fd:" << _self_fd ;
            ss >> error_message;
            throw std::runtime_error(error_message.c_str()); 
    }

    int binding_result = bind(_self_fd, (sockaddr*)&_self_info, sizeof(_self_info));
    if(binding_result < 0) {
            ss << "\t[" << __func__ << "]"
               << "binding_result:" << binding_result ;
            ss >> error_message;
            throw std::runtime_error(error_message.c_str()); 
    }

    listen(_self_fd, client_num);
    return;
}



void Socket_TCP::accept() {
    _partner_fd = ::accept(_self_fd, (sockaddr*)&_partner_info, &_partner_addr_len);
    _attach_client = true;
    return;
}


void Socket_TCP::send(std::vector<unsigned char> message) {
    _message_queue.push(std::move(message));
    if(_message_queue.size()>255) flush();
}


ssize_t Socket_TCP::recv(void *buf, size_t len) {
    return ::recv(_partner_fd, buf, len, 0);
}


void Socket_TCP::flush() {
    if(!_attach_client){
        std::queue<std::vector<unsigned char>> empty;
        std::swap(empty, _message_queue);
        return;
    }


    while(_message_queue.size()>0) {
    auto& msg = _message_queue.front();
    ::write(_partner_fd, msg.data(), msg.size());
    _message_queue.pop();
    }
}


Socket_TCP::Socket_TCP(int port)
:Socket() {
    init_server(port);
}

Socket_TCP::~Socket_TCP() {
    if(_partner_fd>0) close(_partner_fd);
    close(_self_fd);
}