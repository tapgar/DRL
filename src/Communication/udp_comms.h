#ifndef TCP_CLIENT_H
#define TCP_CLIENT_H

#include <stdio.h> //printf
#include <sys/socket.h>    //socket
#include <arpa/inet.h> //inet_addr
#include <netdb.h> //hostent
#include <vector>
#include "CommandInterface.h"

/**
    TCP Client class
*/
class udp_comms
{
private:
    int sock;
    struct sockaddr_in server;
    struct sockaddr_in remaddr;

    bool server_conn();
    bool client_conn();

    bool process_state_info();
    bool process_trajectory();
    bool process_usr_cmd();


    bool receive(char* buff, unsigned int num_bytes);
    bool send(char* buff, unsigned int numBytes);


    bool m_bClient;
    unsigned int PORT;

public:
    udp_comms(bool bClient, unsigned int port);
    bool conn();

    bool receive_command(CommandInterface::MESSAGE_TYPE* cmd);

    bool send_state_info(CommandInterface::StateInfo_Struct state);

    bool send_usr_cmd(CommandInterface::USER_Params usr_cmd);

    bool send_trajectory();

    void get_state(CommandInterface::StateInfo_Struct* state) { *state = newDataBuff.state; }

    void get_trajectory(CommAction_Struct* pCA)
    {
    	pCA = &newDataBuff;
    }

	CommAction_Struct newDataBuff; //for comm thread to put data into



};

#endif
