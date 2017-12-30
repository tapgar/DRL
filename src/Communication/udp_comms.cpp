#include "udp_comms.h"
#include <stdint.h>
#include <cstring>

using namespace std;

#define SERVER "127.0.0.1"
#define BUFLEN 512

udp_comms::udp_comms(bool bClient, unsigned int port)
{
	m_bClient = bClient;
	sock = -1;
	PORT = port;
}

/**
    Connect to a host on a certain port number
 */
bool udp_comms::conn()
{
	if (m_bClient)
		return client_conn();
	else
		return server_conn();
}

bool udp_comms::client_conn()
{
	//create socket if it is not already created
	if(sock == -1)
	{
		//Create socket
		sock = socket(AF_INET , SOCK_DGRAM , IPPROTO_UDP);
		if (sock == -1)
		{
			printf("Could not create socket\n");
		}
	}
	else    {   /* OK , nothing */  }

	server.sin_family = AF_INET;
	server.sin_port = htons( PORT );
	if (inet_aton(SERVER , &server.sin_addr) == 0)
	{
		printf("inet_aton() failed\n");
		return false;
	}

	return true;
}

bool udp_comms::server_conn()
{

	//create socket if it is not already created
	if(sock == -1)
	{
		//Create socket
		sock = socket(AF_INET , SOCK_DGRAM , IPPROTO_UDP);
		if (sock == -1)
		{
			printf("Could not create socket\n");
		}
	}
	else    {   /* OK , nothing */  }

	//	server.sin_addr.s_addr = htonl(INADDR_ANY);
	server.sin_family = AF_INET;
	server.sin_port = htons( PORT );
	if (inet_aton(SERVER , &server.sin_addr) == 0)
	{
		printf("inet_aton() failed\n");
		return false;
	}

	//pretty much the same as client but bind socket for server
	if( bind(sock, (struct sockaddr*)&server, sizeof(server) ) == -1)
	{
		printf("failed to bind socket...\n");
		return false;
	}


	return true;

}




/**
    Receive data from the connected host
 */
bool udp_comms::receive_command(CommandInterface::MESSAGE_TYPE* cmd)
{
	char buff[1];
	if (!receive(buff, 1))
		return false;

	*cmd = (CommandInterface::MESSAGE_TYPE)(buff[0]);

//	printf("Received Command: %d\n",*cmd);

	switch (*cmd) {

	case CommandInterface::StateInfo:
		return process_state_info();
		break;

	case CommandInterface::NewTrajectory:
		return process_trajectory();
		break;

	case CommandInterface::StandingObjective:

		break;

	case CommandInterface::UserParams:
		return process_usr_cmd();
		break;
	}

	return true;
}

bool udp_comms::process_trajectory() {
	unsigned int numBytes = sizeof(CommandInterface::ROM_TargTraj_Struct);
	char buff[numBytes];

	if (!receive(buff, numBytes))
		return false;

//	printf("processing traje...\n");

	memcpy(&newDataBuff.trajInfo, buff, sizeof(CommandInterface::ROM_TargTraj_Struct));
	newDataBuff.trajInfo.run_count = ntohl(newDataBuff.trajInfo.run_count);
	newDataBuff.trajInfo.step_height = ntohl(newDataBuff.trajInfo.step_height);
	newDataBuff.trajInfo.numPoints = ntohs(newDataBuff.trajInfo.numPoints);
	newDataBuff.trajInfo.dt_c = ntohl(newDataBuff.trajInfo.dt_c);

//	printf("traj points\t%d\t%d\n", newDataBuff.trajInfo.numContactSwitch, newDataBuff.trajInfo.numPoints);
	numBytes = sizeof(CommandInterface::ContactInfo_Struct)*newDataBuff.trajInfo.numContactSwitch;
	char conBuff[numBytes];
	if (!receive(conBuff, numBytes))
		return false;

	for (unsigned int i = 0; i < newDataBuff.trajInfo.numContactSwitch; i++)
	{
//		printf("memcpy contacts %d\n", i);
		memcpy(&(newDataBuff.conSched[i]), &(conBuff[sizeof(CommandInterface::ContactInfo_Struct)*i]), sizeof(CommandInterface::ContactInfo_Struct));
		newDataBuff.conSched[i].T_start = ntohl(newDataBuff.conSched[i].T_start);
		newDataBuff.conSched[i].T_end = ntohl(newDataBuff.conSched[i].T_end);
		for (unsigned int j = 0; j < 4; j++)
		{
			newDataBuff.conSched[i].left[j] = ntohl(newDataBuff.conSched[i].left[j]);
			newDataBuff.conSched[i].right[j] = ntohl(newDataBuff.conSched[i].right[j]);
		}
	}

	numBytes = sizeof(CommandInterface::ROM_TrajPt_Struct)*newDataBuff.trajInfo.numPoints;
	char ptBuff[numBytes];
	if (!receive(ptBuff, numBytes))
		return false;

	for (unsigned int i = 0; i < newDataBuff.trajInfo.numPoints; i++)
	{
//		printf("memcpy points %d\n", i);
		memcpy(&(newDataBuff.posTraj[i]), &(ptBuff[sizeof(CommandInterface::ROM_TrajPt_Struct)*i]), sizeof(CommandInterface::ROM_TrajPt_Struct));
		for (unsigned int j = 0; j < 4; j++)
		{
			newDataBuff.posTraj[i].com[j] = ntohl(newDataBuff.posTraj[i].com[j]);
			newDataBuff.posTraj[i].com_xdd[j] = ntohl(newDataBuff.posTraj[i].com_xdd[j]);
		}
	}

	return true;
}

bool udp_comms::send_trajectory() {

	char cmdbuff[] = {CommandInterface::NewTrajectory};
	if (!send(cmdbuff, 1))
		return false;

	unsigned int numBytes = sizeof(CommandInterface::ROM_TargTraj_Struct);
	char buff[numBytes];

	uint8_t con_pts = newDataBuff.trajInfo.numContactSwitch;
	uint16_t traj_pts = newDataBuff.trajInfo.numPoints;
	newDataBuff.trajInfo.run_count = htonl(newDataBuff.trajInfo.run_count);
	newDataBuff.trajInfo.step_height = htonl(newDataBuff.trajInfo.step_height);
	newDataBuff.trajInfo.numPoints = htons(newDataBuff.trajInfo.numPoints);
	newDataBuff.trajInfo.dt_c = htonl(newDataBuff.trajInfo.dt_c);

	memcpy(buff, &newDataBuff.trajInfo, sizeof(CommandInterface::ROM_TargTraj_Struct));
	if (!send(buff, numBytes))
		return false;

	numBytes = sizeof(CommandInterface::ContactInfo_Struct)*con_pts;
	char conBuff[numBytes];


	for (unsigned int i = 0; i < con_pts; i++)
	{
		newDataBuff.conSched[i].T_start = htonl(newDataBuff.conSched[i].T_start);
		newDataBuff.conSched[i].T_end = htonl(newDataBuff.conSched[i].T_end);
		for (unsigned int j = 0; j < 4; j++)
		{
			newDataBuff.conSched[i].left[j] = htonl(newDataBuff.conSched[i].left[j]);
			newDataBuff.conSched[i].right[j] = htonl(newDataBuff.conSched[i].right[j]);
		}
		memcpy( &(conBuff[sizeof(CommandInterface::ContactInfo_Struct)*i]), &(newDataBuff.conSched[i]), sizeof(CommandInterface::ContactInfo_Struct));
	}

	if (!send(conBuff, numBytes))
		return false;

	numBytes = sizeof(CommandInterface::ROM_TrajPt_Struct)*traj_pts;
	char ptBuff[numBytes];

	for (unsigned int i = 0; i < traj_pts; i++)
	{
		for (unsigned int j = 0; j < 4; j++)
		{
			newDataBuff.posTraj[i].com[j] = htonl(newDataBuff.posTraj[i].com[j]);
			newDataBuff.posTraj[i].com_xdd[j] = htonl(newDataBuff.posTraj[i].com_xdd[j]);
		}
		memcpy(&(ptBuff[sizeof(CommandInterface::ROM_TrajPt_Struct)*i]), &(newDataBuff.posTraj[i]), sizeof(CommandInterface::ROM_TrajPt_Struct));
	}

	if (!send(ptBuff, numBytes))
		return false;

	return true;
}

bool udp_comms::process_state_info() {

	unsigned int numBytes = sizeof(CommandInterface::StateInfo_Struct);
	char buff[numBytes];

	if (!receive(buff, numBytes))
		return false;

	memcpy(&newDataBuff.state, buff, sizeof(CommandInterface::StateInfo_Struct));
	newDataBuff.state.run_count = ntohl(newDataBuff.state.run_count);

	for (int i = 0; i < nX; i++)
		newDataBuff.state.x_mm[i] = ntohl(newDataBuff.state.x_mm[i]);
	for (int i = 0; i < 4; i++)
	{
		newDataBuff.state.com[i] = ntohl(newDataBuff.state.com[i]);
		newDataBuff.state.com_vel[i] = ntohl(newDataBuff.state.com_vel[i]);
		newDataBuff.state.left[i] = ntohl(newDataBuff.state.left[i]);
		newDataBuff.state.right[i] = ntohl(newDataBuff.state.right[i]);
	}

	return true;
}

bool udp_comms::send_state_info(CommandInterface::StateInfo_Struct state) {

	char cmdbuff[] = {CommandInterface::StateInfo};
	if (!send(cmdbuff, 1))
		return false;

	unsigned int numBytes = sizeof(CommandInterface::StateInfo_Struct);
	char buff[numBytes];

//	printf("sending state info\n");
	state.run_count = htonl(state.run_count);

	for (int i = 0; i < nX; i++)
		state.x_mm[i] = htonl(state.x_mm[i]);
	for (int i = 0; i < 4; i++)
	{
		state.com[i] = htonl(state.com[i]);
		state.com_vel[i] = htonl(state.com_vel[i]);
		state.left[i] = htonl(state.left[i]);
		state.right[i] = htonl(state.right[i]);
	}
	memcpy(buff, &state, sizeof(CommandInterface::StateInfo_Struct));
	if (!send(buff, numBytes))
		return false;
	return true;
}


bool udp_comms::process_usr_cmd() {

	unsigned int numBytes = sizeof(CommandInterface::USER_Params);
	char buff[numBytes];

	if (!receive(buff, numBytes))
		return false;

	memcpy(&newDataBuff.usr_cmd, buff, sizeof(CommandInterface::USER_Params));
	for (int i = 0; i < 4; i++)
		newDataBuff.usr_cmd.dxT[i] = ntohl(newDataBuff.usr_cmd.dxT[i]);

	newDataBuff.usr_cmd.ds_perc = ntohl(newDataBuff.usr_cmd.ds_perc);
	newDataBuff.usr_cmd.step_time_us = ntohl(newDataBuff.usr_cmd.step_time_us);
	newDataBuff.usr_cmd.step_height = ntohl(newDataBuff.usr_cmd.step_height);

	return true;
}

bool udp_comms::send_usr_cmd(CommandInterface::USER_Params usr_cmd) {

	char cmdbuff[] = {CommandInterface::UserParams};
	if (!send(cmdbuff, 1))
		return false;

	unsigned int numBytes = sizeof(CommandInterface::USER_Params);
	char buff[numBytes];

	for (int i = 0; i < 4; i++)
		usr_cmd.dxT[i] = htonl(usr_cmd.dxT[i]);

	usr_cmd.ds_perc = htonl(usr_cmd.ds_perc);
	usr_cmd.step_time_us = htonl(usr_cmd.step_time_us);
	usr_cmd.step_height = htonl(usr_cmd.step_height);
	memcpy(buff, &usr_cmd, sizeof(CommandInterface::USER_Params));
	if (!send(buff, numBytes))
		return false;
	return true;
}

bool udp_comms::receive(char* buff, unsigned int num_bytes)
{
//	printf("attempting read...\n");
	socklen_t rcv_len = num_bytes;
	//Receive a reply from the server
	if (!m_bClient)
	{
//		printf("sock: %d, address: %x, port: %x\n", sock, remaddr.sin_addr.s_addr, remaddr.sin_port);
		if( recvfrom(sock , buff , num_bytes , 0, (struct sockaddr *) &remaddr, &rcv_len) < 0)
		{
			printf("recv failed: %s\n", strerror(errno));
			return false;
		}
	}
	else
	{
//		printf("sock: %d, address: %x, port: %x\n",sock, server.sin_addr.s_addr, server.sin_port);
		if( recvfrom(sock , buff , num_bytes , 0, (struct sockaddr *) &server, &rcv_len) < 0)
		{
			printf("recv failed: %s\n", strerror(errno));
			return false;
		}
	}
//	printf("Expected: %x\t Received: %x\n", num_bytes, (unsigned int)rcv_len);

//	printf("receiving %d bytes\n", num_bytes);
//	for (int i = 0; i < num_bytes; i++)
//		printf("%hhx,", buff[i]);
//	printf("\n");


	return true;
}

/**
    Send data to the connected host
 */
bool udp_comms::send(char* buff, unsigned int numBytes)
{
//	printf("sending %d bytes\n", numBytes);
//
//	for (int i = 0; i < numBytes; i++)
//		printf("%hhx,", buff[i]);
//	printf("\n");

	if (!m_bClient)
	{
		//Send some data
		if( sendto(sock , buff ,numBytes , 0, (struct sockaddr *) &remaddr, sizeof(server)) < 0)
		{
			printf("Send failed : %s\n", strerror(errno));
			return false;
		}
	}
	else
	{
		//Send some data
		if( sendto(sock , buff , numBytes , 0, (struct sockaddr *) &server, sizeof(server)) < 0)
		{
			printf("Send failed : %s\n", strerror(errno));
			return false;
		}
	}

	return true;
}
