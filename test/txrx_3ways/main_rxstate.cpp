#include <stdio.h>
#include <iostream>
#include <thread>

#include "CommHandler.h"
#include "CommandInterface.h"

#include "mujoco.h"
#include "Planner.h"

using namespace std;

int main() {

   CommHandler* comms = new CommHandler(false, 8888);
   if (!comms->Connect())
   {
      printf("Failed to connect... returning\n");
      return -1;
   }

   list<CommandInterface::MESSAGE_TYPE>* cmd_list = new list<CommandInterface::MESSAGE_TYPE>;
   thread comm_thread(&CommHandler::ReceiveDataThread, comms, cmd_list);

   CommHandler* txcomms = new CommHandler(true, 8880);
   if (!txcomms->Connect())
   {
      printf("Failed to connect... returning\n");
      return -1;
   }

   list<CommandInterface::MESSAGE_TYPE>* tx_cmd_list = new list<CommandInterface::MESSAGE_TYPE>;
   thread txcomm_thread(&CommHandler::ReceiveDataThread, txcomms, tx_cmd_list);

   Planner* planner = new Planner();

   while (true) {
	   planner->Run(cmd_list, comms, tx_cmd_list, txcomms);
   }

}
