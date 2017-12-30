#include <stdio.h>
#include <iostream>
#include <thread>

#include "SimHandler.h"
#include "CommHandler.h"
#include "CommandInterface.h"

using namespace std;

int main() {

   CommHandler* comms = new CommHandler(true, 8888);
   if (!comms->Connect())
   {
      printf("Failed to connect... returning\n");
      return -1;
   }

   list<CommandInterface::MESSAGE_TYPE>* cmd_list = new list<CommandInterface::MESSAGE_TYPE>;
   thread comm_thread(&CommHandler::ReceiveDataThread, comms, cmd_list);

   SimHandler* sim = new SimHandler();
   while (true) {
      sim->Run(cmd_list, comms);
   }

}
