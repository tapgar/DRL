#include <stdio.h>
#include <iostream>
#include <thread>

#include "CommHandler.h"

using namespace std;

int main() {

   CommHandler* comms = new CommHandler();
   if (!comms->Connect())
   {
      printf("Failed to connect... returning\n");
      return -1;
   }

   list<int>* cmd_list = new list<int>;
   thread comm_thread(&CommHandler::ReceiveDataThread, comms, cmd_list);

   while (true) {
      comms->Send();
   }

}
