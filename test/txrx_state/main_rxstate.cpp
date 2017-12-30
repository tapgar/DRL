#include <stdio.h>
#include <iostream>
#include <thread>

#include "CommHandler.h"
#include "CommandInterface.h"
#include <mutex>          // std::mutex

#include "mujoco.h"
#include "Visualizer.h"

using namespace std;

int main() {

   CommHandler* comms = new CommHandler(false);
   if (!comms->Connect())
   {
      printf("Failed to connect... returning\n");
      return -1;
   }

   list<MESSAGE_TYPE>* cmd_list = new list<MESSAGE_TYPE>;
   thread comm_thread(&CommHandler::ReceiveDataThread, comms, cmd_list);

   mutex mtx;
   double qpos[nX];

   mj_activate("/home/tapgar/.mujoco/mjkey.txt");
   char error[1000] = "Could not load binary model";
   mjModel* mj_Model = mj_loadXML("/home/tapgar/cuda-workspace/DRL/models/cassie/cassie3d_stiff.xml", 0, error, 1000);
   if (!mj_Model) {
      mju_error_s("Load model error: %s", error);
      return -1;
   }
   // Initialize mjData
   mjData* mj_Data = mj_makeData(mj_Model);

   Visualizer* vis = new Visualizer(mj_Model, false, "remote_cassie");

   while (true) {

      mtx.lock();
      while (!cmd_list->empty())
      {
         bool bNew = false;
         MESSAGE_TYPE cmd = cmd_list->front();
         if (cmd == StateInfo)
         {
            comms->GetState(mj_Data->qpos);
            mj_forward(mj_Model, mj_Data);
            //for (int i = 0; i < nX; i++)
            //   printf("%f\t", qpos[i]);
            //printf("\n");
            bNew = true;
         }
         cmd_list->pop_front();
         if (bNew)
	    vis->Draw(mj_Data);
      }
      mtx.unlock();
   }

}
