g++ -std=c++11 main_server.cpp ../src/Communication/udp_comms.cpp ../src/Communication/CommHandler.cpp -I../src/Communication -o server -lpthread
g++ -std=c++11 main_client.cpp ../src/Communication/udp_comms.cpp ../src/Communication/CommHandler.cpp -I../src/Communication -DUDP_CLIENT -o client -lpthread
