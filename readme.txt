The source code has been expanded to provide the functionality needed
for this project. 

Here the main contributions to the files is listed as
the methods developed. The remaining contributions consists of 
changes in already existing methods, and are hard to list (and isolate).

Contributions to main.cpp:

void wifiSetup()
void receivedCharFromSerial1(uint8_t n)

Contributions to control.cpp:

void teamwork()
void turn(float angle, int nextState)
void radee(float radee, int nextState)
void vel(float vel, int nextState)
void tcpMsg(const char* msg, int nextState)
void test_leader()
void connect_network()