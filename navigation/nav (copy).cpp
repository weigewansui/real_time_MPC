// file: listener.cpp
//
// LCM example program.
//
// compile with:
//  $ gcc -o listener listener.cpp -llcm
//
// On a system with pkg-config, you can also use:
//  $ gcc -o listener listener.cpp `pkg-config --cflags --libs lcm`

#include <stdio.h>
#include <lapacke.h>
#include <unistd.h>
#include <inttypes.h>
#include <lcm/lcm-cpp.hpp>
#include "../lcmtypes/state_t.hpp"
#include "../lcmtypes/vicon_state_t.hpp"
#include "../lcmtypes/real_accel_t.hpp"
#include <thread> 

lcm::LCM lcm1;
state_t imu_msg;
vicon_state_t vicon_msg;
real_accel_t real_accel_msg;

double ax, ay, az;
double phi, theta, psi;
double g = 9885.5;
void updateState();
void getWorldAccel();

class Handler 
{
    public:
        ~Handler() {}

        void handle_imu(const lcm::ReceiveBuffer* rbuf,
                const std::string& chan, 
                const state_t* msg)
        {
            imu_msg = *msg;
        }

        void handle_vicon(const lcm::ReceiveBuffer* rbuf,
                const std::string& chan, 
                const vicon_state_t* msg)
        {
            vicon_msg = *msg;
        }

};

Handler handlerObject;

int main(int argc, char** argv)
{

    if(!lcm1.good())
        return 1;

    lcm1.subscribe("state", &Handler::handle_imu, &handlerObject);
    lcm1.subscribe("vicon_state", &Handler::handle_vicon, &handlerObject);

    std::thread updateState_thread(updateState);
    std::thread getWorldAccel_thread(getWorldAccel);

    updateState_thread.join();
    getWorldAccel_thread.join();


    return 0;
}


void updateState() {

    while(0 == lcm1.handle()){
    }

}

void getWorldAccel() {

    while(1) {

        ax = vicon_msg.DCM[0]*imu_msg.accel[0] + vicon_msg.DCM[1]*imu_msg.accel[1] + vicon_msg.DCM[2]*imu_msg.accel[2];     
        ay = vicon_msg.DCM[3]*imu_msg.accel[0] + vicon_msg.DCM[4]*imu_msg.accel[1] + vicon_msg.DCM[5]*imu_msg.accel[2];
        az = vicon_msg.DCM[6]*imu_msg.accel[0] + vicon_msg.DCM[7]*imu_msg.accel[1] + vicon_msg.DCM[8]*imu_msg.accel[2] - g;
       
        real_accel_msg.accel[0] = ax;
        real_accel_msg.accel[1] = ay;
        real_accel_msg.accel[2] = az;   

        lcm1.publish("real_accel", &real_accel_msg);
        usleep(10000);
    }

}

