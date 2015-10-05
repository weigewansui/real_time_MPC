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
#include "../lcmtypes/kalman_v_t.hpp"
#include <thread> 
#include <armadillo>
#include <mutex>
#include <ctime>

#define dT 0.01

using namespace arma;

lcm::LCM lcm1;
state_t imu_msg;
vicon_state_t vicon_msg;
real_accel_t real_accel_msg;
kalman_v_t v_msg;
clock_t start;
double duration;

double ax, ay, az;
double phi, theta, psi;
double v_x, v_y, v_z;
double g = 9885.5;

void updateState();
void getWorldAccel();
void kalman();
void getVelocity();
void blkdiag(mat A, mat B, mat& C);

mat A, H, Q, R, K;
mat P_est, P;
mat P_prev;
vec x_prev, x_result, x_est, y_sensor;

std::mutex mtx; 

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

    //initialize Kalman Filter
    mat A_one;
    A_one = { {1, dT, dT*dT/2},
          {0, 1, dT*dT/2},
          {0, 0, 1} };
    A = A_one;
    A.insert_cols(A.n_cols, zeros(3,3));
    A.insert_cols(A.n_cols, zeros(3,3));
    mat tmp = A_one;
    tmp.insert_cols(0, zeros(3,3));
    tmp.insert_cols(tmp.n_cols, zeros(3,3));
    A.insert_rows(A.n_rows, tmp);
    tmp = zeros(3,6);
    tmp.insert_cols(tmp.n_cols, A_one);
    A.insert_rows(A.n_rows, tmp);

    mat H_one = {{1, 0, 0}, {0, 0, 1}};
    blkdiag(H, H_one, H);
    blkdiag(H, H_one, H);
    blkdiag(H, H_one, H);

    Q = eye(9,9)*1e-3;

    // Q.at(0, 1) = 4e-2;
    // Q.at(1, 0) = 4e-2;
    // Q.at(3, 4) = 4e-2;
    // Q.at(4, 3) = 4e-2;
    // Q.at(6, 7) = 4e-2;
    // Q.at(7, 6) = 4e-2;

    // Q.at(2, 1) = 2;
    // Q.at(1, 2) = 2;
    // Q.at(5, 4) = 2;
    // Q.at(4, 5) = 2;
    // Q.at(8, 7) = 2;
    // Q.at(7, 8) = 2;



    R = eye(6,6);

    
    R.at(0,0) = 4e-4;
    R.at(1,1) = 0.1322;
    R.at(2,2) = 4e-4;
    R.at(3,3) = 0.0245;
    R.at(4,4) = 4e-4;
    R.at(5,5) = 0.0301;

    x_prev = zeros(9,1);
    P_prev = zeros(9,9);
    y_sensor = zeros(6,1);


    std::thread updateState_thread(updateState);
    std::thread getVelocity_thread(getVelocity);
    std::thread getWorldAccel_thread(getWorldAccel);

    updateState_thread.join();
    getVelocity_thread.join();
    getWorldAccel_thread.join();

    return 0;
}


void updateState() {

    while(0 == lcm1.handle()){

    }

}

void getWorldAccel() {

    while(1) {

        mtx.lock();
        ax = (vicon_msg.DCM[0]*imu_msg.accel[0] + vicon_msg.DCM[1]*imu_msg.accel[1] + vicon_msg.DCM[2]*imu_msg.accel[2])/1000.0;     
        ay = (vicon_msg.DCM[3]*imu_msg.accel[0] + vicon_msg.DCM[4]*imu_msg.accel[1] + vicon_msg.DCM[5]*imu_msg.accel[2])/1000.0;
        az = (vicon_msg.DCM[6]*imu_msg.accel[0] + vicon_msg.DCM[7]*imu_msg.accel[1] + vicon_msg.DCM[8]*imu_msg.accel[2] - g)/1000.0;
        mtx.unlock();

        real_accel_msg.accel[0] = ax;
        real_accel_msg.accel[1] = ay;
        real_accel_msg.accel[2] = az;   

        lcm1.publish("real_accel", &real_accel_msg);
        usleep(10000);

    }

}

void getVelocity() {

    while(1) {

        mtx.lock();
        y_sensor.at(0,0) = vicon_msg.position[0]/1000.0;
        y_sensor.at(1,0) = ax;   
        y_sensor.at(2,0) = vicon_msg.position[1]/1000.0; 
        y_sensor.at(3,0) = ay;   
        y_sensor.at(4,0) = vicon_msg.position[2]/1000.0;
        y_sensor.at(5,0) = az;
        mtx.unlock();
        // start = clock();
        kalman();
        // duration = ( clock() - start ) / (double) CLOCKS_PER_SEC;
        // std::cout<<duration<<std::endl;
        v_msg.vel[0] = x_result.at(1);
        v_msg.vel[1] = x_result.at(4);
        v_msg.vel[2] = x_result.at(7);
        v_msg.pos[0] = x_result.at(0);
        v_msg.pos[1] = x_result.at(3);
        v_msg.pos[2] = x_result.at(6);
        v_msg.accel[0] = x_result.at(2);
        v_msg.accel[1] = x_result.at(5);
        v_msg.accel[2] = x_result.at(6);

        lcm1.publish("kalman_v", &v_msg);
        usleep(10000);

    }

}

void kalman() {

    x_est = A*x_prev;
    P_est = A*P_prev*A.t() + Q;

    K = P_est*H.t()*inv(H*P_est*H.t() + R);
    x_result = x_est + K*(y_sensor - H*x_est);
    P_prev = (eye(9,9) - K*H)*P_est;
    x_prev = x_result;

}

void blkdiag(mat A, mat B, mat& C) {

    mat C_tmp;
    C_tmp.resize(A.n_rows + B.n_rows, A.n_cols + B.n_cols);
    C_tmp.zeros();
    
    for( int i = 0; i < A.n_rows; i ++) {

        for (int j = 0; j < A.n_cols; j ++) {

            C_tmp.at(i,j) = A.at(i,j);

        }

    }

    for( int i = 0; i < B.n_rows; i ++) {

        for( int j = 0; j < B.n_cols; j ++ ) {

            C_tmp.at( i + A.n_rows, j + A.n_cols ) = B.at(i,j);

        }

    }

    C = C_tmp;

}