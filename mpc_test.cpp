// g++ mpc_test.cpp -o mpctest -std=c++11 -larmadillo

#include <armadillo>
#include <iostream>
#include <stdlib.h>
#include <ctime>
#include <thread>
#include <stdio.h>
#include <unistd.h>
#include <sys/time.h>

#include <lcm/lcm-cpp.hpp>
#include "lcmtypes/accel_t.hpp"
#include "lcmtypes/vicon_state_t.hpp"
// #include "lcmtypes/debug_t.hpp"

// settings
#define HORIZON  10
#define dT  0.01
#define simu_height 16
#define better_height 18

using namespace std;
using namespace arma;

mat A_d_all;
mat B_d;

mat H, C;

vicon_state_t current_state;
accel_t accel_msg;

class Handler 
{
    public:
        ~Handler() {}

        void handleMessage(const lcm::ReceiveBuffer* rbuf,
                const std::string& chan, 
                const vicon_state_t* vicon_msg)
        {
        	current_state = *vicon_msg;

        }
};

void blkdiag(mat, mat, mat&);
void bldMatrixDiag(float*, mat&, int);
void pushMatrix_row(mat&, mat);
void pushMatrix_col(mat&, mat);
void initialize_Matrices();
void updateStateThread();
int64_t utime_now (void);
// void delete_arr(float*);
// 

//assign limitations
float u1_min = -5;
float u1_max = 5;
float u2_min = -5;
float u2_max = 5;
float u3_min = -5;
float u3_max = 7;

float z_max = 5;
float z_min = -5;

float tan_half_phi = 0.5;
float tan_half_psi = 0.5;

// Coefficients for the barrier

float mu_state = 0.5;
float mu_u = 10;

mat x0;


double accumu_err_x = 0;
double accumu_err_y = 0;
double accumu_err_z = 0;

lcm::LCM lcm1;
Handler handlerObject;

int main() {

	if(!lcm1.good())
        return 1;


    lcm1.subscribe("vicon_state", &Handler::handleMessage, &handlerObject);

	initialize_Matrices();

	// initial conditions
	mat tmp0;
	tmp0 = {0, 0, 0, 1.2, 0, 0, 1.3, 0, 0, 0, 0, 0};
	tmp0 = trans(tmp0);

	mat z0(12*HORIZON, 1);

	mat beq(9*HORIZON,1);

	x0 = {1, 0, 0, 1, 0, 0, 0, 0, 0};
	x0 = trans(x0);

	//for testing purpose
	float noise_mu = 0.0001;
	mat noise = noise_mu * x0;
	mat beqFirstHorizon = A_d_all*x0 + noise;
	
	for(int i = 0; i < 9; i++) {

		beq(i,0) = beqFirstHorizon(i, 0);

	}

	for(int i = 0; i < 12; i ++) {
		
		z0(i,0) = tmp0(i,0);
	
	}

	// set the values of entrices of beq for other horizons
	for(int i = 9; i < HORIZON*9; i ++) {

		beq(i, 0) = noise(i%9,0);

	}

	for (int i = 12; i < HORIZON*12; i++) {

		z0(i,0) = tmp0(i%12,0);

	}

	mat v0 =  0.05 * randi<mat>(9*HORIZON, 1, distr_param(0, 20));

	mat z = z0;
	mat v = v0;

 	vec r(21);
 	r.ones();
	vec r_d(12*HORIZON);
	vec r_p(9*HORIZON);

	float w_r_p = 1e-4;
	float w_d_p = 1e-4;

	int loop_num = 0;

	float fval;
	float x, y, z_h;
	mat HESSIAN, GRADIENT;

	float u1, u2, u3, h;
	float tmp_denum_grad_x_1, tmp_denum_grad_x_2, tmp_denum_grad_y_1, tmp_denum_grad_y_2;
	float d_phi_dx, d_phi_dy, d_phi_dz;
	float d_phi_du1, d_phi_du2, d_phi_du3;

	float tmp_denum_x_1, tmp_denum_x_2, tmp_denum_y_1, tmp_denum_y_2;
	float dd_phi_dxdx, dd_phi_dxdz, dd_phi_dydy, dd_phi_dydz, dd_phi_dzdz;
	float dd_phi_du1du1, dd_phi_du2du2, dd_phi_du3du3;
	
	// infeasible start newton method params 
	vec d_v, d_z, beta;
	mat inv_PHI, Y;

	//linear search parameters
	float beta_line_search; //parameter
	float d; //step size
	vec line_search_tmp;

	clock_t start;
	double duration;
	start = clock();

	thread first(updateStateThread);


	while(1) {
		start = clock();
		beqFirstHorizon = A_d_all*x0 + noise;
	
		for(int i = 0; i < 9; i++) {

			beq(i,0) = beqFirstHorizon(i, 0);

		}

		while(norm(r) > 1e-4) {
			duration = ( clock() - start ) / (double) CLOCKS_PER_SEC;

			if(duration > 0.2) {
				continue;
			}
			
			fval = dot(trans(z), (H * z));


			HESSIAN.zeros(12*HORIZON, 12*HORIZON);
			GRADIENT.zeros(12*HORIZON, 1);


			for (int i = 0; i < HORIZON; i++) {

				x = z.at(3 + i * 12);

				y = z.at(6 + i * 12);

				z_h = z.at(9 + i * 12);

				u1 = z.at(0 + 12 * i);


				u2 = z.at(1 + 12 * i);

				u3 = z.at(2 + 12 * i);


				h = -z_h + better_height;

				tmp_denum_grad_x_1 = h*tan_half_phi - x;
				tmp_denum_grad_x_2 = h*tan_half_phi + x;
				tmp_denum_grad_y_1 = h*tan_half_psi - y;
				tmp_denum_grad_y_2 = h*tan_half_psi + y;

				d_phi_dx = 1/tmp_denum_grad_x_1 - 1/tmp_denum_grad_x_2;
	         	d_phi_dy = 1/tmp_denum_grad_y_1 - 1/tmp_denum_grad_y_2;
	         	d_phi_dz = 1/(z_max - z_h) - 1/(z_h - z_min) + tan_half_phi/tmp_denum_grad_x_1 + tan_half_phi/tmp_denum_grad_x_2
	             	+ tan_half_psi/tmp_denum_grad_y_1 + tan_half_psi/tmp_denum_grad_y_2;

	         	d_phi_du1 = 1/(u1_max - u1) - 1/(u1 - u1_min);
	        	d_phi_du2 = 1/(u2_max - u2) - 1/(u2 - u2_min);
	         	d_phi_du3 = 1/(u3_max - u3) - 1/(u3 - u3_min);

	         	GRADIENT.at(i*12) = mu_u*d_phi_du1;
	         	GRADIENT.at(i*12 + 1) = mu_u*d_phi_du2;
	         	GRADIENT.at(i*12 + 2) = mu_u*d_phi_du3;
	         	GRADIENT.at(i*12 + 3) = mu_state*d_phi_dx;
	         	GRADIENT.at(i*12 + 6) = mu_state*d_phi_dy;
	         	GRADIENT.at(i*12 + 9) = mu_state*d_phi_dz;

	         	tmp_denum_x_1 = (h*tan_half_phi - x)*(h*tan_half_phi - x);
	         	tmp_denum_x_2 = (h*tan_half_phi + x)*(h*tan_half_phi + x);

	         	tmp_denum_y_1 = (h*tan_half_psi - y)*(h*tan_half_psi - y);
	         	tmp_denum_y_2 = (h*tan_half_psi + y)*(h*tan_half_psi + y);

	         	dd_phi_dxdx = 1/tmp_denum_x_1 + 1/tmp_denum_x_2;
	         	dd_phi_dxdz = tan_half_phi/tmp_denum_x_1 - tan_half_phi/tmp_denum_x_2;

	         	dd_phi_dydy = 1/tmp_denum_y_1 + 1/tmp_denum_y_2;
	         	dd_phi_dydz = tan_half_psi/tmp_denum_y_1 - tan_half_psi/tmp_denum_y_2;

	         	dd_phi_dzdz = 1/(z_max - z_h)/(z_max - z_h) + 1/(z_h - z_min)/(z_h - z_min) + tan_half_phi*tan_half_phi/tmp_denum_x_1
	            	 + tan_half_phi*tan_half_phi/tmp_denum_x_2 + tan_half_psi*tan_half_psi/tmp_denum_y_1 + tan_half_psi*tan_half_psi/tmp_denum_y_2;

	         	dd_phi_du1du1 = 1/(u1_max - u1)/(u1_max - u1) + 1/(u1 - u1_min)/(u1 - u1_min);
	         	dd_phi_du2du2 = 1/(u2_max - u2)/(u2_max - u2) + 1/(u2 - u2_min)/(u2 - u2_min);
	         	dd_phi_du3du3 = 1/(u3_max - u3)/(u3_max - u3) + 1/(u3 - u3_min)/(u3 - u3_min);

	         	HESSIAN.at(12*i, 12*i) = mu_u*dd_phi_du1du1;
	         	HESSIAN.at(12*i + 1, 12*i + 1) = mu_u*dd_phi_du2du2;
	         	HESSIAN.at(12*i + 2, 12*i + 2) = mu_u*dd_phi_du3du3;

	         	HESSIAN.at(12*i + 3, 12*i + 3) = mu_state*dd_phi_dxdx;
	         	HESSIAN.at(12*i + 3, 12*i + 9) = mu_state*dd_phi_dxdz;
	         	HESSIAN.at(12*i + 6, 12*i + 6) = mu_state*dd_phi_dydy;
	         	HESSIAN.at(12*i + 6, 12*i + 9) = mu_state*dd_phi_dydz;
	         	HESSIAN.at(12*i + 9, 12*i + 9) = mu_state*dd_phi_dzdz;
	         	HESSIAN.at(12*i + 9, 12*i + 3) = mu_state*dd_phi_dxdz;
	         	HESSIAN.at(12*i + 9, 12*i + 6) = mu_state*dd_phi_dydz;

			}

			r_d = 2*H*z + GRADIENT + trans(C)*v;
			r_p = C*z - beq;

			r = r_d;
			r.insert_rows(12*HORIZON, r_p);
			// start = clock();

			inv_PHI = inv(2*H + HESSIAN);
			// duration = ( clock() - start ) / (double) CLOCKS_PER_SEC;
			// cout<<"duration: " <<duration <<endl;
			// return 0;
			// inv_PHI.print("inv_PHI");
			// C.print("C");
			Y = C*inv_PHI*C.t();

			// Y.print("Y");
			beta = -r_p + C*inv_PHI*r_d;

			d_v = -inv_sympd(Y)*beta;
			d_z = inv_PHI*(-r_d - C.t()*d_v);

			// line search
			beta_line_search = 0.5;
			d = 1;
			line_search_tmp = 2*H*(z + d*d_z) + GRADIENT + C.t()*(v + d*d_v);
			line_search_tmp.insert_rows(line_search_tmp.n_rows,  (C*(z + d*d_z)-beq));
			
			while(norm(line_search_tmp)  > (1 - 0.4*d)*norm(r) ) {

				d = beta_line_search*d;

			}

			// update z and v
			z = z + d*d_z;
			v = v + d*d_v;
			// r.print("r");

			// cout<<__LINE__<<endl;

		}

		accel_msg.timestamp = utime_now();
		accel_msg.accel[0] = z(0);
		accel_msg.accel[1] = z(1);
		accel_msg.accel[2] = z(2);

		lcm1.publish("acceleration", &accel_msg);


		
		cout<<"duration: " <<duration <<endl;
		
		r.ones();

		usleep(5e4);

	}

	first.join();


	cout<<"control:u1 u2 u3 " <<z(0,0)<<", " <<z(1,0) <<", " <<z(2,0)<<endl;


}



void updateStateThread() {

	while(0 == lcm1.handle()) {

		x0.at(0,0) = current_state.position[0]/1000.0;
		x0.at(3,0) = current_state.position[1]/1000.0;
		x0.at(6,0) = -(current_state.position[2]/1000.0 + simu_height) + better_height;

		x0.at(1,0) = current_state.velocity[0];
		x0.at(4,0) = current_state.velocity[1];
		x0.at(7,0) = -current_state.velocity[2];

		accumu_err_x = accumu_err_x + x0.at(0,0) * dT;
		accumu_err_y = accumu_err_y + x0.at(3,0) * dT;
		accumu_err_z = accumu_err_z + x0.at(6,0) * dT;

		x0.at(2,0) = accumu_err_x;
		x0.at(5,0) = accumu_err_y;
		x0.at(8,0) = accumu_err_z;

		usleep(5e4);

		// cout <<__LINE__<<endl;
	
	}

}

int64_t utime_now (void)
{
    struct timeval tv;
    gettimeofday (&tv, NULL);
    return (int64_t) tv.tv_sec * 1000000 + tv.tv_usec;
}

void initialize_Matrices () {

	//assign matrices
	mat A_d_triple = {{1, dT, 0}, {0,1,0}, {dT, dT*dT/2, 1}};

	mat tmp;
	blkdiag( A_d_triple, A_d_triple, tmp );
	blkdiag( A_d_triple, tmp, A_d_all);

	mat B_one_state;
	B_one_state << dT*dT/2 <<endr
				<< dT <<endr
				<< dT*dT*dT/6 <<endr;

	blkdiag( B_one_state, B_one_state, tmp );
	blkdiag( B_one_state, tmp, B_d);

	mat R(3,3);
	R.eye();
	R = R*0.001;

	float a[] = {1,1,0.001,1,1,0.001,0.001,0.01,0.001};
	
	mat Q_end, Q;
	bldMatrixDiag(a, Q_end, sizeof(a)/sizeof(*a));

	float b[] = {1, 0.05, 0.01, 1, 0.05, 0.01, 0.01, 0.01, 0.01};

	bldMatrixDiag(b, Q, sizeof(b)/sizeof(*b));

	H.reset();
	C.reset();
	mat zero_mat, eye_mat;
	
	for (int i = 0; i < HORIZON; i ++) {

		// cout<<"i: "<<i<<endl;

		if(i != HORIZON - 1) {

			blkdiag(H, R, H);
			blkdiag(H, Q, H);

		} else {

			blkdiag(H, R, H);
			blkdiag(H, Q_end, H);

		}

		if ( i == 0) {
			// cout<<__LINE__<<endl;

			tmp.reset();
			pushMatrix_row(tmp, -B_d);

			eye_mat.eye(9,9);
			pushMatrix_row(tmp, eye_mat);


			eye_mat.zeros(9, 12*(HORIZON - 1));

			pushMatrix_row(tmp, eye_mat);
			C.insert_rows(C.n_rows, tmp);


		}

		else if (i == HORIZON - 1) {

			// cout<<__LINE__<<endl;

			tmp.reset();
			zero_mat.zeros(9,3);

			pushMatrix_row(tmp, zero_mat);

			zero_mat.zeros(9, 12*((i + 1) - 2));
			pushMatrix_row(tmp, zero_mat);
			pushMatrix_row(tmp, -A_d_all);
			pushMatrix_row(tmp, -B_d);
			// tmp.print("tmp at i = 1");

			eye_mat.eye(9,9);
			pushMatrix_row(tmp, eye_mat);
			// tmp.print("tmp at i = 1");
			C.insert_rows(C.n_rows, tmp);
			// pushMatrix_col(C, tmp);	
			// C.print("C");


		}

		else if (i == 1) {
			// cout<<__LINE__<<endl;

			tmp.reset();
			zero_mat.zeros(9, 3);

			pushMatrix_row(tmp, zero_mat);
			pushMatrix_row(tmp, -A_d_all);
			pushMatrix_row(tmp, -B_d);

			eye_mat.eye(9,9);
			pushMatrix_row(tmp, eye_mat);
			
			zero_mat.zeros(9, 12 * (HORIZON - 2));
			pushMatrix_row(tmp, zero_mat);
			// tmp.print("tmp at i = 1");
			C.insert_rows(C.n_rows, tmp);

		}

		else {
			// cout<<__LINE__<<endl;

			tmp.reset();
			zero_mat.zeros(9, 3);
			pushMatrix_row(tmp, zero_mat);
			zero_mat.zeros(9, ((i + 1) - 2)*12);
			pushMatrix_row(tmp, zero_mat);
			pushMatrix_row(tmp, -A_d_all);
			pushMatrix_row(tmp, -B_d);
			eye_mat.eye(9, 9);
			pushMatrix_row(tmp, eye_mat);
			zero_mat.zeros(9, 12*(HORIZON - (i + 1)));
			pushMatrix_row(tmp, zero_mat);

			C.insert_rows(C.n_rows, tmp);

		}


	}

	// cout << "C: " << C.n_rows << ", " <<C.n_cols << endl;

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

void bldMatrixDiag(float *Vec, mat& MATRIX, int Length) {

	MATRIX.zeros(Length, Length);

	for (int i = 0; i < Length; i ++) {

		MATRIX.at(i,i) = Vec[i];

	}

}

void pushMatrix_row(mat& MATRIX, mat ToBePushed) {


	// cout<< "MATRIX" << MATRIX.n_rows<< ", " <<MATRIX.n_cols <<endl;
	// cout<< "ToBePushed" << ToBePushed.n_rows << ", " << ToBePushed.n_cols <<endl;

	if (MATRIX.is_empty()) {

		// cout <<__LINE__<<endl;
		MATRIX.resize(ToBePushed.n_rows, ToBePushed.n_cols);
		for(int i = 0; i < ToBePushed.n_rows; i ++) {

			for (int j = 0; j < ToBePushed.n_cols; j ++) {

				MATRIX.at(i, j) = ToBePushed.at(i, j);

			}

		}

		return;

	}

	if (MATRIX.n_rows != ToBePushed.n_rows) {

		cout << " wrong dimension!" <<endl;
		abort();

	}

	mat tmp;
	tmp.zeros(MATRIX.n_rows, MATRIX.n_cols + ToBePushed.n_cols);

	// cout<<"tmp: " << tmp.n_rows <<", " <<tmp.n_cols <<endl;
	for(int i = 0; i < MATRIX.n_rows; i ++) {

		// cout << i <<endl;
		for(int j = 0; j < MATRIX.n_cols; j ++) {

			tmp.at(i,j) = MATRIX.at(i,j);

		}

	}

	for(int i = 0; i < ToBePushed.n_rows; i  ++) {

		for(int j = 0; j < ToBePushed.n_cols; j ++) {

			tmp.at(i, j + MATRIX.n_cols) = ToBePushed.at(i, j);
						// cout<<__LINE__<<endl;

		}

	}

	MATRIX = tmp;

}

void pushMatrix_col(mat& MATRIX, mat ToBePushed) {

	if (MATRIX.is_empty()) {

		MATRIX.resize(ToBePushed.n_rows, ToBePushed.n_cols);
		for(int i = 0; i < ToBePushed.n_rows; i ++) {

			for (int j = 0; j < ToBePushed.n_cols; j ++) {

				MATRIX.at(i, j) = ToBePushed.at(i, j);

			}

		}

		return;

	}

	
	if(MATRIX.n_cols != ToBePushed.n_cols) {

		cout << " wrong dimension!" <<endl;
		abort();	

	}

	mat tmp;

	tmp.zeros(MATRIX.n_rows + ToBePushed.n_rows, MATRIX.n_cols);
	
	for (int i = 0; i < MATRIX.n_rows; i ++) {

		for (int j = 0; j < MATRIX.n_cols; j ++) {

			tmp.at(i,j) = MATRIX.at(i,j);

		}

	}

	for (int i = 0; i < ToBePushed.n_rows; i ++) {

		for ( int j = 0; j < ToBePushed.n_rows; j ++) {

			tmp.at(i + MATRIX.n_rows, j) = ToBePushed.at(i,j);

		}

	}

	MATRIX = tmp;


}