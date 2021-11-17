#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <vector>
#include <iostream>
#include <memory>


#if defined(__linux__) || defined(__APPLE__)

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#define STDIN_FILENO 0
#elif defined(_WIN32) || defined(_WIN64)
#include <conio.h>
#endif

#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <string>
#include <cstring>
#include <time.h>


#include "dynamixel_sdk.h"                                  // Uses Dynamixel SDK library

// Control table address
#define ADDR_OPERATING_MODE             11                  // Control table address is different in Dynamixel model
#define ADDR_TORQUE_ENABLE              64
#define ADDR_GOAL_POSITION              116
#define ADDR_PRESENT_POSITION           132

#define LEN_PRO_GOAL_POSITION           4
#define LEN_PRO_PRESENT_POSITION        4

// Protocol version
#define PROTOCOL_VERSION                2.0                 // See which protocol version is used in the Dynamixel

// Default setting
#define DXL1_ID                          1                   // Dynamixel ID: 1
#define DXL2_ID                          2                   // Dynamixel ID: 2
#define DXL3_ID                          3                   // Dynamixel ID: 3
#define DXL4_ID                          4                   // Dynamixel ID: 4
#define BAUDRATE                        115200
#define DEVICENAME                      "/dev/ttyACM1"      // Check which port is being used on your controller
// ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

#define TORQUE_ENABLE                   1                   // Value for enabling the torque
#define TORQUE_DISABLE                  0                   // Value for disabling the torque
#define MAX_POSITION_VALUE              1048575
#define DXL_MOVING_STATUS_THRESHOLD     20                  // Dynamixel moving status threshold
#define EXT_POSITION_CONTROL_MODE       4                   // Value for extended position control mode (operating mode)

#define ESC_ASCII_VALUE                 0x1b
#define SPACE_ASCII_VALUE               0x20
#define _USE_MATH_DEFINES

int getch() {
#if defined(__linux__) || defined(__APPLE__)
	struct termios oldt, newt;
	int ch;
	tcgetattr(STDIN_FILENO, &oldt);
	newt = oldt;
	newt.c_lflag &= ~(ICANON | ECHO);
	tcsetattr(STDIN_FILENO, TCSANOW, &newt);
	ch = getchar();
	tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
	return ch;
#elif defined(_WIN32) || defined(_WIN64)
	return _getch();
#endif
}

int kbhit(void) {
#if defined(__linux__) || defined(__APPLE__)
	struct termios oldt, newt;
	int ch;
	int oldf;

	tcgetattr(STDIN_FILENO, &oldt);
	newt = oldt;
	newt.c_lflag &= ~(ICANON | ECHO);
	tcsetattr(STDIN_FILENO, TCSANOW, &newt);
	oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
	fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

	ch = getchar();

	tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
	fcntl(STDIN_FILENO, F_SETFL, oldf);

	if (ch != EOF) {
		ungetc(ch, stdin);
		return 1;
	}

	return 0;
#elif defined(_WIN32) || defined(_WIN64)
	return _kbhit();
#endif
}

void msecSleep(int waitTime) {
#if defined(__linux__) || defined(__APPLE__)
	usleep(waitTime * 1000);
#elif defined(_WIN32) || defined(_WIN64)
	_sleep(waitTime);
#endif
}

void Cleaner_Arm_Generate_Ori(std::array<double, 3> &targ_P, std::array<double, 9> &Ori) {
	double a_vector_idx_0;

	double a_vector_idx_1;

	double absxk;

	double n_vector_idx_2;

	double scale;

	double t;

	scale = 3.3121686421112381E-170;

	absxk = std::abs(targ_P[0]);

	if (absxk > 3.3121686421112381E-170) {

		n_vector_idx_2 = 1.0;

		scale = absxk;

	} else {

		t = absxk / 3.3121686421112381E-170;

		n_vector_idx_2 = t * t;
	}


	absxk = std::abs(targ_P[1]);

	if (absxk > scale) {

		t = scale / absxk;

		n_vector_idx_2 = n_vector_idx_2 * t * t + 1.0;

		scale = absxk;

	} else {

		t = absxk / scale;

		n_vector_idx_2 += t * t;

	}


	absxk = std::abs(targ_P[2] - 200.0);

	if (absxk > scale) {

		t = scale / absxk;

		n_vector_idx_2 = n_vector_idx_2 * t * t + 1.0;

		scale = absxk;

	} else {

		t = absxk / scale;

		n_vector_idx_2 += t * t;

	}


	n_vector_idx_2 = scale * std::sqrt(n_vector_idx_2);

	a_vector_idx_0 = targ_P[0] / n_vector_idx_2;

	a_vector_idx_1 = targ_P[1] / n_vector_idx_2;

	scale = (targ_P[2] - 200.0) / n_vector_idx_2;



	//  targ_P 는 열벡터

	absxk = 0.0 * scale - (-a_vector_idx_1);

	t = -a_vector_idx_0 - 0.0 * scale;

	n_vector_idx_2 = 0.0 * a_vector_idx_1 - 0.0 * a_vector_idx_0;

	Ori[3] = a_vector_idx_1 * n_vector_idx_2 - scale * t;

	Ori[4] = scale * absxk - a_vector_idx_0 * n_vector_idx_2;

	Ori[5] = a_vector_idx_0 * t - a_vector_idx_1 * absxk;

	Ori[0] = absxk;

	Ori[6] = a_vector_idx_0;

	Ori[1] = t;

	Ori[7] = a_vector_idx_1;

	Ori[2] = n_vector_idx_2;

	Ori[8] = scale;
}

void Cleaner_Arm_Trajectory_IK(std::array<double, 3> &targ_P, std::array<double, 9> &targ_O,

                               double division, std::array<double, 5> &th, std::array<double, 1000> &theta_List) {

	static const signed char b_b[36] = {1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0,

	                                    1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1};


	static const signed char b[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};


	double Jacobian[30];

	double g_tmp[30];

	double A[25];

	double b_d[25];

	double b_vrr[9];

	double curr_O[9];

	double path_O[9];

	double targetori[9];

	double vrr[9];

	double e[6];

	double B[5];

	double thetas[5];

	double Nv_idx_0;

	double Nv_idx_1;

	double Nv_idx_2;

	double Ori_tmp;

	double Posi_tmp;

	double Posi_tmp_tmp;

	double Posi_tmp_tmp_tmp;

	double absxk;

	double b_Ori_tmp;

	double b_Posi_tmp;

	double b_index;

	double curr_P_idx_0;

	double curr_P_idx_1;

	double curr_P_idx_2;

	double d;

	double scale;

	double t;

	double theta1;

	int i;

	int ix;

	int iy;

	int jA;

	signed char b_I[25];

	signed char ipiv[5];

	std::memset(&theta_List[0], 0, 1000U * sizeof(double));

	t = std::cos(th[0]);

	Posi_tmp = std::sin(th[0]);

	Posi_tmp_tmp_tmp = th[1] + th[2];

	Posi_tmp_tmp = Posi_tmp_tmp_tmp + th[3];

	b_Posi_tmp = std::cos(Posi_tmp_tmp);

	absxk = std::sin(Posi_tmp_tmp);

	scale = 475.0 * std::cos(Posi_tmp_tmp_tmp) + 499.0 * std::cos(th[1] -

	                                                              1.3854483767992019);

	curr_P_idx_0 = 13.0 * t * scale / 50.0 - 285.0 * absxk * t;

	curr_P_idx_1 = 13.0 * Posi_tmp * scale / 50.0 - 285.0 * std::sin((th[1] + th[2])

	                                                                 + th[3]) * Posi_tmp;

	curr_P_idx_2 = ((76.5 - 247.0 * std::sin(Posi_tmp_tmp_tmp) / 2.0) - 6487.0 *

	                                                                    std::sin(th[1] - 1.3854483767992019) / 50.0) -
	               285.0 *

	               b_Posi_tmp;

	Ori_tmp = std::cos(th[4]);

	b_Ori_tmp = std::sin(th[4]);

	curr_O[0] = Posi_tmp * b_Ori_tmp + b_Posi_tmp * t * Ori_tmp;

	curr_O[3] = Ori_tmp * Posi_tmp - std::cos((th[1] + th[2]) + th[3]) * std::cos

			(th[0]) * b_Ori_tmp;

	curr_O[6] = -absxk * t;

	curr_O[1] = b_Posi_tmp * Ori_tmp * Posi_tmp - t * b_Ori_tmp;

	curr_O[4] = -t * Ori_tmp - b_Posi_tmp * Posi_tmp * b_Ori_tmp;

	curr_O[7] = -std::sin((th[1] + th[2]) + th[3]) * Posi_tmp;

	curr_O[2] = -std::sin((th[1] + th[2]) + th[3]) * Ori_tmp;

	curr_O[5] = absxk * b_Ori_tmp;

	curr_O[8] = -b_Posi_tmp;

	theta1 = std::acos((targ_O[6] * curr_O[6] + targ_O[7] * curr_O[7]) + targ_O[8]

	                                                                     * -b_Posi_tmp);

	if (theta1 > 0.005) {

		Nv_idx_0 = curr_O[7] * targ_O[8] - -b_Posi_tmp * targ_O[7];

		Nv_idx_1 = -b_Posi_tmp * targ_O[6] - curr_O[6] * targ_O[8];

		Nv_idx_2 = curr_O[6] * targ_O[7] - curr_O[7] * targ_O[6];

		scale = 3.3121686421112381E-170;

		absxk = std::abs(Nv_idx_0);

		if (absxk > 3.3121686421112381E-170) {

			Posi_tmp = 1.0;

			scale = absxk;

		} else {

			t = absxk / 3.3121686421112381E-170;

			Posi_tmp = t * t;

		}


		absxk = std::abs(Nv_idx_1);

		if (absxk > scale) {

			t = scale / absxk;

			Posi_tmp = Posi_tmp * t * t + 1.0;

			scale = absxk;

		} else {

			t = absxk / scale;

			Posi_tmp += t * t;

		}


		absxk = std::abs(Nv_idx_2);

		if (absxk > scale) {

			t = scale / absxk;

			Posi_tmp = Posi_tmp * t * t + 1.0;

			scale = absxk;

		} else {

			t = absxk / scale;

			Posi_tmp += t * t;

		}


		Posi_tmp = scale * std::sqrt(Posi_tmp);

		Nv_idx_0 /= Posi_tmp;

		Nv_idx_1 /= Posi_tmp;

		Nv_idx_2 /= Posi_tmp;

	} else {

		Nv_idx_0 = 1.0;

		Nv_idx_1 = 0.0;

		Nv_idx_2 = 0.0;

		theta1 = 0.0;

	}


	for (i = 0; i < 5; i++) {

		thetas[i] = 0.0;

	}


	b_index = 0.0;

	d = 1.0 / division;

	i = static_cast<int>(((d - d) + 1.0) / d);

	if (0 <= i - 1) {

		Jacobian[24] = 0.0;

		Jacobian[25] = 0.0;

		Jacobian[2] = 0.0;

		Jacobian[26] = 0.0;

		Jacobian[3] = 0.0;

		Jacobian[4] = 0.0;

		Jacobian[5] = 1.0;

		Jacobian[11] = 0.0;

		Jacobian[17] = 0.0;

		Jacobian[23] = 0.0;

	}


	for (int j = 0; j < i; j++) {

		double path_P_idx_0;

		double path_P_idx_1;

		double path_P_idx_2;

		int b_i;

		int i1;

		int i2;

		scale = d + static_cast<double>(j) * d;



		//  이번 루프 목표 위치, 자세 계산

		path_P_idx_0 = targ_P[0];

		path_P_idx_1 = targ_P[1];

		path_P_idx_2 = targ_P[2];

		absxk = std::cos(3.1415926535897931 * scale);

		if (scale < 1.0) {

			path_P_idx_0 = curr_P_idx_0 + (curr_P_idx_0 - targ_P[0]) * (absxk - 1.0) *

			                              0.5;

			path_P_idx_1 = curr_P_idx_1 + (curr_P_idx_1 - targ_P[1]) * (absxk - 1.0) *

			                              0.5;

			path_P_idx_2 = curr_P_idx_2 + (curr_P_idx_2 - targ_P[2]) * (absxk - 1.0) *

			                              0.5;

		}


		scale *= 1.1;

		if (scale < 1.0) {

			scale = -theta1 * 0.5 * (std::cos(3.1415926535897931 * scale) - 1.0);

		} else {

			scale = theta1;

		}


		vrr[0] = Nv_idx_0;

		vrr[3] = Nv_idx_0;

		vrr[6] = Nv_idx_0;

		vrr[1] = Nv_idx_1;

		vrr[4] = Nv_idx_1;

		vrr[7] = Nv_idx_1;

		vrr[2] = Nv_idx_2;

		vrr[5] = Nv_idx_2;

		vrr[8] = Nv_idx_2;

		for (i1 = 0; i1 < 3; i1++) {

			b_vrr[3 * i1] = vrr[3 * i1] * vrr[i1];

			iy = 3 * i1 + 1;

			b_vrr[iy] = vrr[iy] * vrr[i1 + 3];

			iy = 3 * i1 + 2;

			b_vrr[iy] = vrr[iy] * vrr[i1 + 6];

		}


		std::memcpy(&vrr[0], &b_vrr[0], 9U * sizeof(double));

		absxk = std::cos(scale);

		scale = std::sin(scale);

		b_vrr[0] = scale * 0.0;

		b_vrr[3] = scale * -Nv_idx_2;

		b_vrr[6] = scale * Nv_idx_1;

		b_vrr[1] = scale * Nv_idx_2;

		b_vrr[4] = scale * 0.0;

		b_vrr[7] = scale * -Nv_idx_0;

		b_vrr[2] = scale * -Nv_idx_1;

		b_vrr[5] = scale * Nv_idx_0;

		b_vrr[8] = scale * 0.0;

		for (i1 = 0; i1 < 9; i1++) {

			vrr[i1] = (absxk * static_cast<double>(b[i1]) + (1.0 - absxk) * vrr[i1]) +

			          b_vrr[i1];

		}


		for (i1 = 0; i1 < 3; i1++) {

			scale = vrr[i1];

			absxk = vrr[i1 + 3];

			t = vrr[i1 + 6];

			for (i2 = 0; i2 < 3; i2++) {

				path_O[i1 + 3 * i2] = (scale * curr_O[3 * i2] + absxk * curr_O[3 * i2 +

				                                                               1]) + t * curr_O[3 * i2 + 2];

			}

		}


		for (b_i = 0; b_i < 50; b_i++) {

			double Jacobian_tmp;

			double b_Posi_tmp_tmp;

			double c_Posi_tmp_tmp;

			double d_Posi_tmp_tmp;

			double e_Posi_tmp_tmp;

			int b_j;

			int k;

			Posi_tmp_tmp = std::cos(thetas[0]);

			b_Posi_tmp_tmp = std::sin(thetas[0]);

			Posi_tmp_tmp_tmp = thetas[1] + thetas[2];

			scale = Posi_tmp_tmp_tmp + thetas[3];

			c_Posi_tmp_tmp = std::cos(scale);

			scale = std::sin(scale);

			d_Posi_tmp_tmp = std::cos(Posi_tmp_tmp_tmp);

			e_Posi_tmp_tmp = std::cos(thetas[1] - 1.3854483767992019);

			Ori_tmp = std::cos(thetas[4]);

			b_Ori_tmp = std::sin(thetas[4]);

			vrr[0] = b_Posi_tmp_tmp * b_Ori_tmp + c_Posi_tmp_tmp * Posi_tmp_tmp *

			                                      Ori_tmp;

			vrr[3] = Ori_tmp * b_Posi_tmp_tmp - std::cos((thetas[1] + thetas[2]) +

			                                             thetas[3]) * std::cos(thetas[0]) * b_Ori_tmp;

			vrr[6] = -scale * Posi_tmp_tmp;

			vrr[1] = c_Posi_tmp_tmp * Ori_tmp * b_Posi_tmp_tmp - Posi_tmp_tmp *

			                                                     b_Ori_tmp;

			vrr[4] = -Posi_tmp_tmp * Ori_tmp - c_Posi_tmp_tmp * b_Posi_tmp_tmp *

			                                   b_Ori_tmp;

			vrr[7] = -std::sin((thetas[1] + thetas[2]) + thetas[3]) * b_Posi_tmp_tmp;

			vrr[2] = -std::sin((thetas[1] + thetas[2]) + thetas[3]) * Ori_tmp;

			vrr[5] = scale * b_Ori_tmp;

			vrr[8] = -c_Posi_tmp_tmp;

			Jacobian_tmp = 475.0 * d_Posi_tmp_tmp + 499.0 * e_Posi_tmp_tmp;

			t = 285.0 * scale;

			Ori_tmp = 13.0 * b_Posi_tmp_tmp * Jacobian_tmp / 50.0;

			b_Ori_tmp = t * b_Posi_tmp_tmp;

			Jacobian[0] = b_Ori_tmp - Ori_tmp;

			Posi_tmp = 247.0 * std::sin(Posi_tmp_tmp_tmp) / 2.0;

			scale = 285.0 * c_Posi_tmp_tmp + Posi_tmp;

			b_Posi_tmp = 6487.0 * std::sin(thetas[1] - 1.3854483767992019) / 50.0;

			absxk = scale + b_Posi_tmp;

			Jacobian[6] = -std::cos(thetas[0]) * absxk;

			Jacobian[12] = -std::cos(thetas[0]) * scale;

			Jacobian[18] = -285.0 * c_Posi_tmp_tmp * Posi_tmp_tmp;

			Jacobian_tmp = 13.0 * Posi_tmp_tmp * Jacobian_tmp / 50.0 - t *

			                                                           Posi_tmp_tmp;

			Jacobian[1] = Jacobian_tmp;

			Jacobian[7] = -b_Posi_tmp_tmp * absxk;

			Jacobian[13] = -std::sin(thetas[0]) * scale;

			Jacobian[19] = -285.0 * std::cos((thetas[1] + thetas[2]) + thetas[3]) *

			               b_Posi_tmp_tmp;

			Jacobian[8] = (t - 247.0 * d_Posi_tmp_tmp / 2.0) - 6487.0 * e_Posi_tmp_tmp

			                                                   / 50.0;

			Jacobian[14] = 285.0 * std::sin((thetas[1] + thetas[2]) + thetas[3]) -

			               247.0 * std::cos(thetas[1] + thetas[2]) / 2.0;

			Jacobian[20] = t;

			Jacobian[9] = -std::sin(thetas[0]);

			Jacobian[15] = -std::sin(thetas[0]);

			Jacobian[21] = -std::sin(thetas[0]);

			Jacobian[27] = -std::sin((thetas[1] + thetas[2]) + thetas[3]) * std::cos

					(thetas[0]);

			Jacobian[10] = Posi_tmp_tmp;

			Jacobian[16] = Posi_tmp_tmp;

			Jacobian[22] = Posi_tmp_tmp;

			Jacobian[28] = -std::sin((thetas[1] + thetas[2]) + thetas[3]) * std::sin

					(thetas[0]);

			Jacobian[29] = -std::cos((thetas[1] + thetas[2]) + thetas[3]);

			for (i1 = 0; i1 < 3; i1++) {

				scale = path_O[i1];

				absxk = path_O[i1 + 3];

				t = path_O[i1 + 6];

				for (i2 = 0; i2 < 3; i2++) {

					targetori[i1 + 3 * i2] = (scale * vrr[i2] + absxk * vrr[i2 + 3]) + t *

					                                                                   vrr[i2 + 6];

				}

			}


			for (i1 = 0; i1 < 3; i1++) {

				b_vrr[3 * i1] = targetori[3 * i1] - targetori[i1];

				iy = 3 * i1 + 1;

				b_vrr[iy] = targetori[iy] - targetori[i1 + 3];

				iy = 3 * i1 + 2;

				b_vrr[iy] = targetori[iy] - targetori[i1 + 6];

			}


			std::memcpy(&targetori[0], &b_vrr[0], 9U * sizeof(double));

			e[0] = path_P_idx_0 - Jacobian_tmp;

			e[1] = path_P_idx_1 - (Ori_tmp - b_Ori_tmp);

			e[2] = path_P_idx_2 - (((76.5 - Posi_tmp) - b_Posi_tmp) - 285.0 *

			                                                          c_Posi_tmp_tmp);

			e[3] = targetori[5];

			e[4] = targetori[6];

			e[5] = targetori[1];

			scale = 0.0;

			for (i1 = 0; i1 < 6; i1++) {

				absxk = 0.0;

				for (i2 = 0; i2 < 6; i2++) {

					absxk += 0.5 * e[i2] * static_cast<double>(b_b[i2 + 6 * i1]);

				}


				scale += absxk * e[i1];

			}


			for (i1 = 0; i1 < 5; i1++) {

				for (i2 = 0; i2 < 6; i2++) {

					absxk = 0.0;

					for (iy = 0; iy < 6; iy++) {

						absxk += Jacobian[iy + 6 * i1] * static_cast<double>(b_b[iy + 6 * i2]);

					}


					g_tmp[i1 + 5 * i2] = absxk;

				}

			}


			for (i1 = 0; i1 < 25; i1++) {

				b_I[i1] = 0;

			}


			for (k = 0; k < 5; k++) {

				b_I[k + 5 * k] = 1;

			}


			std::memset(&b_d[0], 0, 25U * sizeof(double));

			for (b_j = 0; b_j < 5; b_j++) {

				b_d[b_j + 5 * b_j] = 0.001;

			}


			for (i1 = 0; i1 < 5; i1++) {

				for (i2 = 0; i2 < 5; i2++) {

					absxk = 0.0;

					for (iy = 0; iy < 6; iy++) {

						absxk += g_tmp[i1 + 5 * iy] * Jacobian[iy + 6 * i2];

					}


					iy = i1 + 5 * i2;

					A[iy] = absxk + (scale * static_cast<double>(b_I[iy]) + b_d[iy]);

				}


				absxk = 0.0;

				for (i2 = 0; i2 < 6; i2++) {

					absxk += g_tmp[i1 + 5 * i2] * e[i2];

				}


				B[i1] = absxk;

				ipiv[i1] = static_cast<signed char>(i1 + 1);

			}


			for (b_j = 0; b_j < 4; b_j++) {

				int b_tmp;

				int jp1j;

				int mmj_tmp;

				mmj_tmp = 3 - b_j;

				b_tmp = b_j * 6;

				jp1j = b_tmp + 2;

				iy = 5 - b_j;

				jA = 0;

				ix = b_tmp;

				scale = std::abs(A[b_tmp]);

				for (k = 2; k <= iy; k++) {

					ix++;

					absxk = std::abs(A[ix]);

					if (absxk > scale) {

						jA = k - 1;

						scale = absxk;

					}

				}


				if (A[b_tmp + jA] != 0.0) {

					if (jA != 0) {

						iy = b_j + jA;

						ipiv[b_j] = static_cast<signed char>(iy + 1);

						ix = b_j;

						for (k = 0; k < 5; k++) {

							scale = A[ix];

							A[ix] = A[iy];

							A[iy] = scale;

							ix += 5;

							iy += 5;

						}

					}


					i1 = (b_tmp - b_j) + 5;

					for (jA = jp1j; jA <= i1; jA++) {

						A[jA - 1] /= A[b_tmp];

					}

				}


				iy = b_tmp + 5;

				jA = b_tmp;

				for (jp1j = 0; jp1j <= mmj_tmp; jp1j++) {

					scale = A[iy];

					if (A[iy] != 0.0) {

						ix = b_tmp + 1;

						i1 = jA + 7;

						i2 = (jA - b_j) + 10;

						for (k = i1; k <= i2; k++) {

							A[k - 1] += A[ix] * -scale;

							ix++;

						}

					}


					iy += 5;

					jA += 5;

				}

			}


			if (ipiv[0] != 1) {

				scale = B[0];

				B[0] = B[ipiv[0] - 1];

				B[ipiv[0] - 1] = scale;

			}


			if (ipiv[1] != 2) {

				scale = B[1];

				B[1] = B[ipiv[1] - 1];

				B[ipiv[1] - 1] = scale;

			}


			if (ipiv[2] != 3) {

				scale = B[2];

				B[2] = B[ipiv[2] - 1];

				B[ipiv[2] - 1] = scale;

			}


			if (ipiv[3] != 4) {

				scale = B[3];

				B[3] = B[ipiv[3] - 1];

				B[ipiv[3] - 1] = scale;

			}


			for (k = 0; k < 5; k++) {

				iy = 5 * k;

				if (B[k] != 0.0) {

					i1 = k + 2;

					for (jA = i1; jA < 6; jA++) {

						B[jA - 1] -= B[k] * A[(jA + iy) - 1];

					}

				}

			}


			for (k = 4; k >= 0; k--) {

				iy = 5 * k;

				scale = B[k];

				if (scale != 0.0) {

					scale /= A[k + iy];

					B[k] = scale;

					for (jA = 0; jA < k; jA++) {

						B[jA] -= B[k] * A[jA + iy];

					}

				}

			}


			for (i1 = 0; i1 < 5; i1++) {

				thetas[i1] += B[i1];

			}

		}


		for (b_i = 0; b_i < 5; b_i++) {

			theta_List[static_cast<int>((static_cast<double>(b_i) + 1.0) + b_index) -

			           1] = thetas[b_i];

		}


		b_index += 5.0;

	}

}

void Cleaner_Arm_FK(std::array<double, 5> &th, std::array<double, 9> &Ori, std::array<double, 3> &Posi) {

	double Posi_tmp;

	double Posi_tmp_tmp;

	double Posi_tmp_tmp_tmp;

	double b_Posi_tmp;

	double c_Posi_tmp;

	double d_Posi_tmp;

	Posi_tmp = std::cos(th[0]);

	b_Posi_tmp = std::sin(th[0]);

	Posi_tmp_tmp_tmp = th[1] + th[2];

	Posi_tmp_tmp = Posi_tmp_tmp_tmp + th[3];

	c_Posi_tmp = std::cos(Posi_tmp_tmp);

	d_Posi_tmp = std::sin(Posi_tmp_tmp);

	Posi_tmp_tmp = 475.0 * std::cos(Posi_tmp_tmp_tmp) + 499.0 * std::cos(th[1] -

	                                                                     1.3854483767992019);

	Posi[0] = 13.0 * Posi_tmp * Posi_tmp_tmp / 50.0 - 285.0 * d_Posi_tmp *

	                                                  Posi_tmp;

	Posi[1] = 13.0 * b_Posi_tmp * Posi_tmp_tmp / 50.0 - 285.0 * std::sin((th[1] +

	                                                                      th[2]) + th[3]) * b_Posi_tmp;

	Posi[2] = ((76.5 - 247.0 * std::sin(Posi_tmp_tmp_tmp) / 2.0) - 6487.0 * std::

	sin(th[1] - 1.3854483767992019) / 50.0) - 285.0 * c_Posi_tmp;

	Posi_tmp_tmp = std::cos(th[4]);

	Posi_tmp_tmp_tmp = std::sin(th[4]);

	Ori[0] = b_Posi_tmp * Posi_tmp_tmp_tmp + c_Posi_tmp * Posi_tmp * Posi_tmp_tmp;

	Ori[3] = Posi_tmp_tmp * b_Posi_tmp - std::cos((th[1] + th[2]) + th[3]) * std::

	cos(th[0]) * Posi_tmp_tmp_tmp;

	Ori[6] = -d_Posi_tmp * Posi_tmp;

	Ori[1] = c_Posi_tmp * Posi_tmp_tmp * b_Posi_tmp - Posi_tmp * Posi_tmp_tmp_tmp;

	Ori[4] = -Posi_tmp * Posi_tmp_tmp - c_Posi_tmp * b_Posi_tmp * Posi_tmp_tmp_tmp;

	Ori[7] = -std::sin((th[1] + th[2]) + th[3]) * b_Posi_tmp;

	Ori[2] = -std::sin((th[1] + th[2]) + th[3]) * Posi_tmp_tmp;

	Ori[5] = d_Posi_tmp * Posi_tmp_tmp_tmp;

	Ori[8] = -c_Posi_tmp;

}


int target_arm(int target_x, int target_y, int target_z) {

	std::array<double, 5> curr_th;
	std::array<int32_t, 5> curr_DXLs;
	std::array<double, 3> targ_Pos;
	std::array<double, 3> targ_Pos_temp;
	std::array<double, 9> targ_Ori;
	std::array<double, 5> path_th;
	std::array<double, 5> path_DXL;

	int division = 200;
	int step_time_1;
	int step_time_2;

	std::array<double, 1000> rest2init;
	std::array<double, 1000> init2targ_temp;
	std::array<double, 1000> targ_temp2targ;
	std::array<double, 1000> targ2init;
	std::array<double, 1000> init2rest;
	std::array<double, 9> init_Ori;
	std::array<double, 3> init_Pos;
	std::array<double, 9> rest_Ori;
	std::array<double, 3> rest_Pos;
	std::array<double, 9> curr_Ori;
	std::array<double, 3> curr_Pos;
	std::array<double, 3> Cam_Pos;

	rest_Pos[0] = 1.299351466271795e+02;
	rest_Pos[1] = 0;
	rest_Pos[2] = 43.862349263583056;


	rest_Ori[0] = 0.428941292055329;
	rest_Ori[1] = 0;
	rest_Ori[2] = 0.903332368494512;
	rest_Ori[3] = 0;
	rest_Ori[4] = -1;
	rest_Ori[5] = 0;
	rest_Ori[6] = 0.903332368494512;
	rest_Ori[7] = 0;
	rest_Ori[8] = -0.428941292055329;

	init_Ori[0] = 0;
	init_Ori[1] = 0;
	init_Ori[2] = 1;
	init_Ori[3] = 0;
	init_Ori[4] = -1;
	init_Ori[5] = 0;
	init_Ori[6] = 1;
	init_Ori[7] = 0;
	init_Ori[8] = 0;

	init_Pos[0] = 1.574821624671411e+02;
	init_Pos[1] = 8.394366312178620e-15;
	init_Pos[2] = 2.244095945374110e+02;

	// Initialize PortHandler instance
	// Set the port path
	// Get methods and members of PortHandlerLinux or PortHandlerWindows
	dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);

	// Initialize PacketHandler instance
	// Set the protocol version
	// Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
	dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

	// Initialize GroupSyncWrite instance
	dynamixel::GroupSyncWrite groupSyncWrite(portHandler, packetHandler, ADDR_GOAL_POSITION, LEN_PRO_GOAL_POSITION);

	// Initialize Groupsyncread instance for Present Position
	dynamixel::GroupSyncRead groupSyncRead(portHandler, packetHandler, ADDR_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);

	int dxl_comm_result = COMM_TX_FAIL;             // Communication result
	bool dxl_addparam_result = false;                 // addParam result
	bool dxl_getdata_result = false;                  // GetParam result

	uint8_t dxl_error = 0;                          // Dynamixel error
	uint8_t param_goal_position[4];
	int32_t dxl1_present_position = 0;               // Present position
	int32_t dxl2_present_position = 0;               // Present position
	int32_t dxl3_present_position = 0;               // Present position
	int32_t dxl4_present_position = 0;               // Present position

	// Open port
	if (portHandler->openPort()) {
		printf("Succeeded to open the port!\n");
	} else {
		printf("Failed to open the port!\n");
		printf("Press any key to terminate...\n");
		getch();
		return 0;
	}

	// Set port baudrate
	if (portHandler->setBaudRate(BAUDRATE)) {
		printf("Succeeded to change the baudrate!\n");
	} else {
		printf("Failed to change the baudrate!\n");
		printf("Press any key to terminate...\n");
		getch();
		return 0;
	}

	// Add parameter storage for Dynamixel#1 present position value
	dxl_addparam_result = groupSyncRead.addParam(DXL1_ID);
	if (dxl_addparam_result != true) {
		fprintf(stderr, "[ID:%03d] groupSyncRead addparam failed", DXL1_ID);
		return 0;
	}

	dxl_addparam_result = groupSyncRead.addParam(DXL2_ID);
	if (dxl_addparam_result != true) {
		fprintf(stderr, "[ID:%03d] groupSyncRead addparam failed", DXL2_ID);
		return 0;
	}

	dxl_addparam_result = groupSyncRead.addParam(DXL3_ID);
	if (dxl_addparam_result != true) {
		fprintf(stderr, "[ID:%03d] groupSyncRead addparam failed", DXL3_ID);
		return 0;
	}

	dxl_addparam_result = groupSyncRead.addParam(DXL4_ID);
	if (dxl_addparam_result != true) {
		fprintf(stderr, "[ID:%03d] groupSyncRead addparam failed", DXL4_ID);
		return 0;
	}

	// Wait for Data...........

	Cam_Pos[0] = 26.0;
	Cam_Pos[1] = -1 * 3.0;
	Cam_Pos[2] = -1 * 220.0;

	targ_Pos[0] = (double) target_z + Cam_Pos[0];

	targ_Pos[1] = ((-1 * ((double) target_x - 424)) * 297 / 327 / 341 * (double) target_z) + Cam_Pos[1];

	targ_Pos[2] = ((-1 * ((double) target_y - 240)) * 210 / 234 / 341 * (double) target_z) + Cam_Pos[2];


	printf("tar_x:%d", target_x);
	printf("tar_y:%d", target_y);
	printf("tar_z:%d", target_z);
	printf("p0: %lf  ", targ_Pos[0]);
	printf("p1: %lf  ", targ_Pos[1]);
	printf("p2: %lf  \n", targ_Pos[2]);


	// Calculate Desired Orientation

	Cleaner_Arm_Generate_Ori(targ_Pos, targ_Ori);

	targ_Pos_temp[0] = targ_Pos[0] - targ_Ori[6] * 30.0f;
	targ_Pos_temp[1] = targ_Pos[1] - targ_Ori[7] * 30.0f;
	targ_Pos_temp[2] = targ_Pos[2] - targ_Ori[8] * 30.0f;

	// Torque ON....................................................
	dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL1_ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE,
	                                                &dxl_error);
	if (dxl_comm_result != COMM_SUCCESS) {
		printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
	} else if (dxl_error != 0) {
		printf("%s\n", packetHandler->getRxPacketError(dxl_error));
	} else {
		printf("Dynamixel has been successfully connected \n");
	}
	dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL2_ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE,
	                                                &dxl_error);
	if (dxl_comm_result != COMM_SUCCESS) {
		printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
	} else if (dxl_error != 0) {
		printf("%s\n", packetHandler->getRxPacketError(dxl_error));
	} else {
		printf("Dynamixel has been successfully connected \n");
	}
	dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL3_ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE,
	                                                &dxl_error);
	if (dxl_comm_result != COMM_SUCCESS) {
		printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
	} else if (dxl_error != 0) {
		printf("%s\n", packetHandler->getRxPacketError(dxl_error));
	} else {
		printf("Dynamixel has been successfully connected \n");
	}
	dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL4_ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE,
	                                                &dxl_error);
	if (dxl_comm_result != COMM_SUCCESS) {
		printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
	} else if (dxl_error != 0) {
		printf("%s\n", packetHandler->getRxPacketError(dxl_error));
	} else {
		printf("Dynamixel has been successfully connected \n");
	}
	// Torque ON.................................................

	// Group_Read()..............................................

	dxl_comm_result = groupSyncRead.txRxPacket();
	if (dxl_comm_result != COMM_SUCCESS) {
		printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
	} else if (groupSyncRead.getError(DXL1_ID, &dxl_error)) {
		printf("[ID:%03d] %s\n", DXL1_ID, packetHandler->getRxPacketError(dxl_error));
	} else if (groupSyncRead.getError(DXL2_ID, &dxl_error)) {
		printf("[ID:%03d] %s\n", DXL2_ID, packetHandler->getRxPacketError(dxl_error));
	} else if (groupSyncRead.getError(DXL3_ID, &dxl_error)) {
		printf("[ID:%03d] %s\n", DXL3_ID, packetHandler->getRxPacketError(dxl_error));
	} else if (groupSyncRead.getError(DXL4_ID, &dxl_error)) {
		printf("[ID:%03d] %s\n", DXL4_ID, packetHandler->getRxPacketError(dxl_error));
	}

// Check if groupsyncread data of Dynamixel is available
	dxl_getdata_result = groupSyncRead.isAvailable(DXL1_ID, ADDR_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);
	if (dxl_getdata_result != true) {
		fprintf(stderr, "[ID:%03d] groupSyncRead getdata failed", DXL1_ID);
		return 0;
	}

	dxl_getdata_result = groupSyncRead.isAvailable(DXL2_ID, ADDR_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);
	if (dxl_getdata_result != true) {
		fprintf(stderr, "[ID:%03d] groupSyncRead getdata failed", DXL2_ID);
		return 0;
	}

	dxl_getdata_result = groupSyncRead.isAvailable(DXL3_ID, ADDR_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);
	if (dxl_getdata_result != true) {
		fprintf(stderr, "[ID:%03d] groupSyncRead getdata failed", DXL3_ID);
		return 0;
	}

	dxl_getdata_result = groupSyncRead.isAvailable(DXL4_ID, ADDR_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);
	if (dxl_getdata_result != true) {
		fprintf(stderr, "[ID:%03d] groupSyncRead getdata failed", DXL4_ID);
		return 0;
	}

// Get Dynamixel present position value
	dxl1_present_position = groupSyncRead.getData(DXL1_ID, ADDR_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);

	dxl2_present_position = groupSyncRead.getData(DXL2_ID, ADDR_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);

	dxl3_present_position = groupSyncRead.getData(DXL3_ID, ADDR_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);

	dxl4_present_position = groupSyncRead.getData(DXL4_ID, ADDR_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);

	curr_th[0] = dxl1_present_position * M_PI / 2048;

	curr_th[1] = dxl2_present_position * M_PI / 2048;

	curr_th[2] = dxl3_present_position * M_PI / 2048;

	curr_th[3] = dxl4_present_position * M_PI / 2048;

	printf("t1:%3f  t2:%3f  t3:%3f  t4:%3f\n", curr_th[0], curr_th[1], curr_th[2], curr_th[3]);

	curr_th[4] = 0;

	division = 200;
	Cleaner_Arm_FK(curr_th, curr_Ori, curr_Pos);
	Cleaner_Arm_Trajectory_IK(init_Pos, init_Ori, division, curr_th, rest2init);
	step_time_1 = (int) (std::sqrt(std::pow(curr_Pos[0] - init_Pos[0], 2) + std::pow(curr_Pos[1] - init_Pos[1], 2) +
	                               std::pow(curr_Pos[2] - init_Pos[2], 2)) * 0.03);

	for (int j = 0; j < 5; j++) {
		curr_th[j] = rest2init[j + (division - 1) * 5];
	}
//for (int j = 0; j < 200; j++){
	//printf("division:%3d  th1:%3d  th2:%3d  th3:%3d  th4:%3d\n",j,(int32_t)std::round(rest2init[j*5 + 0]*2048/M_PI),(int32_t)std::round(rest2init[j*5 + 1]*2048/M_PI),(int32_t)std::round(rest2init[j*5 + 2]*2048/M_PI),(int32_t)std::round(rest2init[j*5 + 3]*2048/M_PI));
//}

	Cleaner_Arm_Trajectory_IK(targ_Pos_temp, targ_Ori, division, curr_th, init2targ_temp);

	for (int j = 0; j < 5; j++) {
		curr_th[j] = init2targ_temp[j + (division - 1) * 5];
	}

	Cleaner_Arm_Trajectory_IK(targ_Pos, targ_Ori, division, curr_th, targ_temp2targ);

	for (int j = 0; j < 5; j++) {
		curr_th[j] = targ_temp2targ[j + (division - 1) * 5];
	}

	Cleaner_Arm_FK(curr_th, curr_Ori, curr_Pos);
	Cleaner_Arm_Trajectory_IK(init_Pos, init_Ori, division, curr_th, targ2init);
	step_time_2 = (int) (std::sqrt(std::pow(curr_Pos[0] - init_Pos[0], 2) + std::pow(curr_Pos[1] - init_Pos[1], 2) +
	                               std::pow(curr_Pos[2] - init_Pos[2], 2)) * 0.03);

	for (int j = 0; j < 5; j++) {
		curr_th[j] = targ2init[j + (division - 1) * 5];
	}

	Cleaner_Arm_Trajectory_IK(rest_Pos, rest_Ori, division, curr_th, init2rest);

	int division_count = 0;
	clock_t start, end;
	do {
		start = clock();

		dynamixel::GroupSyncWrite groupSyncWrite(portHandler, packetHandler, ADDR_GOAL_POSITION, LEN_PRO_GOAL_POSITION);

		// Add Dynamixel goal position value to the Syncwrite parameter storage
		int a0 = int(rest2init[division_count * 5 + 0] * 2048 / M_PI);
		//printf("a0: %d  ",a0);
		param_goal_position[0] = DXL_LOBYTE(DXL_LOWORD(a0));
		param_goal_position[1] = DXL_HIBYTE(DXL_LOWORD(a0));
		param_goal_position[2] = DXL_LOBYTE(DXL_HIWORD(a0));
		param_goal_position[3] = DXL_HIBYTE(DXL_HIWORD(a0));

		dxl_addparam_result = groupSyncWrite.addParam(DXL1_ID, param_goal_position);
		if (dxl_addparam_result != true) {
			fprintf(stderr, "[ID:%03d] groupSyncWrite addparam failed", DXL1_ID);
			return 0;
		}

		int a1 = int(rest2init[division_count * 5 + 1] * 2048 / M_PI);
		//printf("a1: %d  ",a1);
		param_goal_position[0] = DXL_LOBYTE(DXL_LOWORD(a1));
		param_goal_position[1] = DXL_HIBYTE(DXL_LOWORD(a1));
		param_goal_position[2] = DXL_LOBYTE(DXL_HIWORD(a1));
		param_goal_position[3] = DXL_HIBYTE(DXL_HIWORD(a1));

		dxl_addparam_result = groupSyncWrite.addParam(DXL2_ID, param_goal_position);
		if (dxl_addparam_result != true) {
			fprintf(stderr, "[ID:%03d] groupSyncWrite addparam failed", DXL2_ID);
			return 0;
		}

		int a2 = int(rest2init[division_count * 5 + 2] * 2048 / M_PI);
		//printf("a2: %d  ",a2);

		param_goal_position[0] = DXL_LOBYTE(DXL_LOWORD(a2));
		param_goal_position[1] = DXL_HIBYTE(DXL_LOWORD(a2));
		param_goal_position[2] = DXL_LOBYTE(DXL_HIWORD(a2));
		param_goal_position[3] = DXL_HIBYTE(DXL_HIWORD(a2));

		dxl_addparam_result = groupSyncWrite.addParam(DXL3_ID, param_goal_position);
		if (dxl_addparam_result != true) {
			fprintf(stderr, "[ID:%03d] groupSyncWrite addparam failed", DXL1_ID);
			return 0;
		}

		int a3 = int(rest2init[division_count * 5 + 3] * 2048 / M_PI);
		//printf("a3: %d\n",a3);
		param_goal_position[0] = DXL_LOBYTE(DXL_LOWORD(a3));
		param_goal_position[1] = DXL_HIBYTE(DXL_LOWORD(a3));
		param_goal_position[2] = DXL_LOBYTE(DXL_HIWORD(a3));
		param_goal_position[3] = DXL_HIBYTE(DXL_HIWORD(a3));

		dxl_addparam_result = groupSyncWrite.addParam(DXL4_ID, param_goal_position);
		if (dxl_addparam_result != true) {
			fprintf(stderr, "[ID:%03d] groupSyncWrite addparam failed", DXL2_ID);
			return 0;
		}

		dxl_comm_result = groupSyncWrite.txPacket();
		if (dxl_comm_result != COMM_SUCCESS) printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
		groupSyncWrite.clearParam();

		//printf("t1:%3d  t2:%3d  t3:%3d  t4:%3d\n", (int)(int32_t)std::round(rest2init[division_count*5 + 0]*2048/M_PI),(int)(int32_t)std::round(rest2init[division_count*5 + 1]*2048/M_PI),(int)(int32_t)std::round(rest2init[division_count*5 + 2]*2048/M_PI),(int)(int32_t)std::round(rest2init[division_count*5 + 3]*2048/M_PI));
		division_count = division_count + 1;

		while ((clock() - start) < step_time_1 * 1000) {
			//printf("division : %d\n",division_count);
			//printf("time:%3f\n",(clock() - start)/CLOCKS_PER_SEC);
		}
	} while (division_count < 200);

	division_count = 0;

	do {
		start = clock();

		dynamixel::GroupSyncWrite groupSyncWrite(portHandler, packetHandler, ADDR_GOAL_POSITION, LEN_PRO_GOAL_POSITION);

		// Add Dynamixel goal position value to the Syncwrite parameter storage
		int a0 = int(init2targ_temp[division_count * 5 + 0] * 2048 / M_PI);
		printf("a0: %d  ", a0);
		param_goal_position[0] = DXL_LOBYTE(DXL_LOWORD(a0));
		param_goal_position[1] = DXL_HIBYTE(DXL_LOWORD(a0));
		param_goal_position[2] = DXL_LOBYTE(DXL_HIWORD(a0));
		param_goal_position[3] = DXL_HIBYTE(DXL_HIWORD(a0));

		dxl_addparam_result = groupSyncWrite.addParam(DXL1_ID, param_goal_position);
		if (dxl_addparam_result != true) {
			fprintf(stderr, "[ID:%03d] groupSyncWrite addparam failed", DXL1_ID);
			return 0;
		}

		int a1 = int(init2targ_temp[division_count * 5 + 1] * 2048 / M_PI);
		printf("a1: %d  ", a1);
		param_goal_position[0] = DXL_LOBYTE(DXL_LOWORD(a1));
		param_goal_position[1] = DXL_HIBYTE(DXL_LOWORD(a1));
		param_goal_position[2] = DXL_LOBYTE(DXL_HIWORD(a1));
		param_goal_position[3] = DXL_HIBYTE(DXL_HIWORD(a1));

		dxl_addparam_result = groupSyncWrite.addParam(DXL2_ID, param_goal_position);
		if (dxl_addparam_result != true) {
			fprintf(stderr, "[ID:%03d] groupSyncWrite addparam failed", DXL2_ID);
			return 0;
		}

		int a2 = int(init2targ_temp[division_count * 5 + 2] * 2048 / M_PI);
		printf("a2: %d  ", a2);

		param_goal_position[0] = DXL_LOBYTE(DXL_LOWORD(a2));
		param_goal_position[1] = DXL_HIBYTE(DXL_LOWORD(a2));
		param_goal_position[2] = DXL_LOBYTE(DXL_HIWORD(a2));
		param_goal_position[3] = DXL_HIBYTE(DXL_HIWORD(a2));

		dxl_addparam_result = groupSyncWrite.addParam(DXL3_ID, param_goal_position);
		if (dxl_addparam_result != true) {
			fprintf(stderr, "[ID:%03d] groupSyncWrite addparam failed", DXL1_ID);
			return 0;
		}

		int a3 = int(init2targ_temp[division_count * 5 + 3] * 2048 / M_PI);
		printf("a3: %d\n", a3);
		param_goal_position[0] = DXL_LOBYTE(DXL_LOWORD(a3));
		param_goal_position[1] = DXL_HIBYTE(DXL_LOWORD(a3));
		param_goal_position[2] = DXL_LOBYTE(DXL_HIWORD(a3));
		param_goal_position[3] = DXL_HIBYTE(DXL_HIWORD(a3));

		dxl_addparam_result = groupSyncWrite.addParam(DXL4_ID, param_goal_position);
		if (dxl_addparam_result != true) {
			fprintf(stderr, "[ID:%03d] groupSyncWrite addparam failed", DXL2_ID);
			return 0;
		}

		dxl_comm_result = groupSyncWrite.txPacket();
		if (dxl_comm_result != COMM_SUCCESS) printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
		groupSyncWrite.clearParam();

		//printf("t1:%3d  t2:%3d  t3:%3d  t4:%3d\n", (int)(int32_t)std::round(rest2init[division_count*5 + 0]*2048/M_PI),(int)(int32_t)std::round(rest2init[division_count*5 + 1]*2048/M_PI),(int)(int32_t)std::round(rest2init[division_count*5 + 2]*2048/M_PI),(int)(int32_t)std::round(rest2init[division_count*5 + 3]*2048/M_PI));
		division_count = division_count + 1;

		while ((clock() - start) < 12 * 1000) {
			//printf("division : %d\n",division_count);
			//printf("time:%3f\n",(clock() - start)/CLOCKS_PER_SEC);
		}
	} while (division_count < 200);

	division_count = 0;
	do {
		start = clock();

		dynamixel::GroupSyncWrite groupSyncWrite(portHandler, packetHandler, ADDR_GOAL_POSITION, LEN_PRO_GOAL_POSITION);

		// Add Dynamixel goal position value to the Syncwrite parameter storage
		int a0 = int(targ_temp2targ[division_count * 5 + 0] * 2048 / M_PI);
		printf("a0: %d  ", a0);
		param_goal_position[0] = DXL_LOBYTE(DXL_LOWORD(a0));
		param_goal_position[1] = DXL_HIBYTE(DXL_LOWORD(a0));
		param_goal_position[2] = DXL_LOBYTE(DXL_HIWORD(a0));
		param_goal_position[3] = DXL_HIBYTE(DXL_HIWORD(a0));

		dxl_addparam_result = groupSyncWrite.addParam(DXL1_ID, param_goal_position);
		if (dxl_addparam_result != true) {
			fprintf(stderr, "[ID:%03d] groupSyncWrite addparam failed", DXL1_ID);
			return 0;
		}

		int a1 = int(targ_temp2targ[division_count * 5 + 1] * 2048 / M_PI);
		printf("a1: %d  ", a1);
		param_goal_position[0] = DXL_LOBYTE(DXL_LOWORD(a1));
		param_goal_position[1] = DXL_HIBYTE(DXL_LOWORD(a1));
		param_goal_position[2] = DXL_LOBYTE(DXL_HIWORD(a1));
		param_goal_position[3] = DXL_HIBYTE(DXL_HIWORD(a1));

		dxl_addparam_result = groupSyncWrite.addParam(DXL2_ID, param_goal_position);
		if (dxl_addparam_result != true) {
			fprintf(stderr, "[ID:%03d] groupSyncWrite addparam failed", DXL2_ID);
			return 0;
		}

		int a2 = int(targ_temp2targ[division_count * 5 + 2] * 2048 / M_PI);
		printf("a2: %d  ", a2);

		param_goal_position[0] = DXL_LOBYTE(DXL_LOWORD(a2));
		param_goal_position[1] = DXL_HIBYTE(DXL_LOWORD(a2));
		param_goal_position[2] = DXL_LOBYTE(DXL_HIWORD(a2));
		param_goal_position[3] = DXL_HIBYTE(DXL_HIWORD(a2));

		dxl_addparam_result = groupSyncWrite.addParam(DXL3_ID, param_goal_position);
		if (dxl_addparam_result != true) {
			fprintf(stderr, "[ID:%03d] groupSyncWrite addparam failed", DXL1_ID);
			return 0;
		}

		int a3 = int(targ_temp2targ[division_count * 5 + 3] * 2048 / M_PI);
		printf("a3: %d\n", a3);
		param_goal_position[0] = DXL_LOBYTE(DXL_LOWORD(a3));
		param_goal_position[1] = DXL_HIBYTE(DXL_LOWORD(a3));
		param_goal_position[2] = DXL_LOBYTE(DXL_HIWORD(a3));
		param_goal_position[3] = DXL_HIBYTE(DXL_HIWORD(a3));

		dxl_addparam_result = groupSyncWrite.addParam(DXL4_ID, param_goal_position);
		if (dxl_addparam_result != true) {
			fprintf(stderr, "[ID:%03d] groupSyncWrite addparam failed", DXL2_ID);
			return 0;
		}

		dxl_comm_result = groupSyncWrite.txPacket();
		if (dxl_comm_result != COMM_SUCCESS) printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
		groupSyncWrite.clearParam();

		//printf("t1:%3d  t2:%3d  t3:%3d  t4:%3d\n", (int)(int32_t)std::round(rest2init[division_count*5 + 0]*2048/M_PI),(int)(int32_t)std::round(rest2init[division_count*5 + 1]*2048/M_PI),(int)(int32_t)std::round(rest2init[division_count*5 + 2]*2048/M_PI),(int)(int32_t)std::round(rest2init[division_count*5 + 3]*2048/M_PI));
		division_count = division_count + 1;

		while ((clock() - start) < 12 * 1000) {
			//printf("division : %d\n",division_count);
			//printf("time:%3f\n",(clock() - start)/CLOCKS_PER_SEC);
		}
	} while (division_count < 200);

	division_count = 0;
	do {
		start = clock();

		dynamixel::GroupSyncWrite groupSyncWrite(portHandler, packetHandler, ADDR_GOAL_POSITION, LEN_PRO_GOAL_POSITION);

		// Add Dynamixel goal position value to the Syncwrite parameter storage
		int a0 = int(targ2init[division_count * 5 + 0] * 2048 / M_PI);
		printf("a0: %d  ", a0);
		param_goal_position[0] = DXL_LOBYTE(DXL_LOWORD(a0));
		param_goal_position[1] = DXL_HIBYTE(DXL_LOWORD(a0));
		param_goal_position[2] = DXL_LOBYTE(DXL_HIWORD(a0));
		param_goal_position[3] = DXL_HIBYTE(DXL_HIWORD(a0));

		dxl_addparam_result = groupSyncWrite.addParam(DXL1_ID, param_goal_position);
		if (dxl_addparam_result != true) {
			fprintf(stderr, "[ID:%03d] groupSyncWrite addparam failed", DXL1_ID);
			return 0;
		}

		int a1 = int(targ2init[division_count * 5 + 1] * 2048 / M_PI);
		printf("a1: %d  ", a1);
		param_goal_position[0] = DXL_LOBYTE(DXL_LOWORD(a1));
		param_goal_position[1] = DXL_HIBYTE(DXL_LOWORD(a1));
		param_goal_position[2] = DXL_LOBYTE(DXL_HIWORD(a1));
		param_goal_position[3] = DXL_HIBYTE(DXL_HIWORD(a1));

		dxl_addparam_result = groupSyncWrite.addParam(DXL2_ID, param_goal_position);
		if (dxl_addparam_result != true) {
			fprintf(stderr, "[ID:%03d] groupSyncWrite addparam failed", DXL2_ID);
			return 0;
		}

		int a2 = int(targ2init[division_count * 5 + 2] * 2048 / M_PI);
		printf("a2: %d  ", a2);

		param_goal_position[0] = DXL_LOBYTE(DXL_LOWORD(a2));
		param_goal_position[1] = DXL_HIBYTE(DXL_LOWORD(a2));
		param_goal_position[2] = DXL_LOBYTE(DXL_HIWORD(a2));
		param_goal_position[3] = DXL_HIBYTE(DXL_HIWORD(a2));

		dxl_addparam_result = groupSyncWrite.addParam(DXL3_ID, param_goal_position);
		if (dxl_addparam_result != true) {
			fprintf(stderr, "[ID:%03d] groupSyncWrite addparam failed", DXL1_ID);
			return 0;
		}

		int a3 = int(targ2init[division_count * 5 + 3] * 2048 / M_PI);
		printf("a3: %d\n", a3);
		param_goal_position[0] = DXL_LOBYTE(DXL_LOWORD(a3));
		param_goal_position[1] = DXL_HIBYTE(DXL_LOWORD(a3));
		param_goal_position[2] = DXL_LOBYTE(DXL_HIWORD(a3));
		param_goal_position[3] = DXL_HIBYTE(DXL_HIWORD(a3));

		dxl_addparam_result = groupSyncWrite.addParam(DXL4_ID, param_goal_position);
		if (dxl_addparam_result != true) {
			fprintf(stderr, "[ID:%03d] groupSyncWrite addparam failed", DXL2_ID);
			return 0;
		}

		dxl_comm_result = groupSyncWrite.txPacket();
		if (dxl_comm_result != COMM_SUCCESS) printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
		groupSyncWrite.clearParam();

		//printf("t1:%3d  t2:%3d  t3:%3d  t4:%3d\n", (int)(int32_t)std::round(rest2init[division_count*5 + 0]*2048/M_PI),(int)(int32_t)std::round(rest2init[division_count*5 + 1]*2048/M_PI),(int)(int32_t)std::round(rest2init[division_count*5 + 2]*2048/M_PI),(int)(int32_t)std::round(rest2init[division_count*5 + 3]*2048/M_PI));
		division_count = division_count + 1;

		while ((clock() - start) < step_time_2 * 1000) {
			//printf("division : %d\n",division_count);
			//printf("time:%3f\n",(clock() - start)/CLOCKS_PER_SEC);
		}
	} while (division_count < 200);

	division_count = 0;

	do {
		start = clock();

		dynamixel::GroupSyncWrite groupSyncWrite(portHandler, packetHandler, ADDR_GOAL_POSITION, LEN_PRO_GOAL_POSITION);

		// Add Dynamixel goal position value to the Syncwrite parameter storage
		int a0 = int(init2rest[division_count * 5 + 0] * 2048 / M_PI);
		printf("a0: %d  ", a0);
		param_goal_position[0] = DXL_LOBYTE(DXL_LOWORD(a0));
		param_goal_position[1] = DXL_HIBYTE(DXL_LOWORD(a0));
		param_goal_position[2] = DXL_LOBYTE(DXL_HIWORD(a0));
		param_goal_position[3] = DXL_HIBYTE(DXL_HIWORD(a0));

		dxl_addparam_result = groupSyncWrite.addParam(DXL1_ID, param_goal_position);
		if (dxl_addparam_result != true) {
			fprintf(stderr, "[ID:%03d] groupSyncWrite addparam failed", DXL1_ID);
			return 0;
		}

		int a1 = int(init2rest[division_count * 5 + 1] * 2048 / M_PI);
		printf("a1: %d  ", a1);
		param_goal_position[0] = DXL_LOBYTE(DXL_LOWORD(a1));
		param_goal_position[1] = DXL_HIBYTE(DXL_LOWORD(a1));
		param_goal_position[2] = DXL_LOBYTE(DXL_HIWORD(a1));
		param_goal_position[3] = DXL_HIBYTE(DXL_HIWORD(a1));

		dxl_addparam_result = groupSyncWrite.addParam(DXL2_ID, param_goal_position);
		if (dxl_addparam_result != true) {
			fprintf(stderr, "[ID:%03d] groupSyncWrite addparam failed", DXL2_ID);
			return 0;
		}

		int a2 = int(init2rest[division_count * 5 + 2] * 2048 / M_PI);
		printf("a2: %d  ", a2);

		param_goal_position[0] = DXL_LOBYTE(DXL_LOWORD(a2));
		param_goal_position[1] = DXL_HIBYTE(DXL_LOWORD(a2));
		param_goal_position[2] = DXL_LOBYTE(DXL_HIWORD(a2));
		param_goal_position[3] = DXL_HIBYTE(DXL_HIWORD(a2));

		dxl_addparam_result = groupSyncWrite.addParam(DXL3_ID, param_goal_position);
		if (dxl_addparam_result != true) {
			fprintf(stderr, "[ID:%03d] groupSyncWrite addparam failed", DXL1_ID);
			return 0;
		}

		int a3 = int(init2rest[division_count * 5 + 3] * 2048 / M_PI);
		printf("a3: %d\n", a3);
		param_goal_position[0] = DXL_LOBYTE(DXL_LOWORD(a3));
		param_goal_position[1] = DXL_HIBYTE(DXL_LOWORD(a3));
		param_goal_position[2] = DXL_LOBYTE(DXL_HIWORD(a3));
		param_goal_position[3] = DXL_HIBYTE(DXL_HIWORD(a3));

		dxl_addparam_result = groupSyncWrite.addParam(DXL4_ID, param_goal_position);
		if (dxl_addparam_result != true) {
			fprintf(stderr, "[ID:%03d] groupSyncWrite addparam failed", DXL2_ID);
			return 0;
		}

		dxl_comm_result = groupSyncWrite.txPacket();
		if (dxl_comm_result != COMM_SUCCESS) printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
		groupSyncWrite.clearParam();

		//printf("t1:%3d  t2:%3d  t3:%3d  t4:%3d\n", (int)(int32_t)std::round(rest2init[division_count*5 + 0]*2048/M_PI),(int)(int32_t)std::round(rest2init[division_count*5 + 1]*2048/M_PI),(int)(int32_t)std::round(rest2init[division_count*5 + 2]*2048/M_PI),(int)(int32_t)std::round(rest2init[division_count*5 + 3]*2048/M_PI));
		division_count = division_count + 1;

		while ((clock() - start) < 7 * 1000) {
			//printf("division : %d\n",division_count);
			//printf("time:%3f\n",(clock() - start)/CLOCKS_PER_SEC);
		}
	} while (division_count < 200);
// Disable Dynamixel Torque

	dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL1_ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE,
	                                                &dxl_error);

	if (dxl_comm_result != COMM_SUCCESS) {

		printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));

	} else if (dxl_error != 0) {

		printf("%s\n", packetHandler->getRxPacketError(dxl_error));

	}

	dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL2_ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE,
	                                                &dxl_error);

	if (dxl_comm_result != COMM_SUCCESS) {

		printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));

	} else if (dxl_error != 0) {

		printf("%s\n", packetHandler->getRxPacketError(dxl_error));

	}

	dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL3_ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE,
	                                                &dxl_error);

	if (dxl_comm_result != COMM_SUCCESS) {

		printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));

	} else if (dxl_error != 0) {

		printf("%s\n", packetHandler->getRxPacketError(dxl_error));

	}

	dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL4_ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE,
	                                                &dxl_error);

	if (dxl_comm_result != COMM_SUCCESS) {

		printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));

	} else if (dxl_error != 0) {

		printf("%s\n", packetHandler->getRxPacketError(dxl_error));

	}


	// Close port
	portHandler->closePort();

	return 1;
}


int a = 0;
int receive_x = 0;
int receive_y = 0;
int receive_z = 0;


std::vector<std::string> split(const std::string &input, char delimiter) {
	std::vector<std::string> answer;
	std::stringstream ss(input);
	std::string temp;

	while (getline(ss, temp, delimiter)) {
		answer.push_back(temp);
	}

	return answer;
}

class robot_arm_node : public rclcpp::Node {
public:
	robot_arm_node()
			: Node("robot_arm_server") {
		auto func_ = [this](const std_msgs::msg::String::SharedPtr msg) {
			this->callback_button(msg);
		};

		sub_btn = this->create_subscription<std_msgs::msg::String>("/button_info", 10, func_);
	}

private:
	rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_btn;

	void callback_button(const std_msgs::msg::String::SharedPtr msg) const {
		auto result = msg->data;

		auto result_split = split(result, ',');
		int center_x = std::stoi(result_split[0]);
		int center_y = std::stoi(result_split[1]);
		int depth = std::stoi(result_split[2]);

		auto a = target_arm(center_x, center_y, int(depth));                                     // CHANGE
		RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming Target\na: %d" " b: %d" " c: %d",   // CHANGE
		            center_x, center_y, depth);
		RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sending back response: [%s]", result.c_str());
	};
};

int main(int argc, char **argv) {
	rclcpp::init(argc, argv);

	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready for using Robot Arm.");      // CHANGE

	rclcpp::spin(std::make_shared<robot_arm_node>());
	rclcpp::shutdown();
}
