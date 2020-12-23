#ifndef ACTIVECASTER_H
#define ACTIVECASTER_H


#ifdef _WIN32
#include <windows.h>
#else   
#include <unistd.h>
#endif



#include <serial/serial.h>
#include <string>
#include <iostream>
#include <cstdio>
#include <stdlib.h>
#include <math.h>
#include <chrono>
// #include "obo_ganglia_driver_lib/ganglia_control.h"
// #include <Eigen/Dense>

// using namespace Eigen;
using std::string;
using std::cout;
using std::cin;
using std::endl;

#define ACK_start                   0xCC
#define ACK_end                     0xEE
#define request_calibration         0xAC
#define request_force               0xCA
#define request_touch               0x3C
#define request_imu                 0x63
#define request_isOpen              0x36
#define request_enteringState		0x70
#define request_isToggleUp          0x41
#define virtualAdmittance_select    0x42
#define targetVel_to_interface		0x50
#define actualVel_to_interface		0x51
#define force_to_interface          0x52




namespace ACTIVECASTER {


	enum state : uint8_t
	{
		init = 0x81, calibrate_steering, idle, calibrate_handle, hold, push, deaccelerate
	};

	class TIME {

	private:

		std::chrono::high_resolution_clock::time_point lastTime;
		std::chrono::duration<double> period = std::chrono::milliseconds(10);
		std::chrono::high_resolution_clock::time_point timeFlag;

	public:
		std::chrono::high_resolution_clock::time_point thisTime;

		void startTiming() {

			thisTime = std::chrono::high_resolution_clock::now();

		}

		double timeLoop() {

			lastTime = thisTime;
			thisTime = std::chrono::high_resolution_clock::now();
			period = std::chrono::duration_cast<std::chrono::duration<double>>(thisTime - lastTime);

			return period.count();

		}

		void setTimeFlag() {

			timeFlag = std::chrono::high_resolution_clock::now();

		}

		double sinceFlag() {

			period = std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::high_resolution_clock::now() - lastTime);
			return period.count();

		}

	};

	class INTERFACE_SCREEN {
	private:

		uint8_t request = 0;
		int return_byte = 0;
		int byteread = 1;
		int bytesend = 1;
		uint8_t serialBuffer = 0;
		int counter = 0;
		bool is_ToggleUp;
		bool is_open = false;
		uint8_t parameterSet;

	public:

		bool isOpen(serial::Serial& _arduino_interface) {

			request = request_isOpen;
			return_byte = _arduino_interface.write(&request, bytesend);
			return_byte = _arduino_interface.read(&serialBuffer, byteread);
			counter = 0;

			while (serialBuffer != ACK_start) {
				cout << "missing starting signal" << endl;
				return_byte = _arduino_interface.read(&serialBuffer, byteread);
				counter++;
				if (counter > 10) {
					cout << "missing starting signal" << endl;
					return false;
				}
			}

			is_open = false;
			return_byte = _arduino_interface.read(&serialBuffer, byteread);
			is_open = 0xFF & serialBuffer;
			return_byte = _arduino_interface.read(&serialBuffer, byteread);

			while (serialBuffer != ACK_end) {
				cout << "missing ending signal" << endl;
				return_byte = _arduino_interface.read(&serialBuffer, byteread);
				counter++;
				if (counter > 10) {
					cout << "missing starting signal" << endl;
					return false;
				}
			}

			return is_open;
		}

		bool enterState(serial::Serial& _arduino_interface, ACTIVECASTER::state _robot_state) {
			request = request_enteringState;
			return_byte = _arduino_interface.write(&request, bytesend);
			return_byte = _arduino_interface.read(&serialBuffer, byteread);

			if (serialBuffer != ACK_start) {
				return false;
			}

			serialBuffer = 0xFF & _robot_state; 
			return_byte = _arduino_interface.write(&serialBuffer, bytesend);
			return_byte = _arduino_interface.read(&serialBuffer, byteread);

			if (serialBuffer != ACK_end) {
				return false;
			}
			
			return true;
		}
		

		bool isToggleUp(serial::Serial& _arduino_interface) {

			request = request_isToggleUp;
			return_byte = _arduino_interface.write(&request, bytesend);
			return_byte = _arduino_interface.read(&serialBuffer, byteread);
			
			if (serialBuffer != ACK_start) {
				cout << "missing starting signal" << endl;
				return false;
			}

			return_byte = _arduino_interface.read(&serialBuffer, byteread);
			is_ToggleUp = 0xFF & serialBuffer;
			return_byte = _arduino_interface.read(&serialBuffer, byteread);
			
			if (serialBuffer != ACK_end) {
				cout << "missing ending signal" << endl;
				return false;
			 }

			return is_ToggleUp;
		}

		uint8_t interfaceResponse(serial::Serial& _arduino_interface) {

			uint8_t ret = 0;
			request = virtualAdmittance_select;
			return_byte = _arduino_interface.write(&request, bytesend);
			return_byte = _arduino_interface.read(&serialBuffer, byteread);

			if (serialBuffer != ACK_start) {
				return 0;
			}
			return_byte = _arduino_interface.read(&serialBuffer, byteread);
			ret = 0xFF & serialBuffer;
			return_byte = _arduino_interface.read(&serialBuffer, byteread);
			if (serialBuffer != ACK_end) {
				return 0;
			}
			return ret;
		}

	};

	class SENSORACQUISITION {
	private:

		uint8_t request;
		size_t return_byte;
		const size_t byteread = 1;
		const size_t bytesend = 1;
		uint8_t serialBuffer = 0;
		int32_t readBuffer[4] = { 0, 0, 0, 0 };
		int counter = 0;
		int32_t read_force[4] = { 0, 0, 0, 0 };
		double scale = 45000;
		double deadZone[3] = { 0.1, 0.1, 0.05 };
		bool read_touch = false;
		bool is_open = false;

	public:

		bool isOpen(serial::Serial& _arduino_sensorAcquisition) {

			request = request_isOpen;
			return_byte = _arduino_sensorAcquisition.write(&request, bytesend);
			return_byte = _arduino_sensorAcquisition.read(&serialBuffer, byteread);
			counter = 0;
			while (serialBuffer != ACK_start) {
				cout << "missing starting signal" << endl;
				return_byte = _arduino_sensorAcquisition.read(&serialBuffer, byteread);
				counter++;
				if (counter > 10) {
					cout << "missing starting signal" << endl;
					return false;
				}
			}
			is_open = false;
			return_byte = _arduino_sensorAcquisition.read(&serialBuffer, byteread);
			is_open = 0xFF & serialBuffer;
			return_byte = _arduino_sensorAcquisition.read(&serialBuffer, byteread);

			while (serialBuffer != ACK_end) {
				cout << "missing ending signal" << endl;
				return_byte = _arduino_sensorAcquisition.read(&serialBuffer, byteread);
				counter++;
				if (counter > 10) {
					cout << "missing starting signal" << endl;
					return false;
				}
			}

			return is_open;
		}

		bool force_calibration(serial::Serial& _arduino_sensorAcquisition) {

			request = request_calibration;
			return_byte = _arduino_sensorAcquisition.write(&request, bytesend);
			return_byte = _arduino_sensorAcquisition.read(&serialBuffer, byteread);
			counter = 0;
			while (serialBuffer != ACK_start) {
				cout << "missing starting signal" << endl;
				return_byte = _arduino_sensorAcquisition.read(&serialBuffer, byteread);
				counter++;
				if (counter > 10) {
					cout << "missing starting signal" << endl;
					return false;
				}
			}

			cout << "start calibration" << endl;
			return_byte = _arduino_sensorAcquisition.read(&serialBuffer, byteread);

			if (serialBuffer != ACK_end) {

				cout << "missing ending signal" << endl;
				return_byte = _arduino_sensorAcquisition.read(&serialBuffer, byteread);
				return false;
			}

			cout << "done calibration" << endl;
			return true;
		}

		bool forceRead(serial::Serial& _arduino_sensorAcquisition) {

			request = request_force;
			return_byte = _arduino_sensorAcquisition.write(&request, bytesend);
			return_byte = _arduino_sensorAcquisition.read(&serialBuffer, byteread);

			counter = 0;
			while (serialBuffer != ACK_start) {
				cout << serialBuffer << endl;
				return_byte = _arduino_sensorAcquisition.read(&serialBuffer, byteread);
				counter++;

				if (counter > 20) {
					counter = 0;
					return false;
				}
			}

			for (int i = 0; i < 4; i++) {
				readBuffer[i] = 0;
				for (int j = 0; j < 4; j++) {
					return_byte = _arduino_sensorAcquisition.read(&serialBuffer, byteread);
					readBuffer[i] |= (0x000000FF & serialBuffer) << (j * 8);
				}
			}

			return_byte = _arduino_sensorAcquisition.read(&serialBuffer, byteread);

			if (serialBuffer == ACK_end) {
				for (int i = 0; i < 4; i++) {
					read_force[i] = readBuffer[i];
				}
			}

			else {
				cout << "abandon buffer" << endl;
				return false;
			}

			return true;
		}

		void forceMapping(double* _mappedForce) {

			*(_mappedForce + 0) = ((double)read_force[2] - (double)read_force[1]) / (2.0 * scale);
			*(_mappedForce + 2) = ((double)read_force[2] + (double)read_force[1]) * 0.4 / scale;
			*(_mappedForce + 1) = ((-(double)read_force[3] - (double)read_force[0]) / (2.0 * scale)) + 0.26 * *(_mappedForce + 2);

			for (int i = 0; i < 3; i++) {

				if (abs(*(_mappedForce + i)) < deadZone[i]) {
					*(_mappedForce + i) = 0.0;
				}

			}

		}

		bool isTouch(serial::Serial& _arduino_sensorAcquisition) {
			request = request_touch;
			return_byte = _arduino_sensorAcquisition.write(&request, bytesend);
			return_byte = _arduino_sensorAcquisition.read(&serialBuffer, byteread);

			if (serialBuffer != ACK_start) {
				cout << "missing starting signal" << endl;
			}

			return_byte = _arduino_sensorAcquisition.read(&serialBuffer, byteread);
			read_touch = 0xFF & serialBuffer;
			return_byte = _arduino_sensorAcquisition.read(&serialBuffer, byteread);

			if (serialBuffer != ACK_end) {
				read_touch = false;
				cout << "missing ending signal" << endl;
			}
			return read_touch;
		}

		bool checkTouch(serial::Serial& _arduino_sensorAcquisition, SENSORACQUISITION _sensorAcquisition, ACTIVECASTER::TIME _touchTime, double _timeout) {

			read_touch = _sensorAcquisition.isTouch(_arduino_sensorAcquisition);

			if (read_touch) {
				_touchTime.setTimeFlag();
				return true;
			}

			else if (!read_touch && _touchTime.sinceFlag() < _timeout) {
				return true;
			}

			else {
				return false;
			}
		}
	};

	class VIRTUALADMITTANCE {

	private:
		
		double innertia[3] = { 0.0, 1.0, 1.0 };
		double damping[3] = { 1.0, 1.0, 1.0 };
		double output_mag[3] = { 0.0, 0.0, 0.0 };
		double deadZone[3] = { 0.01, 0.01, 0.01 };
		double saturate[3] = { 2.0, 2.0, 2.0 };

	public:

		void getVelAdmittance(double _dT, double* _mappedForce, double* _velRef_Robot) {

			for (int i = 0; i < 3; i++) {

				_velRef_Robot[i] = (_dT * (_mappedForce[i] - (damping[i] * _velRef_Robot[i])) / innertia[i]) + _velRef_Robot[i];
				output_mag[i] = abs(_velRef_Robot[i]);

				if (output_mag[i] > saturate[i]) {

					_velRef_Robot[i] = saturate[i] * _velRef_Robot[i] / output_mag[i];
					  
				}

				else {
				}

			}

		}

		void getVelAdmittance_brake(double _dT, double* _velRef_Robot) {

			for (int i = 0; i < 3; i++) {

				_velRef_Robot[0] = - (damping[i] * _velRef_Robot[i]) / innertia[i] + _velRef_Robot[i];
				output_mag[i] = abs(_velRef_Robot[i]);

				if (output_mag[i] > saturate[i]) {

					_velRef_Robot[i] = saturate[i] * _velRef_Robot[i] / output_mag[i];

				}

				else if (output_mag < deadZone) {

					_velRef_Robot[i] = 0.0;

				}

				else {
				}

			}

		}
		void setVirtualAdmittanceParam(double *_innertia, double* _damping ) {
			
			for (int i = 0; i < 3; i++) {
				innertia[i] = _innertia[i];
				damping[i] = _damping[i];
			}
		}

		void setOutputConstraint(double* _saturate, double* _deadZone, double* _velRef_Robot) {

			for (int i = 0; i < 3; i++) {
				saturate[i] = _saturate[i];
				deadZone[i] = _deadZone[i];
				_velRef_Robot[i] = 0.0;
			}
		}

	};

// 	class KINEMATIC {

// 	private:

// 		Eigen::Matrix<double, 6, 3> jacobian;
// 		Eigen::Matrix<double, 3, 6> jacobianTranspose;
// 		Eigen::Matrix<double, 3, 3> inverseBuffer;
// 		Eigen::Matrix<double, 3, 6> pseudoInverseJacobian;

// 		const double pi = 3.14159265358979323846;
// 		const double alpha[3] = { 0, 2 * pi / 3, 4 * pi / 3 };
// 		const double l = 0.3;
// 		const double d = 0.044252;
// 		const double d_i = 1.00 / d;
// 		const double r = 0.100;
// 		const double r_i = 1.00 / r;
// 		const double gear_Motor_to_Steering = 1.5;
// 		const double gear_Motor_to_Wheel = (40*60)/(18*18);
// 		const double steering_coupling = -40/18;
// 		const double encoderResolution = 4098;
// 		bool invertible = true;

// 		GangliaControl::GangliaState stateBuffer;


// 	public:

// 		double beta[3] = { 0.0, 0.0, 0.0 };
// 		double posMes_Motor[6] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
// 		double velRef_RPM[6] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
// 		double velRef_Joint[6] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
// 		double velRef_Robot[3] = { 0.0, 0.0, 0.0 };
// 		double velMes_RPM[6] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
// 		double velMes_Joint[6] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
// 		double velMes_Robot[3] = { 0.0, 0.0, 0.0 };


// 		void clearValue() {
// 			for (int i = 0; i < 6; i++) {
// 				velRef_RPM[i] = 0;
// 				velRef_Joint[i] = 0;
// 				velMes_RPM[i] = 0;
// 				velMes_Joint[i] = 0;
// 			}

// 			for (int i = 0; i < 3; i++) {
// 				velRef_Robot[i] = 0;
// 				velMes_Robot[i] = 0;
// 			}

// 		}

// 		bool setCurrentState(uint8_t* _id, GangliaControl& _ganglia) {
// 			bool ret = true;
			
// 			for (int i = 0; i < 6; i++) {

// 				ret = ret && _ganglia.getState(*(_id + i), &stateBuffer, false);
// 				velMes_RPM[i] = stateBuffer.velocity;
// 				posMes_Motor[i] = stateBuffer.encoder;
				
// 			}
// 			for (int i = 0; i < 3; i++) {
				
// 				velMes_Joint[i] = (velMes_RPM[i] - steering_coupling * velMes_RPM[i + 3]) / gear_Motor_to_Wheel;
// 				velMes_Joint[i + 3] = velMes_RPM[i + 3] / gear_Motor_to_Steering;
// 				beta[i] = 2 * pi * posMes_Motor[i] / 16384;

// 			}

// 			return ret;
			 
// 		}

// 		bool setJacobian() {

// 			for (int i = 0; i < 3; i++) {

// 				jacobian(i, 0) = -sin(alpha[i] + beta[i]) * r_i;
// 				jacobian(i, 1) = cos(alpha[i] + beta[i]) * r_i;
// 				jacobian(i, 2) = l * cos(beta[i]) * r_i;
// 				jacobian(i + 3, 0) = cos(alpha[i] + beta[i]) * d_i;
// 				jacobian(i + 3, 1) = sin(alpha[i] + beta[i]) * d_i;
// 				jacobian(i + 3, 2) = (d + l * sin(beta[i])) * d_i;

// 			}

// 			jacobianTranspose = jacobian.transpose();
// 			inverseBuffer = jacobianTranspose * jacobian;

// 			inverseBuffer.computeInverseWithCheck(inverseBuffer, invertible);
// 			if (invertible == true) {

// 				pseudoInverseJacobian = inverseBuffer * jacobianTranspose;
// 			}
// 			else {
// 				return false;
// 			}
// 			return invertible;
// 		}

// 		void computeVelRefRPM() {

// 			for (int i = 0; i < 3; i++) {

// 				velRef_Joint[i] = ((jacobian(i, 0) * velRef_Robot[0] + jacobian(i, 1) * velRef_Robot[1] + jacobian(i, 2) * velRef_Robot[2]));
// 				velRef_Joint[i + 3] = ((jacobian(i + 3, 0) * velRef_Robot[0] + jacobian(i + 3, 1) * velRef_Robot[1] + jacobian(i + 3, 2) * velRef_Robot[2]));

// 				velRef_RPM[i + 3] = gear_Motor_to_Steering * velRef_Joint[i + 3];
// 				velRef_RPM[i] =  (gear_Motor_to_Wheel * velRef_Joint[i] + steering_coupling * velRef_Joint[i + 3]);

// 			}
// 		}

// 		void computeVelMes_Robot() {

// 			for (int i = 0; i < 3; i++) {

// 				velMes_Robot[i] = velMes_Joint[0] * pseudoInverseJacobian(i, 0) + velMes_Joint[1] * pseudoInverseJacobian(i, 1)
// 					+ velMes_Joint[2] * pseudoInverseJacobian(i, 2) + velMes_Joint[3] * pseudoInverseJacobian(i, 3)
// 					+ velMes_Joint[4] * pseudoInverseJacobian(i, 4) + velMes_Joint[5] * pseudoInverseJacobian(i, 5);
// 			}

// 		}

// 		bool setVel_Ganglia(uint8_t* _id, GangliaControl& _ganglia) {
// 			bool ret = true;
// 			for (int i = 0; i < 6; i++) {

// 				ret = ret && _ganglia.setVelocityTarget(*(_id + 1), (int32_t)(velRef_RPM[i]));

// 			}
// 			return ret;
// 		}

// 		bool hold(KINEMATIC _kine, uint8_t* _id, GangliaControl& _ganglia) {
// 			_kine.clearValue();
// 			return _kine.setVel_Ganglia(_id, _ganglia);
// 		}

// 		bool checkDeacceralate() {
// 			bool ret = true;
// 			if (abs(velMes_Robot[0]) < 0.1) {
// 				ret = ret && true;
// 			}
// 			else {
// 				ret = false;
// 			}
			
// 			if (abs(velMes_Robot[1]) < 0.1) {
// 				ret = ret && true;
// 			}
// 			else {
// 				ret = false;
// 			}

// 			if (abs(velMes_Robot[2]) < 0.3) {
// 				ret = ret && true;
// 			}
// 			else {
// 				ret = false;
// 			}

// 			return ret;
// 		}

// 		void setVelRef_Robot_Deaccerelate(double _dT, double _acc_linear, double _acc_rotation) {
			
// 			if (velRef_Robot[0] < -0.1) {
// 				velRef_Robot[0] = velRef_Robot[1] + _acc_linear * _dT;
// 			}
// 			else if (velRef_Robot[1] > 0.1) {
// 				velRef_Robot[0] = velRef_Robot[1] - _acc_linear * _dT;
// 			}
// 			else {
// 				velRef_Robot[0] = 0;
// 			}

// 			if (velRef_Robot[1] < -0.1) {
// 				velRef_Robot[1] = velRef_Robot[2] + _acc_linear * _dT;
// 			}
// 			else if (velRef_Robot[1] > 0.1) {
// 				velRef_Robot[1] = velRef_Robot[2] - _acc_linear * _dT;
// 			}
// 			else {
// 				velRef_Robot[1] = 0;
// 			}

// 			if (velRef_Robot[2] < -0.3) {
// 				velRef_Robot[2] = velRef_Robot[3] + _acc_rotation * _dT;
// 			}
// 			else if (velRef_Robot[2] > 0.3) {
// 				velRef_Robot[2] = velRef_Robot[3] - _acc_rotation * _dT;
// 			}
// 			else {
// 				velRef_Robot[2] = 0;
// 			}
// 		}

// 	};

// 	class OUTERPID {

// 	private:

// 		double Mes_error[3] = { 0, 0, 0, };
// 		double last_Mes_error[3] = { 0, 0, 0, };
// 		double integral_value[3] = { 0, 0, 0 };
// 		double derivative_value[3] = { 0, 0, 0 };

// 		double P_value[3] = { 1, 1, 1 };
// 		double I_value[3] = { 1, 1, 1 };
// 		double D_value[3] = { 0, 0, 0 };

// 	public:

// 		void clearValue() {
// 			for (int i = 0; i < 3; i++) {
// 				Mes_error[i] = 0;
// 				last_Mes_error[i] = 0;
// 				integral_value[i] = 0;
// 				derivative_value[i] = 0;
// 			}
// 		}

// 		void set_value(double* _P_value, double* _I_value, double* _D_value) {

// 			for (int i = 0; i < 3; i++) {

// 				P_value[i] = *(_P_value + i);
// 				I_value[i] = *(_I_value + i);
// 				D_value[i] = *(_D_value + i);

// 			}
// 		}

// 		void PIDCompensator(double* _velRef_Robot, double* _velMes_Robot, double _dT) {

// 			for (int i = 0; i < 3; i++) {

// 				last_Mes_error[i] = Mes_error[i];
// 				Mes_error[i] = _velRef_Robot[i] - _velMes_Robot[i];
// 				integral_value[i] = integral_value[i] + Mes_error[i] * _dT;
// 				derivative_value[i] = (Mes_error[i] - last_Mes_error[i]) / _dT;
// 				_velRef_Robot[i] = _velRef_Robot[i] + P_value[i] * Mes_error[i] + I_value[i] * integral_value[i] + D_value[i] * derivative_value[i];

// 			}
// 		}

// 	};

}

#endif
