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
//#include "obo_ganglia_driver_lib/ganglia_control.h"
//#include <Eigen/Dense>

//using namespace Eigen;
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
			cout << "Write Data ->>" << request <<"" << endl;

			return_byte = _arduino_sensorAcquisition.write(&request, bytesend);
			return_byte = _arduino_sensorAcquisition.read(&serialBuffer, byteread);
			counter = 0;
			while (serialBuffer != ACK_start) {
				cout << "missing starting signal" << endl;
				return_byte = _arduino_sensorAcquisition.write(&request, bytesend);
				cout << "Write Data ->>" << request <<"" << endl;
				return_byte = _arduino_sensorAcquisition.read(&serialBuffer, byteread);
				cout << "Read" << return_byte <<" Byte" << endl;
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
			std::cout << "Force Calibration _Serial" << std::endl;
			request = request_calibration;
			return_byte = _arduino_sensorAcquisition.write(&request, bytesend);
			cout << "Write " << return_byte <<" Byte" << endl;
			return_byte = _arduino_sensorAcquisition.read(&serialBuffer, byteread);
			counter = 0;
			while (serialBuffer != ACK_start) {
				cout << "missing starting signal" << endl;
				return_byte = _arduino_sensorAcquisition.read(&serialBuffer, byteread);
				counter++;
				if (counter > 20) {
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
			*(_mappedForce + 1) = (((double)read_force[3] - (double)read_force[0]) / (2.0 * scale)) + 0.26 * *(_mappedForce + 2);

			for (int i = 0; i < 3; i++) {

				if (*(_mappedForce + i) < deadZone[i]) {
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

		double innertia[3] = { 1.0, 1.0, 1.0 };
		double damping[3] = { 1.0, 1.0, 1.0 };
		double output_mag[3] = { 0.0, 0.0, 0.0 };
		double deadZone[3] = { 0.01, 0.01, 0.01 };
		double saturate[3] = { 2.0, 2.0, 2.0 };

	public:

		void getVelAdmittance(double _dT, double* _mappedForce, double* _velRef_Robot) {

			for (int i = 0; i < 3; i++) {

				_velRef_Robot[0] = (_dT * (_mappedForce[i] - (damping[i] * _velRef_Robot[i])) / innertia[i]) + _velRef_Robot[i];
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

}

#endif