#ifndef SKID_STEER_CONTROLLER_ARDUINO_HPP
#define SKID_STEER_CONTROLLER_ARDUINO_HPP

#include <sstream>
#include <libserial/SerialPort.h>
#include <iostream>


class Arduino {

public:
	Arduino() = default;

	void connect() {  
		serial_conn_.Open("/dev/ttyUSB0");
		serial_conn_.SetBaudRate(LibSerial::BaudRate::BAUD_57600);
	}

	void disconnect() {
		serial_conn_.Close();
	}

	bool connected() const {
		return serial_conn_.IsOpen();
	}

	std::string send(const std::string &msg) {
		try {
			std::string response = "";
			serial_conn_.FlushIOBuffers(); 
			serial_conn_.Write(msg);
			serial_conn_.ReadLine(response, '\n', 1000);
			return response;
		}
		catch (const LibSerial::ReadTimeout&) {
			std::cerr << "Communication timeout\n";
		}
	}

	void read(int &motor_l, int &motor_r) {
		std::string response = send_msg("e\r");

		size_t del_pos = response.find(" ");
		motor_l = std::atoi(response.substr(0, del_pos).c_str());
		motor_r = std::atoi(response.substr(del_pos + 1).c_str());
	}
	
	void set_motors(int motor_l, int motor_r) {
		std::stringstream ss;
		ss << "m " << motor_l << " " << motor_l << "\r";
		send(ss.str());
	}

private:
	LibSerial::SerialPort serial_conn_;
};

#endif