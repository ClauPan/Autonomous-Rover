#ifndef SKID_STEER_CONTROLLER_MOTOR_HPP
#define SKID_STEER_CONTROLLER_MOTOR_HPP

#include <string>
#include <cmath>
#include <iostream>

class Motor {

public:
	int enc = 0;
	int enc_last = 0;
	double enc_fw = 0;
	double enc_ba = 0;
	double cmd = 0;
	double pos = 0;
	double vel = 0;

	Motor() = default;

	void setup(double enc_fw_, double enc_ba_) {
		enc_fw = enc_fw_;
		enc_ba = enc_ba_;
	}

	double get_encoder_angle() {
		if (enc >= enc_last) {
			enc_last = enc;
			return enc * enc_fw;
		}
		else {
			enc_last = enc;
			return enc * enc_ba;
		}
	}

	double get_counts_per_loop() {
		if (cmd > 0) {
			return cmd / enc_fw / 30;
		}
		else if (cmd < 0) {
			return cmd / enc_ba / 30;
		}
		else {
			return 0;
		}
	}

};


#endif
