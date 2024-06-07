#ifndef DIFFDRIVE_ARDUINO_WHEEL_HPP
#define DIFFDRIVE_ARDUINO_WHEEL_HPP

#include <string>
#include <cmath>
#include <iostream>

class Wheel {
public:

   	std::string name = "";
    	int enc = 0;
    	int enc_last = 0;
    	double cmd = 0;
    	double pos = 0;
    	double vel = 0;
    	double rads_per_count_fw = 0;
    	double rads_per_count_ba = 0;

    	Wheel() = default;

    	Wheel(const std::string &wheel_name, int counts_per_rev_fw, int counts_per_rev_ba) {
      		setup(wheel_name, counts_per_rev_fw, counts_per_rev_ba);
    	}


    	void setup(const std::string &wheel_name, int counts_per_rev_fw, int counts_per_rev_ba) {
      		name = wheel_name;
      		rads_per_count_fw = (2*M_PI)/counts_per_rev_fw;
      		rads_per_count_ba = (2*M_PI)/counts_per_rev_ba;
    	}

    	double calc_enc_angle() {
		double enc_angle = 0.0;
      		if (enc >= enc_last) {
			enc_angle = enc * rads_per_count_fw;
		}
		else if (enc < enc_last) {
			enc_angle = enc * rads_per_count_ba;
		}
		enc_last = enc;
		return enc_angle;
    	}

	double calc_counts_per_loop(float loop_rate) {
		double counts_per_loop = 0.0;
		if (cmd > 0) {
			counts_per_loop = cmd / rads_per_count_fw / loop_rate;
		}
		else if (cmd < 0) {
			counts_per_loop = cmd / rads_per_count_ba / loop_rate;
		}
		return counts_per_loop;
	}

};


#endif // DIFFDRIVE_ARDUINO_WHEEL_HPP
