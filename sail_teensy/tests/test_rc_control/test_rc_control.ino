/*
File contains functions/etc to interpret RC input.
Currently, just has the switchOn and within functions.
-Matt
*/

int MID_SIGNAL = 1500 //Placeholder.

bool switchOn(int switchPin) {
//Returns true if the pulse_length of the switchPin is greater than predefined MID_SIGNAL.
	int pulse_length = pulseIn(switchPin, HIGH);
	if (pulse_length > MID_SIGNAL) {
		return true;
	}
	else {
		return false;
	}
}


bool within(int value, int band_center, int band_radius) {
//Returns true if value is within band_radius of band_center; false otherwise.
	if ((band_center + band_radius > value) && (band_center - band_radius < value)) {
		return true;
	}
	else {
		return false;
	}
}
