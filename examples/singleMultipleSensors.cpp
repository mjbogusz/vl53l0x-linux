#include <VL53L0X.h>

#include <chrono>
#include <csignal>
#include <iomanip>
#include <iostream>
#include <unistd.h>
#include <wiringPi.h>

volatile sig_atomic_t exitFlag = 0;
void sigintHandler(int) {
	exitFlag = 1;
}

int main() {
	// Configuration constants
	// Number of sensors. If changed, make sure to adjust pins and addresses accordingly (ie to match size).
	const int SENSOR_COUNT = 6;
	// GPIO pins to use for sensors' XSHUT. As exported by WiringPi.
	const uint8_t pins[SENSOR_COUNT] = { 0, 1, 2, 3, 4, 5 };
	// Sensors' addresses that will be set and used. These have to be unique.
	const uint8_t addresses[SENSOR_COUNT] = {
		VL53L0X_ADDRESS_DEFAULT + 2,
		VL53L0X_ADDRESS_DEFAULT + 4,
		VL53L0X_ADDRESS_DEFAULT + 6,
		VL53L0X_ADDRESS_DEFAULT + 10,
		VL53L0X_ADDRESS_DEFAULT + 12,
		VL53L0X_ADDRESS_DEFAULT + 14
	};

	// Register SIGINT handler
	signal(SIGINT, sigintHandler);

	// Initialize GPIO connectivity
	wiringPiSetup();

	// Create sensor objects' array
	VL53L0X* sensors[SENSOR_COUNT];

	// Create sensors (and ensure GPIO pin mode)
	for (int i = 0; !exitFlag && i < SENSOR_COUNT; ++i) {
		pinMode(pins[i], OUTPUT);
		sensors[i] = new VL53L0X(pins[i]);
		sensors[i]->powerOff();
	}
	if (exitFlag) {
		return 0;
	}

	// For each sensor: create object, init the sensor (ensures power on), set timeout and address
	// Note: don't power off - it will reset the address to default!
	for (int i = 0; !exitFlag && i < SENSOR_COUNT; ++i) {
		// Create...
		// ...init...
		sensors[i]->init();
		// ...set timeout...
		sensors[i]->setTimeout(200);
		// ...and set address
		sensors[i]->setAddress(addresses[i]);
		// Also, notify user.
		std::cout << "Sensor " << i << " initialized\n";
	}

	// Durations in nanoseconds
	uint64_t totalDuration = 0;
	uint64_t maxDuration = 0;
	uint64_t minDuration = 1000*1000*1000;
	// Initialize reference time measurement
	std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

	int j = 0;
	for (; !exitFlag && j < 100000; ++j) {
		std::cout << "\rReading" << std::setw(4) << std::setfill('0') << j << " | ";
		for (int i = 0; !exitFlag && i < SENSOR_COUNT; ++i) {
			uint16_t distance;
			try {
				distance = sensors[i]->readRangeSingleMillimeters();
			} catch (std::string & err) {
				std::cerr << err;
				return 1;
			}

			if (sensors[i]->timeoutOccurred()) {
				std::cout << "\ntimeout: " << i << std::endl;
			} else {
				std::cout << std::setw(4) << distance << " | ";
			}
		}
		std::cout << std::flush;

		// Calculate duration of current iteration
		std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
		long duration = (std::chrono::duration_cast<std::chrono::nanoseconds>(t2 - t1)).count();
		// Save current time as reference for next iteration
		t1 = t2;
		// Add total measurements duration
		totalDuration += duration;
		// Skip comparing first measurement against max and min as it's not a full iteration
		if (j == 0) {
			continue;
		}
		// Check and save max and min iteration duration
		if (duration > maxDuration) {
			maxDuration = duration;
		}
		if (duration < minDuration) {
			minDuration = duration;
		}
	}

	// Print duration data
	std::cout << "\nMax duration: " << maxDuration << "ns" << std::endl;
	std::cout << "Min duration: " << minDuration << "ns" << std::endl;
	std::cout << "Avg duration: " << totalDuration/(j+1) << "ns" << std::endl;
	std::cout << "Avg frequency: " << 1000000000/(totalDuration/(j+1)) << "Hz" << std::endl;

	// Clean-up: delete objects, set GPIO/XSHUT pins to low.
	for (int i = 0; i < SENSOR_COUNT; ++i) {
		delete sensors[i];
		digitalWrite(pins[i], LOW);
	}
	return 0;
}
