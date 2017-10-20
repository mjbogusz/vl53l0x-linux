/* This example shows how to get single-shot range
 measurements from the VL53L0X. The sensor can optionally be
 configured with different ranging profiles, as described in
 the VL53L0X API user manual, to get better performance for
 a certain application. This code is based on "Single.ino"
 example from vl53l0x-arduino library, which in turn is based
 on the four "SingleRanging" examples in the VL53L0X API.

 The range readings are in units of mm. */

#include <chrono>
#include <csignal>
#include <iomanip>
#include <iostream>

#include <VL53L0X.h>

volatile sig_atomic_t exitFlag = 0;

void sigintHandler(int) {
    exitFlag = 1;
}

// Uncomment to enable long range measurements
// #define LONG_RANGE
// Uncomment ONE to enable high speed or high accuracy measurements
// #define HIGH_SPEED
// #define HIGH_ACCURACY

int main() {
    // Register SIGINT handler
    signal(SIGINT, sigintHandler);

    // Create and initialize the sensor
    VL53L0X sensor;
    try {
        sensor.init();
        sensor.setTimeout(200);
    } catch (std::string &err) {
        std::cerr << err;
        return 1;
    }

#ifdef LONG_RANGE
    // Lower the return signal rate limit (default is 0.25 MCPS)
    sensor.setSignalRateLimit(0.1);
    // Increase laser pulse periods (defaults are 14 and 10 PCLKs)
    sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
    sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
#endif

#if defined HIGH_SPEED
    // Reduce timing budget to 20 ms (default is about 33 ms)
    sensor.setMeasurementTimingBudget(20000);
#elif defined HIGH_ACCURACY
    // Increase timing budget to 200 ms
    sensor.setMeasurementTimingBudget(200000);
#endif

    // Highly unprobable but check SIGINT exit flag
    if (exitFlag) {
        return 0;
    }

    // Durations in nanoseconds
    uint64_t totalDuration = 0;
    uint64_t maxDuration = 0;
    uint64_t minDuration = 1000 * 1000 * 1000;
    // Initialize reference time measurement
    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

    int i = 0;
    for (; !exitFlag && i < 100000; ++i) {
        // Print iteration number
        std::cout << "\rReading" << std::setw(4) << std::setfill('0') << i << " | ";

        // Do the actual ranging
        uint16_t distance = sensor.readRangeSingleMillimeters();
        // Check IO timeout and print range information
        if (sensor.timeoutOccurred()) {
            std::cout << "timeout!       " << std::endl;
        } else {
            std::cout << std::setw(4) << distance;
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
        if (i == 0) {
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
    std::cout << "Avg duration: " << totalDuration / (i + 1) << "ns" << std::endl;
    std::cout << "Avg frequency: " << 1000000000 / (totalDuration / (i + 1)) << "Hz" << std::endl;

    return 0;
}
