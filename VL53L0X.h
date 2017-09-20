/**
 * Most of the functionality of this library is based on the VL53L0X API provided by ST (STSW-IMG005)
 * and some of the explanatory comments are quoted or paraphrased from the API source code, API user manual (UM2039), and the VL53L0X datasheet.
 **/

#ifndef _VL53L0X_H
#define _VL53L0X_H

#include <cstdint>
#include "../LinuxDuino/LinuxDuino.h"

//#define VL53L0X_ADDRESS_DEFAULT 0b0101001
#define VL53L0X_ADDRESS_DEFAULT 0x29

class VL53L0X {
	public:
		/*** Public fields ***/

		/**
		 * Status of last I2C transmission
		 */
		uint8_t lastStatus;

		/*** Public enums, structures and subclasses ***/

		/**
		 * Register addresses from API vl53l0x_device.h (ordered as listed there)
		 */
		enum REGISTER_ADDRESSES {
			SYSRANGE_START = 0x00,

			SYSTEM_THRESH_HIGH = 0x0C,
			SYSTEM_THRESH_LOW = 0x0E,

			SYSTEM_SEQUENCE_CONFIG = 0x01,
			SYSTEM_RANGE_CONFIG = 0x09,
			SYSTEM_INTERMEASUREMENT_PERIOD = 0x04,

			SYSTEM_INTERRUPT_CONFIG_GPIO = 0x0A,

			GPIO_HV_MUX_ACTIVE_HIGH = 0x84,

			SYSTEM_INTERRUPT_CLEAR = 0x0B,

			RESULT_INTERRUPT_STATUS = 0x13,
			RESULT_RANGE_STATUS = 0x14,

			RESULT_CORE_AMBIENT_WINDOW_EVENTS_RTN = 0xBC,
			RESULT_CORE_RANGING_TOTAL_EVENTS_RTN = 0xC0,
			RESULT_CORE_AMBIENT_WINDOW_EVENTS_REF = 0xD0,
			RESULT_CORE_RANGING_TOTAL_EVENTS_REF = 0xD4,
			RESULT_PEAK_SIGNAL_RATE_REF = 0xB6,

			ALGO_PART_TO_PART_RANGE_OFFSET_MM = 0x28,

			I2C_SLAVE_DEVICE_ADDRESS = 0x8A,

			MSRC_CONFIG_CONTROL = 0x60,

			PRE_RANGE_CONFIG_MIN_SNR = 0x27,
			PRE_RANGE_CONFIG_VALID_PHASE_LOW = 0x56,
			PRE_RANGE_CONFIG_VALID_PHASE_HIGH = 0x57,
			PRE_RANGE_MIN_COUNT_RATE_RTN_LIMIT = 0x64,

			FINAL_RANGE_CONFIG_MIN_SNR = 0x67,
			FINAL_RANGE_CONFIG_VALID_PHASE_LOW = 0x47,
			FINAL_RANGE_CONFIG_VALID_PHASE_HIGH = 0x48,
			FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT = 0x44,

			PRE_RANGE_CONFIG_SIGMA_THRESH_HI = 0x61,
			PRE_RANGE_CONFIG_SIGMA_THRESH_LO = 0x62,

			PRE_RANGE_CONFIG_VCSEL_PERIOD = 0x50,
			PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI = 0x51,
			PRE_RANGE_CONFIG_TIMEOUT_MACROP_LO = 0x52,

			SYSTEM_HISTOGRAM_BIN = 0x81,
			HISTOGRAM_CONFIG_INITIAL_PHASE_SELECT = 0x33,
			HISTOGRAM_CONFIG_READOUT_CTRL = 0x55,

			FINAL_RANGE_CONFIG_VCSEL_PERIOD = 0x70,
			FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI = 0x71,
			FINAL_RANGE_CONFIG_TIMEOUT_MACROP_LO = 0x72,
			CROSSTALK_COMPENSATION_PEAK_RATE_MCPS = 0x20,

			MSRC_CONFIG_TIMEOUT_MACROP = 0x46,

			SOFT_RESET_GO2_SOFT_RESET_N = 0xBF,
			IDENTIFICATION_MODEL_ID = 0xC0,
			IDENTIFICATION_REVISION_ID = 0xC2,

			OSC_CALIBRATE_VAL = 0xF8,

			GLOBAL_CONFIG_VCSEL_WIDTH = 0x32,
			GLOBAL_CONFIG_SPAD_ENABLES_REF_0 = 0xB0,
			GLOBAL_CONFIG_SPAD_ENABLES_REF_1 = 0xB1,
			GLOBAL_CONFIG_SPAD_ENABLES_REF_2 = 0xB2,
			GLOBAL_CONFIG_SPAD_ENABLES_REF_3 = 0xB3,
			GLOBAL_CONFIG_SPAD_ENABLES_REF_4 = 0xB4,
			GLOBAL_CONFIG_SPAD_ENABLES_REF_5 = 0xB5,

			GLOBAL_CONFIG_REF_EN_START_SELECT = 0xB6,
			DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD = 0x4E,
			DYNAMIC_SPAD_REF_EN_START_OFFSET = 0x4F,
			POWER_MANAGEMENT_GO1_POWER_FORCE = 0x80,

			VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV = 0x89,

			ALGO_PHASECAL_LIM = 0x30,
			ALGO_PHASECAL_CONFIG_TIMEOUT = 0x30,
		};

		enum vcselPeriodType {
			VcselPeriodPreRange,
			VcselPeriodFinalRange
		};

		/*** Constructors and destructors ***/

		VL53L0X(const int16_t xshutGPIOPin = -1, const uint8_t address = VL53L0X_ADDRESS_DEFAULT);

		/*** Public methods ***/

		/**
		 * \brief Initialize sensor using sequence Based on VL53L0X_DataInit(), VL53L0X_StaticInit(), and VL53L0X_PerformRefCalibration().
		 *
		 * This function does not perform reference SPAD calibration (VL53L0X_PerformRefSpadManagement()),
		 * since the API user manual says that it is performed by ST on the bare modules;
		 * It seems like that should work well enough unless a cover glass is added.
		 * If ioMode2v8 (optional) is true or not given, the sensor is configured for 2V8 mode.
		 *
		 * \return true if initialization succeeded.
		 */
		bool init(bool ioMode2v8 = true);

		void powerOn();
		void powerOff();

		/**
		 * Write an 8-bit register.
		 */
		void writeRegister(uint8_t register, uint8_t value);
		/**
		 * Write a 16-bit register.
		 */
		void writeRegister16Bit(uint8_t register, uint16_t value);
		/**
		 * Write a 32-bit register.
		 *
		 * Based on VL53L0X_write_dword from VL53L0X kernel driver.
		 */
		void writeRegister32Bit(uint8_t register, uint32_t value);
		/**
		 * Write an arbitrary number of bytes from the given array to the sensor, starting at the given register.
		 */
		void writeRegisterMultiple(uint8_t register, const uint8_t* source, uint8_t count);
		/**
		 * Read an 8-bit register.
		 */
		uint8_t readRegister(uint8_t register);
		/**
		 * Read a 16-bit register.
		 */
		uint16_t readRegister16Bit(uint8_t register);
		/**
		 * Read a 32-bit register.
		 */
		uint32_t readRegister32Bit(uint8_t register);
		/**
		 * Read an arbitrary number of bytes from the sensor, starting at the given register, into the given array.
		 */
		void readRegisterMultiple(uint8_t register, uint8_t* destination, uint8_t count);

		void setAddress(uint8_t newAddress);
		inline uint8_t getAddress() {
			return this->address;
		}
		/**
		 * Set the return signal rate limit check value in units of MCPS (mega counts per second).
		 *
		 * "This represents the amplitude of the signal reflected from the target and detected by the device";
		 * setting this limit presumably determines the minimum measurement necessary for the sensor to report a valid reading.
		 * Setting a lower limit increases the potential range of the sensor but also seems to increase the likelihood
		 * of getting an inaccurate reading because of unwanted reflections from objects other than the intended target.
		 * Defaults to 0.25 MCPS as initialized by the ST API and this library.
		 */
		bool setSignalRateLimit(float limitMCPS);
		/**
		 * Get the return signal rate limit check value in MCPS
		 */
		float getSignalRateLimit();
		/**
		 * Set the measurement timing budget (in microseconds), which is the time allowed for one measurement.
		 *
		 * The ST API and this library take care of splitting the timing budget among the sub-steps in the ranging sequence.
		 * A longer timing budget allows for more accurate measurements.
		 * Increasing the budget by a factor of N decreases the range measurement standard deviation by a factor of sqrt(N).
		 * Defaults to about 33 milliseconds; the minimum is 20 ms.
		 * Based on VL53L0X_set_measurement_timing_budget_micro_seconds().
		 */
		bool setMeasurementTimingBudget(uint32_t budgetMicroseconds);
		/**
		 * Get the measurement timing budget (in microseconds)
		 *
		 * Based on VL53L0X_get_measurement_timing_budget_micro_seconds()
		 */
		uint32_t getMeasurementTimingBudget();
		/**
		 * Set the VCSEL (vertical cavity surface emitting laser) pulse period for the given period type (pre-range or final range) to the given value (in PCLKs).
		 *
		 * Longer periods seem to increase the potential range of the sensor.
		 * Valid values are (even numbers only): pre: 12 to 18 (initialized default: 14), final: 8 to 14 (initialized default: 10).
		 * Based on VL53L0X_set_vcsel_pulse_period().
		 */
		bool setVcselPulsePeriod(vcselPeriodType type, uint8_t periodPCLKs);
		/**
		 * Get the VCSEL pulse period in PCLKs for the given period type.
		 *
		 * Based on VL53L0X_get_vcsel_pulse_period().
		 */
		uint8_t getVcselPulsePeriod(vcselPeriodType type);
		/**
		 * Start continuous ranging measurements.
		 *
		 * If periodMilliseconds (optional) is 0 or not given, continuous back-to-back mode is used (the sensor takes measurements as often as possible);
		 * Otherwise, continuous timed mode is used, with the given inter-measurement period in milliseconds determining how often the sensor takes a measurement.
		 * Based on VL53L0X_StartMeasurement().
		 */
		void startContinuous(uint32_t periodMilliseconds = 0);
		/**
		 * Stop continuous measurements.
		 *
		 * Based on VL53L0X_StopMeasurement().
		 */
		void stopContinuous();
		/**
		 * Returns a range reading in millimeters when continuous mode is active.
		 *
		 * readRangeSingleMillimeters() also calls this function after starting a single-shot range measurement.
		 */
		uint16_t readRangeContinuousMillimeters();
		/**
		 * Performs a single-shot range measurement and returns the reading in millimeters.
		 *
		 * Based on VL53L0X_PerformSingleRangingMeasurement().
		 */
		uint16_t readRangeSingleMillimeters();

		inline void setTimeout(uint16_t timeout) {
			this->ioTimeout = timeout;
		}
		inline uint16_t getTimeout() {
			return this->ioTimeout;
		}
		/**
		 * Whether a timeout occurred in one of the read functions since the last call to timeoutOccurred().
		 */
		bool timeoutOccurred();
	private:
		/*** Private fields ***/

		int i2cFileDescriptor;
    LinuxDuino::Wire *_wire = NULL;
		int16_t xshutGPIOPin;
		uint8_t address;

		uint32_t measurementTimingBudgetMicroseconds;
		uint64_t timeoutStartMilliseconds;
		uint64_t ioTimeout;
		bool didTimeout;
		// read by init and used when starting measurement; is StopVariable field of VL53L0X_DevData_t structure in API
		uint8_t stopVariable;

		/*** Private enums, structures and subclasses ***/

		struct SequenceStepEnables {
			// TCC: Target CentreCheck
			bool tcc;
			// MSRC: Minimum Signal Rate Check
			bool msrc;
			// DSS: Dynamic Spad Selection
			bool dss;
			bool preRange;
			bool finalRange;
		};

		struct SequenceStepTimeouts {
			uint16_t preRangeVCSELPeriodPCLKs;
			uint16_t finalRangeVCSELPeriodPCLKs;

			uint16_t msrcDssTccMCLKs;
			uint16_t preRangeMCLKs;
			uint16_t finalRangeMCLKs;

			uint32_t msrcDssTccMicroseconds;
			uint32_t preRangeMicroseconds;
			uint32_t finalRangeMicroseconds;
		};

		/*** Private methods ***/

		/**
		 * Get reference SPAD (single photon avalanche diode) count and type.
		 *
		 * Based on VL53L0X_get_info_from_device(), but only gets reference SPAD count and type.
		 */
		bool getSPADInfo(uint8_t* count, bool* typeIsAperture);
		/**
		 * Get sequence step enables.
		 *
		 * Based on VL53L0X_GetSequenceStepEnables().
		 */
		void getSequenceStepEnables(SequenceStepEnables* enables);
		/**
		 * Get sequence step timeouts.
		 *
		 * Based on get_sequence_step_timeout(), but gets all timeouts instead of just the requested one, and also stores intermediate values.
		 */
		void getSequenceStepTimeouts(const SequenceStepEnables* enables, SequenceStepTimeouts* timeouts);
		/**
		 * Decode sequence step timeout in MCLKs from register value.
		 *
		 * Based on VL53L0X_decode_timeout().
		 * Note: the original function returned a uint32_t, but the return value is always stored in a uint16_t.
		 */
		static uint16_t decodeTimeout(uint16_t registerValue);
		/**
		 * Encode sequence step timeout register value from timeout in MCLKs.
		 *
		 * Based on VL53L0X_encode_timeout().
		 * Note: the original function took a uint16_t, but the argument passed to it is always a uint16_t.
		 */
		static uint16_t encodeTimeout(uint16_t timeoutMCLKs);
		/**
		 * Convert sequence step timeout from MCLKs to microseconds with given VCSEL period in PCLKs.
		 *
		 * Based on VL53L0X_calc_timeout_us().
		 */
		static uint32_t timeoutMclksToMicroseconds(uint16_t timeoutPeriodMCLKs, uint8_t vcselPeriodPCLKs);
		/**
		 * Convert sequence step timeout from microseconds to MCLKs with given VCSEL period in PCLKs.
		 *
		 * Based on VL53L0X_calc_timeout_mclks().
		 */
		static uint32_t timeoutMicrosecondsToMclks(uint32_t timeoutPeriodMicroseconds, uint8_t vcselPeriodPCLKs);
		/**
		 * Based on VL53L0X_perform_single_ref_calibration().
		 */
		bool performSingleRefCalibration(uint8_t vhvInitByte);
};

#endif
