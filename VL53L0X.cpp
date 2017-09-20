#include "VL53L0X.h"

// errno
#include <cerrno>
// strerror()
#include <cstring>
// struct timespec, clock_gettime()
// std::string etc
#include <string>
// close()
#include <unistd.h>
// GPIO: pinMode(), digitalWrite()
#include <wiringPi.h>
// I2C: LinuxDuino
using namespace LinuxDuino;

/*** Defines ***/

// Record the current time to check an upcoming timeout against
#define startTimeout() (this->timeoutStartMilliseconds = milliseconds())

// Check if timeout is enabled (set to nonzero value) and has expired
#define checkTimeoutExpired() (this->ioTimeout > 0 && (milliseconds() - this->timeoutStartMilliseconds) > this->ioTimeout)

// Decode VCSEL (vertical cavity surface emitting laser) pulse period in PCLKs from register value based on VL53L0X_decode_vcsel_period()
#define decodeVcselPeriod(registerValue) (((registerValue) + 1) << 1)

// Encode VCSEL pulse period register value from period in PCLKs based on VL53L0X_encode_vcsel_period()
#define encodeVcselPeriod(periodPCLKs) (((periodPCLKs) >> 1) - 1)

// Calculate macro period in *nanoseconds* from VCSEL period in PCLKs based on VL53L0X_calc_macro_period_ps()
// PLL_period_ps = 1655, macro_period_vclks = 2304
#define calcMacroPeriod(vcselPeriodPCLKs) ((((uint32_t)2304 * (vcselPeriodPCLKs) * 1655) + 500) / 1000)

/*** Helper functions ***/

uint64_t milliseconds() {
	timespec ts;
	clock_gettime(CLOCK_MONOTONIC, &ts);
	return (ts.tv_sec * 1000 + ts.tv_nsec / 1000000);
}

/*** Constructors ***/

VL53L0X::VL53L0X(const int16_t xshutGPIOPin, const uint8_t address) {
	this->xshutGPIOPin = xshutGPIOPin;
	this->address = address;

	this->ioTimeout = 0;
	this->didTimeout = false;

	this->lastStatus = 0;
	this->measurementTimingBudgetMicroseconds = 33000;
	this->stopVariable = 0;
	this->timeoutStartMilliseconds = milliseconds();
}

/*** Public Methods ***/

bool VL53L0X::init(bool ioMode2v8) {
	// Set XSHUT pin mode (if pin set)
	if (this->xshutGPIOPin >= 0) {
		pinMode(this->xshutGPIOPin, OUTPUT);
	}

	// Enable the sensor
	this->powerOn();

	// Initialize I2C communication
    _wire = new Wire(1);

	// VL53L0X_DataInit() begin

	// sensor uses 1V8 mode for I/O by default; switch to 2V8 mode if necessary
	if (ioMode2v8) {
		// set bit 0
		this->writeRegister(VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV, this->readRegister(VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV) | 0x01);
	}

	// "Set I2C standard mode"
	this->writeRegister(0x88, 0x00);

	this->writeRegister(0x80, 0x01);
	this->writeRegister(0xFF, 0x01);
	this->writeRegister(0x00, 0x00);
	this->stopVariable = this->readRegister(0x91);
	this->writeRegister(0x00, 0x01);
	this->writeRegister(0xFF, 0x00);
	this->writeRegister(0x80, 0x00);

	// disable SIGNAL_RATE_MSRC (bit 1) and SIGNAL_RATE_PRE_RANGE (bit 4) limit checks
	this->writeRegister(MSRC_CONFIG_CONTROL, this->readRegister(MSRC_CONFIG_CONTROL) | 0x12);

	// set final range signal rate limit to 0.25 MCPS (million counts per second)
	this->setSignalRateLimit(0.25);

	this->writeRegister(SYSTEM_SEQUENCE_CONFIG, 0xFF);

	// VL53L0X_DataInit() end

	// VL53L0X_StaticInit() begin

	uint8_t spadCount;
	bool spadTypeIsAperture;
	if (!this->getSPADInfo(&spadCount, &spadTypeIsAperture)) {
		return false;
	}

	// The SPAD map (RefGoodSpadMap) is read by VL53L0X_get_info_from_device() in the API,
	// but the same data seems to be more easily readable from GLOBAL_CONFIG_SPAD_ENABLES_REF_0 through _6, so read it from there
	uint8_t refSPADMap[6];
	this->readRegisterMultiple(GLOBAL_CONFIG_SPAD_ENABLES_REF_0, refSPADMap, 6);

	// -- VL53L0X_set_reference_spads() begin (assume NVM values are valid)

	this->writeRegister(0xFF, 0x01);
	this->writeRegister(DYNAMIC_SPAD_REF_EN_START_OFFSET, 0x00);
	this->writeRegister(DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD, 0x2C);
	this->writeRegister(0xFF, 0x00);
	this->writeRegister(GLOBAL_CONFIG_REF_EN_START_SELECT, 0xB4);

	// 12 is the first aperture spad
	uint8_t firstSPADToEnable = spadTypeIsAperture ? 12 : 0;
	uint8_t spadsEnabled = 0;

	for (uint8_t i = 0; i < 48; i++) {
		if (i < firstSPADToEnable || spadsEnabled == spadCount) {
			// This bit is lower than the first one that should be enabled, or (reference_spad_count) bits have already been enabled, so zero this bit
			refSPADMap[i / 8] &= ~(1 << (i % 8));
		} else if ((refSPADMap[i / 8] >> (i % 8)) & 0x1) {
			spadsEnabled++;
		}
	}

	this->writeRegisterMultiple(GLOBAL_CONFIG_SPAD_ENABLES_REF_0, refSPADMap, 6);

	// -- VL53L0X_set_reference_spads() end

	// -- VL53L0X_load_tuning_settings() begin
	// DefaultTuningSettings from vl53l0x_tuning.h

	this->writeRegister(0xFF, 0x01);
	this->writeRegister(0x00, 0x00);

	this->writeRegister(0xFF, 0x00);
	this->writeRegister(0x09, 0x00);
	this->writeRegister(0x10, 0x00);
	this->writeRegister(0x11, 0x00);

	this->writeRegister(0x24, 0x01);
	this->writeRegister(0x25, 0xFF);
	this->writeRegister(0x75, 0x00);

	this->writeRegister(0xFF, 0x01);
	this->writeRegister(0x4E, 0x2C);
	this->writeRegister(0x48, 0x00);
	this->writeRegister(0x30, 0x20);

	this->writeRegister(0xFF, 0x00);
	this->writeRegister(0x30, 0x09);
	this->writeRegister(0x54, 0x00);
	this->writeRegister(0x31, 0x04);
	this->writeRegister(0x32, 0x03);
	this->writeRegister(0x40, 0x83);
	this->writeRegister(0x46, 0x25);
	this->writeRegister(0x60, 0x00);
	this->writeRegister(0x27, 0x00);
	this->writeRegister(0x50, 0x06);
	this->writeRegister(0x51, 0x00);
	this->writeRegister(0x52, 0x96);
	this->writeRegister(0x56, 0x08);
	this->writeRegister(0x57, 0x30);
	this->writeRegister(0x61, 0x00);
	this->writeRegister(0x62, 0x00);
	this->writeRegister(0x64, 0x00);
	this->writeRegister(0x65, 0x00);
	this->writeRegister(0x66, 0xA0);

	this->writeRegister(0xFF, 0x01);
	this->writeRegister(0x22, 0x32);
	this->writeRegister(0x47, 0x14);
	this->writeRegister(0x49, 0xFF);
	this->writeRegister(0x4A, 0x00);

	this->writeRegister(0xFF, 0x00);
	this->writeRegister(0x7A, 0x0A);
	this->writeRegister(0x7B, 0x00);
	this->writeRegister(0x78, 0x21);

	this->writeRegister(0xFF, 0x01);
	this->writeRegister(0x23, 0x34);
	this->writeRegister(0x42, 0x00);
	this->writeRegister(0x44, 0xFF);
	this->writeRegister(0x45, 0x26);
	this->writeRegister(0x46, 0x05);
	this->writeRegister(0x40, 0x40);
	this->writeRegister(0x0E, 0x06);
	this->writeRegister(0x20, 0x1A);
	this->writeRegister(0x43, 0x40);

	this->writeRegister(0xFF, 0x00);
	this->writeRegister(0x34, 0x03);
	this->writeRegister(0x35, 0x44);

	this->writeRegister(0xFF, 0x01);
	this->writeRegister(0x31, 0x04);
	this->writeRegister(0x4B, 0x09);
	this->writeRegister(0x4C, 0x05);
	this->writeRegister(0x4D, 0x04);

	this->writeRegister(0xFF, 0x00);
	this->writeRegister(0x44, 0x00);
	this->writeRegister(0x45, 0x20);
	this->writeRegister(0x47, 0x08);
	this->writeRegister(0x48, 0x28);
	this->writeRegister(0x67, 0x00);
	this->writeRegister(0x70, 0x04);
	this->writeRegister(0x71, 0x01);
	this->writeRegister(0x72, 0xFE);
	this->writeRegister(0x76, 0x00);
	this->writeRegister(0x77, 0x00);

	this->writeRegister(0xFF, 0x01);
	this->writeRegister(0x0D, 0x01);

	this->writeRegister(0xFF, 0x00);
	this->writeRegister(0x80, 0x01);
	this->writeRegister(0x01, 0xF8);

	this->writeRegister(0xFF, 0x01);
	this->writeRegister(0x8E, 0x01);
	this->writeRegister(0x00, 0x01);
	this->writeRegister(0xFF, 0x00);
	this->writeRegister(0x80, 0x00);

	// -- VL53L0X_load_tuning_settings() end

	// "Set interrupt config to new sample ready"
	// -- VL53L0X_SetGpioConfig() begin

	this->writeRegister(SYSTEM_INTERRUPT_CONFIG_GPIO, 0x04);
	// active low
	this->writeRegister(GPIO_HV_MUX_ACTIVE_HIGH, this->readRegister(GPIO_HV_MUX_ACTIVE_HIGH) & ~0x10);
	this->writeRegister(SYSTEM_INTERRUPT_CLEAR, 0x01);

	// -- VL53L0X_SetGpioConfig() end

	this->measurementTimingBudgetMicroseconds = this->getMeasurementTimingBudget();

	// "Disable MSRC and TCC by default"
	// MSRC = Minimum Signal Rate Check
	// TCC = Target CentreCheck
	// -- VL53L0X_SetSequenceStepEnable() begin

	this->writeRegister(SYSTEM_SEQUENCE_CONFIG, 0xE8);

	// -- VL53L0X_SetSequenceStepEnable() end

	// "Recalculate timing budget"
	this->setMeasurementTimingBudget(this->measurementTimingBudgetMicroseconds);

	// VL53L0X_StaticInit() end

	// VL53L0X_PerformRefCalibration() begin (VL53L0X_perform_ref_calibration())

	// -- VL53L0X_perform_vhv_calibration() begin

	this->writeRegister(SYSTEM_SEQUENCE_CONFIG, 0x01);
	if (!this->performSingleRefCalibration(0x40)) {
		return false;
	}

	// -- VL53L0X_perform_vhv_calibration() end

	// -- VL53L0X_perform_phase_calibration() begin

	this->writeRegister(SYSTEM_SEQUENCE_CONFIG, 0x02);
	if (!this->performSingleRefCalibration(0x00)) {
		return false;
	}

	// -- VL53L0X_perform_phase_calibration() end

	// "restore the previous Sequence Config"
	this->writeRegister(SYSTEM_SEQUENCE_CONFIG, 0xE8);

	// VL53L0X_PerformRefCalibration() end

	return true;
}

void VL53L0X::powerOn() {
	if (this->xshutGPIOPin >= 0) {
		digitalWrite(this->xshutGPIOPin, HIGH);
		// t_boot is 1.2ms max, wait 2ms just to be sure
		usleep(2000);
	}
}

void VL53L0X::powerOff() {
	if (this->xshutGPIOPin >= 0) {
		digitalWrite(this->xshutGPIOPin, LOW);
	}
}

void VL53L0X::writeRegister(uint8_t reg, uint8_t value) {
    _wire->beginTransmission(address);
    _wire->write((uint8_t) reg, (uint8_t) value);
    _wire->endTransmission();
}

void VL53L0X::writeRegister16Bit(uint8_t reg, uint16_t value) {
	// Reverse endianness
	uint16_t valueFixed = ((value & 0xFF) << 8) + ((value & 0xFF00) >> 8);

    _wire->beginTransmission(address);
    _wire->write16((uint8_t) reg, (uint16_t) valueFixed);
    _wire->endTransmission();
}

void VL53L0X::writeRegister32Bit(uint8_t reg, uint32_t value) {
	// Split 32-bit word into MS ... LS bytes
    uint8_t *buffer = new uint8_t[4]();
	buffer[0] = (value >> 24) & 0xFF;
	buffer[1] = (value >> 16) & 0xFF;
	buffer[2] = (value >> 8) & 0xFF;
	buffer[3] = value & 0xFF;

    _wire->beginTransmission(address);
    _wire->writeBlock((uint8_t) reg, buffer, 4);
    _wire->endTransmission();
	delete [] buffer;
}

void VL53L0X::writeRegisterMultiple(uint8_t reg, const uint8_t* source, uint8_t count) {
    uint8_t *buffer = new uint8_t[count]();
	for (int i = 0; i < count; ++i) {
		buffer[i] = source[i];
	}

    _wire->beginTransmission(address);
    _wire->writeBlock((uint8_t) reg, buffer, count);
    _wire->endTransmission();
	delete [] buffer;
}

uint8_t VL53L0X::readRegister(uint8_t reg) {
    _wire->beginTransmission(address);
    int p = _wire->read(reg);
    _wire->endTransmission();
	return p & 0xFF;
}

uint16_t VL53L0X::readRegister16Bit(uint8_t reg) {
    _wire->beginTransmission(address);
    int p = _wire->read16(reg);
    _wire->endTransmission();

	lastStatus = (p == -1 ? errno : 0);
	if (p == -1) {
		throw(std::string("Error reading word from register"));
	}

	// Reverse endianness
	return ((p & 0xFF) << 8) + ((p & 0xFF00) >> 8);
}

uint32_t VL53L0X::readRegister32Bit(uint8_t reg) {
    uint8_t *buffer = new uint8_t[4]();

    _wire->beginTransmission(address);
    int p = _wire->readBlock(reg, buffer, 4);
    _wire->endTransmission();

	lastStatus = (p == -1 ? errno : 0);
	if (p == -1) {
		delete [] buffer;
		throw(std::string("Error reading dword from register"));
	}

	uint32_t value = (uint32_t)buffer[0] << 24;
	value += (uint32_t)buffer[1]<<16;
	value += (uint32_t)buffer[2]<<8;
	value += (uint32_t)buffer[3];

	delete [] buffer;
	return value;
}

void VL53L0X::readRegisterMultiple(uint8_t reg, uint8_t* destination, uint8_t count) {
    uint8_t *buffer = new uint8_t[count]();

    _wire->beginTransmission(address);
    int p = _wire->readBlock(reg, buffer, count);
    _wire->endTransmission();

	lastStatus = (p == -1 ? errno : 0);
	if (p == -1) {
		delete [] buffer;
		throw(std::string("Error reading block from register"));
	}

	for (int i = 0; i < count; ++i) {
		destination[i] = buffer[i] & 0xFF;
	}
}

void VL53L0X::setAddress(uint8_t newAddress) {
	// Ensure power state
	this->powerOn();
	// Set new I2C address
	this->writeRegister(I2C_SLAVE_DEVICE_ADDRESS, newAddress & 0x7F);
	// Close I2C communication on old address
	this->address = newAddress;
}

bool VL53L0X::setSignalRateLimit(float limitMCPS) {
	if (limitMCPS < 0 || limitMCPS > 511.99) {
		return false;
	}

	// Q9.7 fixed point format (9 integer bits, 7 fractional bits)
	this->writeRegister16Bit(FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT, limitMCPS * (1 << 7));
	return true;
}

float VL53L0X::getSignalRateLimit() {
	return (float)this->readRegister16Bit(FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT) / (1 << 7);
}

bool VL53L0X::setMeasurementTimingBudget(uint32_t budgetMicroseconds) {
	// note that these are different than values in get_
	uint16_t const START_OVERHEAD = 1320;
	uint16_t const END_OVERHEAD = 960;
	uint16_t const MSRC_OVERHEAD = 660;
	uint16_t const TCC_OVERHEAD = 590;
	uint16_t const DSS_OVERHEAD = 690;
	uint16_t const PRE_RANGE_OVERHEAD = 660;
	uint16_t const FINAL_RANGE_OVERHEAD = 550;
	uint32_t const MIN_TIMING_BUDGET = 20000;

	if (budgetMicroseconds < MIN_TIMING_BUDGET) {
		return false;
	}

	VL53L0X::SequenceStepEnables enables;
	VL53L0X::SequenceStepTimeouts timeouts;
	this->getSequenceStepEnables(&enables);
	this->getSequenceStepTimeouts(&enables, &timeouts);

	uint32_t usedBudgetMicroseconds = START_OVERHEAD + END_OVERHEAD;
	if (enables.tcc) {
		usedBudgetMicroseconds += (timeouts.msrcDssTccMicroseconds + TCC_OVERHEAD);
	}
	if (enables.dss) {
		usedBudgetMicroseconds += 2 * (timeouts.msrcDssTccMicroseconds + DSS_OVERHEAD);
	} else if (enables.msrc) {
		usedBudgetMicroseconds += (timeouts.msrcDssTccMicroseconds + MSRC_OVERHEAD);
	}
	if (enables.preRange) {
		usedBudgetMicroseconds += (timeouts.preRangeMicroseconds + PRE_RANGE_OVERHEAD);
	}
	if (enables.finalRange) {
		usedBudgetMicroseconds += FINAL_RANGE_OVERHEAD;
	}

	// "Note that the final range timeout is determined by the timing
	// budget and the sum of all other timeouts within the sequence.
	// If there is no room for the final range timeout, then an error
	// will be set. Otherwise the remaining time will be applied to
	// the final range."

	if (usedBudgetMicroseconds > budgetMicroseconds) {
		// "Requested timeout too small."
		return false;
	}

	uint32_t finalRangeTimeoutMicroseconds = budgetMicroseconds - usedBudgetMicroseconds;

	// set_sequence_step_timeout() begin
	// (SequenceStepId == VL53L0X_SEQUENCESTEP_FINAL_RANGE)

	// "For the final range timeout, the pre-range timeout
	// must be added. To do this both final and pre-range
	// timeouts must be expressed in macro periods MClks
	// because they have different vcsel periods."

	uint16_t finalRangeTimeoutMCLKs = timeoutMicrosecondsToMclks(finalRangeTimeoutMicroseconds, timeouts.finalRangeVCSELPeriodPCLKs);

	if (enables.preRange) {
		finalRangeTimeoutMCLKs += timeouts.preRangeMCLKs;
	}

	this->writeRegister16Bit(FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI, encodeTimeout(finalRangeTimeoutMCLKs));

	// set_sequence_step_timeout() end

	// store for internal reuse
	this->measurementTimingBudgetMicroseconds = budgetMicroseconds;

	return true;
}

uint32_t VL53L0X::getMeasurementTimingBudget() {
	// note that these are different than values in set_
	uint16_t const START_OVERHEAD = 1910;
	uint16_t const END_OVERHEAD = 960;
	uint16_t const MSRC_OVERHEAD = 660;
	uint16_t const TCC_OVERHEAD = 590;
	uint16_t const DSS_OVERHEAD = 690;
	uint16_t const PRE_RANGE_OVERHEAD = 660;
	uint16_t const FINAL_RANGE_OVERHEAD = 550;

	VL53L0X::SequenceStepEnables enables;
	VL53L0X::SequenceStepTimeouts timeouts;
	this->getSequenceStepEnables(&enables);
	this->getSequenceStepTimeouts(&enables, &timeouts);

	// "Start and end overhead times always present"
	uint32_t budgetMicroseconds = START_OVERHEAD + END_OVERHEAD;
	if (enables.tcc) {
		budgetMicroseconds += (timeouts.msrcDssTccMicroseconds + TCC_OVERHEAD);
	}
	if (enables.dss) {
		budgetMicroseconds += 2 * (timeouts.msrcDssTccMicroseconds + DSS_OVERHEAD);
	} else if (enables.msrc) {
		budgetMicroseconds += (timeouts.msrcDssTccMicroseconds + MSRC_OVERHEAD);
	}
	if (enables.preRange) {
		budgetMicroseconds += (timeouts.preRangeMicroseconds + PRE_RANGE_OVERHEAD);
	}
	if (enables.finalRange) {
		budgetMicroseconds += (timeouts.finalRangeMicroseconds + FINAL_RANGE_OVERHEAD);
	}

	// store for internal reuse
	this->measurementTimingBudgetMicroseconds = budgetMicroseconds;
	return budgetMicroseconds;
}

bool VL53L0X::setVcselPulsePeriod(vcselPeriodType type, uint8_t periodPCLKs) {
	uint8_t vcselPeriodValue = encodeVcselPeriod(periodPCLKs);

	VL53L0X::SequenceStepEnables enables;
	VL53L0X::SequenceStepTimeouts timeouts;

	this->getSequenceStepEnables(&enables);
	this->getSequenceStepTimeouts(&enables, &timeouts);

	// "Apply specific settings for the requested clock period"
	// "Re-calculate and apply timeouts, in macro periods"

	// "When the VCSEL period for the pre or final range is changed,
	// the corresponding timeout must be read from the device using
	// the current VCSEL period, then the new VCSEL period can be
	// applied. The timeout then must be written back to the device
	// using the new VCSEL period.
	//
	// For the MSRC timeout, the same applies - this timeout being
	// dependant on the pre-range vcsel period."

	if (type == VcselPeriodPreRange) {
		// "Set phase check limits"
		switch (periodPCLKs) {
			case 12:
				this->writeRegister(PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x18);
				break;
			case 14:
				this->writeRegister(PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x30);
				break;
			case 16:
				this->writeRegister(PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x40);
				break;
			case 18:
				this->writeRegister(PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x50);
				break;
			default:
				// invalid period
				return false;
		}
		this->writeRegister(PRE_RANGE_CONFIG_VALID_PHASE_LOW, 0x08);

		// apply new VCSEL period
		this->writeRegister(PRE_RANGE_CONFIG_VCSEL_PERIOD, vcselPeriodValue);

		// update timeouts

		// set_sequence_step_timeout() begin
		// (SequenceStepId == VL53L0X_SEQUENCESTEP_PRE_RANGE)

		uint16_t newPreRangeTimeoutMCLKs = timeoutMicrosecondsToMclks(timeouts.preRangeMicroseconds, periodPCLKs);
		this->writeRegister16Bit(PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI, encodeTimeout(newPreRangeTimeoutMCLKs));

		// set_sequence_step_timeout() end

		// set_sequence_step_timeout() begin
		// (SequenceStepId == VL53L0X_SEQUENCESTEP_MSRC)

		uint16_t newMsrcTimeoutMCLKs = timeoutMicrosecondsToMclks(timeouts.msrcDssTccMicroseconds, periodPCLKs);
		this->writeRegister(MSRC_CONFIG_TIMEOUT_MACROP, (newMsrcTimeoutMCLKs > 256) ? 255 : (newMsrcTimeoutMCLKs - 1));

		// set_sequence_step_timeout() end
	} else if (type == VcselPeriodFinalRange) {
		switch (periodPCLKs) {
			case 8:
				this->writeRegister(FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x10);
				this->writeRegister(FINAL_RANGE_CONFIG_VALID_PHASE_LOW, 0x08);
				this->writeRegister(GLOBAL_CONFIG_VCSEL_WIDTH, 0x02);
				this->writeRegister(ALGO_PHASECAL_CONFIG_TIMEOUT, 0x0C);
				this->writeRegister(0xFF, 0x01);
				this->writeRegister(ALGO_PHASECAL_LIM, 0x30);
				this->writeRegister(0xFF, 0x00);
				break;
			case 10:
				this->writeRegister(FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x28);
				this->writeRegister(FINAL_RANGE_CONFIG_VALID_PHASE_LOW, 0x08);
				this->writeRegister(GLOBAL_CONFIG_VCSEL_WIDTH, 0x03);
				this->writeRegister(ALGO_PHASECAL_CONFIG_TIMEOUT, 0x09);
				this->writeRegister(0xFF, 0x01);
				this->writeRegister(ALGO_PHASECAL_LIM, 0x20);
				this->writeRegister(0xFF, 0x00);
				break;
			case 12:
				this->writeRegister(FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x38);
				this->writeRegister(FINAL_RANGE_CONFIG_VALID_PHASE_LOW, 0x08);
				this->writeRegister(GLOBAL_CONFIG_VCSEL_WIDTH, 0x03);
				this->writeRegister(ALGO_PHASECAL_CONFIG_TIMEOUT, 0x08);
				this->writeRegister(0xFF, 0x01);
				this->writeRegister(ALGO_PHASECAL_LIM, 0x20);
				this->writeRegister(0xFF, 0x00);
				break;
			case 14:
				this->writeRegister(FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x48);
				this->writeRegister(FINAL_RANGE_CONFIG_VALID_PHASE_LOW, 0x08);
				this->writeRegister(GLOBAL_CONFIG_VCSEL_WIDTH, 0x03);
				this->writeRegister(ALGO_PHASECAL_CONFIG_TIMEOUT, 0x07);
				this->writeRegister(0xFF, 0x01);
				this->writeRegister(ALGO_PHASECAL_LIM, 0x20);
				this->writeRegister(0xFF, 0x00);
				break;
			default:
				// invalid period
				return false;
		}

		// apply new VCSEL period
		this->writeRegister(FINAL_RANGE_CONFIG_VCSEL_PERIOD, vcselPeriodValue);

		// update timeouts

		// set_sequence_step_timeout() begin
		// (SequenceStepId == VL53L0X_SEQUENCESTEP_FINAL_RANGE)

		// "For the final range timeout, the pre-range timeout
		// must be added. To do this both final and pre-range
		// timeouts must be expressed in macro periods MClks
		// because they have different vcsel periods."

		uint16_t newFinalRangeTimeoutMCLKs = timeoutMicrosecondsToMclks(timeouts.finalRangeMicroseconds, periodPCLKs);
		if (enables.preRange) {
			newFinalRangeTimeoutMCLKs += timeouts.preRangeMCLKs;
		}
		this->writeRegister16Bit(FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI, encodeTimeout(newFinalRangeTimeoutMCLKs));

		// set_sequence_step_timeout end
	} else {
		// invalid type
		return false;
	}

	// "Finally, the timing budget must be re-applied"

	setMeasurementTimingBudget(this->measurementTimingBudgetMicroseconds);

	// "Perform the phase calibration. This is needed after changing on vcsel period."
	// VL53L0X_perform_phase_calibration() begin

	uint8_t sequenceConfig = this->readRegister(SYSTEM_SEQUENCE_CONFIG);
	this->writeRegister(SYSTEM_SEQUENCE_CONFIG, 0x02);
	performSingleRefCalibration(0x0);
	this->writeRegister(SYSTEM_SEQUENCE_CONFIG, sequenceConfig);

	// VL53L0X_perform_phase_calibration() end

	return true;
}

uint8_t VL53L0X::getVcselPulsePeriod(vcselPeriodType type) {
	if (type == VcselPeriodPreRange) {
		return decodeVcselPeriod(this->readRegister(PRE_RANGE_CONFIG_VCSEL_PERIOD));
	} else if (type == VcselPeriodFinalRange) {
		return decodeVcselPeriod(this->readRegister(FINAL_RANGE_CONFIG_VCSEL_PERIOD));
	} else {
		return 255;
	}
}

void VL53L0X::startContinuous(uint32_t periodMilliseconds) {
	this->writeRegister(0x80, 0x01);
	this->writeRegister(0xFF, 0x01);
	this->writeRegister(0x00, 0x00);
	this->writeRegister(0x91, this->stopVariable);
	this->writeRegister(0x00, 0x01);
	this->writeRegister(0xFF, 0x00);
	this->writeRegister(0x80, 0x00);

	if (periodMilliseconds != 0) {
		// continuous timed mode

		// VL53L0X_SetInterMeasurementPeriodMilliSeconds() begin

		uint16_t oscCalibrateValue = this->readRegister16Bit(OSC_CALIBRATE_VAL);

		if (oscCalibrateValue != 0) {
			periodMilliseconds *= oscCalibrateValue;
		}

		this->writeRegister32Bit(SYSTEM_INTERMEASUREMENT_PERIOD, periodMilliseconds);

		// VL53L0X_SetInterMeasurementPeriodMilliSeconds() end

		// VL53L0X_REG_SYSRANGE_MODE_TIMED
		this->writeRegister(SYSRANGE_START, 0x04);
	} else {
		// continuous back-to-back mode
		// VL53L0X_REG_SYSRANGE_MODE_BACKTOBACK
		this->writeRegister(SYSRANGE_START, 0x02);
	}
}

void VL53L0X::stopContinuous() {
	// VL53L0X_REG_SYSRANGE_MODE_SINGLESHOT
	this->writeRegister(SYSRANGE_START, 0x01);

	this->writeRegister(0xFF, 0x01);
	this->writeRegister(0x00, 0x00);
	this->writeRegister(0x91, 0x00);
	this->writeRegister(0x00, 0x01);
	this->writeRegister(0xFF, 0x00);
}

uint16_t VL53L0X::readRangeContinuousMillimeters() {
	startTimeout();
	while ((this->readRegister(RESULT_INTERRUPT_STATUS) & 0x07) == 0) {
		if (checkTimeoutExpired()) {
			this->didTimeout = true;
			return 65535;
		}
		usleep(1);
	}

	// assumptions: Linearity Corrective Gain is 1000 (default);
	// fractional ranging is not enabled
	// Note: reading 16-bit register was working on Arduino but here it's not, thus double read and manual addition
	uint16_t range = this->readRegister16Bit(RESULT_RANGE_STATUS + 10);
	// uint8_t rangeA = this->readRegister(RESULT_RANGE_STATUS + 10);
	// uint8_t rangeB = this->readRegister(RESULT_RANGE_STATUS + 11);
	// uint16_t range = ((uint16_t)(rangeA)<<8) + (uint16_t)(rangeB);

	this->writeRegister(SYSTEM_INTERRUPT_CLEAR, 0x01);

	return range;
}

uint16_t VL53L0X::readRangeSingleMillimeters() {
	this->writeRegister(0x80, 0x01);
	this->writeRegister(0xFF, 0x01);
	this->writeRegister(0x00, 0x00);
	this->writeRegister(0x91, this->stopVariable);
	this->writeRegister(0x00, 0x01);
	this->writeRegister(0xFF, 0x00);
	this->writeRegister(0x80, 0x00);

	this->writeRegister(SYSRANGE_START, 0x01);

	// "Wait until start bit has been cleared"
	startTimeout();
	while (this->readRegister(SYSRANGE_START) & 0x01) {
		if (checkTimeoutExpired()) {
			this->didTimeout = true;
			return 65535;
		}
		usleep(1);
	}

	return readRangeContinuousMillimeters();
}

bool VL53L0X::timeoutOccurred() {
	bool tmp = this->didTimeout;
	this->didTimeout = false;
	return tmp;
}

/*** Private Methods ***/

bool VL53L0X::getSPADInfo(uint8_t* count, bool* typeIsAperture) {
	uint8_t tmp;

	this->writeRegister(0x80, 0x01);
	this->writeRegister(0xFF, 0x01);
	this->writeRegister(0x00, 0x00);

	this->writeRegister(0xFF, 0x06);
	this->writeRegister(0x83, this->readRegister(0x83) | 0x04);
	this->writeRegister(0xFF, 0x07);
	this->writeRegister(0x81, 0x01);

	this->writeRegister(0x80, 0x01);

	this->writeRegister(0x94, 0x6b);
	this->writeRegister(0x83, 0x00);
	startTimeout();
	while (this->readRegister(0x83) == 0x00) {
		if (checkTimeoutExpired()) {
			return false;
		}
		usleep(1);
	}
	this->writeRegister(0x83, 0x01);
	tmp = this->readRegister(0x92);

	*count = tmp & 0x7f;
	*typeIsAperture = (tmp >> 7) & 0x01;

	this->writeRegister(0x81, 0x00);
	this->writeRegister(0xFF, 0x06);
	this->writeRegister(0x83, this->readRegister(0x83 & ~0x04));
	this->writeRegister(0xFF, 0x01);
	this->writeRegister(0x00, 0x01);

	this->writeRegister(0xFF, 0x00);
	this->writeRegister(0x80, 0x00);

	return true;
}

void VL53L0X::getSequenceStepEnables(VL53L0X::SequenceStepEnables* enables) {
	uint8_t sequenceConfig = this->readRegister(SYSTEM_SEQUENCE_CONFIG);

	enables->tcc = (sequenceConfig >> 4) & 0x1;
	enables->dss = (sequenceConfig >> 3) & 0x1;
	enables->msrc = (sequenceConfig >> 2) & 0x1;
	enables->preRange = (sequenceConfig >> 6) & 0x1;
	enables->finalRange = (sequenceConfig >> 7) & 0x1;
}

void VL53L0X::getSequenceStepTimeouts(const VL53L0X::SequenceStepEnables * enables, VL53L0X::SequenceStepTimeouts * timeouts) {
	timeouts->preRangeVCSELPeriodPCLKs = this->getVcselPulsePeriod(VcselPeriodPreRange);

	timeouts->msrcDssTccMCLKs = this->readRegister(MSRC_CONFIG_TIMEOUT_MACROP) + 1;
	timeouts->msrcDssTccMicroseconds = this->timeoutMclksToMicroseconds(timeouts->msrcDssTccMCLKs, timeouts->preRangeVCSELPeriodPCLKs);

	timeouts->preRangeMCLKs = this->decodeTimeout(this->readRegister16Bit(PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI));
	timeouts->preRangeMicroseconds = this->timeoutMclksToMicroseconds(timeouts->preRangeMCLKs, timeouts->preRangeVCSELPeriodPCLKs);

	timeouts->finalRangeVCSELPeriodPCLKs = this->getVcselPulsePeriod(VcselPeriodFinalRange);

	timeouts->finalRangeMCLKs = this->decodeTimeout(this->readRegister16Bit(FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI));

	if (enables->preRange) {
		timeouts->finalRangeMCLKs -= timeouts->preRangeMCLKs;
	}

	timeouts->finalRangeMicroseconds = this->timeoutMclksToMicroseconds(timeouts->finalRangeMCLKs, timeouts->finalRangeVCSELPeriodPCLKs);
}

uint16_t VL53L0X::decodeTimeout(uint16_t registerValue) {
	// format: "(LSByte * 2^MSByte) + 1"
	return (uint16_t)((registerValue & 0x00FF) << (uint16_t)((registerValue & 0xFF00) >> 8)) + 1;
}

uint16_t VL53L0X::encodeTimeout(uint16_t timeoutMCLKs) {
	// format: "(LSByte * 2^MSByte) + 1"
	if (timeoutMCLKs == 0) {
		return 0;
	}

	uint32_t lsByte = 0;
	uint16_t msByte = 0;

	lsByte = timeoutMCLKs - 1;

	while ((lsByte & 0xFFFFFF00) > 0) {
		lsByte >>= 1;
		msByte++;
	}

	return (msByte << 8) | (lsByte & 0xFF);
}

uint32_t VL53L0X::timeoutMclksToMicroseconds(uint16_t timeoutPeriodMCLKs, uint8_t vcselPeriodPCLKs) {
	uint32_t macroPeriodNanoseconds = calcMacroPeriod(vcselPeriodPCLKs);

	return ((timeoutPeriodMCLKs * macroPeriodNanoseconds) + (macroPeriodNanoseconds / 2)) / 1000;
}

uint32_t VL53L0X::timeoutMicrosecondsToMclks(uint32_t timeoutPeriodMicroseconds, uint8_t vcselPeriodPCLKs) {
	uint32_t macroPeriodNanoseconds = calcMacroPeriod(vcselPeriodPCLKs);

	return (((timeoutPeriodMicroseconds * 1000) + (macroPeriodNanoseconds / 2)) / macroPeriodNanoseconds);
}

bool VL53L0X::performSingleRefCalibration(uint8_t vhvInitByte) {
	// VL53L0X_REG_SYSRANGE_MODE_START_STOP
	this->writeRegister(SYSRANGE_START, 0x01 | vhvInitByte);

	startTimeout();
	while ((this->readRegister(RESULT_INTERRUPT_STATUS) & 0x07) == 0) {
		if (checkTimeoutExpired()) {
			return false;
		}
		usleep(1);
	}

	this->writeRegister(SYSTEM_INTERRUPT_CLEAR, 0x01);
	this->writeRegister(SYSRANGE_START, 0x00);

	return true;
}
