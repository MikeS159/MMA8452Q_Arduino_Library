/******************************************************************************
SparkFun_MMA8452Q.cpp
SparkFun_MMA8452Q Library Source File
Jim Lindblom @ SparkFun Electronics
Original Creation Date: June 3, 2014
https://github.com/sparkfun/MMA8452_Accelerometer

This file implements all functions of the MMA8452Q class. Functions here range
from higher level stuff, like reading/writing MMA8452Q registers to low-level,
hardware I2C reads and writes.

Development environment specifics:
	IDE: Arduino 1.0.5
	Hardware Platform: Arduino Uno

	**Updated for Arduino 1.6.4 5/2015**
	
This code is beerware; if you see me (or any other SparkFun employee) at the
local, and you've found our code helpful, please buy us a round!

Distributed as-is; no warranty is given.
******************************************************************************/

#include "SparkFun_MMA8452Q.h"
#include <Arduino.h>
#include <Wire.h>

// CONSTRUCTUR
//   This function, called when you initialize the class will simply write the
//   supplied address into a private variable for future use.
//   The variable addr should be either 0x1C or 0x1D, depending on which voltage
//   the SA0 pin is tied to (GND or 3.3V respectively).
MMA8452Q::MMA8452Q(byte addr)
{
	address = addr; // Store address into private variable
}

// INITIALIZATION
//	This function initializes the MMA8452Q. It sets up the scale (either 2, 4,
//	or 8g), output data rate, portrait/landscape detection and tap detection.
//	It also checks the WHO_AM_I register to make sure we can communicate with
//	the sensor. Returns a 0 if communication failed, 1 if successful.
//  Default scale is 2g, default odr is 800Hz
byte MMA8452Q::init(MMA8452Q_Scale fsr, MMA8452Q_ODR odr)
{
	scale = fsr; // Haul fsr into our class variable, scale
	
	Wire.begin(); // Initialize I2C
	
	if (whoAmI() != 0x2A) // WHO_AM_I should always be 0x2A
	{
		return 0;
	}
	
	standby();  // Must be in standby to change registers
	
	setScale(scale);  // Set up accelerometer scale
	setODR(odr);  // Set up output data rate
	setupPL();  // Set up portrait/landscape detection
	// Multiply parameter by 0.0625g to calculate threshold.
	setupTap(0x80, 0x80, 0x08); // Disable x, y, set z to 0.5g
	
	active();  // Set to active to start reading
	
	return 1;
}

// READ ACCELERATION DATA
//  This function will read the acceleration values from the MMA8452Q. After
//	reading, it will update two triplets of variables:
//		* int's x, y, and z will store the signed 12-bit values read out
//		  of the acceleromter.
//		* floats cx, cy, and cz will store the calculated acceleration from
//		  those 12-bit values. These variables are in units of g's.
void MMA8452Q::read()
{
	byte rawData[6];  // x/y/z accel register data stored here

	readRegisters(OUT_X_MSB, rawData, 6);  // Read the six raw data registers into data array
	
	x = ((short)(rawData[0]<<8 | rawData[1])) >> 4;
	y = ((short)(rawData[2]<<8 | rawData[3])) >> 4;
	z = ((short)(rawData[4]<<8 | rawData[5])) >> 4;
	cx = (float) x / (float)(1<<11) * (float)(scale);
	cy = (float) y / (float)(1<<11) * (float)(scale);
	cz = (float) z / (float)(1<<11) * (float)(scale);
}

// CHECK IF NEW DATA IS AVAILABLE
//	This function checks the status of the MMA8452Q to see if new data is availble.
//	returns 0 if no new data is present, or a 1 if new data is available.
byte MMA8452Q::available()
{
	return (readRegister(STATUS) & 0x08) >> 3;
}

// GET SYSTEM MODE
// This function get the system mode of the MMA8452Q.
// Possible returns are 0 (STANDBY), 1 (WAKE), and 2 (SLEEP)
byte MMA8452Q::getSystemMode()
{
	return (int)readRegister(SYSMOD);
}

// GET INTERRUPT SOURCES
// This function polls the register INT_SOURCE on the MMA8452Q
// The result is a combination of the status of all interrupt sources.
// In order for these to work, the enableXXXInt() functions must be called.
// For each bit, a logical 1 means that the interrupt was thrown.
// Below are the bit values:
// Bit 0: SRC_DRDY (data ready)
// Bit 2: SRC_FF_MT (freefall/motion detected)
// Bit 3: SRC_PULSE (pulse detected)
// Bit 4: SRC_LNDPRT (landscape/portrait interrupt change)
// Bit 5: SRC_TRANS (transient detected)
// Bit 7: SRC_ASLP (wake-to-sleep or sleep-to-wake)
byte MMA8452Q::getInterruptSources()
{
	return readRegister(INT_SOURCE);
}

// GET DEVICE ID
// This function reads the WHO_AM_I register of the MMA8452Q
// The default return is 0x2A
byte MMA8452Q::whoAmI()
{
	return readRegister(WHO_AM_I);
}


// SET FULL-SCALE RANGE
//	This function sets the full-scale range of the x, y, and z axis accelerometers.
//	Possible values for the fsr variable are SCALE_2G, SCALE_4G, or SCALE_8G.
void MMA8452Q::setScale(MMA8452Q_Scale fsr)
{
	// Must be in standby mode to make changes!!!
	byte cfg = readRegister(XYZ_DATA_CFG);
	cfg &= 0xFC; // Mask out scale bits
	cfg |= (fsr >> 2);  // Neat trick, see page 22. 00 = 2G, 01 = 4A, 10 = 8G
	writeRegister(XYZ_DATA_CFG, cfg);
}

//  ENABLE HIGH-PASS OUTPUT FILTER
//	If true, output is run through a high-pass filter.
//  Set HP cutoff in setHPCutoff.
//	Possible values are true (enable) and false (disable)
void MMA8452Q::enableHighPassOutput(bool enable)
{
	// Must be in standby mode to make changes!!!
	byte ctrl = readRegister(XYZ_DATA_CFG);
	ctrl &= 0xEF; // Mask out HPF_OUT bit (bit 4)
	ctrl |= (enable << 4);
	writeRegister(XYZ_DATA_CFG, ctrl);
}

//  SET HIGH-PASS CUTOFF FREQUENCY
//	Possible values are HP0, HP1, HP2, HP3.
//  Actual frequency depends on wake ODR and noise mode.
//  See data sheet for more information.
//  HP0 is roughly 20% or less of ODR
//  HP1 is 2x lower than HP0 (roughly 10% or less of ODR)
//  HP2 is 4x lower than HP0 (roughly 5% or less of ODR)
//  HP3 is 8x lower than HP0 (roughly 2.5% or less of ODR)
void MMA8452Q::setHPCutoff(MMA8452Q_HPFiltCutoff cutoff)
{
	// Must be in standby mode to make changes!!!
	byte ctrl = readRegister(HP_FILTER_CUTOFF);
	ctrl &= 0xFC; // Mask out SEL0,SEL1 (bits [0:1])
	ctrl |= cutoff;
	writeRegister(HP_FILTER_CUTOFF, ctrl);
}

//  BYPASS HIGH-PASS FILTER FOR PULSE PROCESSING
//	If true, output of pulse processing bypasses the high-pass filter
//	Possible values are true (bypass) and false (use HP filter)
void MMA8452Q::bypassHPonPulse(bool bypass)
{
	// Must be in standby mode to make changes!!!
	byte ctrl = readRegister(HP_FILTER_CUTOFF);
	ctrl &= 0xDF; // Mask out Pulse_HPF_BYP bit (bit 5)
	ctrl |= (bypass << 5);
	writeRegister(HP_FILTER_CUTOFF, ctrl);
}

//  ENABLE LOW-PASS FILTER FOR PULSE PROCESSING
//	If true, output of pulse processing is low-pass filtered
//	Possible values are true (bypass) and false (use HP filter)
void MMA8452Q::enableLPonPulse(bool enable)
{
	// Must be in standby mode to make changes!!!
	byte ctrl = readRegister(HP_FILTER_CUTOFF);
	ctrl &= 0xEF; // Mask out Pulse_LPF_EN bit (bit 4)
	ctrl |= (enable << 4);
	writeRegister(HP_FILTER_CUTOFF, ctrl);
}

// ENABLE EVENT LATCH
// When enabled, the register FF_MT_SRC will remain high
// until it is cleared, once freefall or motion is detected.
// When disabled (false), FF_MT_SRC will show the real-time detection status.
// Possible values are true (hold latch) or false (real-time status)
void MMA8452Q::enableFFMotionEventLatch(bool enable)
{
	// Must be in standby mode to make changes!!!
	byte ctrl = readRegister(FF_MT_CFG);
	ctrl &= 0x7F; // Mask out ELE bit (bit 7)
	ctrl |= (enable << 7);
	writeRegister(FF_MT_CFG, ctrl);
}

// CHOOSE FREEFALL OR MOTION DETECTION
// The detector can sense one or the other (rather self-explanatory)
// Possible values: FREEFALL, MOTION
void MMA8452Q::chooseFFMotionDetection(MMA8452Q_FF_MT_Selection FF_or_MT)
{
	// Must be in standby mode to make changes!!!
	byte ctrl = readRegister(FF_MT_CFG);
	ctrl &= 0xBF; // Mask out OAE bit (bit 6)
	ctrl |= (FF_or_MT << 6);
	writeRegister(FF_MT_CFG, ctrl);
}

// CHOOSE AXES FOR FREEFALL/MOTION DETECTION
// The freefall/motion detector can trigger on any combination of axes.
// Possible values: X, Y, Z, XY, XZ, YZ, XYZ
void MMA8452Q::setFFMotionAxes(MMA8452Q_FF_MT_EventAxes axes)
{
	// Must be in standby mode to make changes!!!
	byte ctrl = readRegister(FF_MT_CFG);
	ctrl &= 0xC7; // Mask out XEFE,YEFE,ZEFE bits (bits [3:5])
	ctrl |= (axes << 3);
	writeRegister(FF_MT_CFG, ctrl);
}

// CLEAR FREEFALL/MOTION INTERRUPT
// The interrupt is cleared by reading the register which returns
// the following information:
// FFMotionIntData.isDetected: true if an enabled axis triggered the interrupt
// FFMotionIntData.axes: Boolean true/false indicators of the detected 
//				status on the X, Y, and Z axes respectively.  For example, 
//				a return of 2 (binary 0010) means that Y was detected, 
//				a return of 7 (binary 0111) means that X, Y, and Z were all triggered.
//				Possible values: decimal values 0-7.
// FFMotionIntData.sign: Boolean sign (0 = "+", 1 = "-") of the detected 
//				event on the X, Y, and Z axes respectively.  For example, 
//				a return of 3 (binary 0011) means that the disturbance on 
//				X was negative, on Y was negative, and on Z was positive.  
//				The sign is relative to the threshold values set in FF_MT_THS.
FFMotionIntData MMA8452Q::clearFFMotionInterrupt(){
	FFMotionIntData data;
	byte reg = readRegister(FF_MT_SRC);

	// Mask EA bit (bit 7)
	data.isDetected = (bool)(reg & 0x80);

	// Mask individual bits and combine
	// X: bit 1, Y: bit 3, Z: bit 5
	byte xVal = reg & 0x02;
	byte yVal = reg & 0x08;
	byte zVal = reg & 0x20;
	// Reorder bits to be at bits 0:2
	data.axes = (xVal >> 1) | (yVal >> 2) | (zVal >> 3);

	// Mask individual bits and combine
	// X: bit 1, Y: bit 3, Z: bit 5
	byte xSign = reg & 0x01;
	byte ySign = reg & 0x04;
	byte zSign = reg & 0x10;
	// Reorder bits to be at bits 0:2
	data.sign = xSign | (ySign >> 1) | (zSign >> 2);
	
	return data;
}

// SETUP FREEFALL/MOTION THRESHOLD AND DEBOUNCE BEHAVIOR
// INPUTS:
//		decrementORreset (bool): When a freefall/motion event is detected, 
//			it increments a debounce counter. When the event is no longer 
//			present, you can choose to either decrement (true) or reset (false) 
//			the debounce counter.  Decrementing acts as a median filter.
//
//		threshold (float): Threshold in g's for triggering freefall/motion
//			interrupt.  The maximum value is 8g, with resolution of 0.063g.
void MMA8452Q::setFFMotionThreshold(float threshold, bool decrementORreset){
	// Must be in standby mode to make changes!!!

	// Convert value to counts between 0-127, place in bits 0:6
	byte reg = (byte)(threshold * 15.875);
	// Place bool flag in bit 7
	reg |= ((byte)decrementORreset) << 7;
	writeRegister(FF_MT_THS, reg);
}

// SET MIN SAMPLES IN DEBOUNCE COUNTER FOR FREEFALL/MOTION INTERRUPT TRIGGER
// Here you can set a value between 0-255 for the number of
// counts needed in the debounce counter before the interrupt
// flag is thrown.  Time constant depends on data rate and oversampling mode.
void MMA8452Q::setFFMotionDebounceSamples(byte samples)
{
	// Must be in standby mode to make changes!!!
	writeRegister(FF_MT_COUNT, samples);
}

// SET THE OUTPUT DATA RATE
//	This function sets the output data rate of the MMA8452Q.
//	Possible values for the odr parameter are: ODR_800, ODR_400, ODR_200, 
//	ODR_100, ODR_50, ODR_12, ODR_6, or ODR_1
void MMA8452Q::setODR(MMA8452Q_ODR odr)
{
	// Must be in standby mode to make changes!!!
	byte ctrl = readRegister(CTRL_REG1);
	ctrl &= 0xC7; // Mask out data rate bits [3:5]
	ctrl |= (odr << 3);
	writeRegister(CTRL_REG1, ctrl);
}

// SET THE SLEEP OUTPUT DATA RATE
//	This function sets the sleep output data rate of the MMA8452Q.
//	Possible values for the odr parameter are: 
//	ODR_SLEEP_50, ODR_SLEEP_12, ODR_SLEEP_6, ODR_SLEEP_1
void MMA8452Q::setSleepRate(MMA8452Q_Sleep_Rate rate)
{
	// Must be in standby mode to make changes!!!
	byte ctrl = readRegister(CTRL_REG1);
	ctrl &= 0x3F; // Mask out data rate bits [6:7]
	ctrl |= (rate << 6);
	writeRegister(CTRL_REG1, ctrl);
}

//  CHOOSE BETWEEN LOW NOISE VS. MAX DYNAMIC RANGE
//	This function sets the low noise flag of the MMA8452Q.
//	Possible values are MAX_DYNAMIC_RANGE, LOW_NOISE
//  When LOW_NOISE, dynamic range is limited to 4g,
//  regardless of full scale range set in XYZ_DATA_CFG.
void MMA8452Q::setNoiseMode(MMA8452Q_Mode mode)
{
	// Must be in standby mode to make changes!!!
	byte ctrl = readRegister(CTRL_REG1);
	ctrl &= 0xFB; // Mask LNOISE Bit 2
	ctrl |= (mode << 2);
	writeRegister(CTRL_REG1, ctrl);
}

//  ENABLE/DISABLE FAST READ
//	This function sets the F_READ flag of the MMA8452Q.
//	Possible values are true or false
//  When true, fast I2C is enabled. When false, standard I2C read.
//  Default value if never called is false.
void MMA8452Q::enableFastRead(bool enable)
{
	// Must be in standby mode to make changes!!!
	byte ctrl = readRegister(CTRL_REG1);
	ctrl &= 0xFD; // Mask F_READ Bit 1
	ctrl |= (enable << 1);
	writeRegister(CTRL_REG1, ctrl);
}

//  SET THE POWER SCHEME FOR WAKE OPERATION
//	This function sets the wake-mode power scheme of the MMA8452Q.
//	Possible values for the mode parameter are (in descending consumption): 
//	HIGH_RES, NORMAL, LOWNOISE_LOWPOWER,LOW_POWER
void MMA8452Q::setWakeOversampling(MMA8452Q_Oversampling os)
{
	// Must be in standby mode to make changes!!!
	byte ctrl = readRegister(CTRL_REG2);
	ctrl &= 0xFC; // Mask MODS0 MODS1 [0:1]
	ctrl |= os;
	writeRegister(CTRL_REG2, ctrl);
}

//  SET THE POWER SCHEME FOR WAKE OPERATION
//	This function sets the sleep-mode power scheme of the MMA8452Q.
//	Possible values for the mode parameter are (in descending consumption): 
//	HIGH_RES, NORMAL, LOWNOISE_LOWPOWER,LOW_POWER
void MMA8452Q::setSleepOversampling(MMA8452Q_Oversampling os)
{
	// Must be in standby mode to make changes!!!
	byte ctrl = readRegister(CTRL_REG2);
	ctrl &= 0xFC; // Mask SMODS0 SMODS1 [3:4]
	ctrl |=  (os << 3);
	writeRegister(CTRL_REG2, ctrl);
}

//  ENABLE Auto-Sleep (Interrupts must be configured using CTRL_REG3)
//	This function enables the Auto-wake/sleep functionality of the MMA8452Q.
//	Enabled using true input.
void MMA8452Q::enableAutoSleep(bool enable)
{
	// Must be in standby mode to make changes!!!
	byte ctrl = readRegister(CTRL_REG2);
	ctrl &= 0xFB; // Mask SLPE Bit 2
	ctrl |=  (enable << 2);
	writeRegister(CTRL_REG2, ctrl);
}

// WAKE ON TRANSIENT
// This function sets the WAKE_TRANS bit of the MMA8452Q.
// Possible values are true or false (self-explanatory)
// When true, an a transient function interrupt can wake the system.
void MMA8452Q::wakeOnTransient(bool enable)
{
	// Must be in standby mode to make changes!!!
	byte ctrl = readRegister(CTRL_REG3);
	ctrl &= 0xBF; // Mask WAKE_TRANS (bit 6)
	ctrl |= (enable << 6);
	writeRegister(CTRL_REG3, ctrl);
}

// WAKE ON LANDSCAPE/PORTRAIT CHANGE
// This function sets the WAKE_LNDPRT bit of the MMA8452Q.
// Possible values are true or false (self-explanatory)
// When true, an an orientation function interrupt can wake the system.
void MMA8452Q::wakeOnLandPortChange(bool enable)
{
	// Must be in standby mode to make changes!!!
	byte ctrl = readRegister(CTRL_REG3);
	ctrl &= 0xDF; // Mask WAKE_LNDPRT (bit 5)
	ctrl |= (enable << 5);
	writeRegister(CTRL_REG3, ctrl);
}

// WAKE ON PULSE
// This function sets the WAKE_PULSE bit of the MMA8452Q.
// Possible values are true or false (self-explanatory)
// When true, an a pulse function interrupt can wake the system.
void MMA8452Q::wakeOnPulse(bool enable)
{
	// Must be in standby mode to make changes!!!
	byte ctrl = readRegister(CTRL_REG3);
	ctrl &= 0xEF; // Mask WAKE_PULSE (bit 4)
	ctrl |= (enable << 4);
	writeRegister(CTRL_REG3, ctrl);
}

// WAKE ON FREEFALL/MOTION
// This function sets the WAKE_FF_MT bit of the MMA8452Q.
// Possible values are true or false (self-explanatory)
// When true, an a pulse function interrupt can wake the system.
void MMA8452Q::wakeOnFFMotion(bool enable)
{
	// Must be in standby mode to make changes!!!
	byte ctrl = readRegister(CTRL_REG3);
	ctrl &= 0xF7; // Mask WAKE_FF_MT (bit 3)
	ctrl |= (enable << 3);
	writeRegister(CTRL_REG3, ctrl);
}

// SET INTERRUPT POLARITY
// This function sets the IPOL bit of the MMA8452Q.
// Possible values are ACTIVE_HIGH or ACTIVE_LOW
// Default value is ACTIVE_LOW if not set
void MMA8452Q::setInterruptPolarity(MMA8452Q_IntPolarity polarity)
{
	// Must be in standby mode to make changes!!!
	byte ctrl = readRegister(CTRL_REG3);
	ctrl &= 0xFD; // Mask IPOL (bit 1)
	ctrl |= (polarity << 1);
	writeRegister(CTRL_REG3, ctrl);
}

// SET INTERRUPT PIN CONFIGURATION
// This function sets the PP_OD bit of the MMA8452Q.
// Possible values are ACTIVE_HIGH or ACTIVE_LOW
// Default value is ACTIVE_LOW if not set
void MMA8452Q::setInterruptPinConfig(MMA8452Q_IntPinCfg pinCfg)
{
	// Must be in standby mode to make changes!!!
	byte ctrl = readRegister(CTRL_REG3);
	ctrl &= 0xFE; // Mask PP_OD (bit 0)
	ctrl |= pinCfg;
	writeRegister(CTRL_REG3, ctrl);
}

// ENABLE SLEEP INTERRUPT
// This function sets the INT_EN_ASLP bit of the MMA8452Q.
// Possible values are true or false
// Default value is false if not set
void MMA8452Q::enableSleepInt(bool enable)
{
	// Must be in standby mode to make changes!!!
	byte ctrl = readRegister(CTRL_REG4);
	ctrl &= 0x7F; // Mask INT_EN_ASLP (bit 7)
	ctrl |= (enable << 7);
	writeRegister(CTRL_REG4, ctrl);
}

// ENABLE TRANSIENT INTERRUPT
// This function sets the INT_EN_TRANS bit of the MMA8452Q.
// Possible values are true or false
// Default value is false if not set
void MMA8452Q::enableTransientInt(bool enable)
{
	// Must be in standby mode to make changes!!!
	byte ctrl = readRegister(CTRL_REG4);
	ctrl &= 0xDF; // Mask INT_EN_TRANS (bit 5)
	ctrl |= (enable << 5);
	writeRegister(CTRL_REG4, ctrl);
}

// ENABLE LANDSCAPE/PORTRAIT INTERRUPT
// This function sets the INT_EN_LNDPRT bit of the MMA8452Q.
// Possible values are true or false
// Default value is false if not set
void MMA8452Q::enableLandPortInt(bool enable)
{
	// Must be in standby mode to make changes!!!
	byte ctrl = readRegister(CTRL_REG4);
	ctrl &= 0xEF; // Mask INT_EN_LNDPRT (bit 4)
	ctrl |= (enable << 4);
	writeRegister(CTRL_REG4, ctrl);
}

// ENABLE PULSE INTERRUPT
// This function sets the INT_EN_PULSE bit of the MMA8452Q.
// Possible values are true or false
// Default value is false if not set
void MMA8452Q::enablePulseInt(bool enable)
{
	// Must be in standby mode to make changes!!!
	byte ctrl = readRegister(CTRL_REG4);
	ctrl &= 0xF7; // Mask INT_EN_PULSE (bit 3)
	ctrl |= (enable << 3);
	writeRegister(CTRL_REG4, ctrl);
}

// ENABLE FREEFALL/MOTION INTERRUPT
// This function sets the INT_EN_FF_MT bit of the MMA8452Q.
// Possible values are true or false
// Default value is false if not set
void MMA8452Q::enableFFMotionInt(bool enable)
{
	// Must be in standby mode to make changes!!!
	byte ctrl = readRegister(CTRL_REG4);
	ctrl &= 0xFB; // Mask INT_EN_FF_MT (bit 2)
	ctrl |= (enable << 2);
	writeRegister(CTRL_REG4, ctrl);
}

// ENABLE DATA READY INTERRUPT
// This function sets the INT_EN_DRDY bit of the MMA8452Q.
// Possible values are true or false
// Default value is false if not set
void MMA8452Q::enableDataReadyInt(bool enable)
{
	// Must be in standby mode to make changes!!!
	byte ctrl = readRegister(CTRL_REG4);
	ctrl &= 0xFE; // Mask INT_EN_DRDY (bit 0)
	ctrl |= enable;
	writeRegister(CTRL_REG4, ctrl);
}

// ROUTE SLEEP INTERRUPT
// This function sets the INT_CFG_ASLP bit of the MMA8452Q.
// Possible values are INT_PIN1 and INT_PIN2
// Default value is INT_PIN2 if not set
void MMA8452Q::routeSleepInt(MMA8452Q_IntPinRoute interruptPin)
{
	// Must be in standby mode to make changes!!!
	byte ctrl = readRegister(CTRL_REG5);
	ctrl &= 0x7F; // Mask INT_CFG_ASLP (bit 7)
	ctrl |= (interruptPin << 7);
	writeRegister(CTRL_REG5, ctrl);
}

// ROUTE TRANSIENT INTERRUPT
// This function sets the INT_CFG_TRANS bit of the MMA8452Q.
// Possible values are INT_PIN1 and INT_PIN2
// Default value is INT_PIN2 if not set
void MMA8452Q::routeTransientInt(MMA8452Q_IntPinRoute interruptPin)
{
	// Must be in standby mode to make changes!!!
	byte ctrl = readRegister(CTRL_REG5);
	ctrl &= 0xDF; // Mask INT_CFG_TRANS (bit 5)
	ctrl |= (interruptPin << 5);
	writeRegister(CTRL_REG5, ctrl);
}

// ROUTE LANDSCAPE/PORTRAIT INTERRUPT
// This function sets the INT_CFG_LNDPRT bit of the MMA8452Q.
// Possible values are INT_PIN1 and INT_PIN2
// Default value is INT_PIN2 if not set
void MMA8452Q::routeLandPortInt(MMA8452Q_IntPinRoute interruptPin)
{
	// Must be in standby mode to make changes!!!
	byte ctrl = readRegister(CTRL_REG5);
	ctrl &= 0xEF; // Mask INT_CFG_LNDPRT (bit 4)
	ctrl |= (interruptPin << 4);
	writeRegister(CTRL_REG5, ctrl);
}

// ROUTE PULSE INTERRUPT
// This function sets the INT_CFG_PULSE bit of the MMA8452Q.
// Possible values are INT_PIN1 and INT_PIN2
// Default value is INT_PIN2 if not set
void MMA8452Q::routePulseInt(MMA8452Q_IntPinRoute interruptPin)
{
	// Must be in standby mode to make changes!!!
	byte ctrl = readRegister(CTRL_REG5);
	ctrl &= 0xF7; // Mask INT_CFG_PULSE (bit 3)
	ctrl |= (interruptPin << 3);
	writeRegister(CTRL_REG5, ctrl);
}

// ROUTE FREEFALL/MOTION INTERRUPT
// This function sets the INT_CFG_FF_MT bit of the MMA8452Q.
// Possible values are INT_PIN1 and INT_PIN2
// Default value is INT_PIN2 if not set
void MMA8452Q::routeFFMotionInt(MMA8452Q_IntPinRoute interruptPin)
{
	// Must be in standby mode to make changes!!!
	byte ctrl = readRegister(CTRL_REG5);
	ctrl &= 0xFB; // Mask INT_CFG_FF_MT (bit 2)
	ctrl |= (interruptPin << 2);
	writeRegister(CTRL_REG5, ctrl);
}

// ROUTE DATA READY INTERRUPT
// This function sets the INT_CFG_DRDY bit of the MMA8452Q.
// Possible values are INT_PIN1 and INT_PIN2
// Default value is INT_PIN2 if not set
void MMA8452Q::routeDataReadyInt(MMA8452Q_IntPinRoute interruptPin)
{
	// Must be in standby mode to make changes!!!
	byte ctrl = readRegister(CTRL_REG5);
	ctrl &= 0xFE; // Mask INT_CFG_DRDY (bit 0)
	ctrl |= interruptPin;
	writeRegister(CTRL_REG5, ctrl);
}

// SET UP TAP DETECTION
//	This function can set up tap detection on the x, y, and/or z axes.
//	The xThs, yThs, and zThs parameters serve two functions:
//		1. Enable tap detection on an axis. If the 7th bit is SET (0x80)
//			tap detection on that axis will be DISABLED.
//		2. Set tap g's threshold. The lower 7 bits will set the tap threshold
//			on that axis.
void MMA8452Q::setupTap(byte xThs, byte yThs, byte zThs)
{
	// Set up single and double tap - 5 steps:
	// for more info check out this app note:
	// http://cache.freescale.com/files/sensors/doc/app_note/AN4072.pdf
	// Set the threshold - minimum required acceleration to cause a tap.
	byte temp = 0;
	if (!(xThs & 0x80)) // If top bit ISN'T set
	{
		temp |= 0x3; // Enable taps on x
		writeRegister(PULSE_THSX, xThs);  // x thresh
	}
	if (!(yThs & 0x80))
	{
		temp |= 0xC; // Enable taps on y
		writeRegister(PULSE_THSY, yThs);  // y thresh
	}
	if (!(zThs & 0x80))
	{
		temp |= 0x30; // Enable taps on z
		writeRegister(PULSE_THSZ, zThs);  // z thresh
	}
	// Set up single and/or double tap detection on each axis individually.
	writeRegister(PULSE_CFG, temp | 0x40);
	// Set the time limit - the maximum time that a tap can be above the thresh
	writeRegister(PULSE_TMLT, 0x30);  // 30ms time limit at 800Hz odr
	// Set the pulse latency - the minimum required time between pulses
	writeRegister(PULSE_LTCY, 0xA0);  // 200ms (at 800Hz odr) between taps min
	// Set the second pulse window - maximum allowed time between end of
	//	latency and start of second pulse
	writeRegister(PULSE_WIND, 0xFF);  // 5. 318ms (max value) between taps max
}

// READ TAP STATUS
//	This function returns any taps read by the MMA8452Q. If the function 
//	returns no new taps were detected. Otherwise the function will return the
//	lower 7 bits of the PULSE_SRC register.
byte MMA8452Q::readTap()
{
	byte tapStat = readRegister(PULSE_SRC);
	if (tapStat & 0x80) // Read EA bit to check if a interrupt was generated
	{
		return tapStat & 0x7F;
	}
	else
		return 0;
}

// SET UP PORTRAIT/LANDSCAPE DETECTION
//	This function sets up portrait and landscape detection.
void MMA8452Q::setupPL()
{
	// Must be in standby mode to make changes!!!
	// For more info check out this app note:
	//	http://cache.freescale.com/files/sensors/doc/app_note/AN4068.pdf
	// 1. Enable P/L
	writeRegister(PL_CFG, readRegister(PL_CFG) | 0x40); // Set PL_EN (enable)
	// 2. Set the debounce rate
	writeRegister(PL_COUNT, 0x50);  // Debounce counter at 100ms (at 800 hz)
}

// READ PORTRAIT/LANDSCAPE STATUS
//	This function reads the portrait/landscape status register of the MMA8452Q.
//	It will return either PORTRAIT_U, PORTRAIT_D, LANDSCAPE_R, LANDSCAPE_L,
//	or LOCKOUT. LOCKOUT indicates that the sensor is in neither p or ls.
byte MMA8452Q::readPL()
{
	byte plStat = readRegister(PL_STATUS);
	
	if (plStat & 0x40) // Z-tilt lockout
		return LOCKOUT;
	else // Otherwise return LAPO status
		return (plStat & 0x6) >> 1;
}

// SETUP AUTO-WAKE/SLEEP CONFIGURATION, as per AN4601 (application note)
void MMA8452Q::setupAutoWakeSleep()
{
	// Reset all values to default
	reset();

	// System must be in standby to change most register values
	standby();

	// Set data rates (CTRL_REG1 values)
	setODR(ODR_50);
	setSleepRate(ODR_SLEEP_12);
	setNoiseMode(LOW_NOISE);
	enableFastRead(false);
	
	// Enable sleep, normal mode
	enableAutoSleep(true);
	setWakeOversampling(NORMAL);
	setSleepOversampling(NORMAL);

	// Set wake-up condition
	// transient enabled wake up from auto-sleep
	wakeOnTransient(true);

	// Enable interrupts
	// ASLP INT enabled, Transient INT enabled
	enableSleepInt(true);
	enableTransientInt(true);

	// Route interrupts to external pins
	// auto-sleep -> INT1,transient -> INT2
	routeSleepInt(INT_PIN1);
	routeTransientInt(INT_PIN2);

	// HPF output enabled , 2g mode
	setScale(SCALE_2G);
	enableHighPassOutput(true);

	// HPF=2Hz (since ODR is 50Hz)
	setHPCutoff(HP0);
	bypassHPonPulse(false);

	// P/L disabled
	writeRegister(PL_CFG,0x80);

	// MT/FF disabled
	writeRegister(FF_MT_CFG,0x00);

	// TRANSIENT
	// ELE latch disabled, HPF enable, transient x/y/z axes enabled
	writeRegister(TRANSIENT_CFG,0x0E);

	// transient threshold=0.126g
	writeRegister(TRANSIENT_THS,0x01);
	writeRegister(TRANSIENT_COUNT,0x00);

	// tap/double tap disabled
	writeRegister(PULSE_CFG,0x00);
	// the minimum time required to be judged from move to stop,set to 3 sec
	// the time can also be set up to 81 sec (when set to 0xFF)
	writeRegister(ASLP_COUNT,0x0A);
	writeRegister(CTRL_REG1,0x65);
}

//// SETUP MOTION DETECTION CONFIGURATION, as per AN4070 (application note)
//void MMA8452Q::setupMotionDetection()
//{
//	// Put the device into Standby Mode: Register 0x2A CTRL_REG1
//	writeRegister(CTRL_REG1,0x18); // Set the device in 100 Hz ODR, Standby
//
//	// Set Configuration Register for Motion Detection by setting the “OR” condition OAE = 1, enabling X, Y, and the latch
//	writeRegister(FF_MT_CFG,0xD8);
//
//	// Threshold Setting Value for the Motion detection of > 3g
//	// Note: The step count is 0.063g/ count
//	// • 3g/0.063g = 47.6; //Round up to 48
//	writeRegister(FF_MT_THS,0x30);
//
//	// Set the debounce counter to eliminate false readings for 100 Hz sample rate with a requirement
//	// of 100 ms timer.
//	// Note: 100 ms/10 ms (steps) = 10 counts
//	writeRegister(FF_MT_COUNT,0x0A);
//
//	// Enable Motion/Freefall Interrupt Function in the System (CTRL_REG4)
//	writeRegister(CTRL_REG4,0x04);
//
//	// Route the Motion/Freefall Interrupt Function to INT1 hardware pin (CTRL_REG5)
//	writeRegister(CTRL_REG5,0x04);
//
//	// Put the device in Active Mode
//	byte c = readRegister(CTRL_REG1);
//	writeRegister(CTRL_REG1,c|0x01);
//}

// SETUP MOTION DETECTION CONFIGURATION, as per AN4070 (application note)
void MMA8452Q::setupMotionDetection()
{
	// Configuration registers must be in stanby before setting
	standby();
	
	// Set wake data rates and max acceleration scale
	setScale(SCALE_2G);
	setODR(ODR_800);

	// Configure Motion Detection Interrupt conditions
	enableFFMotionEventLatch(true);
	chooseFFMotionDetection(MOTION);
	setFFMotionAxes(XY);

	// Threshold Setting Value for Motion detection (Max is 8g)
	setFFMotionThreshold(0.63, false);

	// Set the debounce counter to eliminate false readings for 100 Hz sample rate with a requirement
	// of 100 ms timer.
	// Note: 100 ms/10 ms (steps) = 10 counts
	setFFMotionDebounceSamples(10);

	// Enable Motion/Freefall Interrupt Function in the System (CTRL_REG4)
	enableFFMotionInt(true);

	// Route the Motion/Freefall Interrupt Function to INT1 hardware pin (CTRL_REG5)
	routeFFMotionInt(INT_PIN1);

	// Put the device in Active Mode
	active();
}

//  RESET all registers! (All settings will be lost and zeros will be written!)
void MMA8452Q::reset()
{
	writeRegister(CTRL_REG2, 0x40); //Flip reset bit
}

// SET STANDBY MODE
//	Sets the MMA8452 to standby mode. It must be in standby to change most register settings
void MMA8452Q::standby()
{
	byte c = readRegister(CTRL_REG1);
	writeRegister(CTRL_REG1, c & ~(0x01)); //Clear the active bit to go into standby
}

// SET ACTIVE MODE
//	Sets the MMA8452 to active mode. Needs to be in this mode to output data
void MMA8452Q::active()
{
	byte c = readRegister(CTRL_REG1);
	writeRegister(CTRL_REG1, c | 0x01); //Set the active bit to begin detection
}

// WRITE A SINGLE REGISTER
// 	Write a single byte of data to a register in the MMA8452Q.
void MMA8452Q::writeRegister(MMA8452Q_Register reg, byte data)
{
	writeRegisters(reg, &data, 1);
}

// WRITE MULTIPLE REGISTERS
//	Write an array of "len" bytes ("buffer"), starting at register "reg", and
//	auto-incrmenting to the next.
void MMA8452Q::writeRegisters(MMA8452Q_Register reg, byte *buffer, byte len)
{
	Wire.beginTransmission(address);
	Wire.write(reg);
	for (int x = 0; x < len; x++)
		Wire.write(buffer[x]);
	Wire.endTransmission(); //Stop transmitting
}

// READ A SINGLE REGISTER
//	Read a byte from the MMA8452Q register "reg".
byte MMA8452Q::readRegister(MMA8452Q_Register reg)
{
	Wire.beginTransmission(address);
	Wire.write(reg);
	Wire.endTransmission(false); //endTransmission but keep the connection active

	Wire.requestFrom(address, (byte) 1); //Ask for 1 byte, once done, bus is released by default

	while(!Wire.available()) ; //Wait for the data to come back

	return Wire.read(); //Return this one byte
}

// READ MULTIPLE REGISTERS
//	Read "len" bytes from the MMA8452Q, starting at register "reg". Bytes are stored
//	in "buffer" on exit.
void MMA8452Q::readRegisters(MMA8452Q_Register reg, byte *buffer, byte len)
{
	Wire.beginTransmission(address);
	Wire.write(reg);
	Wire.endTransmission(false); //endTransmission but keep the connection active

	Wire.requestFrom(address, len); //Ask for bytes, once done, bus is released by default

	while(Wire.available() < len); //Hang out until we get the # of bytes we expect

	for(int x = 0 ; x < len ; x++)
		buffer[x] = Wire.read();    
}