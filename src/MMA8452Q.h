/******************************************************************************
SparkFun_MMA8452Q.h
SparkFun_MMA8452Q Library Header File
Jim Lindblom @ SparkFun Electronics
Original Creation Date: June 3, 2014
https://github.com/sparkfun/MMA8452_Accelerometer

This file prototypes the MMA8452Q class, implemented in SFE_MMA8452Q.cpp. In
addition, it defines every register in the MMA8452Q.

Development environment specifics:
	IDE: Arduino 1.0.5
	Hardware Platform: Arduino Uno

	**Updated for Arduino 1.6.4 5/2015**
	
This code is beerware; if you see me (or any other SparkFun employee) at the
local, and you've found our code helpful, please buy us a round!

Distributed as-is; no warranty is given.
******************************************************************************/

#ifndef SparkFun_MMA8452Q_h
#define SparkFun_MMA8452Q_h

#include <Arduino.h>

///////////////////////////////////
// MMA8452Q Register Definitions //
///////////////////////////////////
enum MMA8452Q_Register {
	STATUS = 0x00,				// Real time status
	OUT_X_MSB = 0x01,			// [7:0] are 8 MSBs of 12-bit sample.
	OUT_X_LSB = 0x02,			// [7:4] are 4 LSBs of 12-bit sample.
	OUT_Y_MSB = 0x03,			// [7:0] are 8 MSBs of 12-bit sample.
	OUT_Y_LSB = 0x04,			// [7:4] are 4 LSBs of 12-bit sample.
	OUT_Z_MSB = 0x05,			// [7:0] are 8 MSBs of 12-bit sample.
	OUT_Z_LSB = 0x06,			// [7:4] are 4 LSBs of 12-bit sample.
	SYSMOD = 0x0B,				// Current System Mode
	INT_SOURCE = 0x0C,			// Interrupt status
	WHO_AM_I = 0x0D,			// Device ID (0x2A)
	XYZ_DATA_CFG = 0x0E,		// HPF Data Out and Dynamic Range Settings			
	HP_FILTER_CUTOFF = 0x0F,	// Cutoff frequency is set to 16 Hz @ 800 Hz
	PL_STATUS = 0x10,			// Landscape/Portrait orientation status
	PL_CFG = 0x11,				// Landscape/Portrait configuration
	PL_COUNT = 0x12,			// Landscape/Portrait debounce counter
	PL_BF_ZCOMP = 0x13,			// Back-Front, Z-Lock Trip threshold
	P_L_THS_REG = 0x14,			// Portrait to Landscape Trip Angle is 29°
	FF_MT_CFG = 0x15,			// Freefall/Motion functional block configuration
	FF_MT_SRC = 0x16,			// Freefall/Motion event source register
	FF_MT_THS = 0x17,			// Freefall/Motion threshold register
	FF_MT_COUNT = 0x18,			// Freefall/Motion debounce counter
	TRANSIENT_CFG = 0x1D,		// Transient functional block configuration
	TRANSIENT_SRC = 0x1E,		// Transient event status register
	TRANSIENT_THS = 0x1F,		// Transient event threshold
	TRANSIENT_COUNT = 0x20,		// Transient debounce counter
	PULSE_CFG = 0x21,			// ELE, Double_XYZ or Single_XYZ
	PULSE_SRC = 0x22,			// EA, Double_XYZ or Single_XYZ
	PULSE_THSX = 0x23,			// X pulse threshold
	PULSE_THSY = 0x24,			// Y pulse threshold
	PULSE_THSZ = 0x25,			// Z pulse threshold
	PULSE_TMLT = 0x26,			// Time limit for pulse
	PULSE_LTCY = 0x27,			// Latency time for 2nd pulse
	PULSE_WIND = 0x28,			// Window time for 2nd pulse
	ASLP_COUNT = 0x29,			// Counter setting for Auto-SLEEP
	CTRL_REG1 = 0x2A,			// Data Rate, ACTIVE Mode
	CTRL_REG2 = 0x2B,			// Sleep Enable, OS Modes, RST, ST
	CTRL_REG3 = 0x2C,			// Wake from Sleep, IPOL, PP_OD
	CTRL_REG4 = 0x2D,			// Interrupt enable register
	CTRL_REG5 = 0x2E,			// Interrupt pin (INT1/INT2) map
	OFF_X = 0x2F,				// X-axis offset adjust
	OFF_Y = 0x30,				// Y-axis offset adjust
	OFF_Z = 0x31				// Z-axis offset adjust
};

////////////////////////////////
// MMA8452Q Misc Declarations //
////////////////////////////////
enum MMA8452Q_Scale {SCALE_2G = 2, SCALE_4G = 4, SCALE_8G = 8}; // Possible full-scale settings: SCALE_2G,SCALE_4G,SCALE_8G
enum MMA8452Q_ODR {ODR_800, ODR_400, ODR_200, ODR_100, ODR_50, ODR_12, ODR_6, ODR_1}; // possible data rates: ODR_800, ODR_400, ODR_200, ODR_100, ODR_50, ODR_12, ODR_6, ODR_1
enum MMA8452Q_Sleep_Rate {ODR_SLEEP_50, ODR_SLEEP_12, ODR_SLEEP_6, ODR_SLEEP_1}; // possible data rates in sleep: ODR_SLEEP_50, ODR_SLEEP_12, ODR_SLEEP_6, ODR_SLEEP_1
enum MMA8452Q_Oversampling {NORMAL, LOWNOISE_LOWPOWER,HIGH_RES,LOW_POWER}; // possible modes (see AN4075): NORMAL, LOWNOISE_LOWPOWER,HIGH_RES,LOW_POWER
enum MMA8452Q_Mode {MAX_DYNAMIC_RANGE, LOW_NOISE}; // possible modes (see AN4075): MAX_DYNAMIC_RANGE, LOW_NOISE
enum MMA8452Q_IntPolarity {ACTIVE_LOW, ACTIVE_HIGH}; // possible interrupt polarity: ACTIVE_LOW,ACTIVE_HIGH
enum MMA8452Q_IntPinCfg { PUSH_PULL, OPEN_DRAIN }; // possible interrupt pin configuration: PUSH_PULL, OPEN_DRAIN
enum MMA8452Q_IntPinRoute { INT_PIN2, INT_PIN1 }; // possible interrupt pin configuration: INT_PIN2, INT_PIN1
enum MMA8452Q_HPFiltCutoff{ HP0, HP1, HP2, HP3}; // possible high-pass filter cutoffs: HP0, HP1, HP2, HP3 (see data sheet for more info)
enum MMA8452Q_FF_MT_Selection{ FREEFALL, MOTION }; // possible configurations of the motion/freefall event detector: FREEFALL, MOTION
enum MMA8452Q_FF_MT_EventAxes{ NONE, X, Y, XY, Z, XZ, YZ, XYZ}; // possible axes configurations to set-off motion/freefall detection flag: NONE, X, Y, Z, XY, XZ, YZ, XYZ
enum MMA8452Q_PL_Orientation{PORTRAIT_UP,PORTRAIT_DOWN,LANDSCAPE_RIGHT,LANDSCAPE_LEFT}; // Possible orientations of device:  PORTRAIT_UP,PORTRAIT_DOWN,LANDSCAPE_RIGHT,LANDSCAPE_LEFT (see Figure 3 of data sheet)

typedef struct{
	bool isDetected;
	byte axes;
	byte sign;
}FFMotionIntData;

typedef struct{
	bool newPLChange;
	bool zTiltDetected;
	MMA8452Q_PL_Orientation orientation;
	bool backORfront;
}PLStatusData;

typedef struct{
	short x;
	short y;
	short z;
}rawData;

typedef struct{
	float x;
	float y;
	float z;
}scaledData;

typedef struct{
	rawData raw;
	scaledData scaled;
}accelData;

// Possible portrait/landscape settings
#define PORTRAIT_U 0
#define PORTRAIT_D 1
#define LANDSCAPE_R 2
#define LANDSCAPE_L 3
#define LOCKOUT 0x40

////////////////////////////////
// MMA8452Q Class Declaration //
////////////////////////////////
class MMA8452Q
{
public:	
    MMA8452Q(byte addr = 0x1D); // Constructor
	
	void resetRegisters();
	byte init(MMA8452Q_Scale fsr = SCALE_2G, MMA8452Q_ODR odr = ODR_800);
	byte readTap();
	byte readPL();

	void setupAutoSleep(MMA8452Q_Sleep_Rate sleepRate, MMA8452Q_Oversampling powerMode, byte wakeTriggers, float sleepTime, MMA8452Q_IntPinRoute intPin);
	void setupFreefallOrMotionDetection(MMA8452Q_FF_MT_Selection FForMT, MMA8452Q_FF_MT_EventAxes axes, float threshold_g, byte debounceCounts, MMA8452Q_IntPinRoute intPin);
	void setupPortraitLandscapeDetection(byte debounceCounts, MMA8452Q_IntPinRoute intPin);
	void clearAllInterrupts();

    short x, y, z;
	float cx, cy, cz;
	MMA8452Q_ODR m_odr;

//private:
	byte address;
	MMA8452Q_Scale scale;
	
	void setupTap(byte xThs, byte yThs, byte zThs);
	void setupPL();

	// STATUS (0x00), Data Status Register (Read Only)
	byte available();

	// Data Registers (0x01-0x06), (Read Only)
	void read();
	accelData getData();

	//SYSMOD (0x0B), System Mode Register (Read Only)
	byte getSystemMode();

	//INT_SOURCE (0x0C), System Interrupt Status Register (Read Only)
	byte getInterruptSources();

	//WHO_AM_I (0x0D), Device ID Register (Read Only)
	byte whoAmI();

	//XYZ_DATA_CFG (0x0E), XYZ Data Configuration Register (Read/Write)
	void setScale(MMA8452Q_Scale fsr);
	void enableHighPassOutput(bool enable);

	//HP_FILTER_CUTOFF (0x0F), High-Pass Filter Register (Read/Write)
	void setHPCutoff(MMA8452Q_HPFiltCutoff cutoff);
	void bypassHPonPulse(bool bypass);
	void enableLPonPulse(bool enable);

	//PL_STATUS (0x10), Portrait/Landscape Status Register (Read Only)
	PLStatusData getPLStatus();

	//PL_CFG (0x11), Portait/Landscape Configuration Register (Read/Write)
	void enablePLDetection(bool enable);

	// PL_COUNT (0x12), Portrait/Landscape Debounce Register (Read/Write)
	void setPLDebounceSamples(byte samples, bool decrementORreset);

	//FF_MT_CFG (0x15), Freefall/Motion Configuration Register (Read/Write)
	void enableFFMotionEventLatch(bool enable);
	void chooseFFMotionDetection(MMA8452Q_FF_MT_Selection FF_or_MT);
	void setFFMotionAxes(MMA8452Q_FF_MT_EventAxes axes);

	//FF_MT_SRC (0x16), Freefall/Motion Source Register (Read Only)
	FFMotionIntData clearFFMotionInterrupt();

	//FF_MT_THS (0x17), Freefall/Motion Threshold Register (Read/Write)
	void setFFMotionThreshold(float threshold);

	//FF_MT_COUNT (0x18), Debounce Register (Read/Write)
	void setFFMotionDebounceSamples(byte samples, bool decrementORreset);

	//ASLP_COUNT (0x29),  Sleep time register (Read/Write)
	void setSleepTime(float time);

	//CTRL_REG1
	void standby();
	void active();
	void setODR(MMA8452Q_ODR odr);
	void setSleepRate(MMA8452Q_Sleep_Rate rate);
	void setNoiseMode(MMA8452Q_Mode mode);
	void enableFastRead(bool enable);

	//CTRL_REG2
	void reset();
	void enableAutoSleep(bool enable);
	void setWakeOversampling(MMA8452Q_Oversampling os);
	void setSleepOversampling(MMA8452Q_Oversampling os);

	//CTRL_REG3
	void wakeOnTransient(bool enable);
	void wakeOnLandPortChange(bool enable);
	void wakeOnPulse(bool enable);
	void wakeOnFFMotion(bool enable);
	void setInterruptPolarity(MMA8452Q_IntPolarity polarity);
	void setInterruptPinConfig(MMA8452Q_IntPinCfg pinCfg);

	//CTRL_REG4
	void enableSleepInt(bool enable);
	void enableTransientInt(bool enable);
	void enableLandPortInt(bool enable);
	void enablePulseInt(bool enable);
	void enableFFMotionInt(bool enable);
	void enableDataReadyInt(bool enable);

	//CTRL_REG4
	void routeSleepInt(MMA8452Q_IntPinRoute interruptPin);
	void routeTransientInt(MMA8452Q_IntPinRoute interruptPin);
	void routeLandPortInt(MMA8452Q_IntPinRoute interruptPin);
	void routePulseInt(MMA8452Q_IntPinRoute interruptPin);
	void routeFFMotionInt(MMA8452Q_IntPinRoute interruptPin);
	void routeDataReadyInt(MMA8452Q_IntPinRoute interruptPin);


	void writeRegister(MMA8452Q_Register reg, byte data);
    void writeRegisters(MMA8452Q_Register reg, byte *buffer, byte len);
	byte readRegister(MMA8452Q_Register reg);
    void readRegisters(MMA8452Q_Register reg, byte *buffer, byte len);
};

#endif