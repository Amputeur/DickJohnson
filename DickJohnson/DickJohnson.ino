#include <LiquidCrystal.h>
#include "avr/eeprom.h"

#define DEBUG_SERIAL
#define ENABLE_SAVE_TO_EEPROM
#define EXTERNAL_PUMP

#define SAVE_DATA_VERSION 4
#define EEPROM_VERSION_ADDRESS 0
#define JOB_CONFIG_RING_COUNT 8
#define ROD_COUNT_RING_COUNT  32
#define STATS_AVERAGE_TIME_SAMPLING_COUNT 10

#define MIN_ROD_SIZE 0.25f * ROD_SIZE_MULTIPLICATOR
#define MAX_ROD_SIZE 1.25f * ROD_SIZE_MULTIPLICATOR
#define ROD_SIZE_MULTIPLICATOR 2000

#define MIN_EXTRUDE_LENGTH 1.0f * EXTRUDE_LENGTH_MULTIPLICATOR
#define MAX_EXTRUDE_LENGTH 6.0f * EXTRUDE_LENGTH_MULTIPLICATOR
#define EXTRUDE_LENGTH_MULTIPLICATOR 1250

#define STOPPER_THICKNESS 0.375f * positionMultiplicator
#define STOPPER_PADDING 0.25f * positionMultiplicator
#define STOPPER_TIMER 75ul

#define MAX_PRESSURE 512
#define HOME_PRESSURE 400
#define PRESSURE_ANALOG_TO_PSI 1.953125f

#define RECALIBRATE_HOLD_TIME 1500l
#define NON_CRITICAL_INPUTS_HOLD_TIME 150l

#define PISTON_POSITION_ZERO_OFFSET 10000
#define PISTON_POSITION_PADDING 0.125f * positionMultiplicator

#define VICE_RAISE_TIMER_MIN 100
#define VICE_RAISE_TIMER_MAX 5000

#define LEARNING_COUNT 5
#define LEARNING_DELAY 100ul

#define COUP_BELIER_DELAY 90

#define COVER_OPENNED_PAUSE_DELAY 100ul

#define THREAD_NC true
#define THREAD_NF false
#define UNIT_MM true
#define UNIT_INCH false
#define UnitType bool
#define ThreadType bool

#define CONFIG_ENCODER_A 2
#define CONFIG_ENCODER_A_INTERRUPT 0
#define CONFIG_ENCODER_B 3

#define PISTON_POSITION_ENCODER_A 20
#define PISTON_POSITION_ENCODER_A_INTERRUPT 3
#define PISTON_POSITION_ENCODER_B 21

#define IN_MODE_MANUAL 36
#define IN_MODE_AUTO 44

#define OUT_VALVE_FORWARD 27
#define OUT_VALVE_FORWARD_LED 47
#define OUT_VALVE_BACKWARD 29
#define OUT_VALVE_BACKWARD_LED 45

#define IN_UNIT_SELECTOR 48
#define IN_THREAD_TYPE_SELECTOR 19
#define IN_SET_EXTRUDE_LENGTH 50
#define IN_SET_ROD_SIZE 52
#define IN_HOME 46

#define IN_ANALOG_PRESSURE 0

#define OUT_RAISE_STOP 23
#define OUT_LOWER_STOP 25
#define IN_STOP_RAISED 22
#define IN_STOP_LOWERED 34
#define OUT_STOPPER_RAISED_LED 43

#define IN_MANUAL_TOGGLE_STOPPER 16
#define IN_MANUAL_OPEN_VICE 17
#define IN_MANUAL_CLOSE_VICE 18
#define IN_MANUAL_PISTON_FORWARD 24
#define IN_MANUAL_PISTON_BACKWARD 28

#define OUT_VICE_OPEN 31
#define OUT_VICE_OPEN_LED 49
#define OUT_VICE_CLOSE 33
#define OUT_VICE_CLOSE_LED 51

#define OUT_DROP_OIL 35

#define IN_PEDAL 38

#define IN_START_PUMP 32
#define IN_STOP_PUMP 40
#define OUT_PUMP 37

#define IN_COOLANT 30
#define OUT_COOLANT 53
#define OUT_COOLANT_LED 41

#define OUT_RELAY_ACTIVATOR 39

#define IN_PANIC 42
#define IN_COVER 15

#define IN_MAXIMUM_PISTON_POSITION 26

enum Mode {
	ModeNone,
	ModeInit,
	ModeManual,
	ModeAuto
};
Mode currentMode = ModeNone;

enum InitState {
	InitStateWaiting,
	InitStateZeroing,
	InitStateCalibrateStroke,
	InitStateCalibrateStopper,
	InitStateCalibrateViceRaiseTime,
	InitStateCalibrateVicePressure,
	InitStateCalibrateExtrudePressure,
};
InitState initState = InitStateWaiting;

enum Message {
	MessageNone,
	MessageErrorPumpNotStarted,
	MessageErrorSystemNotInitialized,
	MessageErrorExtrudePressure,
	MessageErrorAutoModeGeneric,
	MessageErrorCoverOpenned,
	MessageConfigModePumpNotStarted,
	MessageConfigModeNotInitialized,
	MessageConfigModeMissingConfig,
	MessageConfigModeZeroInProgress,
	MessageConfigModeCalibrateStroke,
	MessageConfigModeCalibrateStrokeEnterReal,
	MessageConfigModeCalibrateStropper,
	MessageConfigModeCalibrateViceRaiseTimer,
	MessageConfigModeCalibrateVicePressure,
	MessageConfigModeCalibrateExtrudePressure,
	MessageConfigModeInitialized,
	MessageAutoModeRunning,
	MessageAutoModeStats,
	MessagePANIC,

	MessageCount,
};

char* messages[MessageCount][MessageCount] = {
	{" A BEbit Studio", "    MACHINE."},	//	MessageNone
	{"Start the pump.", ""},	//	MessageErrorPumpNotStarted
	{"Init the system", "first."},	//	MessageErrorSystemNotInitialized
	{"Max pressure", "reached."},	//	MessageErrorExtrudePressure
	{"Error.", "Reset Auto Mode"}, 	//	MessageErrorAutoModeGeneric
	{"Error.", "Close the cover."}, 	//	MessageErrorCoverOpenned
	{"D:      L:", "Start the pump."},	//	MessageConfigModePumpNotStarted
	{"D:      L:", "Press HOME"},	//	MessageConfigModeNotInitialized
	{"D:      L:", "Missing Config"},	//	MessageConfigModeMissingConfig
	{"D:      L:", "Zero in progress"},	//	MessageConfigModeZeroInProgress
	{"D:      L:", "Calc Stroke"},	//	MessageConfigModeCalibrateStroke
	{"D:      L:", "Real:"},	//	MessageConfigModeCalibrateStrokeEnterReal
	{"D:      L:", "HOME when clear"},	//	MessageConfigModeCalibrateStropper
	{"D:      L:", "Raise Time: "},	//	MessageConfigModeCalibrateViceRaiseTimer
	{"D:      L:", "Vice P: "},	//	MessageConfigModeCalibrateVicePressure
	{"D:      L:", "Extr P: "},	//	MessageConfigModeCalibrateExtrudePressure
	{"D:      L:", ""},	//	MessageConfigModeInitialized
	{"D:      L:", "C:"},	//	MessageAutoModeRunning
	{"TC:       A:", "C:        W:"},	//	MessageAutoModeStats
	{"Emergency stop.", ""},	//	MessagePANIC
};

Message currentMessage = MessageNone;
Message lastMessage = MessageCount;

struct RodSizeParams {
	float size;
	float ncExtrudeNeeded;
	float nfExtrudeNeeded;
	float dieEntryNC;
	float dieEntryNF;
	unsigned int viceMaxPressure;
	unsigned int extrudeMaxPressure;
	char* name;
};

#define SIZE_COUNT 11
RodSizeParams extrusionTable[SIZE_COUNT] = {
//	 Size		NC			NF			DieEntryNC	DieEntryNF	viceP	extrudeP	Name
	{0.25f,		0.7355f,	0.8035f,	0.4375f,	0.4375f,	151,	255,		"  1/4"},
	{0.3125f,	0.7626f,	0.8161f,	0.4375f,	0.4375f,	189,	255,		" 5/16"},
	{0.375f,	0.7772f,	0.8454f,	0.4375f,	0.4375f,	226,	255,		"  3/8"},
	{0.4375f,	0.7829f,	0.8409f,	0.4375f,	0.4375f,	264,	255,		" 7/16"},
	{0.5f,		0.7950f,	0.8601f,	0.4375f,	0.4375f,	377,	512,		"  1/2"},
	{0.625f,	0.8068f,	0.8761f,	0.4375f,	0.4375f,	377,	512,		"  5/8"},
	{0.75f,		0.8218f,	0.8841f,	0.4375f,	0.4375f,	453,	255,		"  3/4"},
	{0.875f,	0.8301f,	0.8868f,	0.4375f,	0.4375f,	528,	255,		"  7/8"},
	{1.0f,		0.8338f,	0.8851f,	0.4375f,	0.4375f,	604,	255,		"1    "},
	{1.125f,	0.8321f,	0.8974f,	0.4375f,	0.4375f,	679,	255,		"1 1/8"},
	{1.25f,		0.8479f,	0.9072f,	0.4375f,	0.4375f,	755,	255,		"1 1/4"}
};

LiquidCrystal lcd(8, 9, 10, 11, 12, 7);

struct JobConfig {
	unsigned int jobID;
	int rodSize;
	int extrudeLength;
};

JobConfig currentJobConfig;
JobConfig loadedJobConfig;

int prevRodSize;
bool canReadRodSize = false;

int currentRodSizeIndex = -1;

int prevExtrudeLength;
bool canReadExtrudeLength = false;

bool pumpStarted = false;
bool stopRaised = false;

bool isPanicked = true;
bool isCoverOpenned = false;
bool initialized = false;

UnitType unitType = UNIT_MM;
ThreadType threadType = THREAD_NC;

unsigned int pistonPosition = 0;
unsigned int pistonStrokeLength = 0;
unsigned int realStrokeLength = (int)(7.0f * (float)EXTRUDE_LENGTH_MULTIPLICATOR);
float positionMultiplicator = 333.333333f;

unsigned int stopperSafePosition = 0;
unsigned int minPistonPosition = 0;
unsigned int maxPistonPosition = 0;
unsigned int viceRaiseTimer = 250;

int currentPressure = 0;

int initModeHomeCount = 0;
unsigned long homePressTime = -1;
unsigned long calibrateVicePressTime = -1;
unsigned long manualViceOpenPressTime = -1;
unsigned long manualViceClosePressTime = -1;
unsigned long toggleStopperPressTime = -1;

bool stopperToHigh = false;
bool stopperToLow = false;
unsigned long stopperTimer;

//	Callbacks
void (*maximumPositionCallback)() = 0;

byte saveDataVersion = 0;
int jobConfigRingPosition = 0;
unsigned long rodCount = 0;
int rodCountRingPosition = 0;

enum AutoState {
	AutoStateFirstRunGoHome,
	AutoStateFirstRunLowerStopper,
	AutoStateFirstRunOpenVice,
	AutoStateFirstRunCalibrateForwardOvershoot,
	AutoStateFirstRunCalibrateForwardOvershootWait,
	AutoStateFirstRunGotoStartPos,
	AutoStateStartOil,
	AutoStateWaitPedal,
	AutoStateClosingVice,
	AutoStateMoveBackwardForStopper,
	AutoStateStopOil,
	AutoStateRaiseStopper,
	AutoStateMoveForwardExtrude,
	AutoStateMoveBackwardPostExtrude,
	AutoStateLowerStopper,
	AutoStateOpenVice,
	AutoStateMoveForwardToStartPos,
	AutoStateErrorExtrudePressure,
	AutoStateErrorPump,
};
AutoState autoState = AutoStateFirstRunGoHome;

int autoModeStartPos;
int autoModeBackPos;
int autoModeRaiseStopperPos;
int autoModeExtrudePos;
int autoModeVicePressure;
int autoModeExtrudePressure;

long autoModeViceTimer = 0;
bool displayStats = false;
unsigned int thisJobRodCount = 0;
bool autoModeHomeWasDown = false;

unsigned long lastExtrudeTimes[STATS_AVERAGE_TIME_SAMPLING_COUNT] = {0ul};
unsigned long lastWaitTimes[STATS_AVERAGE_TIME_SAMPLING_COUNT] = {0ul};
unsigned int currentSamplingIndex = 0;
unsigned long refWaitTime = 0;
unsigned long refExtrudeTime = 0;

bool predalWasReleased = false;

bool isLearning = false;
int learningDirection = 0;
unsigned int learningTargetPosition = 0;
unsigned int learningPrevPistonPosition = 0;

int learningForwardValues[LEARNING_COUNT] = {-1};
unsigned int learningForwardValuesCount = 0;
unsigned long learningTargetTime = 0ul;

unsigned int forwardOvershoot = 0;

unsigned long ignorePressureTime = 0;
bool waitingForHomePressure = false;

unsigned long autoModeViceTimerDelta = 0ul;
unsigned long coverOpennedTime = 0ul;

#ifdef DEBUG_SERIAL
unsigned long nextPistonPositionLogTime = 0;
unsigned int logPrevPistonPosition = 0;
#endif

void setup() {
	SetupPin(OUT_RELAY_ACTIVATOR, false);
	digitalWrite(OUT_RELAY_ACTIVATOR, false);

	lcd.begin(16, 2);
	lcd.clear();
	UpdateDisplayComplete();

	SetupPin(CONFIG_ENCODER_A, true, true);
	SetupPin(CONFIG_ENCODER_B, true, true);

	SetupPin(PISTON_POSITION_ENCODER_A, true, true);
	SetupPin(PISTON_POSITION_ENCODER_B, true, true);

	SetupPin(IN_MODE_MANUAL, true, true);
	SetupPin(IN_MODE_AUTO, true, true);

	SetupRelay(OUT_VALVE_FORWARD, OUT_VALVE_FORWARD_LED);
	SetupRelay(OUT_VALVE_BACKWARD, OUT_VALVE_BACKWARD_LED);
	
	SetupPin(IN_UNIT_SELECTOR, true, true);
	SetupPin(IN_THREAD_TYPE_SELECTOR, true, true);
	SetupPin(IN_SET_EXTRUDE_LENGTH, true, true);
	SetupPin(IN_SET_ROD_SIZE, true, true);
	
	SetupPin(IN_HOME, true, true);

	SetupPin(IN_STOP_RAISED, true, true);
	SetupPin(IN_STOP_LOWERED, true, true);
	SetupRelay(OUT_RAISE_STOP);
	SetupRelay(OUT_LOWER_STOP);
	SetupPin(OUT_STOPPER_RAISED_LED, false);

	SetupPin(IN_MANUAL_PISTON_FORWARD, true, true);
	SetupPin(IN_MANUAL_PISTON_BACKWARD, true, true);
	SetupPin(IN_MANUAL_TOGGLE_STOPPER, true, true);
	SetupPin(IN_MANUAL_OPEN_VICE, true, true);
	SetupPin(IN_MANUAL_CLOSE_VICE, true, true);

	SetupRelay(OUT_VICE_OPEN, OUT_VICE_OPEN_LED);
	SetupRelay(OUT_VICE_CLOSE, OUT_VICE_CLOSE_LED);

	SetupRelay(OUT_DROP_OIL);
	SetupPin(IN_COOLANT, true, true);
	SetupRelay(OUT_COOLANT, OUT_COOLANT_LED);
	
	SetupPin(IN_START_PUMP, true, true);
	SetupPin(IN_STOP_PUMP, true, true);
	SetupRelay(OUT_PUMP);

	SetupPin(IN_PEDAL, true, true);

	SetupPin(IN_PANIC, true, true);
	SetupPin(IN_COVER, true, true);

	SetupPin(IN_MAXIMUM_PISTON_POSITION, true, true);

#ifdef DEBUG_SERIAL
	Serial.begin(9600);
#endif

	//	Read from an unused Analog pin to use electrical noise as seed.
	randomSeed(analogRead(15));

	LoadEEPROM();

	digitalWrite(OUT_RELAY_ACTIVATOR, true);
}

void loop() {
#ifdef DEBUG_SERIAL
	if (millis() > nextPistonPositionLogTime) {
		nextPistonPositionLogTime = millis() + 1000;

		if (pistonPosition != logPrevPistonPosition) {
			logPrevPistonPosition = pistonPosition;
			Serial.print("Current Piston Position: ");
			Serial.print(pistonPosition);
			Serial.print("\n");
		}
	}
#endif

#ifdef DEBUG_SERIAL
	if (Serial.available() > 0) {
		takeADump(Serial.peek());
		Serial.read();
	}
#endif

	bool newPanic = PURead(IN_PANIC);

	if (newPanic != isPanicked) {
		if (newPanic) {
			StateChangeCleanup();
			initialized = false;
			pumpStarted = false;
			RelayWrite(OUT_PUMP, false);
			RelayWrite(OUT_COOLANT, false, OUT_COOLANT_LED);

			currentMessage = MessagePANIC;
			UpdateDisplayComplete();

			currentMode = ModeNone;
			return;
		} else {

		}

		isPanicked = newPanic;
	}

	if (isPanicked) {
		return;
	}

	bool newPumpStarted = pumpStarted;
	if (newPumpStarted) {
		if (!PURead(IN_STOP_PUMP)) {
			newPumpStarted = false;
		}
	} else {
		if (PURead(IN_START_PUMP) && PURead(IN_STOP_PUMP)) {
			newPumpStarted = true;
		}
	}

	if (newPumpStarted != pumpStarted) {
		pumpStarted = newPumpStarted;
		
		if (pumpStarted) {
#ifndef EXTERNAL_PUMP
			RelayWrite(OUT_PUMP, true);
#endif
		} else {
			StateChangeCleanup();
#ifndef EXTERNAL_PUMP
			RelayWrite(OUT_PUMP, false);
#endif
			initState = InitStateWaiting;
			initModeHomeCount = 0;
		}
	}

	//	Always keep this in sync.
	digitalWrite(OUT_STOPPER_RAISED_LED, PURead(IN_STOP_RAISED));
	RelayWrite(OUT_COOLANT, PURead(IN_COOLANT), OUT_COOLANT_LED);

	if (ignorePressureTime < millis()) {
		currentPressure = analogRead(IN_ANALOG_PRESSURE);
	} else {
		currentPressure = 0;
	}

	if (currentPressure >= MAX_PRESSURE) {
		RelayWrite(OUT_VALVE_BACKWARD, false, OUT_VALVE_BACKWARD_LED);
		RelayWrite(OUT_VALVE_FORWARD, false, OUT_VALVE_FORWARD_LED);
		RelayWrite(OUT_VICE_OPEN, false, OUT_VICE_OPEN_LED);
		RelayWrite(OUT_VICE_CLOSE, false, OUT_VICE_CLOSE_LED);
	}

	Learn();

	if (PURead(IN_MAXIMUM_PISTON_POSITION)) {
		if (maximumPositionCallback != 0) {
			maximumPositionCallback();
		}
		RelayWrite(OUT_VALVE_FORWARD, false, OUT_VALVE_FORWARD_LED);
	}

	bool modeManual = PURead(IN_MODE_MANUAL);
	bool modeAuto = PURead(IN_MODE_AUTO);

	if (!modeManual && !modeAuto) {
		if (currentMode != ModeInit) {
#ifdef DEBUG_SERIAL
			Serial.print("Set mode Init\n");
#endif
			StateChangeCleanup(true);
			currentMode = ModeInit;

			initModeHomeCount = 0;
			initState = InitStateWaiting;

			waitingForHomePressure = false;
			maximumPositionCallback = 0;

			if (canReadRodSize || canReadExtrudeLength) {
				detachInterrupt(CONFIG_ENCODER_A_INTERRUPT);
				canReadExtrudeLength = canReadRodSize = false;
			}
		}
		LoopInit();
	} else if (modeManual && !modeAuto) {
		if (currentMode != ModeManual) {
#ifdef DEBUG_SERIAL
			Serial.print("Set mode Manual\n");
#endif
			toggleStopperPressTime = -1;
			StateChangeCleanup(true);
			currentMode = ModeManual;
			manualViceOpenPressTime = -1;
			manualViceClosePressTime = -1;
			toggleStopperPressTime = -1;
			
			prevRodSize = prevExtrudeLength = 0;
			canReadRodSize = canReadExtrudeLength = false;
		}
		LoopManual();
	} else if (!modeManual && modeAuto) {
		if (currentMode != ModeAuto) {
#ifdef DEBUG_SERIAL
			Serial.print("Set mode Auto\n");
#endif
			StateChangeCleanup(true);
			currentMode = ModeAuto;
			autoState = AutoStateFirstRunGoHome;
			isLearning = false;
			isCoverOpenned = false;
			autoModeViceTimer = 0ul;
			
			if (!PURead(IN_COVER)) {
				coverOpennedTime = COVER_OPENNED_PAUSE_DELAY;
			} else {
				coverOpennedTime = 0ul;
			}

			if (canReadRodSize || canReadExtrudeLength) {
				detachInterrupt(CONFIG_ENCODER_A_INTERRUPT);
				canReadExtrudeLength = canReadRodSize = false;
			}

			if (currentRodSizeIndex >= 0 && currentRodSizeIndex < SIZE_COUNT) {
				displayStats = false;
				thisJobRodCount = 0;
				autoModeHomeWasDown = false;

				currentSamplingIndex = 0;
				for (int i=0; i<STATS_AVERAGE_TIME_SAMPLING_COUNT; i++) {
					lastWaitTimes[i] = lastExtrudeTimes[i] = 0;
				}

				waitingForHomePressure = true;
				maximumPositionCallback = &MaximumPositionReached;
			} else {
				//	TODO Error.
			}
		}
		LoopAuto();
	} else {
		//	Something went wrong. Cleanup everything!
		StateChangeCleanup();
	}
}

void StateChangeCleanup() {
	StateChangeCleanup(false);
}

void StateChangeCleanup(bool leaveMode) {
	stopperToHigh = stopperToLow = false;
	stopperTimer = 0ul;
	RelayWrite(OUT_VALVE_FORWARD, false, OUT_VALVE_FORWARD_LED);
	RelayWrite(OUT_VALVE_BACKWARD, false, OUT_VALVE_BACKWARD_LED);
	RelayWrite(OUT_RAISE_STOP, false);
	RelayWrite(OUT_LOWER_STOP, false);
	RelayWrite(OUT_VICE_OPEN, false, OUT_VICE_OPEN_LED);
	RelayWrite(OUT_VICE_CLOSE, false, OUT_VICE_CLOSE_LED);
	RelayWrite(OUT_DROP_OIL, false);

	if (leaveMode) {
		switch (currentMode) {
		case ModeInit:
			LeaveModeInit();
			break;
		case ModeManual:
			LeaveModeManual();
			break;
		case ModeAuto:
			LeaveModeAuto();
			break;
		}
	}
}

void LoopInit() {
	if (!pumpStarted) {
		currentMessage = MessageConfigModePumpNotStarted;
	} else if (initState != InitStateWaiting) {
		switch (initState) {
		case InitStateZeroing:
			currentMessage = MessageConfigModeZeroInProgress;
			break;
		case InitStateCalibrateStroke:
			if (waitingForHomePressure) {
				currentMessage = MessageConfigModeCalibrateStroke;
			} else {
				currentMessage = MessageConfigModeCalibrateStrokeEnterReal;
			}
			break;
		case InitStateCalibrateStopper:
			currentMessage = MessageConfigModeCalibrateStropper;
			break;
		case InitStateCalibrateViceRaiseTime:
			currentMessage = MessageConfigModeCalibrateViceRaiseTimer;
			break;
		case InitStateCalibrateVicePressure:
			currentMessage = MessageConfigModeCalibrateVicePressure;
			break;
		case InitStateCalibrateExtrudePressure:
			currentMessage = MessageConfigModeCalibrateExtrudePressure;
			break;
		}
	} else if (initialized) {
		if (pistonStrokeLength == 0 || stopperSafePosition == 0) {
			currentMessage =  MessageConfigModeMissingConfig;
		} else {
			currentMessage =  MessageConfigModeInitialized;
		}
	} else {
		currentMessage = MessageConfigModeNotInitialized;
	}
	
	UpdateDisplayComplete();

	switch (initState) {
	case InitStateZeroing:
		Zeroing();
		break;
	case InitStateCalibrateStroke:
		CalibrateStroke();
		break;
	case InitStateCalibrateStopper:
		CalibrateStopper();
		break;
	case InitStateCalibrateViceRaiseTime:
		CalibrateViceRaiseTime();
		return;
	case InitStateCalibrateVicePressure:
		CalibrateVicePressure();
		return;
	case InitStateCalibrateExtrudePressure:
		CalibrateExtrudePressure();
		return;
	default:
		InitWaitInput();
		break;
	}

	ReadConfig();
}

void InitWaitInput() {
	if (!pumpStarted) {
		return;
	}

	if (PURead(IN_MANUAL_OPEN_VICE)) {
		if (calibrateVicePressTime == -1) {
			calibrateVicePressTime = millis();
		}
		if ((millis() - calibrateVicePressTime) > RECALIBRATE_HOLD_TIME && initState != InitStateCalibrateViceRaiseTime) {
			calibrateVicePressTime = -1;
			initState = InitStateCalibrateViceRaiseTime;
			detachInterrupt(CONFIG_ENCODER_A_INTERRUPT);
			canReadRodSize = false;
			canReadExtrudeLength = false;

			attachInterrupt(CONFIG_ENCODER_A_INTERRUPT, UpdateViceRaiseTimer, CHANGE);
		}
	} else if (PURead(IN_MANUAL_CLOSE_VICE)) {
		if (calibrateVicePressTime == -1) {
			calibrateVicePressTime = millis();
		}
		if ((millis() - calibrateVicePressTime) > RECALIBRATE_HOLD_TIME && initState != InitStateCalibrateVicePressure) {
			calibrateVicePressTime = -1;
			initState = InitStateCalibrateVicePressure;
			detachInterrupt(CONFIG_ENCODER_A_INTERRUPT);
			canReadRodSize = false;
			canReadExtrudeLength = false;

			attachInterrupt(CONFIG_ENCODER_A_INTERRUPT, UpdateVicePressure, CHANGE);
		}
	} else if (PURead(IN_MANUAL_PISTON_FORWARD)) {
		if (calibrateVicePressTime == -1) {
			calibrateVicePressTime = millis();
		}
		if ((millis() - calibrateVicePressTime) > RECALIBRATE_HOLD_TIME && initState != InitStateCalibrateExtrudePressure) {
			calibrateVicePressTime = -1;
			initState = InitStateCalibrateExtrudePressure;
			detachInterrupt(CONFIG_ENCODER_A_INTERRUPT);
			canReadRodSize = false;
			canReadExtrudeLength = false;

			attachInterrupt(CONFIG_ENCODER_A_INTERRUPT, UpdateExtrudePressure, CHANGE);
		}
	} else if (PURead(IN_HOME)) {
		if (homePressTime == -1) {
			homePressTime = millis();
		} else if (initModeHomeCount > 0 && (millis() - homePressTime) > RECALIBRATE_HOLD_TIME) {
			lcd.setCursor(15, 1);
			lcd.print("*");
		}
	} else if (homePressTime != -1) {
		unsigned long heldTime = millis() - homePressTime;

		homePressTime = -1;
		if (!initialized || initModeHomeCount == 0 || heldTime < RECALIBRATE_HOLD_TIME) {
			initState = InitStateZeroing;
			waitingForHomePressure = true;
			maximumPositionCallback = 0;
			if (!PURead(OUT_VALVE_BACKWARD)) {
				RelayWrite(OUT_VALVE_BACKWARD, true, OUT_VALVE_BACKWARD_LED);
				ignorePressureTime = millis() + COUP_BELIER_DELAY;
			}
			initModeHomeCount = 1;
		} else {
			switch (initModeHomeCount) {
				case 1:
					canReadRodSize = false;
					canReadExtrudeLength = false;
					initState = InitStateCalibrateStroke;
					waitingForHomePressure = true;
					maximumPositionCallback = &CalibrateStrokeCallback;

					initModeHomeCount = 2;
					break;
				case 2:
					initState = InitStateCalibrateStopper;
					initModeHomeCount = 0;
					break;
				default:
					initModeHomeCount = 0;
					break;
			}
		}
	}
}

void Zeroing() {
	if (waitingForHomePressure) {
		if (currentPressure >= HOME_PRESSURE) {
			ZeroingPressureCallback();
			stopperTimer = 0ul;
		}
	} else {
		if (PURead(IN_STOP_LOWERED)) {
			if (stopperTimer == 0ul) {
				stopperTimer = millis() + STOPPER_TIMER;
			} else if (millis() >= stopperTimer) {
				RelayWrite(OUT_LOWER_STOP, false);
				initialized = true;
				initState = InitStateWaiting;
			}
		} else {
			RelayWrite(OUT_LOWER_STOP, true);
		}
	}
}

void ZeroingPressureCallback() {
	pistonPosition = PISTON_POSITION_ZERO_OFFSET;
	minPistonPosition = pistonPosition + PISTON_POSITION_PADDING;
	maxPistonPosition = minPistonPosition + pistonStrokeLength;

	waitingForHomePressure = false;

	RelayWrite(OUT_VALVE_BACKWARD, false, OUT_VALVE_BACKWARD_LED);

	attachInterrupt(PISTON_POSITION_ENCODER_A_INTERRUPT, PistonPositionInterrupt, CHANGE);
}

void CalibrateStroke() {
	if (!waitingForHomePressure && maximumPositionCallback == 0) {
		UpdateDisplayRealStrokeLength();
		if (PURead(IN_HOME)) {
			if (homePressTime == -1) {
				homePressTime = millis();
			}
		} else if (homePressTime != -1) {
			homePressTime = -1;

			initState = InitStateWaiting;
			canReadRodSize = true;
			canReadExtrudeLength = true;

			positionMultiplicator = (float)pistonStrokeLength / ((float)realStrokeLength/(float)EXTRUDE_LENGTH_MULTIPLICATOR);
			if (pistonStrokeLength != 0 && stopperSafePosition != 0) {
				SaveSystemSettings();
			}
#ifdef DEBUG_SERIAL
			Serial.print("Stroke Length: ");
			Serial.print(pistonStrokeLength);
			Serial.print("\n");
			Serial.print("Piston Position Multiplicator: ");
			Serial.print(positionMultiplicator);
			Serial.print("\n");
#endif
		}
	} else {
		if (waitingForHomePressure && currentPressure >= HOME_PRESSURE) {
			CalibrateStrokeCallback();
		} else if (!PURead(IN_STOP_RAISED)) {
			RelayWrite(OUT_RAISE_STOP, true);
			RelayWrite(OUT_VALVE_FORWARD, false, OUT_VALVE_FORWARD_LED);
			return;
		} else {
			RelayWrite(OUT_RAISE_STOP, false);
			if (!PURead(OUT_VALVE_FORWARD)) {
				RelayWrite(OUT_VALVE_FORWARD, true, OUT_VALVE_FORWARD_LED);
				ignorePressureTime = millis() + COUP_BELIER_DELAY;
			}
		}
	}
}

void CalibrateStrokeCallback() {
	if (PURead(IN_STOP_RAISED)) {
		maxPistonPosition = pistonPosition;
		pistonStrokeLength = maxPistonPosition - minPistonPosition;

		waitingForHomePressure = false;
		maximumPositionCallback = 0;
		attachInterrupt(CONFIG_ENCODER_A_INTERRUPT, UpdateStrokeLength, CHANGE);
	}
}

void CalibrateStopper() {
	UpdatePositionManual();

	if (PURead(IN_HOME)) {
		if (homePressTime == -1) {
			homePressTime = millis();
		}
	} else if (homePressTime != -1) {
		stopperSafePosition = pistonPosition - minPistonPosition;
		
		initState = InitStateWaiting;

		homePressTime = -1;

		if (pistonStrokeLength != 0 && stopperSafePosition != 0) {
			SaveSystemSettings();
		}

#ifdef DEBUG_SERIAL
		Serial.print("Stopper Safe pos: ");
		Serial.print(stopperSafePosition);
		Serial.print("\n");
#endif
	}
}

void CalibrateViceRaiseTime() {
	if (PURead(IN_MANUAL_OPEN_VICE)) {
		UpdateDisplayViceRaiseTime();
	} else {
		initState = InitStateWaiting;
		detachInterrupt(CONFIG_ENCODER_A_INTERRUPT);

		SaveSystemSettings();
	}
}

void CalibrateVicePressure() {
	if (PURead(IN_MANUAL_CLOSE_VICE)) {
		UpdateDisplayRodSize();
		UpdateDisplayExtureLength();
		UpdateDisplayVicePressure();
	} else {
		initState = InitStateWaiting;
		detachInterrupt(CONFIG_ENCODER_A_INTERRUPT);

		SaveSystemSettings();
	}
}

void CalibrateExtrudePressure() {
	if (PURead(IN_MANUAL_PISTON_FORWARD)) {
		UpdateDisplayRodSize();
		UpdateDisplayExtureLength();
		UpdateDisplayExtrudePressure();
	} else {
		initState = InitStateWaiting;
		detachInterrupt(CONFIG_ENCODER_A_INTERRUPT);

		SaveSystemSettings();
	}
}

void ReadConfig() {
	UnitType setUnit = PURead(IN_UNIT_SELECTOR);

	if (prevRodSize != currentJobConfig.rodSize) {
		prevRodSize = currentJobConfig.rodSize;
		UpdateDisplayRodSize();
	}

	if (prevExtrudeLength != currentJobConfig.extrudeLength) {
		prevExtrudeLength = currentJobConfig.extrudeLength;
		UpdateDisplayExtureLength();
	}

	if (setUnit != unitType) {
		unitType = setUnit;

		UpdateDisplayRodSize();
		UpdateDisplayExtureLength();
	}

	bool setRodSizeButton = PURead(IN_SET_ROD_SIZE);
	bool setExtrudeLengthButton = PURead(IN_SET_EXTRUDE_LENGTH);

	if (canReadRodSize && !setRodSizeButton) {
		detachInterrupt(CONFIG_ENCODER_A_INTERRUPT);
		canReadRodSize = false;
	} else if (!canReadRodSize && setRodSizeButton) {
		attachInterrupt(CONFIG_ENCODER_A_INTERRUPT, UpdateRodSizeInterrupt, CHANGE);
		canReadRodSize = true;
	} else if (canReadExtrudeLength && !setExtrudeLengthButton) {
		detachInterrupt(CONFIG_ENCODER_A_INTERRUPT);
		canReadExtrudeLength = false;
	} else if (!canReadExtrudeLength && setExtrudeLengthButton) {
		attachInterrupt(CONFIG_ENCODER_A_INTERRUPT, UpdateExtrudeLengthInterrupt, CHANGE);
		canReadExtrudeLength = true;
	}
}

void LeaveModeInit() {
	SaveJobConfig();
}

void UpdatePositionManual() {
	if (PURead(IN_MANUAL_PISTON_FORWARD) && PURead(IN_STOP_RAISED)) {
		if (!PURead(IN_MAXIMUM_PISTON_POSITION)) {
			RelayWrite(OUT_VALVE_BACKWARD, false, OUT_VALVE_BACKWARD_LED);
			if (!PURead(OUT_VALVE_FORWARD)) {
				RelayWrite(OUT_VALVE_FORWARD, true, OUT_VALVE_FORWARD_LED);
				ignorePressureTime = millis() + COUP_BELIER_DELAY;
			}
		} else {
			RelayWrite(OUT_VALVE_FORWARD, false, OUT_VALVE_FORWARD_LED);
			RelayWrite(OUT_VALVE_BACKWARD, false, OUT_VALVE_BACKWARD_LED);
			return;
		}
	} else if (PURead(IN_MANUAL_PISTON_BACKWARD)) {
		RelayWrite(OUT_VALVE_FORWARD, false, OUT_VALVE_FORWARD_LED);
		if (!PURead(OUT_VALVE_BACKWARD)) {
			RelayWrite(OUT_VALVE_BACKWARD, true, OUT_VALVE_BACKWARD_LED);
			ignorePressureTime = millis() + COUP_BELIER_DELAY;
		}
	} else {
		RelayWrite(OUT_VALVE_FORWARD, false, OUT_VALVE_FORWARD_LED);
		RelayWrite(OUT_VALVE_BACKWARD, false, OUT_VALVE_BACKWARD_LED);
		return;
	}

	//	Fail safe.
	if (currentPressure > MAX_PRESSURE) {
		RelayWrite(OUT_VALVE_FORWARD, false, OUT_VALVE_FORWARD_LED);
		RelayWrite(OUT_VALVE_BACKWARD, false, OUT_VALVE_BACKWARD_LED);
	}
}

void UpdateStopperManual() {
	//	Check in safe zone to raise.
	//if (pistonPosition <= minPistonPosition + stopperSafePosition) {
		if (PURead(IN_MANUAL_TOGGLE_STOPPER)) {
			if (toggleStopperPressTime == -1) {
				toggleStopperPressTime = millis();
			}
		} else {
			if (millis() - NON_CRITICAL_INPUTS_HOLD_TIME > toggleStopperPressTime) {
				if (PURead(IN_STOP_LOWERED)) {
					RelayWrite(OUT_LOWER_STOP, false);
					RelayWrite(OUT_RAISE_STOP, true);
					stopperToHigh = true;
					stopperToLow = false;
					stopperTimer = 0ul;
				} else if (PURead(IN_STOP_RAISED)) {
					RelayWrite(OUT_LOWER_STOP, true);
					RelayWrite(OUT_RAISE_STOP, false);
					stopperToHigh = false;
					stopperToLow = true;
					stopperTimer = 0ul;
				}
			}

			toggleStopperPressTime = -1;
		}
	/*} else {
		toggleStopperPressTime = -1;
	}*/
}

void UpdateViceManual() {
	bool open = PURead(IN_MANUAL_OPEN_VICE);
	bool close = PURead(IN_MANUAL_CLOSE_VICE);

	if (open) {
		if (manualViceOpenPressTime == -1) {
			manualViceOpenPressTime = millis();
		}
	} else {
		manualViceOpenPressTime = -1;
	}

	if (close) {
		if (manualViceClosePressTime == -1) {
			manualViceClosePressTime = millis();
		}
	} else {
		manualViceClosePressTime = -1;
	}

	if (open && (millis() - manualViceOpenPressTime) > NON_CRITICAL_INPUTS_HOLD_TIME) {
		RelayWrite(OUT_VICE_CLOSE, false, OUT_VICE_CLOSE_LED);
		if (!PURead(OUT_VICE_OPEN)) {
			RelayWrite(OUT_VICE_OPEN, true, OUT_VICE_OPEN_LED);
			ignorePressureTime = millis() + COUP_BELIER_DELAY;
		}
	} else if (((close && (millis() - manualViceClosePressTime) > NON_CRITICAL_INPUTS_HOLD_TIME)
			|| PURead(IN_PEDAL))
			&& currentRodSizeIndex >= 0
			&& currentPressure <= extrusionTable[currentRodSizeIndex].viceMaxPressure) {

		RelayWrite(OUT_VICE_OPEN, false, OUT_VICE_OPEN_LED);
		if (!PURead(OUT_VICE_CLOSE)) {
			RelayWrite(OUT_VICE_CLOSE, true, OUT_VICE_CLOSE_LED);
			ignorePressureTime = millis() + COUP_BELIER_DELAY;
		}
	} else {
		RelayWrite(OUT_VICE_OPEN, false, OUT_VICE_OPEN_LED);
		RelayWrite(OUT_VICE_CLOSE, false, OUT_VICE_CLOSE_LED);
	}
}

int prevPistonPosition = 0;

void LoopManual() {
	currentMessage = MessageConfigModeInitialized;

	if (!pumpStarted) {
		currentMessage = MessageConfigModePumpNotStarted;
	} else if (stopperToHigh) {
		if (PURead(IN_STOP_RAISED)) {
			if (stopperTimer == 0ul) {
				stopperTimer = millis() + STOPPER_TIMER;
			} else if (millis() >= stopperTimer) {
				stopperToHigh = false;
				RelayWrite(OUT_RAISE_STOP, false);
				stopperTimer = 0ul;
			}
		}
	} else if (stopperToLow) {
		if (PURead(IN_STOP_LOWERED)) {
			if (stopperTimer == 0ul) {
				stopperTimer = millis() + STOPPER_TIMER;
			} else if (millis() >= stopperTimer) {
				stopperToLow = false;
				RelayWrite(OUT_LOWER_STOP, false);
				stopperTimer = 0ul;
			}
		}
	} else {
		UpdatePositionManual();
		UpdateStopperManual();
		UpdateViceManual();
	}

	UpdateDisplayComplete();

	if (currentMessage == MessageConfigModeInitialized) {
		if (prevRodSize != currentJobConfig.rodSize) {
			prevRodSize = currentJobConfig.rodSize;
			UpdateDisplayRodSize();
		}

		if (prevExtrudeLength != currentJobConfig.extrudeLength) {
			prevExtrudeLength = currentJobConfig.extrudeLength;
			UpdateDisplayExtureLength();
		}

		bool prevUnitType = unitType;
		unitType = PURead(IN_UNIT_SELECTOR);

		bool prevThreadType = threadType;
		threadType = PURead(IN_THREAD_TYPE_SELECTOR);

		if (pistonPosition != prevPistonPosition || threadType != prevThreadType || unitType != prevUnitType) {
			UpdateDisplayResultingLength();
			UpdateDisplayExtureLength();
		}
	}
}

void LeaveModeManual() {

}

void LoopAuto() {
	if (pistonStrokeLength == 0 || stopperSafePosition == 0) {
		currentMessage = MessageErrorSystemNotInitialized;
		UpdateDisplayComplete();
		return;
	}

	if (!pumpStarted) {
		currentMessage = MessageErrorPumpNotStarted;
		UpdateDisplayComplete();
		autoState = AutoStateErrorPump;
		return;
	}

	if (!PURead(IN_COVER)) {
		if (coverOpennedTime == 0ul) {
			coverOpennedTime = millis();
		} else if (coverOpennedTime >= COVER_OPENNED_PAUSE_DELAY) {
			if (!isCoverOpenned) {
				isCoverOpenned = true;
				StateChangeCleanup();

				currentMessage = MessageErrorCoverOpenned;
				UpdateDisplayComplete();

				if (autoModeViceTimer != 0) {
					autoModeViceTimerDelta = autoModeViceTimer - millis();
				}
			}
			return;
		}
	} else if (isCoverOpenned) {
		isCoverOpenned = false;

		if (autoModeViceTimer != 0) {
			autoModeViceTimer = millis() + autoModeViceTimerDelta;
		}
	} else if (coverOpennedTime != 0ul) {
		coverOpennedTime = 0ul;
	}

	/*if (currentPressure >= MAX_PRESSURE) {
		//	TODO Error Message.
		return;
	}*/

	if (stopperTimer != 0ul && millis() >= stopperTimer) {
		stopperTimer = 0ul;
		RelayWrite(OUT_RAISE_STOP, false);
		RelayWrite(OUT_LOWER_STOP, false);
	}

	switch (autoState) {
	case AutoStateFirstRunGoHome:
		if (!PURead(OUT_VALVE_BACKWARD)) {
			RelayWrite(OUT_VALVE_BACKWARD, true, OUT_VALVE_BACKWARD_LED);
			ignorePressureTime = millis() + COUP_BELIER_DELAY;
		}

		if (currentPressure >= HOME_PRESSURE) {
			AutoFirstRunHomePressureCallback();
		}
		break;
	case AutoStateFirstRunLowerStopper:
		if (MoveStopper(LOW)) {
			autoState = AutoStateFirstRunOpenVice;
			//	Set the timer for the Vice to open.
			autoModeViceTimer = millis() + (unsigned long)viceRaiseTimer;
		}
		break;
	case AutoStateFirstRunOpenVice:
		if (MoveVice(HIGH)) {
			autoState = AutoStateFirstRunCalibrateForwardOvershoot;
		}
		break;
	case AutoStateFirstRunCalibrateForwardOvershoot:
		if (GotoDestination((int)((float)(autoModeStartPos - minPistonPosition) * 0.5f) + minPistonPosition, LOW)) {
			autoState = AutoStateFirstRunCalibrateForwardOvershootWait;
		}
		break;
	case AutoStateFirstRunCalibrateForwardOvershootWait:
		if (!isLearning) {
			autoState = AutoStateFirstRunGotoStartPos;
		}
		break;
	case AutoStateFirstRunGotoStartPos:
		if (GotoDestination(autoModeStartPos, LOW)) {
			autoState = AutoStateStartOil;
			refExtrudeTime = refWaitTime = millis();
		}
		break;
	case AutoStateStartOil:
		RelayWrite(OUT_DROP_OIL, true);
		autoState = AutoStateWaitPedal;

		predalWasReleased = !PURead(IN_PEDAL);
		break;
	case AutoStateWaitPedal:
		if (PURead(IN_PEDAL)) {
			if (predalWasReleased) {
				autoState = AutoStateClosingVice;
				lastWaitTimes[currentSamplingIndex] = millis() - refWaitTime;
				if (displayStats) {
					UpdateDisplayStats();
				}
			}
		} else {
			predalWasReleased = true;
		}
		break;
	case AutoStateClosingVice:
		if (MoveVice(LOW)) {
			autoState = AutoStateMoveBackwardForStopper;
		}
		break;
	case AutoStateMoveBackwardForStopper:
		if (GotoDestination(autoModeRaiseStopperPos, HIGH)) {
			autoState = AutoStateStopOil;
		}
		break;
	case AutoStateStopOil:
		RelayWrite(OUT_DROP_OIL, false);
		autoState = AutoStateRaiseStopper;
		break;
	case AutoStateRaiseStopper:
		if (MoveStopper(HIGH)) {
			autoState = AutoStateMoveForwardExtrude;
		}
		break;
	case AutoStateMoveForwardExtrude:
		if (MoveStopper(HIGH) && GotoDestination(autoModeExtrudePos, LOW)) {
			autoState = AutoStateMoveBackwardPostExtrude;
		} else {
			if (currentPressure > autoModeExtrudePressure) {
				RelayWrite(OUT_VALVE_BACKWARD, false, OUT_VALVE_BACKWARD_LED);
				RelayWrite(OUT_VALVE_FORWARD, false, OUT_VALVE_FORWARD_LED);
				autoState = AutoStateErrorExtrudePressure;
				currentMessage = MessageErrorExtrudePressure;
				UpdateDisplayComplete();
				return;
			}
		}
		break;
	case AutoStateMoveBackwardPostExtrude:
		if (GotoDestination(autoModeBackPos, HIGH)) {
			autoState = AutoStateLowerStopper;
		}
		break;
	case AutoStateLowerStopper:
		if (MoveStopper(LOW)) {
			autoState = AutoStateOpenVice;
			//	Set the timer for the Vice to open.
			autoModeViceTimer = millis() + (unsigned long)viceRaiseTimer;
		}
		break;
	case AutoStateOpenVice:
		if (MoveVice(HIGH)) {
			autoState = AutoStateMoveForwardToStartPos;
		}
		break;
	case AutoStateMoveForwardToStartPos:
		if (GotoDestination(autoModeStartPos, LOW)) {
			autoState = AutoStateStartOil;

			unsigned long curMillis = millis();
			lastExtrudeTimes[currentSamplingIndex] = curMillis - refExtrudeTime;
			refExtrudeTime = refWaitTime = curMillis;

			currentSamplingIndex = (currentSamplingIndex + 1) % STATS_AVERAGE_TIME_SAMPLING_COUNT;
			IncrementCount();
		}
		break;
	case AutoStateErrorPump:
		currentMessage = MessageErrorAutoModeGeneric;
		UpdateDisplayComplete();
		return;
	}

	bool home = PURead(IN_HOME);

	if (!autoModeHomeWasDown && home) {
		displayStats = !displayStats;
	}
	autoModeHomeWasDown = home;

	if (displayStats) {
		if (currentMessage != MessageAutoModeStats) {
			currentMessage = MessageAutoModeStats;
			UpdateDisplayComplete();
			UpdateDisplayStats();
			UpdateDisplayCount();
		}
	} else {
		if (currentMessage != MessageAutoModeRunning) {
			currentMessage = MessageAutoModeRunning;
			UpdateDisplayComplete();
			UpdateDisplayRodSize();
			UpdateDisplayExtureLength();
			UpdateDisplayCount();
		}
	}
}

void Learn() {
	if (isLearning && currentMode == ModeAuto) {
		//	Check if pistonPosition stabilized for a while...
		if (pistonPosition == learningPrevPistonPosition) {
			if (millis() > learningTargetTime) {
				isLearning = false;

				int delta = learningTargetPosition - pistonPosition;

				if (learningDirection == 1) {
					//	Forward
					delta *= -1;

					learningForwardValues[learningForwardValuesCount%LEARNING_COUNT] = delta + forwardOvershoot;
					learningForwardValuesCount++;

					unsigned int total = 0;
					byte learnedCount = 0;
					for (int i=0; i<LEARNING_COUNT; i++) {
						if (i < learningForwardValuesCount) {
							learnedCount++;
							total += learningForwardValues[i];
						}
					}

					if (learnedCount > 0) {
						forwardOvershoot = (int)((float)total / (float)learnedCount);
#ifdef DEBUG_SERIAL
						Serial.print("forwardOvershoot average: ");
						Serial.print(forwardOvershoot);
						Serial.print("\n");
#endif
					}
				}
			}
		} else {
			learningPrevPistonPosition = pistonPosition;
			learningTargetTime = millis() + LEARNING_DELAY;
		}
	}
}

bool GotoDestination(int destination, bool continueIf) {
	if (continueIf == HIGH) {
		if (pistonPosition <= destination) {
			RelayWrite(OUT_VALVE_BACKWARD, false, OUT_VALVE_BACKWARD_LED);
			RelayWrite(OUT_VALVE_FORWARD, false, OUT_VALVE_FORWARD_LED);

#ifdef DEBUG_SERIAL
			Serial.print("GotoDestination reached target: ");
			Serial.print((destination));
			Serial.print(" -- The current pistonPosition is: ");
			Serial.print(pistonPosition);
			Serial.print("\n");
#endif
			return true;
		} else {
			RelayWrite(OUT_VALVE_FORWARD, false, OUT_VALVE_FORWARD_LED);
			if (!PURead(OUT_VALVE_BACKWARD)) {
				RelayWrite(OUT_VALVE_BACKWARD, true, OUT_VALVE_BACKWARD_LED);
				ignorePressureTime = millis() + COUP_BELIER_DELAY;
			}
		}
	} else {
		if (pistonPosition >= (destination - forwardOvershoot)) {
			RelayWrite(OUT_VALVE_BACKWARD, false, OUT_VALVE_BACKWARD_LED);
			RelayWrite(OUT_VALVE_FORWARD, false, OUT_VALVE_FORWARD_LED);

#ifdef DEBUG_SERIAL
			Serial.print("GotoDestination reached target: ");
			Serial.print((destination));
			Serial.print(" -- The current pistonPosition is: ");
			Serial.print(pistonPosition);
			Serial.print("\n");
#endif
			isLearning = true;
			learningTargetTime = millis() + LEARNING_DELAY;
			learningDirection = 1;
			learningTargetPosition = destination;
			return true;
		} else {
			RelayWrite(OUT_VALVE_BACKWARD, false, OUT_VALVE_BACKWARD_LED);
			if (!PURead(OUT_VALVE_FORWARD)) {
				RelayWrite(OUT_VALVE_FORWARD, true, OUT_VALVE_FORWARD_LED);
				ignorePressureTime = millis() + COUP_BELIER_DELAY;
			}
		}
	}

	isLearning = false;
	return false;
}

bool MoveStopper(bool destination) {
	if (destination == HIGH) {
		if (PURead(IN_STOP_RAISED)) {
			stopperTimer = millis() + STOPPER_TIMER;
			/*RelayWrite(OUT_RAISE_STOP, false);
			RelayWrite(OUT_LOWER_STOP, false);*/
			return true;
		} else {
			RelayWrite(OUT_LOWER_STOP, false);
			RelayWrite(OUT_RAISE_STOP, true);
		}
	} else {
		if (PURead(IN_STOP_LOWERED)) {
			stopperTimer = millis() + STOPPER_TIMER;
			/*RelayWrite(OUT_RAISE_STOP, false);
			RelayWrite(OUT_LOWER_STOP, false);*/
			return true;
		} else {
			RelayWrite(OUT_RAISE_STOP, false);
			RelayWrite(OUT_LOWER_STOP, true);
		}
	}

	return false;
}

bool MoveVice(bool destination) {
	if (destination == HIGH) {
		unsigned long curTime = millis();
		if (autoModeViceTimer <= curTime) {
			autoModeViceTimer = 0;
			RelayWrite(OUT_VICE_CLOSE, false, OUT_VICE_CLOSE_LED);
			RelayWrite(OUT_VICE_OPEN, false, OUT_VICE_OPEN_LED);
			return true;
		} else {
			RelayWrite(OUT_VICE_CLOSE, false, OUT_VICE_CLOSE_LED);
			if (!PURead(OUT_VICE_OPEN)) {
				RelayWrite(OUT_VICE_OPEN, true, OUT_VICE_OPEN_LED);
				ignorePressureTime = millis() + COUP_BELIER_DELAY;
			}
		}
	} else {
		if (currentPressure >= autoModeVicePressure) {
			RelayWrite(OUT_VICE_CLOSE, false, OUT_VICE_CLOSE_LED);
			RelayWrite(OUT_VICE_OPEN, false, OUT_VICE_OPEN_LED);
			return true;
		} else {
			RelayWrite(OUT_VICE_OPEN, false, OUT_VICE_OPEN_LED);

			if (!PURead(OUT_VICE_CLOSE)) {
				RelayWrite(OUT_VICE_CLOSE, true, OUT_VICE_CLOSE_LED);
				ignorePressureTime = millis() + COUP_BELIER_DELAY;
			}
		}
	}

	return false;
}

void AutoFirstRunHomePressureCallback() {
	autoState = AutoStateFirstRunLowerStopper;
	waitingForHomePressure = false;

	if (!initialized) {
		initialized = true;
		attachInterrupt(PISTON_POSITION_ENCODER_A_INTERRUPT, PistonPositionInterrupt, CHANGE);
	}

	pistonPosition = PISTON_POSITION_ZERO_OFFSET;
	minPistonPosition = pistonPosition + PISTON_POSITION_PADDING;
	maxPistonPosition = minPistonPosition + pistonStrokeLength;

	RelayWrite(OUT_VALVE_BACKWARD, false, OUT_VALVE_BACKWARD_LED);

	float len = 1.0f;
	threadType = PURead(IN_THREAD_TYPE_SELECTOR);
	float dieEntry = 0.0f;
	if (threadType == THREAD_NC) {
		len = extrusionTable[currentRodSizeIndex].ncExtrudeNeeded;
		dieEntry = extrusionTable[currentRodSizeIndex].dieEntryNC;
	} else {
		len = extrusionTable[currentRodSizeIndex].nfExtrudeNeeded;
		dieEntry = extrusionTable[currentRodSizeIndex].dieEntryNF;
	}

	float realLen = (float)currentJobConfig.extrudeLength/(float)EXTRUDE_LENGTH_MULTIPLICATOR;
	autoModeStartPos = maxPistonPosition - (int)((realLen / (1.0f/len)) * (float)positionMultiplicator) - STOPPER_THICKNESS - (int)(dieEntry * positionMultiplicator);
	autoModeBackPos = maxPistonPosition - (int)(realLen * (float)positionMultiplicator) - STOPPER_THICKNESS - STOPPER_PADDING - (int)(dieEntry * positionMultiplicator);
	autoModeBackPos = min(minPistonPosition + stopperSafePosition, autoModeBackPos);
	autoModeRaiseStopperPos = autoModeStartPos - STOPPER_PADDING;
	autoModeRaiseStopperPos = min(minPistonPosition + stopperSafePosition, autoModeRaiseStopperPos);
	autoModeExtrudePos = maxPistonPosition + 1000;
	autoModeVicePressure = extrusionTable[currentRodSizeIndex].viceMaxPressure;
	autoModeExtrudePressure = extrusionTable[currentRodSizeIndex].extrudeMaxPressure;

#ifdef DEBUG_SERIAL
	Serial.print("Current Position: ");
	Serial.print(((float)(maxPistonPosition - pistonPosition) / (float)positionMultiplicator));
	Serial.print("\n");
	Serial.print("autoModeStartPos: ");
	Serial.print(((float)(maxPistonPosition - autoModeStartPos) / (float)positionMultiplicator));
	Serial.print("\n");
	Serial.print("autoModeBackPos: ");
	Serial.print(((float)(maxPistonPosition - autoModeBackPos) / (float)positionMultiplicator));
	Serial.print("\n");
	Serial.print("autoModeRaiseStopperPos: ");
	Serial.print(((float)(maxPistonPosition - autoModeRaiseStopperPos) / (float)positionMultiplicator));
	Serial.print("\n");
	Serial.print("autoModeExtrudePos: ");
	Serial.print(((float)(maxPistonPosition - autoModeExtrudePos) / (float)positionMultiplicator));
	Serial.print("\n");
	Serial.print("autoModeVicePressure: ");
	Serial.print(autoModeVicePressure);
	Serial.print("\n");
	Serial.print("autoModeExtrudePressure: ");
	Serial.print(autoModeExtrudePressure);
	Serial.print("\n");
#endif
}

void MaximumPositionReached() {
	pistonPosition = maxPistonPosition;

	autoState = AutoStateMoveBackwardPostExtrude;
}

void LeaveModeAuto() {

}

//	Interrupts
void PistonPositionInterrupt() {
	if (digitalRead(PISTON_POSITION_ENCODER_A) == digitalRead(PISTON_POSITION_ENCODER_B)) {
		pistonPosition++;
	} else {
		pistonPosition--;
	}
}

void UpdateRodSizeInterrupt() {
	if (digitalRead(CONFIG_ENCODER_A) != digitalRead(CONFIG_ENCODER_B)) {
		currentJobConfig.rodSize = min(currentJobConfig.rodSize + 1, MAX_ROD_SIZE);
	} else {
		currentJobConfig.rodSize = max(currentJobConfig.rodSize - 1, MIN_ROD_SIZE);
	}
}

void UpdateExtrudeLengthInterrupt() {
	if (digitalRead(CONFIG_ENCODER_A) != digitalRead(CONFIG_ENCODER_B)) {
		currentJobConfig.extrudeLength = min(currentJobConfig.extrudeLength + 1, MAX_EXTRUDE_LENGTH);
	} else {
		currentJobConfig.extrudeLength = max(currentJobConfig.extrudeLength - 1, MIN_EXTRUDE_LENGTH);
	}
}

void UpdateViceRaiseTimer() {
	if (digitalRead(CONFIG_ENCODER_A) != digitalRead(CONFIG_ENCODER_B)) {
		viceRaiseTimer = min(viceRaiseTimer + 1, VICE_RAISE_TIMER_MAX);
	} else {
		viceRaiseTimer = max(viceRaiseTimer - 1, VICE_RAISE_TIMER_MIN);
	}
}

void UpdateVicePressure() {
	if (digitalRead(CONFIG_ENCODER_A) != digitalRead(CONFIG_ENCODER_B)) {
		extrusionTable[currentRodSizeIndex].viceMaxPressure = min(extrusionTable[currentRodSizeIndex].viceMaxPressure + 1, 1024);
	} else {
		extrusionTable[currentRodSizeIndex].viceMaxPressure = max(extrusionTable[currentRodSizeIndex].viceMaxPressure - 1, 1);
	}
}

void UpdateExtrudePressure() {
	if (digitalRead(CONFIG_ENCODER_A) != digitalRead(CONFIG_ENCODER_B)) {
		extrusionTable[currentRodSizeIndex].extrudeMaxPressure = min(extrusionTable[currentRodSizeIndex].extrudeMaxPressure + 1, 1024);
	} else {
		extrusionTable[currentRodSizeIndex].extrudeMaxPressure = max(extrusionTable[currentRodSizeIndex].extrudeMaxPressure - 1, 1);
	}
}

void UpdateStrokeLength() {
	if (digitalRead(CONFIG_ENCODER_A) != digitalRead(CONFIG_ENCODER_B)) {
		realStrokeLength ++;
	} else {
		realStrokeLength = max(realStrokeLength - 1, 1);
	}
}

//	Display
void UpdateDisplayComplete() {
	if (currentMessage != lastMessage) {
		lcd.clear();
		lcd.print(messages[currentMessage][0]);
		lcd.setCursor(0, 1);
		lcd.print(messages[currentMessage][1]);

		lastMessage = currentMessage;

		prevExtrudeLength = 0;
		prevRodSize = 0;
	}
}

void UpdateDisplayRodSize() {
	float fSize = (float)currentJobConfig.rodSize / (float)ROD_SIZE_MULTIPLICATOR;

	int closestIndex = -1;
	float closest = 999.0f;
	char* result = "";
	for (int i=0; i<SIZE_COUNT; i++) {
		float dist = extrusionTable[i].size - fSize;
		if (dist < 0.0f) {
			dist *= -1.0f;
		}

		if (dist < closest) {
			closest = dist;
			closestIndex = i;
		}
	}

	currentRodSizeIndex = closestIndex;

	if (currentRodSizeIndex >= 0) {
		result = extrusionTable[currentRodSizeIndex].name;
	} else {
		result = "NA";
	}

	lcd.setCursor(2, 0);
	lcd.print(result);
}

void UpdateDisplayExtureLength() {
	lcd.setCursor(10, 0);

	float fLength = (float)currentJobConfig.extrudeLength / (float)EXTRUDE_LENGTH_MULTIPLICATOR;

	if (unitType == UNIT_MM) {
		fLength *= 25.4f;

		if (fLength < 10) {
			lcd.print("  ");
		} else if (fLength < 100) {
			lcd.print(" ");
		}
		lcd.print((int)fLength);
		lcd.print("mm");
	} else {
		int major = currentJobConfig.extrudeLength / EXTRUDE_LENGTH_MULTIPLICATOR;
		int minor = 100 * ((float)(currentJobConfig.extrudeLength % EXTRUDE_LENGTH_MULTIPLICATOR) / (float)EXTRUDE_LENGTH_MULTIPLICATOR);

		lcd.print(major);
		lcd.print(".");
		if (minor < 10) {
			lcd.print("0");
		}
		lcd.print(minor);
		lcd.print("\"");
	}
}

void UpdateDisplayCount() {
	lcd.setCursor(3, 1);
	lcd.print(thisJobRodCount);
}

void UpdateDisplayStats() {
	//	"TC:       A:",
	//	"C:        W:"
	lcd.setCursor(4, 0);
	lcd.print(rodCount);

	//	Average time per extrude.
	unsigned long total = 0;
	unsigned int count = 0;
	for (int i=0; i<STATS_AVERAGE_TIME_SAMPLING_COUNT; i++) {
		if (lastExtrudeTimes[i] > 0ul) {
			total += lastExtrudeTimes[i];
			count++;
		}
	}
	lcd.setCursor(13, 0);
	if (count > 0) {
		total = (total / (unsigned long)count) / 1000ul;
		lcd.print(total);
		if (total < 10) {
			lcd.print("  ");
		} else if (total < 100) {
			lcd.print(" ");
		}
	} else {
		lcd.print("0_0");
	}

	//	Average time wasted.
	total = 0;
	count = 0;
	for (int i=0; i<STATS_AVERAGE_TIME_SAMPLING_COUNT; i++) {
		if (lastWaitTimes[i] > 0ul) {
			total += lastWaitTimes[i];
			count++;
		}
	}
	lcd.setCursor(13, 1);
	if (count > 0) {
		total = (total / (unsigned long)count) / 1000ul;
		lcd.print(total);
		if (total < 10) {
			lcd.print("  ");
		} else if (total < 100) {
			lcd.print(" ");
		}
	} else {
		lcd.print("0_0");
	}
}

void UpdateDisplayViceRaiseTime() {
	//	"Raise Time: "
	lcd.setCursor(12, 1);
	lcd.print(viceRaiseTimer);
	lcd.print("   ");
}

void UpdateDisplayVicePressure() {
	//	"Vice P: "
	lcd.setCursor(8, 1);
	lcd.print(extrusionTable[currentRodSizeIndex].viceMaxPressure * PRESSURE_ANALOG_TO_PSI);
	lcd.print("   ");
}

void UpdateDisplayExtrudePressure() {
	//	"Extr P: "
	lcd.setCursor(8, 1);
	lcd.print(extrusionTable[currentRodSizeIndex].extrudeMaxPressure * PRESSURE_ANALOG_TO_PSI);
	lcd.print("   ");
}

void UpdateDisplayRealStrokeLength() {
	//	"Real: "
	lcd.setCursor(6, 1);
	lcd.print((float)realStrokeLength/(float)EXTRUDE_LENGTH_MULTIPLICATOR);
	lcd.print("   ");
}

void UpdateDisplayResultingLength() {
	prevPistonPosition = pistonPosition;
	lcd.setCursor(0, 1);

	float len = 1.0f;
	float dieEntry = 1.0f;
	if (threadType == THREAD_NC) {
		len = extrusionTable[currentRodSizeIndex].ncExtrudeNeeded;
		dieEntry = extrusionTable[currentRodSizeIndex].dieEntryNC;
	} else {
		len = extrusionTable[currentRodSizeIndex].nfExtrudeNeeded;
		dieEntry = extrusionTable[currentRodSizeIndex].dieEntryNF;
	}

	float fLength = (float)(maxPistonPosition - pistonPosition - STOPPER_THICKNESS - (int)(dieEntry * positionMultiplicator)) / positionMultiplicator;
	fLength = fLength / len;

	if (unitType == UNIT_MM) {
		fLength *= 25.4f;
		lcd.print((int)fLength);
		lcd.print("mm");
	} else {
		lcd.print(fLength);
	}

	lcd.print("      ");
}

void LoadEEPROM() {
	eeprom_busy_wait();

	saveDataVersion = eeprom_read_byte((byte*)EEPROM_VERSION_ADDRESS);

	if (saveDataVersion == SAVE_DATA_VERSION) {

		//	System Settings.
		int curAdd = EEPROM_VERSION_ADDRESS + sizeof(saveDataVersion);

#ifdef DEBUG_SERIAL
		Serial.print("Loading System settings from address: ");
		Serial.print(curAdd);
		Serial.print("\n");
#endif

		pistonStrokeLength = eeprom_read_word((unsigned int*)curAdd);
		curAdd += sizeof(pistonStrokeLength);
		
		eeprom_read_block(&positionMultiplicator, (void*)curAdd, sizeof(positionMultiplicator));
		curAdd += sizeof(positionMultiplicator);

		stopperSafePosition = eeprom_read_word((unsigned int*)curAdd);

		maxPistonPosition = minPistonPosition + pistonStrokeLength;
		curAdd += sizeof(stopperSafePosition);

		viceRaiseTimer = eeprom_read_word((unsigned int*)curAdd);
		viceRaiseTimer = max(viceRaiseTimer, VICE_RAISE_TIMER_MIN);
		curAdd += sizeof(viceRaiseTimer);

		for (int i=0; i<SIZE_COUNT; i++) {
			extrusionTable[i].viceMaxPressure = eeprom_read_word((unsigned int*)curAdd);
			curAdd += sizeof(extrusionTable[i].viceMaxPressure);

			extrusionTable[i].extrudeMaxPressure = eeprom_read_word((unsigned int*)curAdd);
			curAdd += sizeof(extrusionTable[i].extrudeMaxPressure);
		}

#ifdef DEBUG_SERIAL
		Serial.print("Piston Stroke Length: ");
		Serial.print(pistonStrokeLength);
		Serial.print("\n");
		Serial.print("Position Multiplicator: ");
		Serial.print(positionMultiplicator);
		Serial.print("\n");
		Serial.print("Vice Raise Timer: ");
		Serial.print(viceRaiseTimer);
		Serial.print("\n");
		Serial.print("\n");
		Serial.print("Reading JobConfig from address: ");
		Serial.print(curAdd);
		Serial.print("\n");
#endif
		//	JobConfig
		int highestJobRing = 0;
		loadedJobConfig.jobID = 0;
		loadedJobConfig.rodSize = 0.75f * ROD_SIZE_MULTIPLICATOR;
		loadedJobConfig.extrudeLength = 4.0f * EXTRUDE_LENGTH_MULTIPLICATOR;
		JobConfig readJobCfg;

		for (int i=0; i<JOB_CONFIG_RING_COUNT; i++) {
			int readAdd = curAdd + sizeof(currentJobConfig)*i;
			eeprom_read_block(&readJobCfg, (void*)readAdd, sizeof(currentJobConfig));

			if (readJobCfg.jobID > loadedJobConfig.jobID) {
				loadedJobConfig = readJobCfg;
				
				highestJobRing = i;
			}
		}
		jobConfigRingPosition = highestJobRing;

		currentJobConfig = loadedJobConfig;
		prevRodSize = currentJobConfig.rodSize;
		prevExtrudeLength = currentJobConfig.extrudeLength;

#ifdef DEBUG_SERIAL
		Serial.print("--JobConfig--\nID:");
		Serial.print(currentJobConfig.jobID);
		Serial.print("\nSize: ");
		Serial.print(currentJobConfig.rodSize);
		Serial.print("\nLen: ");
		Serial.print(currentJobConfig.extrudeLength);
		Serial.print("\nRing: ");
		Serial.print(jobConfigRingPosition);
		Serial.print("\n");
#endif
		//	RodCount
		curAdd += sizeof(currentJobConfig) * JOB_CONFIG_RING_COUNT;

		rodCount = 0ul;
		unsigned int rodCountI = 0u;
#ifdef DEBUG_SERIAL
		Serial.print("Rod Count Rings: ");
#endif
		for (int i=0; i<ROD_COUNT_RING_COUNT; i++) {
			rodCountI = eeprom_read_word((unsigned int*)(curAdd + sizeof(rodCountI)*i));

			rodCount += (long)rodCountI;

#ifdef DEBUG_SERIAL
			Serial.print(i);
			Serial.print(": ");
			Serial.print(rodCountI);
			Serial.print("\t");
#endif
		}

		rodCountRingPosition = random(0, ROD_COUNT_RING_COUNT);

#ifdef DEBUG_SERIAL
		Serial.print("\nLifetime Rod Count: ");
		Serial.print(rodCount);
		Serial.print("\n");
#endif
	}

	eeprom_busy_wait();
}

void SaveSystemSettings() {
#ifdef ENABLE_SAVE_TO_EEPROM
	bool initEEPROM = saveDataVersion != SAVE_DATA_VERSION;

	saveDataVersion = SAVE_DATA_VERSION;
	int curAdd = EEPROM_VERSION_ADDRESS;
	if (eeprom_read_byte((byte*)curAdd) != saveDataVersion) {
		eeprom_write_byte((byte*)curAdd, saveDataVersion);
	}

	curAdd += sizeof(saveDataVersion);

	if (eeprom_read_word((unsigned int*)curAdd) != pistonStrokeLength) {
		eeprom_write_word((unsigned int*)curAdd, pistonStrokeLength);
	}
	
	curAdd += sizeof(pistonStrokeLength);

	float curPositionMultiplicator = 0;
	eeprom_read_block(&curPositionMultiplicator, (void*)curAdd, sizeof(curPositionMultiplicator));

	if (curPositionMultiplicator != positionMultiplicator) {
		eeprom_write_block((void*)&positionMultiplicator, (void*)curAdd, sizeof(positionMultiplicator));
	}

	curAdd += sizeof(positionMultiplicator);

	if (eeprom_read_word((unsigned int*)curAdd) != stopperSafePosition) {
		eeprom_write_word((unsigned int*)curAdd, stopperSafePosition);
	}

	curAdd += sizeof(stopperSafePosition);

	if (eeprom_read_word((unsigned int*)curAdd) != viceRaiseTimer) {
		eeprom_write_word((unsigned int*)curAdd, viceRaiseTimer);
	}

	curAdd += sizeof(viceRaiseTimer);

	for (int i=0; i<SIZE_COUNT; i++) {
		if (eeprom_read_word((unsigned int*)curAdd) != extrusionTable[i].viceMaxPressure) {
			eeprom_write_word((unsigned int*)curAdd, extrusionTable[i].viceMaxPressure);
		}

		curAdd += sizeof(extrusionTable[i].viceMaxPressure);

		if (eeprom_read_word((unsigned int*)curAdd) != extrusionTable[i].extrudeMaxPressure) {
			eeprom_write_word((unsigned int*)curAdd, extrusionTable[i].extrudeMaxPressure);
		}

		curAdd += sizeof(extrusionTable[i].extrudeMaxPressure);
	}

	if (initEEPROM) {
		//	JobConfig area.
		int eepromReservedSize = sizeof(currentJobConfig) * JOB_CONFIG_RING_COUNT;

		//	Count area.
		eepromReservedSize += sizeof(unsigned int) * ROD_COUNT_RING_COUNT;

		//	Zero the reserved range!
#ifdef DEBUG_SERIAL
		Serial.print("Zeroing ");
		Serial.print(eepromReservedSize);
		Serial.print(" bytes of data at address: ");
		Serial.print(curAdd);
		Serial.print("\n");
#endif

		for (int i=0; i<eepromReservedSize; i++) {
			if (eeprom_read_byte((byte*)curAdd + i) != 0) {
				eeprom_write_byte((byte *)curAdd + i, 0);
			}
		}
	}

	eeprom_busy_wait();
#ifdef DEBUG_SERIAL
	Serial.print("Save complete!\n");
#endif
#endif
}

int GetJobConfigEEPROMAddress() {
	return EEPROM_VERSION_ADDRESS +
	sizeof(saveDataVersion) +
	sizeof(pistonStrokeLength) +
	sizeof(positionMultiplicator) +
	sizeof(stopperSafePosition) +
	sizeof(viceRaiseTimer) +
	((sizeof(unsigned int) + sizeof(unsigned int)) * SIZE_COUNT);
}

int GetRodCountEEPROM_Address() {
	return GetJobConfigEEPROMAddress() + sizeof(currentJobConfig)*JOB_CONFIG_RING_COUNT;
}

void SaveJobConfig() {
#ifdef ENABLE_SAVE_TO_EEPROM
	if (loadedJobConfig.rodSize != currentJobConfig.rodSize &&
		loadedJobConfig.extrudeLength != currentJobConfig.extrudeLength &&
		saveDataVersion == SAVE_DATA_VERSION) {

		currentJobConfig.jobID++;

		jobConfigRingPosition = (jobConfigRingPosition + 1) % JOB_CONFIG_RING_COUNT;
		int curAdd = GetJobConfigEEPROMAddress() + jobConfigRingPosition*sizeof(currentJobConfig);
#ifdef DEBUG_SERIAL
		Serial.print("Write JobConfig at address: ");
		Serial.print(curAdd);
		Serial.print("\nJobID:");
		Serial.print(currentJobConfig.jobID);
		Serial.print("\nSize: ");
		Serial.print(currentJobConfig.rodSize);
		Serial.print("\nLen: ");
		Serial.print(currentJobConfig.extrudeLength);
		Serial.print("\n");
#endif
		eeprom_write_block((void*)&currentJobConfig, (void*)curAdd, sizeof(currentJobConfig));

		eeprom_busy_wait();
	}
#endif
}

void IncrementCount() {
	thisJobRodCount++;
	rodCount++;

	if (saveDataVersion == SAVE_DATA_VERSION) {
		int curAdd = GetRodCountEEPROM_Address();

		rodCountRingPosition = (rodCountRingPosition + 1) % ROD_COUNT_RING_COUNT;

		curAdd += rodCountRingPosition * sizeof(unsigned int);

		unsigned int c = eeprom_read_word((unsigned int*)curAdd);
		c++;
#ifdef ENABLE_SAVE_TO_EEPROM
		eeprom_write_word((unsigned int*)curAdd, c);
#endif
	}

	if (currentMessage == MessageAutoModeRunning || currentMessage == MessageAutoModeStats) {
		UpdateDisplayCount();
		if (displayStats) {
			UpdateDisplayStats();
		}
	}
}

//	Helper functions
void SetupPin(int pin, bool in) {
	SetupPin(pin, in, false);
}

void SetupPin(int pin, bool in, bool pullUp) {
	if (in) {
		pinMode(pin, INPUT);
		digitalWrite(pin, pullUp);
	} else {
		pinMode(pin, OUTPUT);
	}
}

void SetupRelay(int pin) {
	SetupRelay(pin, -1);
}

void SetupRelay(int pin, int led) {
	pinMode(pin, OUTPUT);
	digitalWrite(pin, true);
	if (led > -1) {
		pinMode(led, OUTPUT);
	}
}

bool PURead(int pin) {
	return !digitalRead(pin);
}

void RelayWrite(int relayPin, bool enabled) {
	RelayWrite(relayPin, enabled, -1);
}

void RelayWrite(int relayPin, bool enabled, int ledPin) {
	if (digitalRead(relayPin) == enabled) {
		digitalWrite(relayPin, !enabled);
		if (ledPin > -1) {
			digitalWrite(ledPin, enabled);
		}
#ifdef EXTERNAL_PUMP
		switch (relayPin) {
		case OUT_VALVE_FORWARD:
		case OUT_VALVE_BACKWARD:
		case OUT_VICE_CLOSE:
		case OUT_VICE_OPEN:
			digitalWrite(OUT_PUMP, !enabled);
			break;
		}
#endif
	}
}

#ifdef DEBUG_SERIAL
void takeADump(char dumpType) {
	//	States
	if (dumpType == 'a' || dumpType == 's') {
		printSerialVariableI("currentMode", currentMode);
		printSerialVariableI("initState", initState);
		printSerialVariableI("currentMessage", currentMessage);
		printSerialVariableI("lastMessage", lastMessage);
	}

	//	Job Info
	if (dumpType == 'a' || dumpType == 'j') {
		printSerialVariableJobConfig("currentJobConfig", currentJobConfig);
		printSerialVariableJobConfig("loadedJobConfig", loadedJobConfig);
		printSerialVariableI("prevRodSize", prevRodSize);
		printSerialVariableB("canReadRodSize", canReadRodSize);
		printSerialVariableI("currentRodSizeIndex", currentRodSizeIndex);
		printSerialVariableI("prevExtrudeLength", prevExtrudeLength);
		printSerialVariableB("canReadExtrudeLength", canReadExtrudeLength);
	}

	//	Flags
	if (dumpType == 'a' || dumpType == 'f') {
		printSerialVariableB("pumpStarted", pumpStarted);
		printSerialVariableB("stopRaised", stopRaised);
		printSerialVariableB("isPanicked", isPanicked);
		printSerialVariableB("isCoverOpenned", isCoverOpenned);
		printSerialVariableB("initialized", initialized);
		printSerialVariableB("unitType", unitType);
		printSerialVariableB("threadType", threadType);
		printSerialVariableB("displayStats", displayStats);
	}

	//	Piston
	if (dumpType == 'a' || dumpType == 'p') {
		printSerialVariableUI("pistonPosition", pistonPosition);
		printSerialVariableUI("pistonStrokeLength", pistonStrokeLength);
		printSerialVariableUI("realStrokeLength", realStrokeLength);
		printSerialVariableF("positionMultiplicator", positionMultiplicator);
		printSerialVariableUI("stopperSafePosition", stopperSafePosition);
		printSerialVariableUI("minPistonPosition", minPistonPosition);
		printSerialVariableUI("maxPistonPosition", maxPistonPosition);
	}

	//	Misc
	if (dumpType == 'a' || dumpType == 'm') {
		printSerialVariableUI("viceRaiseTimer", viceRaiseTimer);
		printSerialVariableI("currentPressure", currentPressure);
		printSerialVariableI("initModeHomeCount", initModeHomeCount);
		printSerialVariableUI("homePressTime", homePressTime);
		printSerialVariableUI("calibrateVicePressTime", calibrateVicePressTime);
		printSerialVariableUI("manualViceOpenPressTime", manualViceOpenPressTime);
		printSerialVariableUI("manualViceClosePressTime", manualViceClosePressTime);
		printSerialVariableUI("toggleStopperPressTime", toggleStopperPressTime);
		printSerialVariableB("stopperToHigh", stopperToHigh);
		printSerialVariableB("stopperToLow", stopperToLow);
		printSerialVariableUL("stopperTimer", stopperTimer);
		printSerialVariableBYTE("saveDataVersion", saveDataVersion);
		printSerialVariableI("jobConfigRingPosition", jobConfigRingPosition);
	}

	//	Auto
	if (dumpType == 'a' || dumpType == 'o') {
		printSerialVariableUL("rodCount", rodCount);
		printSerialVariableI("rodCountRingPosition", rodCountRingPosition);
		printSerialVariableI("autoState", autoState);
		printSerialVariableI("autoModeStartPos", autoModeStartPos);
		printSerialVariableI("autoModeBackPos", autoModeBackPos);
		printSerialVariableI("autoModeRaiseStopperPos", autoModeRaiseStopperPos);
		printSerialVariableI("autoModeExtrudePos", autoModeExtrudePos);
		printSerialVariableI("autoModeVicePressure", autoModeVicePressure);
		printSerialVariableI("autoModeExtrudePressure", autoModeExtrudePressure);
		printSerialVariableI("autoModeViceTimer", autoModeViceTimer);
		printSerialVariableUI("thisJobRodCount", thisJobRodCount);
		printSerialVariableB("autoModeHomeWasDown", autoModeHomeWasDown);

		printSerialVariableUL("ignorePressureTime", ignorePressureTime);
		printSerialVariableB("waitingForHomePressure", waitingForHomePressure);
		printSerialVariableUL("autoModeViceTimerDelta", autoModeViceTimerDelta);
		printSerialVariableUL("coverOpennedTime", coverOpennedTime);

		printSerialVariableUL("nextPistonPositionLogTime", nextPistonPositionLogTime);
		printSerialVariableUI("logPrevPistonPosition", logPrevPistonPosition);

		printSerialVariableB("predalWasReleased", predalWasReleased);
	}

	//	Timing
	if (dumpType == 'a' || dumpType == 't') {
		printSerialVariableULArray("lastExtrudeTimes", lastExtrudeTimes, STATS_AVERAGE_TIME_SAMPLING_COUNT);
		printSerialVariableULArray("lastWaitTimes", lastWaitTimes, STATS_AVERAGE_TIME_SAMPLING_COUNT);

		printSerialVariableUI("currentSamplingIndex", currentSamplingIndex);
		printSerialVariableUL("refWaitTime", refWaitTime);
		printSerialVariableUL("refExtrudeTime", refExtrudeTime);
	}

	//	Learning
	if (dumpType == 'a' || dumpType == 'l') {
		printSerialVariableB("isLearning", isLearning);
		printSerialVariableI("learningDirection", learningDirection);
		printSerialVariableUI("learningTargetPosition", learningTargetPosition);
		printSerialVariableUI("learningPrevPistonPosition", learningPrevPistonPosition);

		printSerialVariableIArray("learningForwardValues", learningForwardValues, LEARNING_COUNT);

		printSerialVariableUI("learningForwardValuesCount", learningForwardValuesCount);
		printSerialVariableUL("learningTargetTime", learningTargetTime);
		printSerialVariableUI("forwardOvershoot", forwardOvershoot);
	}
}

void printSerialVariableI(char* variableName, int variableValue) {
	Serial.print(variableName);
	Serial.print(": ");
	Serial.print(variableValue);
	Serial.print("\n");
}

void printSerialVariableUI(char* variableName, unsigned int variableValue) {
	Serial.print(variableName);
	Serial.print(": ");
	Serial.print(variableValue);
	Serial.print("\n");
}

void printSerialVariableUL(char* variableName, unsigned long variableValue) {
	Serial.print(variableName);
	Serial.print(": ");
	Serial.print(variableValue);
	Serial.print("\n");
}

void printSerialVariableBYTE(char* variableName, byte variableValue) {
	Serial.print(variableName);
	Serial.print(": ");
	Serial.print(variableValue);
	Serial.print("\n");
}

void printSerialVariableB(char* variableName, bool variableValue) {
	Serial.print(variableName);
	Serial.print(": ");
	Serial.print(variableValue ? "TRUE" : "FALSE");
	Serial.print("\n");
}

void printSerialVariableF(char* variableName, float variableValue) {
	Serial.print(variableName);
	Serial.print(": ");
	Serial.print(variableValue);
	Serial.print("\n");
}

void printSerialVariableJobConfig(char* variableName, JobConfig variableValue) {
	Serial.print(variableName);
	Serial.print("\n");
	Serial.print("	jobID: ");
	Serial.print(variableValue.jobID);
	Serial.print("\n");
	Serial.print("	rodSize: ");
	Serial.print(variableValue.rodSize);
	Serial.print("\n");
	Serial.print("	extrudeLength: ");
	Serial.print(variableValue.extrudeLength);
	Serial.print("\n");
}

void printSerialVariableULArray(char* variableName, unsigned long variableValue[], int arrayLen) {
	Serial.print(variableName);
	Serial.print("\n");
	for (int i=0; i<arrayLen; i++) {
		Serial.print("	");
		Serial.print(i);
		Serial.print(": ");
		Serial.print(variableValue[i]);
		Serial.print("\n");
	}
}

void printSerialVariableIArray(char* variableName, int variableValue[], int arrayLen) {
	Serial.print(variableName);
	Serial.print("\n");
	for (int i=0; i<arrayLen; i++) {
		Serial.print("	");
		Serial.print(i);
		Serial.print(": ");
		Serial.print(variableValue[i]);
		Serial.print("\n");
	}
}
#endif
