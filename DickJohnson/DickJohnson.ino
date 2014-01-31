#include <LiquidCrystal.h>

#define MIN_ROD_SIZE 0.25f * ROD_SIZE_MULTIPLICATOR
#define MAX_ROD_SIZE 1.25f * ROD_SIZE_MULTIPLICATOR
#define ROD_SIZE_MULTIPLICATOR 2000

#define MIN_EXTRUDE_LENGTH 0.5f * EXTRUDE_LENGTH_MULTIPLICATOR
#define MAX_EXTRUDE_LENGTH 6.0f * EXTRUDE_LENGTH_MULTIPLICATOR
#define EXTRUDE_LENGTH_MULTIPLICATOR 1250

#define MAX_PRESSURE 255
#define RECALIBRATE_HOLD_TIME 2500
#define PISTON_POSITION_ZERO_OFFSET 100

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

#define IN_MODE_INIT 22
#define IN_MODE_MANUAL 53
#define IN_MODE_AUTO 52

#define OUT_MODE_MANUAL_LED 51
#define OUT_MODE_AUTO_LED 50
#define OUT_MODE_INIT_LED 47
#define OUT_PUMP_LED 24

#define OUT_VALVE_FORWARD 26
#define OUT_VALVE_BACKWARD 27

#define IN_UNIT_SELECTOR 23
#define IN_PUMP 25
#define IN_SET_EXTRUDE_LENGTH 48
#define IN_SET_ROD_SIZE 49
#define IN_HOME 46

#define IN_ANALOG_PRESSURE 0

#define min(a, b) (((a) < (b)) ? (a) : (b))
#define max(a, b) (((a) < (b)) ? (b) : (a))
#define clamp(in, low, high) min(max((in), (low)), (high))

enum Mode {
	ModeNone,
	ModeInit,
	ModeManual,
	ModeAuto
};

Mode currentMode = ModeNone;

enum Message {
	MessageNone,
	MessageErrorPumpNotStarted,
	MessageErrorSystemNotInitialized,
	MessageConfigModePumpNotStarted,
	MessageConfigModeNotInitialized,
	MessageConfigModeZeroInProgress,
	MessageConfigModeInitialized,
	MessagePressHomeWhenStopClear,
	MessageZeroInProgress,

	MessageCount
};

String messages[MessageCount][MessageCount] = {
	{"A BEbit Studio", "    MACHINE."},	//	MessageNone
	{"Pump not Started", ""},	//	MessageErrorPumpNotStarted
	{"Init the system", "first."},	//	MessageErrorSystemNotInitialized
	{"D:      L:", "Pump not Started"},	//	MessageConfigModePumpNotStarted
	{"D:      L:", "Press HOME"},	//	MessageConfigModeNotInitialized
	{"D:      L:", "Zero in progress"},	//	MessageConfigModeZeroInProgress
	{"D:      L:", ""},	//	MessageConfigModeInitialized
	{"Press HOME when", "stop is clear"},	//	MessagePressHomeWhenStopClear
	{"Zero in progress", ""}	//	MessageZeroInProgress
};

Message currentMessage = MessageNone;
Message lastMessage = MessageCount;

float extrusionTable[11][3] = {
//	 Size		NC		NF
	{0.25f,		1.0f,	1.0f},
	{0.3125f,	1.0f,	1.0f},
	{0.375,		1.0f,	1.0f},
	{0.4375f,	1.0f,	1.0f},
	{0.5f,		1.0f,	1.0f},
	{0.625f,	1.0f,	1.0f},
	{0.75f,		1.0f,	1.0f},
	{0.875f,	1.0f,	1.0f},
	{1.0f,		1.0f,	1.0f},
	{1.125f,	1.0f,	1.0f},
	{1.25f,		1.0f,	1.0f}
};

String sizeNames[11] = {
	"  1/4",
	" 5/16",
	"  3/8",
	" 7/16",
	"  1/2",
	"  5/8",
	"  3/4",
	"  7/8",
	"1    ",
	"1 1/8",
	"1 1/4"
};

LiquidCrystal lcd(8, 9, 10, 11, 12, 7);

int rodSize = 0.75f * ROD_SIZE_MULTIPLICATOR;
int prevRodSize;
bool canReadRodSize = false;

int extrudeLength = 4.0f * EXTRUDE_LENGTH_MULTIPLICATOR;
int prevExtrudeLength;
bool canReadExtrudeLength = false;

bool pumpStarted = false;
bool initialized = false;
bool zeroInprogress = false;

UnitType unitType = UNIT_MM;
ThreadType threadType = THREAD_NC;

unsigned int pistonPosition = 0;
int currentPressure = 0;

int initModeHomeCount = 0;
unsigned long homePressTime = -1;

void setup() {
	lcd.begin(16, 2);
	lcd.clear();
	UpdateDisplayComplete();

	SetupPin(CONFIG_ENCODER_A, true, true);
	SetupPin(CONFIG_ENCODER_B, true, true);

	SetupPin(PISTON_POSITION_ENCODER_A, true, true);
	SetupPin(PISTON_POSITION_ENCODER_B, true, true);

	SetupPin(IN_MODE_INIT, true, true);
	SetupPin(IN_MODE_MANUAL, true, true);
	SetupPin(IN_MODE_AUTO, true, true);

	SetupPin(OUT_MODE_INIT_LED, false);
	SetupPin(OUT_MODE_AUTO_LED, false);
	SetupPin(OUT_MODE_MANUAL_LED, false);
	SetupPin(OUT_PUMP_LED, false);

	SetupPin(OUT_VALVE_FORWARD, false);
	SetupPin(OUT_VALVE_BACKWARD, false);

	SetupPin(IN_UNIT_SELECTOR, true, true);
	SetupPin(IN_SET_EXTRUDE_LENGTH, true, true);
	SetupPin(IN_SET_ROD_SIZE, true, true);
	SetupPin(IN_PUMP, true, true);

	SetupPin(IN_HOME, true, true);

	SetupPin(13, false);
}

void loop() {
	bool pump = !digitalRead(IN_PUMP);
	if (pump != pumpStarted) {
		pumpStarted = pump;
		digitalWrite(OUT_PUMP_LED, pumpStarted);
	}

	currentPressure = analogRead(IN_ANALOG_PRESSURE);

	bool modeInit = !digitalRead(IN_MODE_INIT);
	bool modeManual = !digitalRead(IN_MODE_MANUAL);
	bool modeAuto = !digitalRead(IN_MODE_AUTO);

	if (modeInit && !modeManual && !modeAuto) {
		if (currentMode != ModeInit) {
			currentMode = ModeInit;

			initModeHomeCount = 0;

			digitalWrite(OUT_MODE_MANUAL_LED, false);
			digitalWrite(OUT_MODE_AUTO_LED, false);
			digitalWrite(OUT_MODE_INIT_LED, true);

			if (canReadRodSize || canReadExtrudeLength) {
				detachInterrupt(CONFIG_ENCODER_A_INTERRUPT);
				canReadExtrudeLength = canReadRodSize = false;
			}
		}
		LoopInit();
	} else if (modeManual && !modeAuto && !modeInit) {
		if (currentMode != ModeManual) {
			currentMode = ModeManual;
			digitalWrite(OUT_MODE_INIT_LED, false);
			digitalWrite(OUT_MODE_AUTO_LED, false);
			digitalWrite(OUT_MODE_MANUAL_LED, true);
			canReadRodSize = canReadExtrudeLength = false;
		}
		LoopManual();
	} else if (!modeInit && !modeManual && modeAuto) {
		if (currentMode != ModeAuto) {
			currentMode = ModeAuto;
			digitalWrite(OUT_MODE_INIT_LED, false);
			digitalWrite(OUT_MODE_AUTO_LED, true);
			digitalWrite(OUT_MODE_MANUAL_LED, false);

			if (canReadRodSize || canReadExtrudeLength) {
				detachInterrupt(CONFIG_ENCODER_A_INTERRUPT);
				canReadExtrudeLength = canReadRodSize = false;
			}
		}
		LoopAuto();
	}
}

void LoopInit() {
	if (!pumpStarted) {
		currentMessage = MessageConfigModePumpNotStarted;
	} else if (zeroInprogress) {
		currentMessage = MessageConfigModeZeroInProgress;
	} else if (initialized) {
		currentMessage =  MessageConfigModeInitialized;
	} else {
		currentMessage = MessageConfigModeNotInitialized;
	}
	
	UpdateDisplayComplete();

	if (zeroInprogress) {
		//	Check Preasure.
		if (currentPressure > MAX_PRESSURE) {
			pistonPosition = PISTON_POSITION_ZERO_OFFSET;
			attachInterrupt(PISTON_POSITION_ENCODER_A_INTERRUPT, PistonPositionInterrupt, CHANGE);

			digitalWrite(OUT_VALVE_BACKWARD, false);
			initialized = true;
			zeroInprogress = false;
		}
	} else {
		//	Wait HOME button.
		if (!digitalRead(IN_HOME)) {
			if (homePressTime == -1) {
				homePressTime = millis();
			}
		} else if (homePressTime != -1) {
			unsigned long heldTime = millis() - homePressTime;

			homePressTime = -1;
			if (!initialized || initModeHomeCount == 0 || heldTime < RECALIBRATE_HOLD_TIME) {
				zeroInprogress = true;
				digitalWrite(OUT_VALVE_BACKWARD, true);	
				initModeHomeCount = 1;
			} else {
				switch (initModeHomeCount) {
				    case 1:
				      //	Recalibrate Course.
				      initModeHomeCount = 2;
				      break;
				    case 2:
				      //	Recalibrate Stopper.
				      initModeHomeCount = 0;
				      break;
				    default:
				      initModeHomeCount = 0;
				      break;
				}
			}
			
		}
	}

	ReadConfig();
}

void ReadConfig() {
	bool updateDisplay = false;

	bool setSize = !digitalRead(IN_SET_ROD_SIZE);
	bool setLength = !digitalRead(IN_SET_EXTRUDE_LENGTH);

	UnitType setUnit = !digitalRead(IN_UNIT_SELECTOR);

	if (prevRodSize != rodSize) {
		updateDisplay = true;
		prevRodSize = rodSize;
		UpdateDisplayRodSize();
	}

	if (prevExtrudeLength != extrudeLength) {
		updateDisplay = true;
		prevExtrudeLength = extrudeLength;
		UpdateDisplayExtureLength();
	}

	if (setUnit != unitType) {
		updateDisplay = true;
		unitType = setUnit;

		UpdateDisplayRodSize();
		UpdateDisplayExtureLength();
	}

	bool setRodSizeButton = !digitalRead(IN_SET_ROD_SIZE);
	bool setExtrudeLengthButton = !digitalRead(IN_SET_EXTRUDE_LENGTH);

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

void LoopManual() {

}

void LoopAuto() {
	if (!initialized) {
		currentMessage = MessageErrorSystemNotInitialized;
	}

	UpdateDisplayComplete();
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
	if (digitalRead(CONFIG_ENCODER_A) == digitalRead(CONFIG_ENCODER_B)) {
		rodSize = min(rodSize + 1, MAX_ROD_SIZE);
	} else {
		rodSize = max(rodSize - 1, MIN_ROD_SIZE);
	}
}

void UpdateExtrudeLengthInterrupt() {
	if (digitalRead(CONFIG_ENCODER_A) == digitalRead(CONFIG_ENCODER_B)) {
		extrudeLength = min(extrudeLength + 1, MAX_EXTRUDE_LENGTH);
	} else {
		extrudeLength = max(extrudeLength - 1, MIN_EXTRUDE_LENGTH);
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
	float fSize = (float)rodSize / (float)ROD_SIZE_MULTIPLICATOR;

	int closestIndex = -1;
	float closest = 999.0f;
	String result = (String)rodSize;
	for (int i=0; i<11; i++) {
		float dist = extrusionTable[i][0] - fSize;
		if (dist < 0.0f) {
			dist *= -1.0f;
		}

		if (dist < closest) {
			closest = dist;
			closestIndex = i;
		}
	}

	if (closestIndex >= 0) {
		result = sizeNames[closestIndex];
	}

	lcd.setCursor(2, 0);
	lcd.print(result);
}

void UpdateDisplayExtureLength() {
	lcd.setCursor(10, 0);

	float fLength = (float)extrudeLength / (float)EXTRUDE_LENGTH_MULTIPLICATOR;

	if (unitType == UNIT_MM) {
		fLength *= 25.4f;

		String result = "";
		if (fLength < 10) {
			result = "00";
		} else if (fLength < 100) {
			result = "0";
		}
		lcd.print(result + (String)(int)fLength + "mm");
	} else {
		int major = extrudeLength / EXTRUDE_LENGTH_MULTIPLICATOR;
		int minor = 100 * ((float)(extrudeLength % EXTRUDE_LENGTH_MULTIPLICATOR) / (float)EXTRUDE_LENGTH_MULTIPLICATOR);

		lcd.print((String)major + "." + (minor<10?("0" + (String)minor):(String)minor) + "\"");
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