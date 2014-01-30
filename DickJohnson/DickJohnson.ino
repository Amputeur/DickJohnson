#include <LiquidCrystal.h>

#define MIN_ROD_SIZE 0.25f * ROD_SIZE_MULTIPLICATOR
#define MAX_ROD_SIZE 1.25f * ROD_SIZE_MULTIPLICATOR
#define ROD_SIZE_MULTIPLICATOR 2000

#define MIN_EXTRUDE_LENGTH 1.0f * EXTRUDE_LENGTH_MULTIPLICATOR
#define MAX_EXTRUDE_LENGTH 7.0f * EXTRUDE_LENGTH_MULTIPLICATOR
#define EXTRUDE_LENGTH_MULTIPLICATOR 1250

#define THREAD_NC true
#define THREAD_NF false
#define UNIT_MM true
#define UNIT_INCH false
#define UnitType bool
#define ThreadType bool

#define CONFIG_ENCODER_A 2
#define CONFIG_ENCODER_A_INTERRUPT 0
#define CONFIG_ENCODER_B 3

#define IN_MODE_INIT 22
#define IN_MODE_CONFIG 53
#define IN_MODE_RUN 52

#define OUT_MODE_CONFIG_LED 51
#define OUT_MODE_RUN_LED 50
#define OUT_MODE_INIT_LED 47

#define IN_UNIT_SELECTOR 23
#define IN_SET_EXTRUDE_LENGTH 48
#define IN_SET_ROD_SIZE 49

#define min(a, b) (((a) < (b)) ? (a) : (b))
#define max(a, b) (((a) < (b)) ? (b) : (a))
#define clamp(in, low, high) min(max((in), (low)), (high))

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

bool initialized = false;

bool wasConfig = false;
bool wasRun = false;
bool wasInit = false;

UnitType unitType = UNIT_MM;
ThreadType threadType = THREAD_NC;

void setup() {
	lcd.begin(16, 2);
	lcd.clear();
	UpdateDisplayComplete();

	SetupPin(CONFIG_ENCODER_A, true, true);
	SetupPin(CONFIG_ENCODER_B, true, true);

	SetupPin(IN_MODE_INIT, true, true);
	SetupPin(IN_MODE_CONFIG, true, true);
	SetupPin(IN_MODE_RUN, true, true);

	SetupPin(OUT_MODE_INIT_LED, false);
	SetupPin(OUT_MODE_RUN_LED, false);
	SetupPin(OUT_MODE_CONFIG_LED, false);

	SetupPin(IN_UNIT_SELECTOR, true, true);
	SetupPin(IN_SET_EXTRUDE_LENGTH, true, true);
	SetupPin(IN_SET_ROD_SIZE, true, true);

	SetupPin(13, false);
}

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

void loop() {
	bool modeInit = !digitalRead(IN_MODE_INIT);
	bool modeConfig = !digitalRead(IN_MODE_CONFIG);
	bool modeRun = !digitalRead(IN_MODE_RUN);

	if (modeInit && !modeConfig && !modeRun) {
		if (!wasInit) {
			wasRun = wasConfig = false;
			wasInit = true;
			digitalWrite(OUT_MODE_CONFIG_LED, false);
			digitalWrite(OUT_MODE_RUN_LED, false);
			digitalWrite(OUT_MODE_INIT_LED, true);

			if (canReadRodSize || canReadExtrudeLength) {
				detachInterrupt(CONFIG_ENCODER_A_INTERRUPT);
				canReadExtrudeLength = canReadRodSize = false;
			}
		}
		LoopInit();
	} else if (!initialized) {
		//	Can't go into another mode if not initialized. Display an error message on screen.
		lcd.clear();
		lcd.print("Select INIT mode");
	} else if (modeConfig && !modeRun && !modeInit) {
		if (!wasConfig) {
			wasRun = wasInit = false;
			wasConfig = true;
			digitalWrite(OUT_MODE_INIT_LED, false);
			digitalWrite(OUT_MODE_RUN_LED, false);
			digitalWrite(OUT_MODE_CONFIG_LED, true);
			canReadRodSize = canReadExtrudeLength = false;
		}
		LoopConfig();
	} else if (!modeConfig && modeRun) {
		if (!wasRun) {
			wasConfig = wasInit = false;
			wasRun = true;
			digitalWrite(OUT_MODE_INIT_LED, false);
			digitalWrite(OUT_MODE_RUN_LED, true);
			digitalWrite(OUT_MODE_CONFIG_LED, false);

			if (canReadRodSize || canReadExtrudeLength) {
				detachInterrupt(CONFIG_ENCODER_A_INTERRUPT);
				canReadExtrudeLength = canReadRodSize = false;
			}
		}
		LoopRun();
	}
}

void LoopInit() {
	ZeroCompleted();
}

void ZeroCompleted() {
	initialized = true;
}

void LoopConfig() {
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

void LoopRun() {
	
}

//	Interupt
void PistonPositionInterrupt() {

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
	lcd.clear();

	lcd.print("D:      L:");
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
