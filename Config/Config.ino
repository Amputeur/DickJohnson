#define IN_ROD_SIZE 0
#define IN_ROD_EXTRUDE_LENGTH 7
#define IN_CONFIG_POT 5	//	Analog


#define MIN_EXTRUDE_LENGTH 25
#define MAX_EXTRUDE_LENGTH 150
#define MIN_ROD_SIZE 312
#define MAX_ROD_SIZE 1500


#include <LiquidCrystal.h>

LiquidCrystal lcd(1, 6, 5, 4, 3, 2);

int rodSize = 500;
int extrudeLength = 100;

int lastPotVal;

void setup() {
	pinMode(IN_ROD_SIZE, INPUT);
	digitalWrite(IN_ROD_SIZE, true);
	pinMode(IN_ROD_EXTRUDE_LENGTH, INPUT);
	digitalWrite(IN_ROD_EXTRUDE_LENGTH, true);

	lcd.begin(16, 2);
	DisplaySettings();

	lastPotVal = analogRead(IN_CONFIG_POT); 
}

void loop() {
	if (!digitalRead(IN_ROD_SIZE)) {
		int curPot = analogRead(IN_CONFIG_POT);
		/*int diff = curPot - lastPotVal;

		rodSize += diff;
		rodSize = constrain(rodSize, MIN_ROD_SIZE, MAX_ROD_SIZE);*/
		rodSize = map(curPot, 0, 1023, MIN_ROD_SIZE, MAX_ROD_SIZE);
		DisplaySettings();
	} else if (!digitalRead(IN_ROD_EXTRUDE_LENGTH)) {
		int curPot = analogRead(IN_CONFIG_POT);
		/*int diff = curPot - lastPotVal;

		extrudeLength += diff;
		extrudeLength = constrain(extrudeLength, MIN_EXTRUDE_LENGTH, MAX_EXTRUDE_LENGTH);*/
		extrudeLength = map(curPot, 0, 1023, MIN_EXTRUDE_LENGTH, MAX_EXTRUDE_LENGTH);
		DisplaySettings();
	}

	lastPotVal = analogRead(IN_CONFIG_POT);
}

void DisplaySettings() {
	lcd.clear();
	char buffer [16];
	snprintf(buffer, 16, "Size: %2d.%3d\"", rodSize/1000, rodSize%1000);

	lcd.print(buffer);
	lcd.setCursor(0, 1);
	snprintf(buffer, 16, "Length: %3d mm", extrudeLength);
	lcd.print(buffer);
}
