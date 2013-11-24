#define IN_PEDAL 0
#define IN_VICE_PREASURE 5	//	Analog
#define IN_PISTON_RANGE 4	//	Analog
#define IN_VICE_OPEN 7

#define OUT_CLOSE_VICE 8
#define OUT_OPEN_VICE 13
#define OUT_STOPPER_SERVO 11
#define OUT_DROP_OIL 9
#define OUT_PISTON_BACKWARD 10
#define OUT_PISTON_FORWARD 12
#define OUT_DEBUG_LIGHT 13

#include <Servo.h>
#include <LiquidCrystal.h>

LiquidCrystal lcd(1, 6, 5, 4, 3, 2);
Servo stopperServo;

void setup() {
	pinMode(IN_PEDAL, INPUT);
	digitalWrite(IN_PEDAL, true);
	pinMode(IN_VICE_OPEN, INPUT);
	digitalWrite(IN_VICE_OPEN, true);

	pinMode(OUT_DEBUG_LIGHT, OUTPUT);
	pinMode(OUT_CLOSE_VICE, OUTPUT);
	pinMode(OUT_OPEN_VICE, OUTPUT);
	pinMode(OUT_STOPPER_SERVO, OUTPUT);
	pinMode(OUT_DROP_OIL, OUTPUT);
	pinMode(OUT_PISTON_BACKWARD, OUTPUT);
	pinMode(OUT_PISTON_FORWARD, OUTPUT);

	stopperServo.attach(OUT_STOPPER_SERVO);
	stopperServo.write(0);

	lcd.begin(16, 2);
}

void loop() {
	lcd.clear();
	lcd.print("Wait pedal!");
	// NOT on because of PullUp resistor.
	if (!digitalRead(IN_PEDAL)) {
		//	Close vice.
		digitalWrite(OUT_CLOSE_VICE, true);

				
		lcd.clear();
		// Wait for target preasure.
		lcd.print("Wait preasure.");
		while (analogRead(IN_VICE_PREASURE) < 512) {
			delay(5);
		}
		digitalWrite(OUT_CLOSE_VICE, false);
		
		lcd.clear();
		lcd.print("Dropping oil!");
		//	Drop some oil!
		digitalWrite(OUT_DROP_OIL, true);
		delay(500);
		digitalWrite(OUT_DROP_OIL, false);
		
		lcd.clear();
		lcd.print("Wait Piston back.");
		//	Move piston back a little to prevent scratching noise.... Atchoum!
		digitalWrite(OUT_PISTON_BACKWARD, true);
		
		while (analogRead(IN_PISTON_RANGE) > 250) {
			delay(1);
		}
		digitalWrite(OUT_PISTON_BACKWARD, false);
		
		lcd.clear();
		lcd.print("Raising Stopper.");
		//	Raise Stopper
		stopperServo.write(179);
		delay(250);
		
		lcd.clear();
		lcd.print("EXTRUDING!!!");
		lcd.setCursor(0, 1);
		lcd.print("Move forward.");
		//	EXTRUDE!!!
		digitalWrite(OUT_PISTON_FORWARD, true);
		while (analogRead(IN_PISTON_RANGE) < 900) {
			delay(1);
		}
		digitalWrite(OUT_PISTON_FORWARD, false);

		lcd.clear();
		lcd.print("Move backward.");
		//	Move back.
		digitalWrite(OUT_PISTON_BACKWARD, true);
		while (analogRead(IN_PISTON_RANGE) > 250) {
			delay(1);
		}
		digitalWrite(OUT_PISTON_BACKWARD, false);

		lcd.clear();
		lcd.print("Lowering Stopper");
		//	Lower Stopper.
		stopperServo.write(0);

		lcd.clear();
		lcd.print("Open vice");
		//	Open vice.
		digitalWrite(OUT_OPEN_VICE, true);
		// Wait for vice open micro switch.
		while (digitalRead(IN_VICE_OPEN)) {
			delay(5);
		}
		digitalWrite(OUT_OPEN_VICE, false);
	}
}
