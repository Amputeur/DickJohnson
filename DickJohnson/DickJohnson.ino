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
}

void loop() {
	// NOT on because of PullUp resistor.
	if (!digitalRead(IN_PEDAL)) {
		//	Close vice.
		digitalWrite(OUT_CLOSE_VICE, true);

		// Wait for target preasure.
		while (analogRead(IN_VICE_PREASURE) < 512) {
			delay(5);
		}
		digitalWrite(OUT_CLOSE_VICE, false);

		//	Drop some oil!
		digitalWrite(OUT_DROP_OIL, true);
		delay(500);
		digitalWrite(OUT_DROP_OIL, false);

		//	Move piston back a little to prevent scratching noise.... Atchoum!
		digitalWrite(OUT_PISTON_BACKWARD, true);
		
		while (analogRead(IN_PISTON_RANGE) > 250) {
			delay(1);
		}
		digitalWrite(OUT_PISTON_BACKWARD, false);

		//	Raise Stopper
		stopperServo.write(179);
		delay(250);

		//	EXTRUDE!!!
		digitalWrite(OUT_PISTON_FORWARD, true);
		while (analogRead(IN_PISTON_RANGE) < 900) {
			delay(1);
		}
		digitalWrite(OUT_PISTON_FORWARD, false);

		//	Move back.
		digitalWrite(OUT_PISTON_BACKWARD, true);
		while (analogRead(IN_PISTON_RANGE) > 250) {
			delay(1);
		}
		digitalWrite(OUT_PISTON_BACKWARD, false);

		//	Lower Stopper.
		stopperServo.write(0);

		//	Open vice.
		digitalWrite(OUT_OPEN_VICE, true);
		// Wait for vice open micro switch.
		while (digitalRead(IN_VICE_OPEN)) {
			delay(5);
		}
		digitalWrite(OUT_OPEN_VICE, false);
	}
}
