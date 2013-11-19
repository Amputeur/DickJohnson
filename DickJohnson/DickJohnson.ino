// DickJohnson.ino

void setup() {
	pinMode(13, OUTPUT);
}

void loop() {
	delay(500);
	digitalWrite(13, true);
	delay(500);
	digitalWrite(13, false);
}

