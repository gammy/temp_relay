/* WARNING: This code was written for an arduino mega, not duelanove! */
#include <LiquidCrystal.h>
#include <dht11.h>

#define TEMP_RELAY_ON   	26
#define DELAY_LOOP_SECONDS	(5 * 1000)
#define PIN_TEMP		3
#define PIN_RELAY		2
#define SIGN_DEGREES		(char) 0xDF
#define PRINT_TEMP(x) 		{ lcd.print(x); lcd.print(SIGN_DEGREES); lcd.print("C"); }

// Instantiate DHT11 (Temperature sensor); doesn't initialize here
dht11 DHT11;

// Create LCD instance. Pins are: RS, EN, D4, D5, D6, D7
LiquidCrystal lcd(6, 7,             // Register Select & Data Enable
		  8, 9, 10, 11);    // Bit 1, 2, 3, 4

void relay_set(int state) {

	digitalWrite(PIN_RELAY, state);

	lcd.setCursor(6, 0);
	lcd.print("Fan is ");
	lcd.print(state ? "on " : "off");
}

void setup() {

	// Initialize relay pin and set to LOW (off)
	pinMode(PIN_RELAY, OUTPUT);

	// Initialize LCD
	lcd.begin(16, 2);

	lcd.clear();
	lcd.setCursor(0, 1);
	lcd.print("Threshold ");
	PRINT_TEMP(TEMP_RELAY_ON);

	// Initialize relay
	relay_set(LOW);

}

void loop() {

	int status_dht11 = DHT11.read(PIN_TEMP);

	lcd.setCursor(0, 0);

	if(status_dht11 != DHTLIB_OK) {
		relay_set(LOW); // XXX For safety
		lcd.print("Error");
		delay(5000);
		return;
	}

	int t = DHT11.temperature;

	PRINT_TEMP(t);

	if(t >= TEMP_RELAY_ON) {
		relay_set(HIGH);
	} else {
		relay_set(LOW);
	}

	delay(DELAY_LOOP_SECONDS);
}
