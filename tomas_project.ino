/* Temperature-driven mains(AC, 230V) relay with 
 * a graph display and threshold adjust.
 *
 * - DHT11 1-wire temperature sensor
 * - 1602 display in 4-bit mode, with 8-character scrolling pixel graph
 * - Threshold-adjustable
 * - Developed on Atmega1280 (Arduino Mega)
 *
 * Libraries used:
 * - LiquidCrystal
 * - niesteszeck's interrupt-driven DHT11 library:
 * 	https://github.com/niesteszeck/idDHT11
 *
 * For Tomas
 * By gammy
 */


#include <LiquidCrystal.h>
#include <idDHT11.h> // Interrupt-driven library

#define THRESHOLD_BASE   	25 // In degrees C
#define DELAY_LOOP_MS           2000

#define PIN_TEMP		3
#define PIN_RELAY		2
#define PIN_KNOB 		15

#define SIGN_DEGREES		(char) 0xdf
#define SIGN_THRESHOLD		(char) 126
#define PRINT_TEMP(x) 		{ lcd.print(x); lcd.print(SIGN_DEGREES); lcd.print("C"); }
#define GRAPH_WIDTH		8

// Graph buffer
byte graph[GRAPH_WIDTH + 1][8];

// Timers
uint32_t timer_beg;

// Create DHT11 (Temperature sensor) class
void dht11_callback(); // must be declared before the lib initialization
idDHT11 DHT11(PIN_TEMP, 1 /* XXX Pin 3 = int 1 on Arduino Mega */, dht11_callback);

// Create LCD class. Pins are: RS, EN, D4, D5, D6, D7
LiquidCrystal lcd(6, 7,             // Register Select & Data Enable
		  8, 9, 10, 11);    // Bit 1, 2, 3, 4

void dht11_callback() {
	DHT11.isrCallback();
}

void relay_set(int state) {

	digitalWrite(PIN_RELAY, state);

	lcd.setCursor(13, 0);
	lcd.print(state ? " On" : "Off");
}

void setup() {

	// Initialize relay pin and set to LOW (off)
	pinMode(PIN_RELAY, OUTPUT);

	// Initialize LCD
	lcd.begin(16, 2);

	lcd.clear();

	// Initialize relay
	relay_set(LOW);

	Serial.begin(9600);

	timer_beg = millis();

}

void loop() {

	static int i, y;
	static int t_scale;
	static int t_min = 100;
	static int t_max = 0;

	/** Temperatue ******************************************************/
	Serial.print("Acquiring...");
	DHT11.acquire();
	while(DHT11.acquiring()); // Can wait forever, SHIT library..
	Serial.print("ok\n");

	if(DHT11.getStatus() != IDDHTLIB_OK) {

		return;

		relay_set(LOW); // XXX For safety

		lcd.setCursor(0, 0);
		lcd.print("Error");

		delay(5000);

		lcd.setCursor(0, 0);
		lcd.print("     ");

		return;
	}

	int t = DHT11.getCelsius();

	// XXX HAX HAX HAX XXX
	// Adjust minimum and maximum observed temperatures
	//if(t < t_min)
	//	t_min = t;
	t_min = 24;
	if(t < 24)
		t = 24;

	t_max = 32;
	if(t > 32)
		t = 32;
	//if(t > t_max)
	//	t_max = t;

	t_scale = t - t_min;
	int tmax = t_max - t_min;

	int tq = ((float) t_scale / (float) tmax) * (float) 7.0f;

	Serial.print("Min:"); Serial.print(t_min);
	Serial.print(", Max:"); Serial.print(t_max);
	Serial.print(", Scale:"); Serial.print(t_scale);
	Serial.print(", Tmax:"); Serial.print(tmax);
	Serial.print(", tq:"); Serial.println(tq);

	lcd.setCursor(GRAPH_WIDTH, 0);
	PRINT_TEMP(t);

	/** Graph ***********************************************************/

	bool graph_update = millis() - timer_beg > DELAY_LOOP_MS;

	if(graph_update) {

		// Insert temperatue into last character, rightmost pixel of the graph
		for(i = 0; i < 7; i++)
			if(i <= tq)
				graph[GRAPH_WIDTH - 1][6 - i] |= 0b00000001;
		lcd.createChar(GRAPH_WIDTH - 1, graph[GRAPH_WIDTH - 1]);

		// Copy last bit of next char to first bit of current char
		for(i = 1; i < GRAPH_WIDTH; i++) {
			for(y = 0; y < 7; y++)
				graph[i - 1][y] ^= ((graph[i][y] << 2) & 0b10000000) >> 7;
			lcd.createChar(i - 1, graph[i - 1]);
		}

		// Print the graph characters to display
		lcd.setCursor(0, 0);

		for(i = 0; i < GRAPH_WIDTH; i++)
			lcd.write((byte) i);

	}
 
	/** Temperatue Threshold ********************************************/

	// Calculate threshold, taking temp pot into consideration
	// (This'll give 32 discrete steps with 0 in the middle)
	int8_t threshold = THRESHOLD_BASE + (512 - analogRead(PIN_KNOB)) / 32;
	if(threshold < 0)
		threshold = 0;

	lcd.setCursor(0, 1);
	lcd.print("Turns on at ");
	PRINT_TEMP(threshold);

	/** Relay Control ***************************************************/

	if(t >= threshold) {
		relay_set(HIGH);
	} else {
		relay_set(LOW);
	}

	if(graph_update) {

		// Shift all pixels on each character one step to the left
		for(i = 0; i < GRAPH_WIDTH; i++) {
			for(y = 0; y < 7; y++)
				graph[i][y] <<= 1;
			lcd.createChar(i, graph[i]);
		}

		timer_beg = millis();
	} else {
		delay(50);
	}

	delay(100); // XXX DHT11 is CRAZY and *REQUIRES* this

}
