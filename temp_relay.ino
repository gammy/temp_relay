/* Temperature-driven mains(AC, 230V) relay with 
 * a graph display and threshold adjust.
 * 
 * Software features:
 * - "All-Above or All-Below" threshold relay switching
 * - Realtime-adjustable threshold
 * - (5 * 8) * 7 pixel scrolling historical bar chart
 * - Custom character for displaying "°C"
 *
 * Hardare:
 * - Analog Device tmp421 I2C: http://shop.moderndevice.com/products/tmp421-temperature-sensor
 * - 1602 display in 4-bit mode, with character scrolling pixel graph (up to 8 chars)
 * - Analog POT for threshold adjust
 * - Developed on Atmega328p (Arduino Duelanove)
 *
 * Libraries used:
 * - LiquidCrystal
 * - LibTempTMP421 (which in turn uses Wire for I2C communication)
 *   https://github.com/moderndevice/LibTempTMP421
 * 
 * TODO:
 * - Periodically store coolest and warmest readings and load on boot
 * - Periodically store the bar chart and load on boot
 * - Add scroll functionality on bottom row to show some stats(min/max, etc)
 *
 * For Tomas
 * By gammy
 */


#include <LiquidCrystal.h>
#include <Wire.h>
#include <LibTempTMP421.h>
#include <EEPROM.h>

#define THRESHOLD_BASE   	25 // In degrees C

#define TIMER_GRAPH_UPDATE_MS   1000 * (60 * 21) // (5*7) = 35 pixels, update once every 21 minutes = ~12 hours total
#define TIMER_RELAY_UPDATE_MS	1000
//#define TIMER_STORE_UPDATE_MS	(10 * TIMER_RELAY_UPDATE_MS)

#define SAMPLE_COUNT		20 // Must be less than sizeof(int) - 1

#define PIN_RELAY		2
#define PIN_KNOB 		0

#define SIGN_THRESHOLD		(char) 126

#define GRAPH_WIDTH		7 // Max 7, since we use 8th (last) char for custom deg/C 

#define PRINT_TEMP_FRACT(a) 	{ lcd.print((int) t); \
	                          lcd.print('.'); \
	                          lcd.print((int) ((t - (int) t) * 10)); \
                                  lcd.write((byte) GRAPH_WIDTH);}

#define PRINT_TEMP(a)		{ lcd.print(a); \
				  lcd.write((byte) GRAPH_WIDTH);}

// Graph buffer
byte graph[GRAPH_WIDTH + 1][8];

// Custom symbol for °C, stored after last graph buffer char if available
byte char_degc[8] = {
	0b00001000,
	0b00010100,
	0b00001000,
	0b00000011,
	0b00000100,
	0b00000100,
	0b00000011,
	0b00000000
};

struct {
	uint32_t graph;
	uint32_t store;
} timers;

typedef struct temp_s {
	LibTempTMP421 *sensor;
	float scale;
	float min;
	float max;
};

temp_s temp;

// Create Analog Device (Temperature sensor) class (which initializes I2C via Wire)
// Create LCD class. Pins are: RS, EN, D4, D5, D6, D7
LiquidCrystal lcd(6, 7,             // Register Select & Data Enable
		  8, 9, 10, 11);    // Bit 1, 2, 3, 4

void relay_set(int state) {

	digitalWrite(PIN_RELAY, state);

	lcd.setCursor(13, 0);
	lcd.print(state ? " On" : "Off");
}

void sample_update(int state) {

	static uint32_t timer_beg = millis();
	static bool samples[SAMPLE_COUNT];
	static unsigned int count_above, count_below;

	unsigned int i;

	if(millis() - timer_beg >= TIMER_RELAY_UPDATE_MS) {

		// Populate last entry of sample buffer
		samples[SAMPLE_COUNT - 1] = state == HIGH;

		count_above = count_below = 0;

		// Count results in buffer
		for(i = 0; i < SAMPLE_COUNT; i++) {
			if(samples[i] == true)
				count_above++;
			else
				count_below++;
		}

		// All samples are above threshold
		if(count_above == SAMPLE_COUNT && count_below == 0) {
			relay_set(HIGH);
		} else 
		// All samples are below threshold
		if(count_below == SAMPLE_COUNT && count_above == 0) {
			relay_set(LOW);
		}

		// Shift all samples a step left
		for(i = 1; i < SAMPLE_COUNT; i++)
			samples[i - 1] = samples[i];

		timer_beg = millis();
	}

}

void setup() {

	// Initialize relay pin (default state is LOW; off)
	pinMode(PIN_RELAY, OUTPUT);

	// Initialize LCD
	lcd.begin(16, 2);
	lcd.clear();
	
	relay_set(LOW);

	lcd.setCursor(0, 0); lcd.print("Temp Relay v0.1 ");
	lcd.setCursor(0, 1); lcd.print("   By gammy 2013");
	delay(2000);

	lcd.clear();
	lcd.setCursor(0, 1);
	lcd.print("Turns on at ");

	Serial.begin(9600);
			
	if(GRAPH_WIDTH < 8)
		lcd.createChar(GRAPH_WIDTH, char_degc);

	temp.sensor = new LibTempTMP421(0);
	temp.min = 100.0f;
	temp.max = 0.0f;

	timers.graph = millis();
	timers.store = timers.graph;

}

void loop() {

	static int i, y;

	/** Temperatue ******************************************************/
	float t = temp.sensor->GetTemperature();

	// Adjust minimum and maximum observed temperatures
	if(t < temp.min)
		temp.min = t;

	if(t > temp.max)
		temp.max = t;

	temp.scale = t - temp.min;
	float tmax = temp.max - temp.min;

	int tq = (temp.scale / tmax) * (float) 8.0f;

	//Serial.print("Min:"); Serial.print(t_min);
	//Serial.print(", Max:"); Serial.print(t_max);
	//Serial.print(", Scale:"); Serial.print(t_scale);
	//Serial.print(", Tmax:"); Serial.print(tmax);
	//Serial.print(", tq:"); Serial.println(tq);

	lcd.setCursor(GRAPH_WIDTH, 0);
	PRINT_TEMP_FRACT(t);
	
	/** Graph ***********************************************************/

	//uint32_t CUNT = millis() - timers_graph_beg;
	uint32_t FUCK = millis() - timers.graph;

	Serial.print((uint32_t) ((uint32_t ) millis() - (uint32_t) timers.graph));
	Serial.print(' ');
	Serial.print(FUCK);
	Serial.print(' ');
	Serial.println((uint32_t) TIMER_GRAPH_UPDATE_MS);

	if(FUCK > (uint32_t) TIMER_GRAPH_UPDATE_MS) {
		Serial.println("FUUUUUUUUUUUUUUUUUUUUUCK");
	} 

	// XXX No averaging
	if(millis() - timers.graph > (uint32_t) TIMER_GRAPH_UPDATE_MS) {
		Serial.println("Updating graph");

		// Shift all pixels on each character one step to the left
		for(i = 0; i < GRAPH_WIDTH; i++) {
			for(y = 0; y < 8; y++)
				graph[i][y] <<= 1;
		}

		// Insert temperatue into last character, rightmost pixel of the graph
		for(i = 0; i < 8; i++) {
			if(i <= tq)
				graph[GRAPH_WIDTH - 1][7 - i] |= 0b00000001;
		}

		// Write last character to lcd
		lcd.createChar(GRAPH_WIDTH - 1, graph[GRAPH_WIDTH - 1]);

		// Copy last bit of next char to first bit of current char
		// and display all characters except for the last one
		for(i = 1; i < GRAPH_WIDTH; i++) {
			for(y = 0; y < 8; y++)
				graph[i - 1][y] ^= ((graph[i][y] << 2) & 0b10000000) >> 7;
			lcd.createChar(i - 1, graph[i - 1]);
		}

		// Print the graph characters to display
		lcd.setCursor(0, 0);

		for(i = 0; i < GRAPH_WIDTH; i++)
			lcd.write((byte) i);
		
		timers.graph = millis();
	}
 
	/** Temperatue Threshold ********************************************/

	// Calculate threshold, taking temp pot into consideration
	// (This'll give 32 discrete steps with 0 in the middle)
	int threshold = THRESHOLD_BASE + (512 - analogRead(PIN_KNOB)) / 32;
	if(threshold < 0)
		threshold = 0;

	lcd.setCursor(12, 1);
	PRINT_TEMP(threshold);

	/** Relay Control (abstracted) **************************************/

	sample_update(t >= threshold);

}
