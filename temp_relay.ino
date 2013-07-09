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
 * - Developed on Atmega1280 (Arduino Mega)
 *
 * Libraries used:
 * - LiquidCrystal
 * - LibTempTMP421 (which in turn uses Wire for I2C communication)
 *   https://github.com/moderndevice/LibTempTMP421
 * 
 * TODO:
 * - Periodically store coolest and warmest readings and load on boot
 * - Periodically store the bar chart and load on boot
 *
 * For Tomas
 * By gammy
 */


#include <LiquidCrystal.h>
#include <Wire.h>
#include <LibTempTMP421.h>

#define THRESHOLD_BASE   	25 // In degrees C

#define TIMER_GRAPH_UPDATE_MS   1000 * (60 * 1)

#define TIMER_RELAY_UPDATE_MS	1000
#define SAMPLE_COUNT		60 // Must be less than sizeof(int) - 1

#define PIN_TEMP		3
#define PIN_RELAY		2
#define PIN_KNOB 		15

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

uint32_t timer_graph_beg;
uint32_t timer_backup_beg;

// Create Analog Device (Temperature sensor) class (which initializes I2C via Wire)
LibTempTMP421 temp = LibTempTMP421(0);

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

	lcd.setCursor(0, 1);
	lcd.print("Turns on at ");

	Serial.begin(9600);
			
	if(GRAPH_WIDTH < 8)
		lcd.createChar(GRAPH_WIDTH, char_degc);

	timer_graph_beg = millis();

}

void loop() {

	static int i, y;
	static int t_scale;
	static int t_min = 100;
	static int t_max = 0;

	/** Temperatue ******************************************************/
	float t = temp.GetTemperature();

	// XXX HAX HAX HAX XXX
	// Adjust minimum and maximum observed temperatures
	if(t < t_min)
		t_min = t;
	//t_min = 23;
	//if(t < t_min)
	//	t = t_min;

	//t_max = 34;
	//if(t > t_max)
	//	t = t_max;
	if(t > t_max)
		t_max = t;

	t_scale = t - t_min;
	int tmax = t_max - t_min;

	int tq = ((float) t_scale / (float) tmax) * (float) 8.0f;

	//Serial.print("Min:"); Serial.print(t_min);
	//Serial.print(", Max:"); Serial.print(t_max);
	//Serial.print(", Scale:"); Serial.print(t_scale);
	//Serial.print(", Tmax:"); Serial.print(tmax);
	//Serial.print(", tq:"); Serial.println(tq);

	lcd.setCursor(GRAPH_WIDTH, 0);
	PRINT_TEMP_FRACT(t);
	
	/** Graph ***********************************************************/

	bool graph_update = (millis() - timer_graph_beg) > (uint32_t) TIMER_GRAPH_UPDATE_MS;
	//uint32_t CUNT = millis() - timer_graph_beg;
	uint32_t FUCK = millis() - timer_graph_beg;

	Serial.print((uint32_t) ((uint32_t ) millis() - (uint32_t) timer_graph_beg));
	Serial.print(' ');
	Serial.print(FUCK);
	Serial.print(' ');
	Serial.println((uint32_t) TIMER_GRAPH_UPDATE_MS);

	if(FUCK > (uint32_t) TIMER_GRAPH_UPDATE_MS) {
		Serial.println("FUUUUUUUUUUUUUUUUUUUUUCK");
	} 

	// XXX No averaging
	if(graph_update) {
		Serial.println("Updating graph");

		// Insert temperatue into last character, rightmost pixel of the graph
		for(i = 0; i < 8; i++)
			if(i <= tq)
				graph[GRAPH_WIDTH - 1][7 - i] |= 0b00000001;
		lcd.createChar(GRAPH_WIDTH - 1, graph[GRAPH_WIDTH - 1]);

		// Copy last bit of next char to first bit of current char
		for(i = 1; i < GRAPH_WIDTH; i++) {
			for(y = 0; y < 8; y++)
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
	int threshold = THRESHOLD_BASE + (512 - analogRead(PIN_KNOB)) / 32;
	if(threshold < 0)
		threshold = 0;

	lcd.setCursor(12, 1);
	PRINT_TEMP(threshold);

	/** Relay Control (abstracted) **************************************/

	sample_update(t >= threshold);

	/** Graph ***********************************************************/
	if(graph_update) {

		// Shift all pixels on each character one step to the left
		for(i = 0; i < GRAPH_WIDTH; i++) {
			for(y = 0; y < 8; y++)
				graph[i][y] <<= 1;
			lcd.createChar(i, graph[i]);
		}

		timer_graph_beg = millis();
	}

}
