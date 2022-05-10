#include <Arduino.h>
#include "HX711.h"
#include <Wire.h>

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <OneWire.h>
#include <DallasTemperature.h>

#include <ESP8266WiFi.h>

#define PIN_I2C_SDA 4
#define PIN_I2C_SCL 5

#define PIN_BTN_RESET 12
#define PIN_BTN_FUNCTION 13

#define PIN_DS18B20 13

#define TIMER_START_THRESHOLD 0.5
#define TIMER_STOP_THRESHOLD 0.1
#define TIMER_HYST_CYCLES 4  // threshold has to be exceeded for this many loop cycles
#define DEBOUNCE_DELAY 25

#define CALIBRATION_FACTOR 206  // remove/comment out to start calibration
#define AVG_SAMPLES 3  // 8 is more stable but measurements take longer
HX711 scale;

OneWire onewire(PIN_DS18B20);
DallasTemperature ds18b20(&onewire);

volatile unsigned long timer_count = 0;
bool timer_running = false;
bool timer_has_run = false;
int debug_display = 0;
volatile int isr1_counter = 0, isr2_counter = 0;
unsigned int delta_below_threshold = 0;

enum {
    TIMER_AUTO = 0,
    TIMER_MANUAL
} timer_mode;

enum {
    UNITS_METRIC = 0,
    UNITS_IMPERIAL
} units_mode;

class Display {
public:
    Display();
    void display_message(const char *m, uint8_t line);
    void display_message(unsigned long v, uint8_t line);
    void display_message(long v, uint8_t line);
    void display_message(double v, uint8_t line);
    void update(double v);
    void clear();

protected:
    Adafruit_SSD1306 *display;
    char buf[16];
};

Display::Display() {
    display = new Adafruit_SSD1306(128, 32, &Wire, -1);
    Serial.println("SSD1306 created");

    display->begin(SSD1306_SWITCHCAPVCC, 0x3C);
    display->setRotation(2);
    Serial.println("SSD1306 initialised");
    /*if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
        Serial.println("Failed to communicate with display at 0x3C");
        while (1) {}
    }*/

    display->clearDisplay();
    display->setTextSize(1);
    display->setTextColor(WHITE);
    display->setCursor(0, 0);
    display->display();
    Serial.println("Cleared display");
}

void Display::display_message(const char *m, uint8_t line) {
    //display->clearDisplay();
    display->setCursor(0, line * 18);
    display->write(m);
    display->display();
}

void Display::display_message(unsigned long v, uint8_t line) {
    sprintf(buf, "%lu", v);
    display_message(buf, line);
}

void Display::display_message(long v, uint8_t line) {
    sprintf(buf, "%ld", v);
    display_message(buf, line);
}

void Display::display_message(double v, uint8_t line) {
    dtostrf(v, 6, 1, buf);
    display_message(buf, line);
}

void Display::clear() {
    display->clearDisplay();
}

void Display::update(double v) {
    display->setTextSize(2);
    clear();
    display->setCursor(0, 0);

    // Weight (convert to imperial if required)
    if (units_mode == UNITS_IMPERIAL)
        v *= 0.03527396;

    dtostrf(v, 6, 1, buf);
    display->print(buf);

    if (units_mode == UNITS_IMPERIAL)
        display->print("oz");

    // Timer
    display->setCursor(2, 18);
    display->setTextSize(2);
    sprintf(buf, "%s %02u:%02u %u", timer_running ? ">" : " ", (unsigned int)(timer_count / 60.0), (unsigned int)(timer_count % 60), delta_below_threshold);
    display->print(buf);
    display->display();
}

Display *display;

#if 0
void calibration() {
    menu_display->clear();
    menu_display->display_message("Clear scale", 0);
    delay(2000);
    scale.set_scale();
    scale.tare();
    Serial.println("Scale tared.");
    Serial.println("Place known weight onto scale!");
    menu_display->clear();
    menu_display->display_message("Place weight", 0);
    delay(2000);
    unsigned long known_value = scale.get_units(10);
    Serial.print("Known weight raw value is ");
    Serial.println(known_value);
    Serial.print("For 12g the SCALE value is ");
    const unsigned long sc = (unsigned long)(known_value / 120.0);
    Serial.println(sc);
    menu_display->clear();
    menu_display->display_message(sc, 0);
    scale.set_scale(sc);
    //scale.set_offset(0);
    delay(1000);
}
#endif

volatile unsigned long isr_btn_function_last_m = 0;

#if 0
void isr_btn_reset() {
    delay(DEBOUNCE_DELAY); // debounce
    isr1_counter++;
    if (isr1_counter == 1) return;

    if (digitalRead(PIN_BTN_RESET) == LOW) {
        scale.tare();
        timer_mode = TIMER_AUTO;
        timer_count = 0;
        timer_running = false;
        timer_has_run = false;
    }
}

void isr_btn_function() {
    delay(DEBOUNCE_DELAY); // debounce
    isr2_counter++;
    if (isr2_counter == 1) return;

    if (digitalRead(PIN_BTN_FUNCTION) == LOW) {
        isr_btn_function_last_m = millis();
    } else {
        const unsigned long m = millis();
        if (m - isr_btn_function_last_m < 1000) { // released within less than 1 second
            timer_mode = TIMER_MANUAL;
            debug_display = (int)timer_running;
            timer_running = !timer_running;
        } else { // released after more than 1 second
            units_mode = units_mode == UNITS_METRIC ? UNITS_IMPERIAL : UNITS_METRIC;
        }
    }
}

void IRAM_ATTR isr_timer() {
    if (timer_running)
        timer_count++;
}
#endif

void setup() {
    Serial.begin(115200);
    delay(10);
    Serial.println("Serial ready");

    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);

    // Power save settings
    // ... TODO

    WiFi.mode(WIFI_OFF);
    WiFi.forceSleepBegin();

    display = new Display;
    display->clear();
    display->display_message("Starting ...", 0);

    Serial.println("Display ready");

    //pinMode(PIN_BTN_RESET, INPUT_PULLUP);
    //pinMode(PIN_BTN_FUNCTION, INPUT_PULLUP);

    //attachInterrupt(digitalPinToInterrupt(PIN_BTN_RESET), isr_btn_reset, CHANGE);
    //attachInterrupt(digitalPinToInterrupt(PIN_BTN_FUNCTION), isr_btn_function, CHANGE);

    scale.begin(12, 14); // DAT, CLK

    Serial.println("Scale started");

    delay(10);
    int scale_retries = 0;
    while (!scale.is_ready()) {
        Serial.println("Failed to init scale.");
        delay(100);
        display->clear();
        scale_retries++;
        if (scale_retries > 8) {
            display->display_message("SCALE ERROR", 0);
            while(1) {}
        }
    }

#ifndef CALIBRATION_FACTOR
    calibration();
#else
    scale.set_scale(CALIBRATION_FACTOR);
#endif

    delay(1000);
    Serial.println("Taring scale ready for measurements");
    scale.tare();
    delay(100);

    // Setting up timer ISR
    //timer1_attachInterrupt(isr_timer);
    //timer1_enable(TIM_DIV256, TIM_EDGE, TIM_LOOP); // 312,500Hz ticks
    //timer1_write(31250); // 0.1s

    ds18b20.begin();
    Serial.print("Found DS18B20: ");
    Serial.print(ds18b20.getDeviceCount(), DEC);
    Serial.print(" - temperature: ");
    ds18b20.requestTemperatures();
    delay(10);
    Serial.println(ds18b20.getTempCByIndex(0));
}

void loop() {
    static double last_v = 0, v = 0;
    static double last_delta = 0;

    v = scale.get_units(AVG_SAMPLES) / 10.0;
    //if (v < 0) v = 0;
    if (v < 0.0 && v > -0.2) v = 0;
    if (v > 1000) v = 9999;

    double delta = v - last_v;
    //if (delta < 0) delta *= -1.0;
    Serial.print("delta: ");
    Serial.println(delta);

    // Start timer if weight registered (only once after reset)
    if (timer_mode == TIMER_AUTO) {
        if (!timer_has_run && !timer_running) {
            // Need two consecutive measurements over the threshold to start timer
            if (v > TIMER_START_THRESHOLD && last_v > TIMER_START_THRESHOLD) {
                timer_running = true;
            }
        }

        if (timer_running) {
            // No (more) weight change: stop timer (two consecutive deltas below the threshold)
            if (delta < TIMER_STOP_THRESHOLD && last_delta < TIMER_STOP_THRESHOLD) {
                if (delta_below_threshold++ > TIMER_HYST_CYCLES) {
                    timer_running = false;
                    timer_has_run = true;
                }
            } else {
                delta_below_threshold = 0;
            }
        }
    }

    last_v = v;
    last_delta = delta;

    //Serial.println(v, 3);
    display->update(v);
}

