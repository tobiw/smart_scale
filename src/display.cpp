#include <Arduino.h>
#include "definitions.h"
#include "display.h"

extern volatile unsigned long timer_count;
extern bool timer_running;
extern unsigned int delta_below_threshold;
extern enum units_mode_t units_mode;

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
