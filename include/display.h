#ifndef __DISPLAY_H__
#define __DISPLAY_H__

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

class Display {
public:
    Display();
    void display_message(const char *m, uint8_t line);
    void display_message(unsigned long v, uint8_t line);
    void display_message(long v, uint8_t line);
    void display_message(double v, uint8_t line);
    void update(double scale_value, int temperature_reading);
    void clear();

protected:
    Adafruit_SSD1306 *display;
    char buf[16];
};

#endif
