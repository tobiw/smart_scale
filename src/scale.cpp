#include "scale.h"


Scale::Scale() {
    scale = new HX711;
}

int Scale::begin(int pin_dat, int pin_clk) {
    scale->begin(pin_dat, pin_clk);

    // It always takes a few retries at startup until the HX711 is ready
    int scale_retries = 0;
    while (!scale->is_ready()) {
        delay(100);
        scale_retries++;
        if (scale_retries > 8) {
            return -1;
        }
    }

#ifndef CALIBRATION_FACTOR
    calibration();
#else
    scale->set_scale(CALIBRATION_FACTOR);
#endif

    return 0;
}

void Scale::calibration() {
#if 0
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
#endif
}

void Scale::tare() {
    scale->tare();
}

double Scale::get() {
    return scale->get_units(AVG_SAMPLES);
}
