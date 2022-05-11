#ifndef __SCALE_H__
#define __SCALE_H__

#include "HX711.h"

#define CALIBRATION_FACTOR 206  // remove/comment out to start calibration
#define AVG_SAMPLES 3  // 8 is more stable but measurements take longer

class Scale {
    public:
        Scale();
        int begin(int pin_dat, int pin_clk);
        void calibration();
        void tare();
        double get();

    private:
        HX711 *scale;
};

#endif
