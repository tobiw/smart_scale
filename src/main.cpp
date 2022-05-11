#include <Arduino.h>
#include <Wire.h>

#include <OneWire.h>
#include <DallasTemperature.h>
#include <PubSubClient.h>

#include <ESP8266WiFi.h>

#include "definitions.h"
#include "display.h"
#include "scale.h"

#define PIN_I2C_SDA 4
#define PIN_I2C_SCL 5

#define PIN_BTN_RESET 12
#define PIN_BTN_FUNCTION 13

#define PIN_DS18B20 13

#define TIMER_START_THRESHOLD 0.5
#define TIMER_STOP_THRESHOLD 0.1
#define TIMER_HYST_CYCLES 4  // threshold has to be exceeded for this many loop cycles
#define DEBOUNCE_DELAY 25

enum timer_mode_t timer_mode;
enum units_mode_t units_mode;

Scale scale;

OneWire onewire(PIN_DS18B20);
DallasTemperature ds18b20(&onewire);

const char* mqtt_user = "pi";
const char* mqtt_password = "00624583";
#define TOPIC_IOT_STATUS "iot/kitchen-scale"
#define TOPIC "home/kitchen-scale/temperature"

void mqtt_cb(char* topic, byte* payload, unsigned int length) {}

WiFiClient client;
PubSubClient mqtt_client(client);

volatile unsigned long timer_count = 0;
bool timer_running = false;
bool timer_has_run = false;
int debug_display = 0;
volatile int isr1_counter = 0, isr2_counter = 0;
unsigned int delta_below_threshold = 0;
bool use_temperature_sensor = false;

Display *display;

void publish_mqtt_status(unsigned int ds18b20_reading) {
  char buf[64], buf_ip[16];
  unsigned long ip_int = (unsigned long)WiFi.localIP();
  sprintf(buf_ip, "%lu.%lu.%lu.%lu", (ip_int & 0xf000) >> 24, (ip_int & 0xf00) >> 16, (ip_int & 0xf0) >> 8, (ip_int & 0xf));
  sprintf(buf, "{ \"running\": 1, \"ip\": %s, \"signal\": %d }", buf_ip, WiFi.RSSI());

#ifdef TOPIC_IOT_STATUS
  mqtt_client.publish(TOPIC_IOT_STATUS, buf);
#endif
  sprintf(buf, "%d", ds18b20_reading);
  mqtt_client.publish(TOPIC, buf);
}


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

    display = new Display;
    display->clear();
    display->display_message("Starting ...", 0);

    Serial.println("Display ready");

    pinMode(PIN_BTN_RESET, INPUT_PULLUP);
    //pinMode(PIN_BTN_FUNCTION, INPUT_PULLUP);

    //attachInterrupt(digitalPinToInterrupt(PIN_BTN_RESET), isr_btn_reset, CHANGE);
    //attachInterrupt(digitalPinToInterrupt(PIN_BTN_FUNCTION), isr_btn_function, CHANGE);

    if (scale.begin(12, 14) < 0) { // DAT, CLK
        Serial.println("Failed to init scale.");
        display->clear();
        display->display_message("SCALE ERROR", 0);
        while(1) {}
    }

    Serial.println("Scale started");
    delay(1000);
    Serial.println("Taring scale ready for measurements");
    scale.tare();
    delay(100);

    // Setting up timer ISR
    //timer1_attachInterrupt(isr_timer);
    //timer1_enable(TIM_DIV256, TIM_EDGE, TIM_LOOP); // 312,500Hz ticks
    //timer1_write(31250); // 0.1s

    ds18b20.begin();
    use_temperature_sensor = ds18b20.getDeviceCount() > 0;
    Serial.print("Found DS18B20: ");
    Serial.print(use_temperature_sensor, DEC);
    Serial.print(" - temperature: ");
    ds18b20.requestTemperatures();
    delay(1);
    Serial.println(ds18b20.getTempCByIndex(0));

    // Only connect to WiFi if the temperature probe is plugged in
    // (scale-only mode doesn't need to be accessible remotely)
    if (use_temperature_sensor) {
        display->clear();
        display->display_message("Connecting WiFi ...", 0);

        WiFi.begin(ssid, password);

        while (WiFi.status() != WL_CONNECTED) {
          delay(500);
        }

        Serial.println("WiFi connected");

        byte retry = 0;
        int r;
        mqtt_client.setCallback(mqtt_cb);
        while (!(r = mqtt_client.connect("car-battery", mqtt_user, mqtt_password)) && retry++ < 2);
        Serial.println("MQTT connected");
        display->display_message("MQTT connected", 1);
        delay(500);
    } else {
        WiFi.mode(WIFI_OFF);
        WiFi.forceSleepBegin();
    }
}

void loop() {
    static double last_v = 0, v = 0;
    static double last_delta = 0;
    static int ds18b20_reading = -255;
    static unsigned long last_temperature_reading = 0;

    const unsigned long m = millis();

    // Buttons
    if (digitalRead(PIN_BTN_RESET) == LOW) {
        scale.tare();
        timer_mode = TIMER_AUTO;
        timer_count = 0;
        timer_running = false;
        timer_has_run = false;
    }

    // Get temperature
    if (ds18b20.getDeviceCount() > 0) { // found a temperature sensor
        if ((m - last_temperature_reading) > 10000) {
            ds18b20.requestTemperatures();
            delay(1);
            ds18b20_reading = (int)ds18b20.getTempCByIndex(0);
            if (ds18b20_reading != -255) {
                publish_mqtt_status(ds18b20_reading);
            }
            last_temperature_reading = m;
        }
    }

    // Get scale
    v = scale.get() / 10.0;
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

    // Debug output and display updates
    //Serial.println(v, 3);
    display->update(v, ds18b20_reading);
}

