#include "esp32-hal.h"
// #include <driver/rmt.h>
#include <Arduino.h>
int camera_trigger_pin = 12; // camera trigger
int camera_strobe_pin = 14;  // camera strobe
int led_pin = 23;            // LED
gpio_num_t led_pin_gpio = GPIO_NUM_23;

// setting PWM properties
const int ledChannel = 0;
const int resolution = 4;

hw_timer_t *timer = NULL;

volatile uint32_t isrTriggerCounter = 0;
volatile uint32_t lastTriggerIsrAt = 0;
volatile SemaphoreHandle_t triggerSemaphore;
portMUX_TYPE triggerMux = portMUX_INITIALIZER_UNLOCKED;

volatile uint32_t isrTimerCounter = 0;
volatile uint32_t lastTimerIsrAt = 0;
volatile SemaphoreHandle_t timerSemaphore;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

rmt_obj_t *rmt_send = NULL;
rmt_data_t led_data[32];

// rmt_config_t config;
// rmt_item32_t items[1];

// void rmt_pulse()
// {

//     rmt_write_items(config.channel, items, 1, 0);
// }

void ARDUINO_ISR_ATTR onTimer()
{
    // Increment the counter and set the time of ISR
    portENTER_CRITICAL_ISR(&timerMux);
    isrTimerCounter++;
    lastTimerIsrAt = micros();
    portEXIT_CRITICAL_ISR(&timerMux);
    // Give a semaphore that we can check in the loop
    xSemaphoreGiveFromISR(timerSemaphore, NULL);
    digitalWrite(led_pin, LOW);
}

void ARDUINO_ISR_ATTR camera_trigger_isr()
{
    rmtWrite(rmt_send, led_data, 32);
    portENTER_CRITICAL_ISR(&triggerMux);
    isrTriggerCounter++;
    lastTriggerIsrAt = micros();
    portEXIT_CRITICAL_ISR(&triggerMux);
    // Give a semaphore that we can check in the loop
    xSemaphoreGiveFromISR(triggerSemaphore, NULL);
    // rmt_pulse();
    //  digitalWrite(led_pin, HIGH);
    //  timerAlarmEnable(timer);
}

void enable_pwm(double freq, int duty)
{
    int freq_out = ledcSetup(ledChannel, freq, resolution);
    Serial.print("Freq out: ");
    Serial.print(freq_out);
    Serial.println();
    ledcAttachPin(led_pin, ledChannel);
    ledcWrite(ledChannel, duty);
}

void disable_pwm()
{
    ledcDetachPin(led_pin);
}

void handleTrigger()
{

    if (xSemaphoreTake(triggerSemaphore, 0) == pdTRUE)
    {
        uint32_t isrCount = 0, isrTime = 0;
        // Read the interrupt count and time
        portENTER_CRITICAL(&triggerMux);
        isrCount = isrTriggerCounter;
        isrTime = lastTriggerIsrAt;
        portEXIT_CRITICAL(&triggerMux);
        uint32_t dm = micros() - isrTime;

        Serial.print("Camera triggered at ");
        Serial.print(isrTime);
        Serial.print(" loop latency ");
        Serial.print(dm);

        Serial.print(" total of ");
        Serial.print(isrCount);
        Serial.println();
    }
}

void handleTimer()
{

    if (xSemaphoreTake(timerSemaphore, 0) == pdTRUE)
    {
        uint32_t isrCount = 0, isrTime = 0;
        // Read the interrupt count and time
        portENTER_CRITICAL(&timerMux);
        isrCount = isrTimerCounter;
        isrTime = lastTimerIsrAt;
        portEXIT_CRITICAL(&timerMux);
        uint32_t dm = micros() - isrTime;

        Serial.print("Timer triggered at ");
        Serial.print(isrTime);
        Serial.print(" loop latency ");
        Serial.print(dm);

        Serial.print(" total of ");
        Serial.print(isrCount);

        Serial.println();
    }
}

void process(String s)
{
    if (s.length() > 0)
    {
        Serial.print("Process: ");
        Serial.println(s);

        char cmd = s[0];
        // Add PWM as alternative to strobe signal
        if (cmd == 'P')
        {
            String arg = s.substring(1);
            int idx = arg.indexOf(' ');
            double freq = arg.substring(0, idx).toDouble();
            int duty = arg.substring(idx).toInt();

            enable_pwm(freq, duty);
            Serial.print("PWM ");
            Serial.print(freq);
            Serial.print(" ");
            Serial.print(duty);
            Serial.println();
        }
        if (cmd == 'S')
        {
            int arg = s.substring(1).toInt();
            disable_pwm();
            digitalWrite(led_pin, HIGH);
            digitalWrite(camera_trigger_pin, HIGH);
            digitalWrite(camera_strobe_pin, HIGH);
            if (arg < 1000)
                delayMicroseconds(arg);
            else
                delay(arg / 1000);
            digitalWrite(led_pin, LOW);
            digitalWrite(camera_trigger_pin, LOW);
            digitalWrite(camera_strobe_pin, LOW);
            Serial.print("Pulse ");
            Serial.print(arg);
            Serial.println();
        }
        if (cmd == 'L')
        {
            int arg = s.substring(1).toInt();
            disable_pwm();
            if (arg == 0)
            {
                digitalWrite(led_pin, LOW);
            }
            else if (arg == 1)
            {
                digitalWrite(led_pin, HIGH);
            }

            Serial.print("Light state ");
            Serial.print(arg);
            Serial.println();
        }
        if (cmd == 'C')
        {
            int arg = s.substring(1).toInt();
            if (arg == 0)
            {
                digitalWrite(camera_trigger_pin, LOW);
            }
            else if (arg == 1)
            {
                digitalWrite(camera_trigger_pin, HIGH);
            }

            Serial.print("Camera state ");
            Serial.print(arg);
            Serial.println();
        }
        if (cmd == 'Q')
        {
            int arg = s.substring(1).toInt();
            digitalWrite(camera_trigger_pin, HIGH);
            if (arg < 1000)
                delayMicroseconds(arg);
            else
                delay(arg / 1000);
            digitalWrite(camera_trigger_pin, LOW);

            Serial.print("Camera strobe ");
            Serial.print(arg);
            Serial.println();
        }
        if (cmd == 'X')
        {
            String arg = s.substring(1);
            int idx = arg.indexOf(' ');
            int light = arg.substring(0, idx).toInt();
            int camera = arg.substring(idx).toInt();
            timerAlarmWrite(timer, light, false);
            Serial.print(" with light delay set to ");
            Serial.println(light);
            // disable_pwm();
            // digitalWrite(led_pin, LOW); // light off
            digitalWrite(camera_trigger_pin, LOW); // camera off
            // delayMicroseconds(0);
            digitalWrite(camera_trigger_pin, HIGH); // camera on
            delayMicroseconds(camera);
            // delayMicroseconds(camera);
            // digitalWrite(led_pin, HIGH); // light on
            // delayMicroseconds(light);
            // digitalWrite(led_pin, LOW); // light off
            digitalWrite(camera_trigger_pin, LOW); // camera off
            // delay(1);

            Serial.print("Sync flash and camera ");
            Serial.print(light);
            Serial.print(" ");
            Serial.print(camera);
            Serial.print(" ");
            Serial.print(micros());
            Serial.println();
        }
        if (cmd == 'Z')
        {
            // rmt_pulse();
        }
        if (cmd == 'A')
        {

            if ((rmt_send = rmtInit(led_pin_gpio, RMT_TX_MODE, RMT_MEM_64)) == NULL)
            {
                Serial.println("init sender failed\n");
            }

            float realTick = rmtSetTick(rmt_send, 3200);
            Serial.printf("real tick set to: %fns\n", realTick);

            rmtWrite(rmt_send, led_data, 32);
            rmtDeinit(rmt_send);
        }
    }
}

String line;

void handleSerial()
{

    while (Serial.available())
    {
        int c = Serial.read();
        Serial.print((char)c);
        if (c == '\n' || c == '\r')
        {
            process(line);
            line = "";
        }
        else
        {
            line += (char)c;
        }
    }
}

void setup()
{
    Serial.begin(115200);
    pinMode(camera_trigger_pin, OUTPUT);
    digitalWrite(camera_trigger_pin, LOW);
    pinMode(camera_strobe_pin, INPUT_PULLUP);
    attachInterrupt(camera_strobe_pin, camera_trigger_isr, FALLING);

    pinMode(led_pin, OUTPUT);
    digitalWrite(led_pin, LOW);

    // Create semaphore to inform us when the timer has fired
    triggerSemaphore = xSemaphoreCreateBinary();
    timerSemaphore = xSemaphoreCreateBinary();

    timer = timerBegin(0, 80, true);
    timerAttachInterrupt(timer, &onTimer, true);

    led_data[0].level0 = 1;
    led_data[0].duration0 = 32767;
    led_data[0].level1 = 0;
    led_data[0].duration1 = 1;

    
    if ((rmt_send = rmtInit(led_pin_gpio, RMT_TX_MODE, RMT_MEM_64)) == NULL)
    {
        Serial.println("init sender failed\n");
    }

    float realTick = rmtSetTick(rmt_send, 3200);
    Serial.printf("real tick set to: %fns\n", realTick);
    
    Serial.println("ready");
}

void loop()
{

    handleTrigger();
    handleTimer();
    handleSerial();
}
