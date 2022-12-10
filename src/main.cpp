#include <Arduino.h>
int camera_pin = 13; // camera trigger
int led_pin = 12; // LED

// setting PWM properties
const int ledChannel = 0;
const int resolution = 4;

void setup()
{
    Serial.begin(115200);
    pinMode(camera_pin, OUTPUT);
    pinMode(led_pin, OUTPUT);
    digitalWrite(camera_pin, LOW);
    digitalWrite(led_pin, LOW);

    Serial.println("ready");
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

String line;
void process(String s)
{
    Serial.print("Process: ");
    Serial.println(s);
    if (s.length() > 0)
    {
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
            String arg = s.substring(1);
            disable_pwm();
            digitalWrite(led_pin, HIGH);
            delay(arg.toInt());
            digitalWrite(led_pin, LOW);
            Serial.print("Pulse ");
            Serial.print(arg.toInt());
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
                digitalWrite(camera_pin, LOW);
            }
            else if (arg == 1)
            {
                digitalWrite(camera_pin, HIGH);
            }

            Serial.print("Camera state ");
            Serial.print(arg);
            Serial.println();
        }
        if (cmd == 'Q')
        {
            int arg = s.substring(1).toInt();
            digitalWrite(camera_pin, HIGH);
            if (arg < 1000)
                delayMicroseconds(arg);
            else
                delay(arg / 1000);
            digitalWrite(camera_pin, LOW);

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

            digitalWrite(led_pin, LOW); // light off
            digitalWrite(camera_pin, LOW); // camera off
            
            digitalWrite(camera_pin, HIGH); // camera on
            delayMicroseconds(20); // Minimum trigger delay
            digitalWrite(led_pin, HIGH); // light on
            delayMicroseconds(light);



            digitalWrite(led_pin, LOW); // light off
            delayMicroseconds(camera);
            digitalWrite(camera_pin, LOW); // camera off
            //delay(1);

            Serial.print("Sync flash and camera ");
            Serial.print(light);
            Serial.print(" ");
            Serial.print(camera);
            Serial.println();
        }
    }
}

void loop()
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