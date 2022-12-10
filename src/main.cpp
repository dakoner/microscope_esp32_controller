#include <Arduino.h>
int pin1 = 13; // camera trigger
int pin2 = 12; // LED

// setting PWM properties
const int ledChannel = 0;
const int resolution = 8;

void setup()
{
    Serial.begin(115200);
    pinMode(pin1, OUTPUT);
    pinMode(pin2, OUTPUT);
    digitalWrite(pin1, LOW);
    digitalWrite(pin2, LOW);

    Serial.println("ready");
}

void enable_pwm(double freq, int duty)
{
    int freq_out = ledcSetup(ledChannel, freq, resolution);
    Serial.print("Freq out: ");
    Serial.print(freq_out);
    Serial.println();
    ledcAttachPin(pin2, ledChannel);
    ledcWrite(ledChannel, duty);
}

void disable_pwm()
{
    ledcDetachPin(pin2);
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
            digitalWrite(pin2, HIGH);
            delay(arg.toInt());
            digitalWrite(pin2, LOW);
            Serial.print("Pulse ");
            Serial.print(arg.toInt());
            Serial.println();
        }
        if (cmd == 'C')
        {
            int arg = s.substring(1).toInt();
            if (arg == 0)
            {
                digitalWrite(pin1, HIGH);
            }
            else if (arg == 1)
            {
                digitalWrite(pin1, LOW);
            }

            Serial.print("Camera state ");
            Serial.print(arg);
            Serial.println();
        }
        if (cmd == 'Q')
        {
            int arg = s.substring(1).toInt();
            digitalWrite(pin1, HIGH);
            if (arg < 1000)
                delayMicroseconds(arg);
            else
                delay(arg / 1000);
            digitalWrite(pin1, LOW);

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

            digitalWrite(pin2, LOW); // light off
            digitalWrite(pin1, LOW); // camera off
            
            digitalWrite(pin1, HIGH); // camera on
            delayMicroseconds(20); // Minimum trigger delay
            digitalWrite(pin2, HIGH); // light on
            delayMicroseconds(light);



            digitalWrite(pin2, LOW); // light off
            delayMicroseconds(camera);
            digitalWrite(pin1, LOW); // camera off
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