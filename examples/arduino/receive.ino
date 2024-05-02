#include <Wire.h>

void setup()
{
    Serial.begin(9600);

    Wire.begin(0x01);
    Wire.onReceive(receive);
}

void loop()
{
    delay(100);
}

void receive(int length)
{
    int command = Wire.read();

    Serial.print("[");
    Serial.print(command, 16);
    Serial.print("] ");

    if (command == 0x01)
    {
        byte b = Wire.read();

        Serial.print("0x");
        Serial.print(b, 16);
    }
    else if (command == 0x80)
    {
        for (int i = 1; i < length; i++)
        {
            char c = Wire.read();
            Serial.print(c);
        }
    }

    Serial.println();
}
