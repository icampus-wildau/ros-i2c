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
    byte command = Wire.read();

    Serial.print("[0x");
    Serial.print(command, 16);
    Serial.print("; ");
    Serial.print(length - 1);
    Serial.print("] ");

    if (command == 0x01)
    {
        byte b = Wire.read();

        Serial.print("0x");
        Serial.print(b, 16);
    }
    else if (command == 0x02)
    {
        byte l = Wire.read();
        byte h = Wire.read();

        int w = (h << 8) | l;

        Serial.print("0x");
        Serial.print(w, 16);
    }
    else if (command == 0x80)
    {
        for (int i = 1; i < length; i++)
        {
            char c = Wire.read();
            Serial.print(c);
        }
    }

    if (Wire.available())
    {
        Serial.print(" + ");

        // Discard unused bytes.
        while (Wire.available())
        {
            char c = Wire.read();
            Serial.print(c);
        }
    }

    Serial.println();
}
