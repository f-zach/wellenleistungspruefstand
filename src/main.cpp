#include <Arduino.h>
#include <i2c_t3.h>
#include <SPI.h>
#include <ADC.h>
#include <TCO.h>
#include <FRQ.h>


#define LED_ERR 5
#define LED_BUSY 6

ADCmodule ADC(0x49,0x48);
TCOmodule TCO(0x20);
FRQmodule FRQ(115200);

//Konstruktoren für alle Module

long t, tMeasurementStart;
int val, sensor_cs, i;
float frequnecy;
char command;
bool fault;

void setup()
{
  delay(3000);
  Serial.begin(9600);

  Serial.println("starting");
  pinMode(LED_ERR, OUTPUT);
  pinMode(LED_BUSY, OUTPUT);

  digitalWrite(LED_BUSY,HIGH);

  delay(5000);
  
  Wire.begin(I2C_MASTER, 0x00, I2C_PINS_18_19);

  SPI.begin();

  ADC.config(B00000001);
  Serial.println(1);
  TCO.config(B11111111, 1);

 digitalWrite(LED_BUSY,LOW);
}

void loop()
{
  //Lese Befehlssymbol
  if (Serial.available())
  {
    command = Serial.read();
  }

  //Switch zur ausführung der Befehle
  switch (command)
  {
  //Case 'r': Lese alle Module aus
  case 'r':
    digitalWriteFast(LED_BUSY, HIGH);

    ADC.readAll();

    TCO.readTempAll();

    FRQ.getFrequency(1);

    //Baue String für die Ausgabe über Serial auf
    for (i = 0; i < ADC.channelCount; i++)
    {
      Serial.print(ADC.Voltage[i],5);
      Serial.print("\t");
    }

    for (i = 0; i < TCO.sensorCount; i++)
    {
      Serial.print(TCO.TemperatureC[i]);
      Serial.print("\t");
    }

    Serial.print(FRQ.frequency1);
    Serial.print("\t");

    Serial.print(FRQ.frequency2);
    Serial.print("\t");

    Serial.println();

    command = 0;
    digitalWriteFast(LED_BUSY, LOW);
    break;

  }
}
