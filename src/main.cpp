#include <Arduino.h>
#include <i2c_t3.h>
#include <SPI.h>
#include <MAIN.h>
#include <ADC.h>
#include <TCO.h>
#include <FRQ.h>
#include <PRS.h>

#define LED_ERR 5
#define LED_BUSY 6

MAINmodule MAIN(24, 0x76);
ADCmodule ADC(0x49, 0x48);
TCOmodule TCO(0x20);
FRQmodule FRQ(115200);
PRSmodule PRS(0x70);

void readGoPowerBox();
// Konstruktoren für alle Module

long t, tMeasurementStart, rpmBrake, tLastSentCont = 0, tLastSentPC = 0;
int val, sensor_cs, i;
float frequnecy, torque, power;
char command;
bool fault;

float pressRanges[8]={150,150,150,150,0,0,0,0};
int unitSensor[8] = {MILLIBAR,MILLIBAR,MILLIBAR,MILLIBAR,0,0,0,0};
int unitReq[8] = {MILLIBAR,MILLIBAR,MILLIBAR,MILLIBAR,0,0,0,0};
byte sensorI2Caddress [8] = {0x28, 0x28, 0x28, 0x28, 0, 0, 0, 0};



void setup()
{
  Serial.begin(9600);
  Serial4.begin(9600);
  Serial5.begin(115200);

  Serial.println("starting");
  pinMode(LED_ERR, OUTPUT);
  pinMode(LED_BUSY, OUTPUT);

  digitalWrite(LED_BUSY, HIGH);

  delay(5000);

  Wire.begin(I2C_MASTER, 0x00, I2C_PINS_18_19);

  SPI.begin();

  MAIN.config();

  ADC.config(B00000111);

  TCO.config(B01111111, 1);

  PRS.config(B00001111, pressRanges, B00001111, unitSensor, unitReq, sensorI2Caddress);

  digitalWrite(LED_BUSY, LOW);
}

void loop()
{
  digitalWriteFast(LED_BUSY, HIGH);

  MAIN.startTmeasurement();


  ADC.readAll();

  TCO.readTempAll();

  FRQ.getFrequency(1);

  PRS.readPressAll();

  MAIN.readEnvP();
  MAIN.readEnvT();

  readGoPowerBox();
  // Lese Befehlssymbol
  if (Serial.available())
  {
    command = Serial.read();
  }

  if (millis() - tLastSentCont > 100)
  {
    Serial5.print("*r" + String(FRQ.frequency1) + ";s" + String(rpmBrake) + ";p" + String(power) + ";#");
    // Serial.println("*r" + String(FRQ.frequency1) + ";s" + String(dataGoPowerBox[0]) + ";p" + String(dataGoPowerBox[2]) + ";#");
    tLastSentCont = millis();
  }

  // Switch zur ausführung der Befehle
  // switch (command)
  // {
  // // Case 'r': Lese alle Module aus
  // case 'r':
  //   // Baue String für die Ausgabe über Serial auf
  //   for (i = 0; i < ADC.channelCount; i++)
  //   {
  //     Serial.print(ADC.Voltage[i], 5);
  //     Serial.print("\t");
  //   }

  //   for (i = 0; i < TCO.sensorCount; i++)
  //   {
  //     Serial.print(TCO.TemperatureC[i]);
  //     Serial.print("\t");
  //   }

  //   Serial.print(FRQ.frequency1);
  //   Serial.print("\t");

  //   // Serial.print(FRQ.frequency2);
  //   // Serial.print("\t");

  //   Serial.print(rpmBrake);
  //   Serial.print("\t");

  //   Serial.print(torque);
  //   Serial.print("\t");

  //   Serial.print(power);
  //   Serial.print("\t");

  //   Serial.print(MAIN.envPressure);
  //   Serial.print("\t");

  //   Serial.print(MAIN.envTemperature);
  //   Serial.print("\t");

  //   Serial.println();

  //   command = 0;
  //   digitalWriteFast(LED_BUSY, LOW);
  //   break;
  // }

  if (millis() - tLastSentPC >= 100)
  {
    // Baue String für die Ausgabe über Serial auf
    for (i = 0; i < ADC.channelCount; i++)
    {
      Serial.print(ADC.Voltage[i], 5);
      Serial.print("\t");
    }

    for (i = 0; i < TCO.sensorCount; i++)
    {
      Serial.print(TCO.TemperatureC[i]);
      Serial.print("\t");
    }

    Serial.print(FRQ.frequency1 * 60);
    Serial.print("\t");

    // Serial.print(FRQ.frequency2);
    // Serial.print("\t");

    Serial.print(rpmBrake);
    Serial.print("\t");

    Serial.print(torque);
    Serial.print("\t");

    Serial.print(power);
    Serial.print("\t");

    for (size_t i = 0; i < PRS.SensorCount; i++)
    {
      Serial.print(PRS.Pressure[i]);
      Serial.print("\t");
    }
    

    Serial.print(MAIN.envPressure);
    Serial.print("\t");

    Serial.print(MAIN.envTemperature);
    Serial.print("\t");

    Serial.println();

    command = 0;

    tLastSentPC = millis();

    digitalWriteFast(LED_BUSY, LOW);
  }
}

void readGoPowerBox()
{
  if (Serial4.available() > 10 && Serial4.available() < 24)
  {
    // Serial.println("Data Recieved");
    for (int i = 0; i < 3; i++)
    {
      Serial4.read();
    }
    rpmBrake = Serial4.readStringUntil('\t').toInt();
    torque = Serial4.readStringUntil('\t').toFloat();
    power = Serial4.readStringUntil('\t').toFloat();

    // Serial.println(String(rpmBrake) + "\t" + String(torque) + "\t" + String(power));
  }
  else if (Serial4.available() >= 24)
  {
    // Serial.println(Serial4.available());
    // Serial.println("Nothing recieved");

    for (i = 0; i < 64; i++)
    {
      Serial4.read();
    }
  }
  else
  {
  }
}