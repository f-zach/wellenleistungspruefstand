#include <Arduino.h>
#include <i2c_t3.h>
#include <SPI.h>
#include <MAIN.h>
#include <ADC.h>
#include <TCO.h>
#include <FRQ.h>


#define LED_ERR 5
#define LED_BUSY 6

MAINmodule MAIN(24,0x76);
ADCmodule ADC(0x49,0x48);
TCOmodule TCO(0x20);
FRQmodule FRQ(115200);

void readGoPowerBox();
//Konstruktoren für alle Module

long t, tMeasurementStart;
int val, sensor_cs, i;
float frequnecy, dataGoPowerBox[3];
char command;
bool fault;

void setup()
{
  Serial.begin(9600);
  Serial4.begin(9600);

  Serial.println("starting");
  pinMode(LED_ERR, OUTPUT);
  pinMode(LED_BUSY, OUTPUT);

  digitalWrite(LED_BUSY,HIGH);

  delay(5000);
  
  Wire.begin(I2C_MASTER, 0x00, I2C_PINS_18_19);

  SPI.begin();

  MAIN.config();
  
  ADC.config(B00000111);

  TCO.config(B01010100, 1);

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

    MAIN.startTmeasurement();

    ADC.readAll();

    TCO.readTempAll();

    FRQ.getFrequency(1);

    MAIN.readEnvP();
    MAIN.readEnvT();

    readGoPowerBox();

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

    for(int i = 0; i<3; i++)
    {
      Serial.print(dataGoPowerBox[i]);
      Serial.print("\t");
    }

    Serial.print(MAIN.envPressure);
    Serial.print("\t");

    Serial.print(MAIN.envTemperature);
    Serial.print("\t");

    Serial.println();

    command = 0;
    digitalWriteFast(LED_BUSY, LOW);
    break;

  }
}

void readGoPowerBox()
{
    if (Serial4.available()>10 && Serial4.available() < 24)
    {
      //Serial.println("Data Recieved");
        Serial4.read();
        for(int i = 0; i < 3; i++)
        {
          String recieveString = Serial4.readStringUntil('\t');
          dataGoPowerBox[i] = recieveString.toFloat();
        }
        
    } 
    else if(Serial4.available() >= 24)
    {
      //Serial.println(Serial4.available());
      //Serial.println("Nothing recieved");

      for(i = 0; i<64; i++)
      {
      Serial4.read();
      }

      dataGoPowerBox[2] = 1;
    }
    else
    {

    }    
}