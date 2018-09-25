/*
 * Listado de Sensores:
 * - ENTRADA DEMUX - (S0,S1,S2):  NOMBRE   :   Descripcion   :   Tipo
 *
 * DEMUX1:
 * - X0 - (0,0,0):  L_W   :   Nivel de Agua           :   4-20mA
 * - X1 - (1,0,0):  L_G   :   Nivel de Aceite         :   4-20mA
 * - X2 - (0,1,0):  T_W   :   Temperatura de Agua     :   NTC
 * - X3 - (1,1,0):  L_O   :   Nivel de Aceite         :   4-20mA
 * - X4 - (0,0,1):  P_T2  :   Presion de Turbo 2      :   4-20mA
 * - X5 - (1,0,1):  P_O   :   Presion de Aceite       :   4-20mA
 * - X6 - (0,1,1):  P_W   :   Presion de Agua         :   4-20mA
 * - X7 - (1,1,1):  P_T1  :   Presion de Turbo 1      :   4-20mA
 *
 * DEMUX2:
 * - X0 - (0,0,0):  AMP1  :   Amperimetro 1           :   0-5V
 * - X1 - (1,0,0):  S_2   :   Entrada no asignada
 * - X2 - (0,1,0):  s_1   :   Entrada no asignada
 * - X3 - (1,1,0):  AMP3  :   Amperimetro 2           :   0-5V
 * - X4 - (0,0,1):  VOLT  :   Voltimetro              :   0-5V
 * - X5 - (1,0,1):  T_O   :   Temperatura de Aceite   :   NTC
 * - X6 - (0,1,1):  AMP4  :   Amperimetro 4           :   0-5V
 * - X7 - (1,1,1):  AMP2  :   Amperimetro 2           :   0-5V
 */

/*
 * Comunicacion Serie:
 * Serial 0   :   TX (1)   :  RX (0)     :   Comunicacion Embarcado/ Salida USB
 * Serial 1   :   TX (18)  :  RX (19)    :   Comunicacion con FlowEZO
 * Serial 2   :   TX (16)  :  RX (17)    :   No asignado   /   Salida RX2/TX2
 * Serial 3   :   TX (14)  :  RX (15)    :   No asignado   /   Salida RX1/TX1
 * Serial 11  :   TX (3)   :  RX (2)     :   No asignado   /   Salida RX3/TX3   /   SIMULADO
 */

//--------  COMUNICACION I2C  ------------------------------------
#include <Wire.h>
//----------------------------------------------------------------

//----------  INICIALIZACION SONDA K  ----------------------------
#include <SPI.h>
#include "Adafruit_MAX31855.h"
int thermoCLK = 52;
int thermoCS = 53;
int thermoDO = 50;
double thermoAux = 0;
Adafruit_MAX31855 thermocouple(thermoCLK, thermoCS, thermoDO);
//----------------------------------------------------------------

//----------  INICIALIZACION RTC  --------------------------------
#include <DS3232RTC.h>
//#include <WireRtcLib.h>
#include <Time.h>
//----------------------------------------------------------------

//-------------INICIALIZACION ADC  -------------------------------
#include <Adafruit_ADS1015.h>
Adafruit_ADS1115 ads;
float refVolt = 4.97;
//----------------------------------------------------------------

//-------------INICIALIZACION LEDS  -------------------------------
#include "Adafruit_LEDBackpack.h"
#include "Adafruit_GFX.h"
Adafruit_24bargraph bar = Adafruit_24bargraph();
boolean estLeds = 1;
boolean ledCiclo = 1;
float WLult;    //Ultima lectura de Nivel de Agua
float OLult;    //Ultima lectura de Nivel de Aceite
float FLult;    //Ultima lectura de Nivel de Combustible
//----------------------------------------------------------------

//-------------  INICIALIZACION NTC  -----------------------------
#define THERMISTORNOMINAL 2       //Resistencia NTC a temperatura nominal
#define TEMPERATURENOMINAL 25     //Temperatura Nominal
#define NUMSAMPLES 1              //Numero de lecturas (5)
#define BCOEFFICIENT 3420         //Coeficiente NTC
#define SERIESRESISTOR 1          //Resistencia en serie
#define VOLTAGEREF 4.98           //Tension de referencia
#define THERMISTORPIN A0
//----------------------------------------------------------------

//--------------  INICIALIZACION PRESION  ------------------------
#define PRESSRES 200
//----------------------------------------------------------------

//--------------  INICIALIZACION ACELEROMETRO  -------------------
#include <Adafruit_MMA8451.h>
#include <Adafruit_Sensor.h>
Adafruit_MMA8451 mma = Adafruit_MMA8451();
//----------------------------------------------------------------

//-------- INICIALIAZACION COMUNICACIONES SERIE  -----------------

//#include <AltSoftSerial.h>          //Include the software serial library
//AltSoftSerial altSerial;            //Name the software serial library altSerial (this cannot be omitted)
//int SerialData0 = 29;                         //Arduino pin 7 to control pin S0
//int SerialData1 = 31;                         //Arduino pin 6 to control pin S1
int enable_1 = 27;             //Arduino pin 5 to  control pin E on board 1
char computerdata[20];               //A 20 byte character array to hold incoming data from a pc/mac/other
char sensordata[30];                 //A 30 byte character array to hold incoming data from the sensors
byte computer_bytes_received = 0;    //We need to know how many characters bytes have been received
byte sensor_bytes_received = 0;      //We need to know how many characters bytes have been received

char *channel;                       //Char pointer used in string parsing
char *cmd;                           //Char pointer used in string parsing

String inputstring = "";                                                       //a string to hold incoming data from the PC
String sensorstring = "";
boolean input_stringcomplete = false;                                          //have we received all the data from the PC
boolean sensor_stringcomplete = false;
String inputstring1 = "";                                                       //a string to hold incoming data from the PC
String sensorstring1 = "";
boolean input_stringcomplete1 = false;                                          //have we received all the data from the PC
boolean sensor_stringcomplete1 = false;
String inputstring2 = "";                                                       //a string to hold incoming data from the PC
String sensorstring2 = "";
boolean input_stringcomplete2 = false;                                          //have we received all the data from the PC
boolean sensor_stringcomplete2 = false;
String inputstring3 = "";                                                       //a string to hold incoming data from the PC
String sensorstring3 = "";
boolean input_stringcomplete3 = false;                                          //have we received all the data from the PC
boolean sensor_stringcomplete3 = false;

boolean rpmOK = false;
boolean caudalOK=false;

String Fuel_String;       //Texto recibido desde FlowEZO. Formato: 0.00,0.00
//----------------------------------------------------------------

//----------  INICIALIZACION MULTIPLEXORES 4851  -----------------
// MUX 1
int s0 = 11;
int s1 = 12;
int s2 = 13;
// MUX 2
int s4 = 4;
int s5 = 5;
int s6 = 6;
//----------------------------------------------------------------

//--------------  VARIABLES SENSORES  ----------------------------
float WT;             //Temperatura Agua
float WTemp;
float WL;             //Nivel Agua
float OT;             //Temperatura Aceite
float OTemp;
float OP;             //Presion Aceite
float OL;             //Nivel Aceite
float TP1;            //Presion Turbo 1
float TP2;            //Presion Turbo 2
double KT;            //Temperatura Gases de Escape
float FL;             //Nivel de Combustible
float FIN;            //Caudal +
float FOUT;           //Caudal -
float FuelConsumo;
float FSUM;           //Caudal Instantaneo
//float DieselInst;
float FACUM;          //Caudal de combustible Acumulado
float FP;             //Presion de Combustible
int RPM;              //RPM Motor
String RPM2;
float GenV;           //Voltimetro
float Gear1A;         //Amperimetro 1
float Gear2A;         //Amperimetro 2
float Gear3A;         //Amperimetro 3
float Gear4A;         //Amperimetro 4
byte EX;
byte EY;
byte EZ;
byte EW;
byte LX;
byte LY;
byte LZ;
byte LW;

float telX;
float telY;
float telZ;


float TTemp;          //Temperatura Telemetria
//----------------------------------------------------------------


//---Variables auxiliares Acelerometro Telemetria---
float ejeX = 0.00;
float ejeY = 0.00;
float ejeZ = 0.00;
float maxX = 0.00;
float maxY = 0.00;
float maxZ = 0.00;
//-------------------------------------------------


char input[40];  // puffer für die Daten
char current;     // puffer für empfangenes Zeichen
int incount = 0;  // Zähler zum schreiben in den Puffer
bool lineComplete = false; //


const int numReadings = 100;

int16_t readings[numReadings];      // the readings from the analog input
int readIndex = 0;              // the index of the current reading
int total = 0;                  // the running total
int16_t average = 0;                // the average

//int inputPin = A0;


//----------------Filtrado Sensores de Nivel---------------------
float WLevel[10];
int WLaux = 10;
int WLaverage = 0;
float FLevel[10];
int FLaux = 10;
int FLaverage = 0;
float OLevel[10];
int OLaux = 10;
int OLaverage = 0;
//---------------------------------------------------------------


long previousMillis = 0;
long intervalo = 1000;

boolean auxRPM =false;

void setup() {

  //Alimentacion Multiplexores
  pinMode(7, OUTPUT);
  digitalWrite(7, HIGH);


  for (int thisReading = 0; thisReading < numReadings; thisReading++) {
    readings[thisReading] = 0;
  }

  //----------  LEDS  -----------------------------------------
  bar.begin(0x70);  // pass in the address
  for (uint8_t b = 0; b < 24; b++ ) {
    bar.setBar(b, LED_OFF);
  }
  bar.writeDisplay();
  //-----------------------------------------------------------

  //########################################################################################################################
  //########              BEGIN SETUP
  //########################################################################################################################


  // put your setup code here, to run once:

  //########################################################################################################################
  //########              BEGIN COMMUNICATIONS
  //########################################################################################################################
  Serial.begin(115200);
  Serial3.begin(9600);  //RPM
 Serial2.begin(9600);
  //Serial1.begin(115200);  //FLOW
  //########################################################################################################################
  //########              END COMMUNICATIONS
  //########################################################################################################################

  //########################################################################################################################
  //########              BEGIN SERIAL MUX
  //########################################################################################################################

  inputstring.reserve(5);                                                   //set aside some bytes for receiving data from the PC
  sensorstring.reserve(30);                                                 //set aside some bytes for receiving data from Atlas Scientific product
  // inputstring1.reserve(5);                                                   //set aside some bytes for receiving data from the PC
  //sensorstring1.reserve(30);                                                 //set aside some bytes for receiving data from Atlas Scientific product
  inputstring2.reserve(5);
  sensorstring2.reserve(30);
  inputstring3.reserve(5);
  sensorstring3.reserve(30);
  Fuel_String.reserve(30);
  //########################################################################################################################
  //########              END SERIAL MUX
  //########################################################################################################################


  //########################################################################################################################
  //########              BEGIN SONDA K
  //########################################################################################################################
#ifndef ESP8266
  while (!Serial);     // will pause Zero, Leonardo, etc until serial console opens
#endif
  //Serial.println("MAX31855 test");
  // //delay(500);
  //########################################################################################################################
  //########              END SONDA K
  //########################################################################################################################

  //########################################################################################################################
  //########              BEGIN RTC
  //########################################################################################################################
  setSyncProvider(RTC.get);
  //Aqui debemos de poner un código para comparar la hora desde un NTP y si hay variación, ajustar fecha y hora
  //además si podemos debemos de comprobar la tensión de la batería del RTC, para asegurarnos que todo funciona.

  //########################################################################################################################
  //########              END RTC
  //########################################################################################################################

  //########################################################################################################################
  //########              BEGIN ACCEL
  //########################################################################################################################
  mma.begin();
  mma.setRange(MMA8451_RANGE_8_G);
  //########################################################################################################################
  //########              END ACCEL
  //########################################################################################################################


  //--------  MULTIPLEXORES  --------------------------------------
  pinMode(s0, OUTPUT);
  pinMode(s1, OUTPUT);
  pinMode(s2, OUTPUT);
  pinMode(s4, OUTPUT);
  pinMode(s5, OUTPUT);
  pinMode(s6, OUTPUT);

  digitalWrite(s0, LOW);
  digitalWrite(s1, LOW);
  digitalWrite(s2, LOW);
  digitalWrite(s4, LOW);
  digitalWrite(s5, LOW);
  digitalWrite(s6, LOW);
  //----------------------------------------------------------------

  //--------  ADC  -------------------------------------------------
  //ads.setGain(GAIN_TWOTHIRDS);  // 2/3x gain +/- 6.144V  1 bit = 3mV      0.1875mV (default)
  ads.setGain(GAIN_ONE);        // 1x gain   +/- 4.096V  1 bit = 2mV      0.125mV
  // ads.setGain(GAIN_TWO);        // 2x gain   +/- 2.048V  1 bit = 1mV      0.0625mV
  // ads.setGain(GAIN_FOUR);       // 4x gain   +/- 1.024V  1 bit = 0.5mV    0.03125mV
  // ads.setGain(GAIN_EIGHT);      // 8x gain   +/- 0.512V  1 bit = 0.25mV   0.015625mV
  // ads.setGain(GAIN_SIXTEEN);    // 16x gain  +/- 0.256V  1 bit = 0.125mV  0.0078125mV
  ads.begin();
  //----------------------------------------------------------------


  for (int thisReading = 0; thisReading < numReadings; thisReading++) {
    readings[thisReading] = 0;
  }


  for (uint8_t b = 0; b < 24; b++ ) {
    bar.setBar(23 - b, LED_GREEN);
    delay(50);
    bar.writeDisplay();
  }
  bar.writeDisplay();
  delay(100);
  for (uint8_t b = 0; b < 24; b++ ) {
    bar.setBar(b, LED_OFF);
  }
  bar.writeDisplay();

  //########################################################################################################################
  //########              END SETUP
  //########################################################################################################################
}

//########################################################################################################################
//########              BEGIN RTC
//########################################################################################################################
void printDigits(int digits)
{
  // utility function for digital clock display: prints preceding colon and leading 0
  Serial.print(':');
  if (digits < 10)
    Serial.print('0');
  Serial.print(digits);
}


void digitalClockDisplay(void)
{
  // digital clock display of the time
  Serial.print(hour());
  printDigits(minute());
  printDigits(second());
  Serial.print(',');
  Serial.print(day());
  Serial.print('/');
  Serial.print(month());
  Serial.print('/');
  Serial.println(year());

}
//########################################################################################################################
//########              END RTC
//########################################################################################################################

/*
//########################################################################################################################
//########              BEGIN SERIAL FUNCTIONS
//########################################################################################################################




void serialEvent() {                                                         //if the hardware serial port_0 receives a char
               char inchar = (char)Serial.read();                               //get the char we just received
               inputstring += inchar;                                           //add it to the inputString
               if(inchar == '\r') {input_stringcomplete = true;}                //if the incoming character is a <CR>, set the flag
              }
*/
void serialEvent2() {
  //if the hardware serial port_3 receives a char
  char inchar2 = (char)Serial2.read();
  if (inchar2 == '\r')
  { Fuel_String = sensorstring2;
    caudalOK=true;
    sensorstring2 = "";
  }
  else//get the char we just received
  {
    sensorstring2 += inchar2;   //add it to the inputString
  }

}
/*
  void serialEvent2(){                                                         //if the hardware serial port_3 receives a char
              char inchar = (char)Serial2.read();                              //get the char we just received
              sensorstring2 += inchar;                                          //add it to the inputString
              if(inchar == '\r') {sensor_stringcomplete2 = true;}               //if the incoming character is a <CR>, set the flag
             }
*/
void serialEvent3() {
  char inchar3 = (char)Serial3.read();                              //get the char we just received
  if (inchar3== '\r')
  { RPM2 = sensorstring3;
  //Serial.println(RPM2);
  auxRPM=true;
    sensorstring3 = "";
  }
  else
  {
    sensorstring3+= inchar3;
  }




  //if the incoming character is a <CR>, set the flag
}



//########################################################################################################################
//########              END SERIAL FUNCTIONS
//########################################################################################################################



//########################################################################################################################
//########              BEGIN MUX CD74HC4067
//########################################################################################################################

float readMux0(int channel) {
  //Cuantosms = 0;
  float voltage;
  int controlPin[] = {s0, s1, s2};
  int16_t adc0;
  int muxChannel[8][3] = {
    {0, 0, 0}, //channel 0
    {1, 0, 0}, //channel 1
    {0, 1, 0}, //channel 2
    {1, 1, 0}, //channel 3
    {0, 0, 1}, //channel 4
    {1, 0, 1}, //channel 5
    {0, 1, 1}, //channel 6
    {1, 1, 1}, //channel 7


  };
  //loop through the 4 sig
  //digitalWrite(Mux0,LOW);

  //digitalWrite(Mux1,HIGH);
  for (int i = 0; i < 3; i ++) {
    digitalWrite(controlPin[i], muxChannel[channel][i]);

  }
  unsigned long currentMillis = millis();
  //delay(1);
  adc0 = ads.readADC_SingleEnded(0);

  voltage = adc0 * 0.000125;
  //return the value
  return voltage;
}

//########################################################################################################################
//########              END MUX CD74HC4067
//########################################################################################################################


//########################################################################################################################
//########              BEGIN MUX CD74HC4067
//########################################################################################################################

float readMux1(int channel) {
  //Cuantosms = 0;
  float voltage;
  int controlPin[] = {s4, s5, s6};
  int16_t  adc1;
  int muxChannel[8][3] = {
    {0, 0, 0}, //channel 0
    {1, 0, 0}, //channel 1
    {0, 1, 0}, //channel 2
    {1, 1, 0}, //channel 3
    {0, 0, 1}, //channel 4
    {1, 0, 1}, //channel 5
    {0, 1, 1}, //channel 6
    {1, 1, 1}, //channel 7

  };
  //loop through the 4 sig
  //digitalWrite(Mux1,LOW);

  //digitalWrite(Mux0,HIGH);
  for (int i = 0; i < 3; i ++) {
    digitalWrite(controlPin[i], muxChannel[channel][i]);
  }
  unsigned long currentMillis = millis();
  adc1 = ads.readADC_SingleEnded(1);
  voltage = adc1 * 0.000125;

  //return the value
  return voltage;
}

//########################################################################################################################
//########              END MUX CD74HC4067
//########################################################################################################################


//########################################################################################################################
//########              BEGIN NTC MEASUREMENT
//########################################################################################################################
float KTY(float SensVoltage) {
 float temp;
 float resistencia;

resistencia = ((VOLTAGEREF * SERIESRESISTOR) / SensVoltage) - SERIESRESISTOR;
resistencia=resistencia*1000;

temp=resistencia-1500;
temp=temp/20;


return temp;

}


float NTCMeasurement(float SensVoltage) {
  float steinhart;
  float resistencia;
  float current;
  float Vres2;

  Serial.println();
  Serial.print("Volt Temperatura: ");
  Serial.println(SensVoltage);
  Vres2 = (VOLTAGEREF - SensVoltage);
  current = (VOLTAGEREF - SensVoltage) / SERIESRESISTOR;
  //resistencia =( VOLTAGEREF * SERIESRESISTOR) / (VOLTAGEREF-SensVoltage);
  //resistencia =(( VOLTAGEREF * SERIESRESISTOR) / (VOLTAGEREF-SensVoltage))-SERIESRESISTOR;
  //resistencia2=SERIESRESISTOR/SensVoltage;
  //Serial.println(resistencia2);
  //resistencia = (VOLTAGEREF - SensVoltage/SensVoltage)*SERIESRESISTOR;
  resistencia = ((VOLTAGEREF * SERIESRESISTOR) / SensVoltage) - SERIESRESISTOR;
  
  //resistencia = (SensVoltage - VOLTAGEREF /VOLTAGEREF)*SERIESRESISTOR;
  steinhart = resistencia / THERMISTORNOMINAL;     // (R/Ro)
  //steinhart = SensVoltage / SERIESRESISTOR;     // (R/Ro)
  steinhart = log(steinhart);                  // ln(R/Ro)
  steinhart /= BCOEFFICIENT;                   // 1/B * ln(R/Ro)
  steinhart += 1.0 / (TEMPERATURENOMINAL + 273.15); // + (1/To)
  steinhart = 1.0 / steinhart;                 // Invert
  steinhart -= 273.15;
  //steinhart = steinhart/1000.00;
  //Serial.println(steinhart);
/*
float steinhart;
  steinhart = average / THERMISTORNOMINAL;     // (R/Ro)
  steinhart = log(steinhart);                  // ln(R/Ro)
  steinhart /= BCOEFFICIENT;                   // 1/B * ln(R/Ro)
  steinhart += 1.0 / (TEMPERATURENOMINAL + 273.15); // + (1/To)
  steinhart = 1.0 / steinhart;                 // Invert
  steinhart -= 273.15;                         // convert to C

  return steinhart;
*/
}

float HoneyPX10 (float SensVoltage)
{
  float PresHoney = 0.000;
  if (SensVoltage < 0)
  {
    PresHoney = 0;
  }
  else {
    PresHoney = (3.125 * SensVoltage - 1.500);
  }


  return PresHoney;
}

String getValue(String data, char separator, int index)
{
  int found = 0;
  int strIndex[] = {
    0, -1
  };
  int maxIndex = data.length() - 1;
  for (int i = 0; i <= maxIndex && found <= index; i++) {
    if (data.charAt(i) == separator || i == maxIndex) {
      found++;
      strIndex[0] = strIndex[1] + 1;
      strIndex[1] = (i == maxIndex) ? i + 1 : i;
    }
  }
  return found > index ? data.substring(strIndex[0], strIndex[1]) : "";
}
//########################################################################################################################
//########              END NTC MEASUREMENT
//########################################################################################################################


float ReadWaterTemp() {
  uint8_t i;
  float average;
  float samples[NUMSAMPLES];
  for (i = 0; i < NUMSAMPLES; i++) {
    samples[i] = readMux0(2);

    //Serial.println(samples[i]);
    //delay(10);
  }
  average = 0;
  for (i = 0; i < NUMSAMPLES; i++) {
    average += samples[i];
  }

  average /= NUMSAMPLES;
 
 // WTemp = NTCMeasurement(average);
WTemp=KTY(average);

  if (isnan(WTemp))
  {
    WTemp = 0.00;
  }

  if (WTemp<=-20)
  {
    WTemp=0;
    }

//WTemp=WTemp-40;


  return (WTemp);
}

int ReadWaterLevel() {

  float voltage = 0.000;
  float Level;
  WLaverage = 0;
  voltage =  readMux0(0);
  
  // Serial.println(voltage);
  //   delay(1);

  Level = (voltage * 31.25) - 25;
  WLult = Level;


  for (int i = WLaux - 1; i > 0; i--) {
    WLevel[i] = WLevel[i - 1];
  }
  WLevel[0] = Level;


  for (int i = 0; i < WLaux; i++) {
    WLaverage += WLevel[i];
  }
  WLaverage = WLaverage / WLaux;

  return WLaverage;





}
float ReadOilTemp() {
  float voltage;
   voltage =  readMux1(5);
   
  uint8_t i;
  float average;
  float samples[NUMSAMPLES];
  //int value = analogRead(A0);
  //loat voltage = value * (5.0 / 1023.0);
  //return (voltage);
  //float temp = (voltage * 30) -10;
  for (i = 0; i < NUMSAMPLES; i++) {
    samples[i] = readMux1(5);
    //Serial.print(samples[i]);
    ///samples[i] = analogRead(THERMISTORPIN);
    //Serial.print(5.0*samples[i]/1024);
    //Serial.print(" - ");
    //delay(10);
  }
  average = 0;
  for (i = 0; i < NUMSAMPLES; i++) {
    average += samples[i];
  }
  average /= NUMSAMPLES;
//  OTemp = NTCMeasurement(voltage);
//OTemp=NTC2(voltage);

OTemp=KTY(average);
  if (isnan(OTemp))
  {
    OTemp = 0.00;

  }

    if (OTemp<=-20)
  {
    OTemp=0;
    }
    




  return (OTemp);
}




float ReadOilLevel() {

  float voltage = 0.000;
  float Level;
  OLaverage = 0;
  voltage =  readMux0(3);
  // Serial.println(voltage);
  //   delay(1);

  Level = (voltage * 31.25) - 25.00;
  Level= (Level - 30.00) * (100.00 - 0.00) / (45.00 - 30.00) + 0.00;
  
  OLult = Level;
  for (int i = OLaux - 1; i > 0; i--) {
    OLevel[i] = OLevel[i - 1];
  }
  OLevel[0] = Level;

  for (int i = 0; i < OLaux; i++) {
    OLaverage += OLevel[i];
  }
  OLaverage = OLaverage / OLaux;



if (OLaverage>100)
{OLaverage=100;}
if (OLaverage<0)
{OLaverage=0;}


  return OLaverage;

}


float ReadOilPress() {
  float voltage = 0.000;
  float Press = 0.000;
  voltage =  readMux0(5);


  Press = HoneyPX10(voltage);

  return Press;
}

float ReadTurboPress1() {
  float voltage;
  float Press;
  voltage =  readMux0(7);
  Press = HoneyPX10(voltage);
  return Press;
}

float ReadTurboPress2() {
  float voltage;
  float Press;
  voltage =  readMux0(4);
  voltage= voltage-0.25;
  Press = HoneyPX10(voltage);
  
  return Press;
}

float ReadEGTTemp() {

  double c = thermocouple.readCelsius();

  if (isnan(c)) {
    //  Serial.println("Something wrong with thermocouple!");

    c = thermoAux;
  } else {
    //  Serial.print("C = ");

    thermoAux = c;
c=c-100;

    //  Serial.println(c);
  }
  return c;
}

int ReadFuelLevel() {

  float voltage = 0.000;
  float Level;
  FLaverage = 0;
  voltage =  readMux0(1);

  //   delay(1);

  Level = (voltage * 31.25) - 25;
  FLult = Level;
  for (int i = FLaux - 1; i > 0; i--) {
    FLevel[i] = FLevel[i - 1];
  }
  FLevel[0] = Level;


  for (int i = 0; i < FLaux; i++) {
    FLaverage += FLevel[i];
  }
  FLaverage = FLaverage / FLaux;



  return FLaverage;

}



float FuelSUM() {

  float DieselInst = 0;
  String yval = "";
  yval.reserve(30);
  yval = getValue(Fuel_String, ',', 1);
  // Serial.println(yval);
  DieselInst = yval.toFloat();


  // Serial.println(DieselInst);
  return DieselInst;
}

float FuelACUM() {
  String yval2;
  float DieselInst2;
  yval2 = getValue(Fuel_String, ',', 0);
  //Serial.println(Fuel_String);
  DieselInst2 = yval2.toFloat();
  //Serial.println(DieselInst);

  return DieselInst2;
}

float ReadFuelPress() {
  float voltage;
  float Press;
  voltage =  readMux0(6);
  Press = HoneyPX10(voltage);
  return Press;
}

String ReadRPM() {
String yval2;
  yval2 = getValue(RPM2, ',', 1);
  return yval2;


}

float ReadGear1A() {
  float voltage;
  float current;
  voltage =  readMux1(0);
  voltage = voltage * 1000;
  current = map(voltage, 0, 5000, -1000, 1000);
   if (abs(current) <= 10) //Si el amperimetro arroja valores entre -10A y 10A la salida sera 0
  {current = 0;  }

  if (abs(current) >= 600) //Si el amperimetro arroja valores entre -10A y 10A la salida sera 0
  {    current = 0;}

  return current;


}

float ReadGear2A() {
  float voltage;
  float current;
  voltage =  readMux1(7);
  voltage = voltage * 1000;
  current = map(voltage, 0, 5000, -1000, 1000);
   if (abs(current) <= 10) //Si el amperimetro arroja valores entre -10A y 10A la salida sera 0
  {current = 0;  }

  if (abs(current) >= 600) //Si el amperimetro arroja valores entre -10A y 10A la salida sera 0
  {    current = 0;}
  return current;

}

float ReadGear3A() {
  float voltage;
  float current;
  voltage =  readMux1(3);
  voltage = voltage * 1000;
  current = map(voltage, 0, 5000, -1000, 1000);
   if (abs(current) <= 10) //Si el amperimetro arroja valores entre -10A y 10A la salida sera 0
  {current = 0;  }

  if (abs(current) >= 600) //Si el amperimetro arroja valores entre -10A y 10A la salida sera 0
  {    current = 0;}
  return current;

}

float ReadGear4A() {
  float voltage;
  float current;
  voltage =  readMux1(6);
  voltage = voltage * 1000;
  
  current = map(voltage, 0, 5000, -1000, 1000);

  if (abs(current) <= 10) //Si el amperimetro arroja valores entre -10A y 10A la salida sera 0
  {current = 0;  }

  if (abs(current) >= 600) //Si el amperimetro arroja valores entre -10A y 10A la salida sera 0
  {    current = 0;}
  
  return current;

}

float ReadGenVoltage() {
  float voltage;
  float current;
  voltage =  readMux1(4);
  
  voltage = voltage * 990;
 
  current = map(voltage, 0, 5000, 0, 1000);

  if (current<20)
  {current=0;}
  
  return current;
}

String locX() {
  
  String yval = "";
  yval.reserve(30);
  yval = getValue(Fuel_String, ',', 5);
  //maxLocX = yval.toFloat();
  //if (maxLocX<=0.10)
  //{maxLocX=0;}

  return yval;
}
String locY() {
  
  String yval = "";
  yval.reserve(50);
  yval = getValue(Fuel_String, ',', 6);
  //maxLocY = yval.toFloat();
  // if (maxLocY<=0.10)
  // {maxLocY=0;}
  return yval;
}
String locZ() {
  
  String yval = "";
  yval.reserve(50);
  yval = getValue(Fuel_String, ',', 7);
  //maxLocZ = yval.toFloat();
  // if (maxLocZ<=0.10)
  // {maxLocZ=0;}
  return yval;
}





float TelemetryT() {
  return thermocouple.readInternal();

}

void IndicadorLed (int dir, boolean a)
{
  if (a == 1)
  {
    bar.setBar(23 - dir, LED_YELLOW);
  }
  else
  {
    if (estLeds == 1) {
      bar.setBar(23 - dir, LED_GREEN);
    }
    else {
      bar.setBar(23 - dir, LED_OFF);
    }
  }
}

boolean ComprobarDatos(float dato, float minimo)
{
  boolean resp;
  if (dato <= minimo)
  {
    resp = true;
  }
  else
  {
    resp = false;
  }
  return resp;
}




void loop() {
  // put your main code here, to run repeatedly:
 //Lectura acelerometro telemetria
  sensors_event_t event;
  mma.getEvent(&event);
  ejeX = event.acceleration.x;
  ejeY = event.acceleration.y;
  ejeZ = event.acceleration.z;

  //Comprobar si son los valores mas altos en el ultimo ciclo
  maximoX(ejeX);
  maximoY(ejeY);
  maximoZ(ejeZ);

caudalOK=false;
rpmOK=false;

  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis > intervalo) {
    previousMillis = currentMillis;


    WT = ReadWaterTemp();
    IndicadorLed(10, ComprobarDatos(WT, 0));
    WL = ReadWaterLevel();
    IndicadorLed(1, ComprobarDatos(WLult, -1));
    OT = ReadOilTemp();
    IndicadorLed(9, ComprobarDatos(OT, 0));
    OL = ReadOilLevel();
    IndicadorLed(0, ComprobarDatos(OLult, -1));
    OP = ReadOilPress();
    IndicadorLed(4, ComprobarDatos(OP, 0));
    TP1 = ReadTurboPress1();
    IndicadorLed(6, ComprobarDatos(TP1, 0));
    TP2 = ReadTurboPress2();
    IndicadorLed(7, ComprobarDatos(TP2, 0));
    KT = ReadEGTTemp();
    IndicadorLed(11, ComprobarDatos(KT, 0));
    FL = ReadFuelLevel();
    IndicadorLed(2, ComprobarDatos(FLult, -1));


    FSUM = FuelSUM();
    FACUM = FuelACUM();
    IndicadorLed(19, 0);
    FP = ReadFuelPress();
    IndicadorLed(5, ComprobarDatos(FP, 0));
    RPM2 = ReadRPM();
    
    IndicadorLed(20, false);
    Gear1A = ReadGear1A() ;
    IndicadorLed(13, ComprobarDatos(Gear1A, -500));
    Gear2A = ReadGear2A() ;
    IndicadorLed(14, ComprobarDatos(Gear2A, -500));
    Gear3A = ReadGear3A() ;
    IndicadorLed(15, ComprobarDatos(Gear3A, -500));
    Gear4A = ReadGear4A() ;
    IndicadorLed(16, ComprobarDatos(Gear4A, -500));
    GenV = ReadGenVoltage();
    IndicadorLed(17, ComprobarDatos(GenV, 20));
        
    
    
    TTemp = TelemetryT();


    //bar.writeDisplay();


    if (ledCiclo == 0)
    { ledCiclo = 1;
      bar.setBar(0, LED_GREEN);
      bar.setBar(1, LED_OFF);
    }
    else
    { ledCiclo = 0;
      bar.setBar(0, LED_OFF);
      bar.setBar(1, LED_GREEN);
    }

    estLeds = 1;
    bar.writeDisplay();

    Serial.print("TW1:");
    Serial.print(WT);     //Temperatura Agua
    Serial.print(",LW1:");
    Serial.print(WL);     //Nivel Agua
    Serial.print(",TO1:");
    Serial.print(OT);     //Temperatura Aceite
    Serial.print(",LO1:");
    Serial.print(OL);     //Nivel Aceite
    Serial.print(",PO1:");
    Serial.print(OP);     //Presion Aceite
    Serial.print(",PT1:");
    Serial.print(TP1);    //Presion Turbo 1
    Serial.print(",PT2:");
    Serial.print(TP2);    //Presion Turbo 2
    Serial.print(",TE1:");
    Serial.print(KT);     //Temperatura Gases de Escape
    Serial.print(",LC1:XX");
    Serial.print(",FI1:");
    Serial.print("XX");   //Caudal Combustible Diferencia
    Serial.print(",FA1:");
    Serial.print("XX");   //Caudal Combustible Diferencia
    Serial.print(",PC1:");
    Serial.print(FP);     //Presion de Combustible
    Serial.print(",RM1:");
    Serial.print(RPM2.toInt());   //RPM Motor
    Serial.print(",CM1:");
    Serial.print(Gear1A); //Amperimetro 1
    Serial.print(",CM2:");
    Serial.print(Gear2A); //Amperimetro 2
    Serial.print(",CM3:");
    Serial.print(Gear3A); //Amperimetro 3
    Serial.print(",CM4:");
    Serial.print(Gear4A); //Amperimetro 4
    Serial.print(",VG1:");
    Serial.print(GenV);   //Voltimetro
    Serial.print(",APX:");
    Serial.print(maxX);     //Acelerometro Eje X
    Serial.print(",APY:");
    Serial.print(maxY);     //Acelerometro Eje Y
    Serial.print(",APZ:");
    Serial.print(maxZ);     //Acelerometro Eje Z
    Serial.print(",AMX:");
    Serial.print(locX());
    Serial.print(",AMY:");
    Serial.print(locY());
    Serial.print(",AMZ:");
    Serial.print(locZ());
    Serial.print(",TP1:");
    Serial.print(TTemp);  //Temperatura Telemetria
    Serial.println(",");
   // digitalClockDisplay();//Fecha y hora Telemetria



maxX=0;
maxY=0;
maxZ=0;

 }
}

float convAmp (float inVolt) {
  float corriente = 0.00;
  corriente = map(inVolt, 800, 4000, -1000, 1000);
  if (abs(corriente) <= 10) //Si el amperimetro arroja valores entre -10A y 10A la salida sera 0
  {
    corriente = 0;
  }
  return corriente;
}

void maximoX(float ultX) {
  if (abs(ultX) > maxX)
  {
    maxX = abs(ultX);
  }
}

void maximoY(float ultY) {
  if (abs(ultY) > maxY)
  {
    maxY = abs(ultY);
  }
}

void maximoZ(float ultZ) {
  if (abs(ultZ) > maxZ)
  {
    maxZ = abs(ultZ);
  }


}
