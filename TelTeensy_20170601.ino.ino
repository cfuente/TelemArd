/*
 * Listado de Sensores:
 * - ENTRADA DEMUX - (S0,S1,S2):  NOMBRE   :   Descripcion   :   Tipo
 *
 * DEMUX1:
 * - X0 - (0,0,0):  4.20  :   Entrada auxiliar        :   4-20mA
 * - X1 - (1,0,0):  L_G   :   Presion de Aceite       :   4-20mA
 * - X2 - (0,1,0):  T_W   :   Presion de Turbo 1      :   4-20mA
 * - X3 - (1,1,0):  L_O   :   Nivel de Aceite         :   4-20mA
 * - X4 - (0,0,1):  P_T2  :   Presion de Combustible  :   4-20mA
 * - X5 - (1,0,1):  P_O   :   Nivel de Combustible    :   4-20mA
 * - X6 - (0,1,1):  P_W   :   Presion de Turbo 2      :   4-20mA
 * - X7 - (1,1,1):  P_T1  :   Nivel de Agua           :   4-20mA
 *
 * DEMUX2:
 * - X0 - (0,0,0):  AMP2  :   Amperimetro 2           :   4-20mA
 * - X1 - (1,0,0):  VOLT  :   Voltimetro              :   4-20mA
 * - X2 - (0,1,0):  EGT   :   Temperatura Gases Escape:   4-20mA
 * - X3 - (1,1,0):  AMP1  :   Amperimetro 1           :   4-20mA
 * - X4 - (0,0,1):  T_W   :   Voltimetro              :   4-20mA
 * - X5 - (1,0,1):  AMP3  :   Amperimetro 3           :   4-20mA
 * - X6 - (0,1,1):  T_O   :   Temperatura de Aceite   :   4-20mA
 * - X7 - (1,1,1):  AMP4  :   Amperimetro 4           :   4-20mA
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

//----------  INICIALIZACION RTC  --------------------------------
#include <DS3232RTC.h>
//#include <WireRtcLib.h>
#include <Time.h>
//----------------------------------------------------------------

//-------------INICIALIZACION ADC  -------------------------------
#include <Adafruit_ADS1015.h>
Adafruit_ADS1115 ads;
float refVolt = 5.01;
//----------------------------------------------------------------

//-------------INICIALIZACION LEDS  -------------------------------
#include "Adafruit_LEDBackpack.h"
#include "Adafruit_GFX.h"
Adafruit_24bargraph bar = Adafruit_24bargraph();
boolean ledCiclo = 1;
float WLult;    //Ultima lectura de Nivel de Agua
float OLult;    //Ultima lectura de Nivel de Aceite
float FLult;    //Ultima lectura de Nivel de Combustible
//----------------------------------------------------------------

//--- LEDS INDICADORES AUXILIARES ---
int LED0 = 20;    //Recepcion Serial1
int LED1 = 21;
int LED2 = 22;
int LED3 = 23;
//------------------------------------

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

String Fuel_String;       //Texto recibido desde FlowEZO. Formato: 0.00,0.00
boolean serial1check = false;
boolean serial2check = false;
//----------------------------------------------------------------

//--- Control de Multiplexores ---
// MUX 0
int s00 = 6;
int s01 = 5;
int s02 = 2;
// MUX 1
int s10 = 14;
int s11 = 11;
int s12 = 12;

int muxChannel[8][3] = {
  {0, 0, 0}, //Y0
  {1, 0, 0}, //Y1
  {0, 1, 0}, //Y2
  {1, 1, 0}, //Y3
  {0, 0, 1}, //Y4
  {1, 0, 1}, //Y5
  {0, 1, 1}, //Y6
  {1, 1, 1}, //Y7
};
//--------------------------------

//--- VARIABLES SENSORES ---
//PRESION
float P_O;    //ACEITE
float P_G;    //COMBUSTIBLE
float P_T1;   //TURBO 1
float P_T2;   //TURBO 2
//NIVEL
float L_O;    //ACEITE
float L_W;    //AGUA
float L_G;    //COMBUSTIBLE
//TEMPERATURA
float T_O;    //ACEITE
float T_W;    //AGUA
float EGT;    //GASES DE ESCAPE
//CAUDAL
String F_G;    //CAUDAL DE COMBUSTIBLE
//REVOLUCIONES MOTOR
String RPM;    //REVOLUCIONES POR MINUTO MOTOR
//ELECTRICOS
float AMP1;   //CORRIENTE MOTOR 1
float AMP2;   //CORRIENTE MOTOR 2
float AMP3;   //CORRIENTE MOTOR 3
float AMP4;   //CORRIENTE MOTOR 4
float VOLT;   //TENSION GENERADOR
//ACELERACIONES
float telX;   //Eje X Telemetria
float telY;   //Eje Y Telemetria
float telZ;   //Eje Z Telemetria
String motX;   //Eje X Motor
String motY;   //Eje Y Motor
String motZ;   //Eje Z Motor

boolean sensores[21] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};



//--------------  VARIABLES SENSORES  ----------------------------

float WL;             //Nivel Agua
float OL;             //Nivel Aceite
float FL;             //Nivel de Combustible
float FIN;            //Caudal +
float FOUT;           //Caudal -
float FuelConsumo;
String FSUM;           //Caudal Instantaneo
//float DieselInst;
String FACUM;          //Caudal de combustible Acumulado
float fAcumPrev;
//int RPM;              //RPM Motor
String RPM2;
String RPM3;
int RPM4;

float TTemp;          //Temperatura Telemetria
//----------------------------------------------------------------


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

//---Variables auxiliares Acelerometro Telemetria---
float ejeX = 0.00;
float ejeY = 0.00;
float ejeZ = 0.00;
float maxX = 0.00;
float maxY = 0.00;
float maxZ = 0.00;
//-------------------------------------------------

//--- Variables Temporizacion ---
long previousMillis = 0;
long intervalo = 1000;
//-----------------------------------

boolean auxRPM = false;
boolean varContHora = false;

void setup() {

  //--- INICIO COMUNICACIONES ---
  Serial.begin(115200);   //COMUNICACION CON EMBARCADO

  Serial3.begin(9600);  //COMUNICACION CON CONTADOR DE REVOLUCIONES
  Serial1.begin(9600);  //COMUNICACION CON CUADALIMETRO


  //----------  LEDS  -----------------------------------------
  bar.begin(0x70);  // pass in the address
  Serial.println("1");
  for (uint8_t b = 0; b < 24; b++ ) {
    bar.setBar(b, LED_OFF);
  }
  bar.writeDisplay();
  //-----------------------------------------------------------


  //########################################################################################################################
  //########              BEGIN SERIAL MUX
  //########################################################################################################################

  inputstring.reserve(5);                                                   //set aside some bytes for receiving data from the PC
  sensorstring.reserve(30);                                                 //set aside some bytes for receiving data from Atlas Scientific product
  // inputstring1.reserve(5);                                                   //set aside some bytes for receiving data from the PC
  sensorstring1.reserve(30);                                                 //set aside some bytes for receiving data from Atlas Scientific product
  //inputstring2.reserve(5);
  //sensorstring2.reserve(30);
  inputstring3.reserve(5);
  sensorstring3.reserve(60);
  Fuel_String.reserve(30);
  //########################################################################################################################
  //########              END SERIAL MUX
  //########################################################################################################################


  //--- INICIALIZACION DS3231 [RELOJ TIEMPO REAL] ---
  setSyncProvider(RTC.get);
  //------------------------------

  //--- INICIALIZACION MMA8451 [ACELEROMETRO] ---
  mma.begin();
  mma.setRange(MMA8451_RANGE_8_G);
  //----------------------------------------------

  //--- INICIALIZACION ADS1115 [ADC] ---
  ads.setGain(GAIN_ONE);        // 1x gain   +/- 4.096V  1 bit = 2mV      0.125mV
  ads.begin();
  //--------------------------------

  //--- Configuracion pines control de Multiplexores ---
  pinMode(s00, OUTPUT);
  pinMode(s01, OUTPUT);
  pinMode(s02, OUTPUT);
  pinMode(s10, OUTPUT);
  pinMode(s11, OUTPUT);
  pinMode(s12, OUTPUT);
  digitalWrite(s00, LOW);
  digitalWrite(s01, LOW);
  digitalWrite(s02, LOW);
  digitalWrite(s10, LOW);
  digitalWrite(s11, LOW);
  digitalWrite(s12, LOW);
  //-----------------------------------------------

  //--- Configuracion pines leds indicadores auxiliares ---
  pinMode(LED0, OUTPUT);
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  pinMode(LED3, OUTPUT);
  digitalWrite(LED0, LOW);
  digitalWrite(LED1, LOW);
  digitalWrite(LED2, LOW);
  digitalWrite(LED3, LOW);
  //-------------------------------------------------------


  //--- ANIMACION LEDS INICIO ---
  for (uint8_t b = 0; b < 24; b++ ) {
    bar.setBar(b, LED_GREEN);
    delay(50);
    bar.writeDisplay();
  }
  bar.writeDisplay();
  delay(100);
  for (uint8_t b = 0; b < 24; b++ ) {
    bar.setBar(b, LED_OFF);
  }
  bar.writeDisplay();
  Serial.println("END SETUP");

  //----------------------------------------------------------------------------
}

void printDigits(int digits)
{
  //Añadir cero a la izquierda en horas y minutos si tienen un solo digito
  Serial.print(':');
  if (digits < 10)
    Serial.print('0');
  Serial.print(digits);
}


void enviarFecha(void)
{
  //Enviar datos reloj puerto serie
  Serial.print(hour());
  printDigits(minute());
  printDigits(second());
  Serial.print(',');
  Serial.print(day());
  Serial.print('/');
  Serial.print(month());
  Serial.print('/');
  Serial.print(year());

}




//--- Interrupciones puertos serie -------
void serialEvent() {                                                         //if the hardware serial port_0 receives a char
  char inchar = (char)Serial.read();                               //get the char we just received
  //add it to the inputString
  if (inchar == '\r') {
    input_stringcomplete = true;

    if (inputstring.length()==21)
    {String prub = "";
    String comp = "H";
    prub = getValue(inputstring, ',', 0);

    if (prub == comp)
    {

    
      int YYYY = getValue(inputstring, ',', 1).toInt();
      int MM = getValue(inputstring, ',', 2).toInt();
      int DD = getValue(inputstring, ',', 3).toInt();
      int hh = getValue(inputstring, ',', 4).toInt();
      int mm = getValue(inputstring, ',', 5).toInt();
      int ss = getValue(inputstring, ',', 6).toInt();

      compMaxHora(MM, 12);
      compDiasMes(DD, MM);
      compMaxHora(hh, 60);
      compMaxHora(mm, 60);
      compMaxHora(ss, 60);

      if (varContHora == false)
      {
        time_t t;
        tmElements_t tm;

        tm.Year = YYYY - 1970;
        tm.Month = MM;
        tm.Day = DD;
        tm.Hour = hh;
        tm.Minute = mm;
        tm.Second = ss;
        t = makeTime(tm);
        RTC.set(t);
        setTime(t);

        Serial.println("Hora cambiada correctamente.");
      }
      else
      {
        Serial.println("Formato no valido.");
      }

    }
    else
    {
      Serial.println("Comando no reconocido.");
    }
    }
    else
    {Serial.println("Formato no valido.");}
    inputstring = "";
    varContHora = false;
  }
  else
  {
    inputstring += inchar; //if the incoming character is a <CR>, set the flag
  }
}



void serialEvent1() {
  /*Subprogama ejecutado al recibir datos a traves del puerto serie 1 (Caudalimetro)
   * Añade los datos recibidos a un string. Si el caracter recibido corresponde a un
   * salto de carro significa que el mensaje recibido esta completo. En ese momento se
   * copia el mensaje obtenido en Fuel_String para su posterior procesado y se borra el string auxiliar.
   */

  digitalWrite(LED0, HIGH);
  serial1check = true;      //Variable de control - Datos recividos a traves de puerto serie 1

  char inchar1 = (char)Serial1.read();
  if (inchar1 == '\r')   {   //Si el caracter recibido es un salto de linea
    Fuel_String = sensorstring1;
    sensorstring1 = "";
  }
  else  {
    sensorstring1 += inchar1;
  }

}

void serialEvent3() {
  /*Subprogama ejecutado al recibir datos a traves del puerto serie 3 (Medidor de revoluciones motor)
  * Añade los datos recibidos a un string. Si el caracter recibido corresponde a un
  * salto de carro significa que el mensaje recibido esta completo. En ese momento se
  * copia el mensaje obtenido en RPM2 para su posterior procesado y se borra el string auxiliar.
  */

  digitalWrite(LED3, HIGH);
  serial2check = true;

  char inchar3 = (char)Serial3.read();
  if (inchar3 == '\r')  {
    RPM2 = sensorstring3;
    auxRPM = true;
    sensorstring3 = "";
  }
  else  {
    sensorstring3 += inchar3;
  }
}

// ------------------------------------------



//--- FUNCIONES MULTIPLEXADO ---
float readMux0(int channel) { //Lectura Multiplexor 0
  float voltage;
  int controlPin[] = {s00, s01, s02};
  int16_t adc0;
  for (int i = 0; i < 3; i ++) {
    digitalWrite(controlPin[i], muxChannel[channel][i]);
  }

  adc0 = ads.readADC_SingleEnded(1);
  voltage = adc0 * 0.125;
  return voltage;
}

float readMux1(int channel) { //Lectura Multiplexor 1
  float voltage = 0.00;
  int controlPin[] = {s10, s11, s12};
  int16_t  adc1;
  for (int i = 0; i < 3; i ++) {
    digitalWrite(controlPin[i], muxChannel[channel][i]);
  }
  adc1 = ads.readADC_SingleEnded(0);
  voltage = adc1 * 0.125;
  return voltage;
}

//--- FUNCIONES CONVERSION DE DATOS ---
float convPress (float inVolt) {
  float presion = 0.00;
  presion = map(inVolt, 800, 4000, 0, 1000);
  return presion;
}

float convTemp (float inVolt) {
  float temperatura = 0.00;
  temperatura = map(inVolt, 800, 4000, -50, 150);
  return temperatura;
}

float convEGT (float inVolt) {
  float temperatura = 0.00;
  temperatura = map(inVolt, 800, 4000, 0, 1200);
  return temperatura;
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

float convVolt (float inVolt) {
  float tension = 0.00;
  tension = map(inVolt, 800, 4000, 0, 1000);
  return tension;
}
//-------------------------------------



//--- FUNCIONES LECTURA DE SENSORES ---
float readT_W() {
  float aux2 = 0.00;
  float volt = 0.00;
  volt = readMux1(4);
  compConexion(volt, 10);
  aux2 = convTemp(volt);
  return aux2;
}
float readT_O() {
  float aux2 = 0.00;
  float volt = 0.00;
  volt = readMux1(6);
  compConexion(volt, 9);
  aux2 = convTemp(volt);
  return aux2;
}
float readEGT() {
  float aux2 = 0.00;
  float volt = 0.00;
  volt = readMux1(2);
  compConexion(volt, 11);
  aux2 = convEGT(volt);
  return aux2;
}

float readP_O() {
  float aux2 = 0.00;
  float volt = 0.00;
  volt = readMux0(1);
  compConexion(volt, 4);
  aux2 = convPress(volt);
  return aux2;
}
float readP_G() {
  float aux2 = 0.00;
  float volt = 0.00;
  volt = readMux0(4);
  compConexion(volt, 5);
  aux2 = convPress(volt);
  return aux2;
}
float readP_T1() {
  float aux2 = 0.00;
  float volt = 0.00;
  volt = readMux0(2);
  compConexion(volt, 6);
  aux2 = convPress(volt);
  return aux2;
}
float readP_T2() {
  float aux2 = 0.00;
  float volt = 0.00;
  volt = readMux0(6);
  compConexion(volt, 7);
  aux2 = convPress(volt);
  return aux2;
}
/*
float readL_O() {

}
float readL_W() {

}
float readL_G() {

}
*/
float readAMP1() {
  float aux2 = 0.00;
  float volt = 0.00;
  volt = readMux1(3);
  compConexion(volt, 13);
  aux2 = convAmp(volt);
  return aux2;
}
float readAMP2() {
  float aux2 = 0.00;
  float volt = 0.00;
  volt = readMux1(0);
  compConexion(volt, 14);
  aux2 = convAmp(volt);
  return aux2;
}
float readAMP3() {
  float aux2 = 0.00;
  float volt = 0.00;
  volt = readMux1(5);
  compConexion(volt, 15);
  aux2 = convAmp(volt);
  return aux2;
}
float readAMP4() {
  float aux2 = 0.00;
  float volt = 0.00;
  volt = readMux1(7);
  compConexion(volt, 16);
  aux2 = convAmp(volt);
  return aux2;
}
float readVOLT() {
  float aux2 = 0.00;
  float volt = 0.00;
  volt = readMux1(1);
  compConexion(volt, 17);
  aux2 = convVolt(volt);
  return aux2;
}

String FuelSUM() {
  String fuelInst = "0";
  String yval = "";
  yval.reserve(30);
  fuelInst = getValue(Fuel_String, ',', 1);
  // Serial.println(yval);
  //fuelInst = yval();
  return fuelInst;
}
String FuelACUM() {
  String yval2;
  String fuelAcum;
  fuelAcum = getValue(Fuel_String, ',', 3);
  //fuelAcum = yval2;
  //if (fuelAcum<fAcumPrev)
  //{fuelAcum=fAcumPrev;}
  //fAcumPrev = fuelAcum;
  return fuelAcum;
}

String locX() {
  //String maxLocX = 0;
  String yval = "";
  yval.reserve(30);
  yval = getValue(Fuel_String, ',', 5);
  //maxLocX = yval.toFloat();
  //if (maxLocX<=0.10)
  //{maxLocX=0;}

  return yval;
}
String locY() {
//  String maxLocY = 0;
  String yval = "";
  yval.reserve(50);
  yval = getValue(Fuel_String, ',', 6);
  //maxLocY = yval.toFloat();
  // if (maxLocY<=0.10)
  // {maxLocY=0;}
  return yval;
}
String locZ() {
//  String maxLocZ = 0;
  String yval = "";
  yval.reserve(50);
  yval = getValue(Fuel_String, ',', 7);
  //maxLocZ = yval.toFloat();
  // if (maxLocZ<=0.10)
  // {maxLocZ=0;}
  return yval;
}

//-------------------------------------

//--- FUNCIONES VERSION ANTIGUA ---
int ReadWaterLevel() {

  float voltage = 0.000;
  float Level;

  WLaverage = 0;
  voltage =  readMux0(7);
  compConexion(voltage, 1);
  voltage = voltage / 1000;
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

float ReadOilLevel() {

  float voltage = 0.000;
  float Level;
  OLaverage = 0;
  voltage =  readMux0(3);
  compConexion(voltage, 0);
  voltage = voltage / 1000;
  Level = (voltage * 31.25) - 25.00;
  Level = (Level - 30.00) * (100.00 - 0.00) / (45.00 - 30.00) + 0.00;

  OLult = Level;
  for (int i = OLaux - 1; i > 0; i--) {
    OLevel[i] = OLevel[i - 1];
  }
  OLevel[0] = Level;

  for (int i = 0; i < OLaux; i++) {
    OLaverage += OLevel[i];
  }
  OLaverage = OLaverage / OLaux;

  if (OLaverage > 100)
  { OLaverage = 100;
  }
  if (OLaverage < 0)
  { OLaverage = 0;
  }
  return OLaverage;
}

int ReadFuelLevel() {

  float voltage = 0.000;
  float Level;
  FLaverage = 0;
  voltage =  readMux0(5);
  compConexion(voltage, 2);
  voltage = voltage / 1000;
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

String ReadRPM() {

  String yval2;
  yval2 = getValue(RPM2, ',', 1);
  return yval2;
}



void compMaxHora (int val, int limit)
{
  if (val >= limit || val < 0)
  {
    varContHora = true;
  }
}

void compDiasMes (int val, int mes)
{
  int diasXmes[] = {31, 29, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
  if (val >= diasXmes[mes - 1] || val < 0)
  {
    varContHora = true;
  }
}

void estadoLed (int direccion, boolean estado)
{
  if (estado == true)
  {
    bar.setBar(direccion, LED_GREEN);
  }
  else
  {
    bar.setBar(direccion, LED_YELLOW);
  }
}

boolean ComprobarDatos(float dato, float minimo) {
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

void compConexion (float volt, int posicion)
{ //Funcion para comprobar si un sensor de 4-20mA se encuentra conectado.
  //La placa dispone de resistencias de 200ohms, por lo que
  // 4mA --> 0.8V (800mV)
  // volt = tension medida en mV

  if (volt >= 800)
  {
    sensores[posicion] = true;
  }
  else
  {
    sensores[posicion] = false;
  }
}


String devolValorSensor (float dato, int posicion)
{
  String resp;
  if (sensores[posicion] == false)
  {
    resp = "XX";
  }
  else
  {
    resp = String(dato);
  }
  return resp;
}

String comprobarSensorInt (int dato, float minimo) {
  String resp;
  if (dato <= minimo)
  { resp = "XX";
  }
  else
  { resp = String(dato);
  }
  return resp;
}

String comprobarCOM (float dato, int serialN) {
  String resp;
  if (serialN == 1)
  { if (serial1check == false)
    {
      resp = "XX";
    }
    else
    {
      resp = String(dato, 2);
    }
  }
  if (serialN == 2)
  { if (serial2check == false)
    {
      resp = "XX";
    }
    else
    {
      resp = String(dato, 2);
    }
  }
  return resp;
}

String comprobarCOM2 (String dato, int serialN)
{

  String resp;
  if (serialN == 1)
  { if (serial1check == false)
    {
      resp = "XX";
    }
    else
    {
      resp = dato;
    }
  }
  if (serialN == 2)
  { if (serial2check == false)
    {
      resp = "XX";
    }
    else
    {
      resp = dato;
    }
  }
  return resp;
}




String getValue(String data, char separator, int index)
{ //Funcion para obtener varios strings de un string separado por un caracter concreto0
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


void loop() {


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

  digitalWrite(LED0, LOW);

  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis > intervalo) {
    previousMillis = currentMillis;

    //Acelerometros motor
    motX = locX();
    motY = locY();
    motZ = locZ();

    //Lectura de sensores:
    //  - Presiones:
    P_O = readP_O() / 100;
    P_G = readP_G() / 100;
    P_T1 = readP_T1() / 100;
    P_T2 = readP_T2() / 100;
    //  - Temperaturas:
    T_O = readT_O();
    T_W = readT_W();
    EGT = readEGT();
    //  - Niveles:
    OL = ReadOilLevel();
    WL = ReadWaterLevel();
    FL = ReadFuelLevel();
    //  - Sensores electricos:
    AMP1 = readAMP1() ;
    AMP2 = readAMP2() ;
    AMP3 = readAMP3() ;
    AMP4 = readAMP4() ;
    VOLT = readVOLT();

    //Lectura de sensores comunicacion serie
    FSUM = FuelSUM();
    FACUM = FuelACUM();
    //estadoLed(19, ComprobarDatos(motX, 0));
    estadoLed(19, serial1check);
    RPM3 = ReadRPM();
    RPM4 = RPM3.toInt();
    //estadoLed(20, ComprobarDatos(RPM3.toInt(), 0));
    estadoLed(20, serial2check);

    TTemp = RTC.temperature() / 5;

    //Modificar LEDS indicadores segun sensores conectados:
    estadoLed(0, sensores[0]);  //L_O
    estadoLed(1, sensores[1]);  //L_W
    estadoLed(2, sensores[2]);  //L_G

    estadoLed(4, sensores[4]); //P_O
    estadoLed(5, sensores[5]); //P:G
    estadoLed(6, sensores[6]); //P_T1
    estadoLed(7, sensores[7]); //P_T2

    estadoLed(9, sensores[9]);  //T_O
    estadoLed(10, sensores[10]);  //T_W
    estadoLed(11, sensores[11]);  //EGT

    estadoLed(13, sensores[13]);  //AMP1
    estadoLed(14, sensores[14]);  //AMP2
    estadoLed(15, sensores[15]);  //AMP3
    estadoLed(16, sensores[16]);  //AMP4
    estadoLed(17, sensores[17]);  //VOLT


    //LEDs INDICADORES DE FUNCIONAMIENTO
    if (ledCiclo == 0)    {
      ledCiclo = 1;
      bar.setBar(22, LED_GREEN);
      bar.setBar(23, LED_OFF);
    }
    else    {
      ledCiclo = 0;
      bar.setBar(22, LED_OFF);
      bar.setBar(23, LED_GREEN)  ;
    }

    bar.writeDisplay();

    //COMPOSICION Y ENVIO DE LA TRAMA

    Serial.print("TW1:");
    Serial.print(devolValorSensor(T_W, 10));
    Serial.print(",LW1:");
    Serial.print(devolValorSensor(WL, 1));    //Nivel Agua
    Serial.print(",TO1:");
    Serial.print(devolValorSensor(T_O, 9));   //Temperatura Aceite
    Serial.print(",LO1:");
    Serial.print(devolValorSensor(OL, 0));    //Nivel Aceite
    Serial.print(",PO1:");
    Serial.print(devolValorSensor(P_O, 4));   //Presion Aceite
    Serial.print(",PT1:");
    Serial.print(devolValorSensor(P_T1, 6));   //Presion Turbo 1
    Serial.print(",PT2:");
    Serial.print(devolValorSensor(P_T2, 7));   //Presion Turbo 2
    Serial.print(",TE1:");
    Serial.print(devolValorSensor(EGT, 11));    //Temperatura Gases de Escape
    Serial.print(",LC1:");
    Serial.print(devolValorSensor(FL, 2));    //Nivel Combustible
    Serial.print(",FI1:");
    Serial.print(comprobarCOM2(FSUM, 1));  //Caudal Combustible Diferencia
    Serial.print(",FA1:");
    Serial.print(comprobarCOM2(FACUM, 1 ));  //Caudal Combustible Diferencia
    Serial.print(",PC1:");
    Serial.print(devolValorSensor(P_G, 5));    //Presion de Combustible
    Serial.print(",RM1:");
    Serial.print(comprobarCOM(RPM4, 2));  //RPM Motor
    Serial.print(",CM1:");
    Serial.print(devolValorSensor(AMP1, 13)); //Amperimetro 1
    Serial.print(",CM2:");
    Serial.print(devolValorSensor(AMP2, 14)); //Amperimetro 2
    Serial.print(",CM3:");
    Serial.print(devolValorSensor(AMP3, 15)); //Amperimetro 3
    Serial.print(",CM4:");
    Serial.print(devolValorSensor(AMP4, 16)); //Amperimetro 4
    Serial.print(",VG1:");
    Serial.print(devolValorSensor(VOLT, 17));  //Voltimetro
    Serial.print(",APX:");
    Serial.print(maxX);     //Acelerometro Eje X
    Serial.print(",APY:");
    Serial.print(maxZ);     //Acelerometro Eje Y
    Serial.print(",APZ:");
    Serial.print(maxY);     //Acelerometro Eje Z
    Serial.print(",AMX::");
    Serial.print(comprobarCOM2(motX, 1));
    Serial.print(",AMY:");
    Serial.print(comprobarCOM2(motY, 1));
    Serial.print(",AMZ:");
    Serial.print(comprobarCOM2(motZ, 1));
    Serial.print(",TP1:");
    Serial.print(TTemp);  //Temperatura Telemetria
    Serial.print(",");
    //enviarFecha();//Fecha y hora Telemetria
    Serial.println();

    maxX = 0.00;
    maxY = 0.00;
    maxZ = 0.00;

    Fuel_String = "XX,XX,XX,XX,XX";
    RPM2 = "";

    serial1check = false;
    serial2check = false;
  }
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

