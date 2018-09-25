
/*
 * Programa para el calculo del caudal mediante la lectura de pulsos
 * del caudalimetro diferencial DFM250D
 *
 * Para PlacaAuxiliarPerifericos_v2.00
 *
 * Leds:
 *  - ledFunc:  pin14 D8   -  Led indicador de ciclo de funcionamiento
 *  - ledFlow:  pin12 D6   -  Led indicador de funcion de placa (Caudal) y entrada de pulso
 *  - ledRPM:   pin11 D5   -  Led indicador de funcion de placa (Revoluciones por minuto) y entrada de pulso
 *  - ledTX:    pin24 D15  -  Led indicador de comunicacion serie (TX)
 *  - ledRX:    pin23 D14  -  Led indicador de comunicacion serie (RX)
 *
 * Entradas:  (Solo se habilitara la correspondiente al objetivo de la placa)
 *  - RPM: pin4 D2  - INT0  - Entrada de pulsos de Littelfuse 55100
 *  - Flow: pin5 D3 - INT1  - Entrada de pulsos de DFM250D
 *
 */

#include <Wire.h>
#include <Adafruit_MMA8451.h>
#include <Adafruit_Sensor.h>

Adafruit_MMA8451 mma = Adafruit_MMA8451();

volatile unsigned long firstPulseTime;    //Variable para guardar el momento del primer pulso de cada ciclo
volatile unsigned long lastPulseTime;     //Variable para guardar el momento del ultimo pulso de cada ciclo
volatile unsigned long numPulses;         //Numero de pulsos en cada ciclo

unsigned long previousMillis = 0;         //Momento del ultimo ciclo
const long interval = 20;               // Intervalo de cada ciclo

//Variables de resultados:
float numPulsosTotales = 0; //Numero de pulsos detectados desde el arranque
int numPulsos = 0;          //Numero de pulsos detectados en el ultimo segundo
float flowInst = 0.00;      //Caudal instantaneo
float flowTotal = 0.00;     //Caudal total desde el arranque
int numPulsosLast=0;
int numPulsosTotalesLast=0;

int numPulses2 = 0;
float result2 = 0.00;
float result3 = 0.00;
int nciclo = 0;
float ntiempo = 0.00;
int numPulses3=0;
//
int contador = 1; //VARIABLE DE COMPROBACION  - PENDIENTE DE RETIRADA
//

float rev = 0.000;   //Revoluciones por minuto
float flow = 0.000;

//Declaracion de I/O
int ledFunc = 8;
int ledFlow = 6;
int ledRPM = 5;
int ledTX = 15;
int ledRX = 14;
int inFlow = 3;

boolean contFlow = false;
boolean contTX = false;

float ejeX = 0.00;
float ejeY = 0.00;
float ejeZ = 0.00;

float maxX = 0.00;
float maxY = 0.00;
float maxZ = 0.00;
float maxXlast = 0.00;
float maxYlast = 0.00;
float maxZlast = 0.00;

void isr()    //Subprograma ejecutado en cada interrupcion por pulso
{
  cambioLed(ledFlow);
  contFlow = true;
  numPulsos++;
  numPulses++;
  numPulses2++;
  numPulsosTotales++;
  unsigned long now = micros();
  if (numPulses == 1)
  {
    firstPulseTime = now;
  }
  else
  {
    lastPulseTime = now;
  }

}

void setup()
{
  Serial.begin(9600);    // Inicio de comunicacion serie

  //Definir tipo de I/O:
  pinMode(ledFunc, OUTPUT);
  pinMode(ledFlow, OUTPUT);
  pinMode(ledRPM, OUTPUT);
  pinMode(ledTX, OUTPUT);
  pinMode(ledRX, OUTPUT);
  pinMode(inFlow, INPUT);
  digitalWrite(ledFunc, 0);
  digitalWrite(ledFlow, 0);
  digitalWrite(ledRPM, 0);
  digitalWrite(ledTX, 0);
  digitalWrite(ledRX, 0);

  mma.begin();
  mma.setRange(MMA8451_RANGE_8_G);

}



void loop()
{

  /*Composicion de la trama:
   *
   * F_G  , L/min , Pulsos/min  , Litros totales  , Pulsos totales  , Eje X , Eje Y , Eje Z
   *
   * - F_G: Indicador de magnitud
   * - L/min: Caudal de combustible en Litros por minuto
   * - Pulsos/min: Estimacion de pulsos por minuto segun pulsos recibidos el ultimo segundo
   * - Litros totales: Litros de combustible consumidos desde el arranque de la placa
   * - Pulsos totales: Pulsos detectados desde el arranque de la placa
   * - Eje X, Eje Y, Eje Z: Valores maximos de aceleracion en cada eje en el ultimo segundo
   */


  attachInterrupt(1, isr, RISING);    // enable the interrupt
  unsigned long currentMillis = millis();

  /* Get a new sensor event */
  sensors_event_t event;
  mma.getEvent(&event);

  /* Display the results (acceleration is measured in m/s^2) */
  ejeX = event.acceleration.x;
  ejeY = event.acceleration.y;
  ejeZ = event.acceleration.z;

  maximoX(ejeX);
  maximoY(ejeY);
  maximoZ(ejeZ);

  if (currentMillis - previousMillis >= interval) {
    // save the last time you blinked the LED
    previousMillis = currentMillis;


    if (contFlow == false)
    {
      digitalWrite(ledFlow, HIGH);
    }
    contFlow = false;
    if (contTX == true)
    { contTX = false;
      cambioLed(ledTX);
    }
    nciclo++;

    if (nciclo == (500 / interval))
    { cambioLed(ledTX);
      contTX = true;
      Serial.print("F_G,");
      Serial.print(result2);
      Serial.print(",");
      Serial.print(numPulsosLast);
      Serial.print(",");
      Serial.print(flowTotal);
      Serial.print(",");
     Serial.print(numPulsosTotalesLast);
      Serial.print(",");
        Serial.print(maxXlast);
        Serial.print(",");
        Serial.print(maxYlast);
        Serial.print(",");
        Serial.println(maxZlast);
      
    }

    if (nciclo == (1000 / interval))
    {
      detachInterrupt(0);
      cambioLed(ledTX);
      contTX = true;

      if (numPulses == 0)
      {
        result2 = 0;
        ntiempo=0;
        result3=0;
      }
      else
      { 
        ntiempo = (float)(lastPulseTime - firstPulseTime);

         if (ntiempo>800000)
         {
         result2=((numPulses-1)*1000000)/ntiempo;
         result2=(result2*60.00)/80.00;
         }
         else
         {
          result2=(numPulses*60.00)/80.00;
          }
      
      result3=(numPulses*60.00)/80.00;
      }

      numPulsosLast=(int)numPulses;
      numPulsosTotalesLast=(int)numPulsosTotales;
     
      flowTotal = numPulsosTotales / 80;

      Serial.print("F_G,");
      Serial.print(result2);
      Serial.print(",");
      //Serial.print(result3);
     // Serial.print(",");
      Serial.print((int)numPulses);
      Serial.print(",");
      Serial.print(flowTotal);
      Serial.print(",");
      Serial.print((int)numPulsosTotales);
      Serial.print(",");
      Serial.print(maxX);
      Serial.print(",");
      Serial.print(maxY);
      Serial.print(",");
      Serial.println(maxZ);

      digitalWrite(ledFlow, HIGH);
      cambioLed(ledFunc);


      maxXlast = maxX;
      maxYlast = maxY;
      maxZlast = maxZ;
      maxX = 0.00;
      maxY = 0.00;
      maxZ = 0.00;

      numPulses2 = 0;
      numPulses = 0;

      contador++;
      nciclo = 0;
    }

  }
}

void cambioLed(int nLed)
{
  digitalWrite(nLed, !digitalRead(nLed));
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

void envioTrama()
{}
