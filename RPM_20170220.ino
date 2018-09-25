
/*
 * Programa para el calculo de las revoluciones por minuto del motor
 * mediante la lectura de pulsos del sensor de efecto Hall Littelfuse 55100
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




volatile unsigned long firstPulseTime;    //Variable para guardar el momento del primer pulso de cada ciclo
volatile unsigned long lastPulseTime;     //Variable para guardar el momento del ultimo pulso de cada ciclo
volatile unsigned long numPulses;         //Numero de pulsos en cada ciclo

unsigned long previousMillis = 0;         //Momento del ultimo ciclo
const long interval = 20;               // Intervalo de cada ciclo

int numPulses2 = 0;
float result2=0.00;
int result3=0;
int nciclo=0;
//
int contador=1; //VARIABLE DE COMPROBACION  - PENDIENTE DE RETIRADA
//

float rev = 0.000;   //Revoluciones por minuto

//Declaracion de I/O
int ledFunc = 8;    //Led de funcionaminento
int ledFlow = 6;    //Led de lectura caudalimetro
int ledRPM = 5;     //Led de lectura de contador de revoluciones
int ledTX = 15;     //Led de transmision puerto serie
int ledRX = 14;     //Led de recepcion puerto serie
    
boolean contRPM=false;
boolean contTX=false;


void isr()    //Subprograma ejecutado en cada interrupcion por pulso
{
  cambioLed(ledRPM);
 //digitalWrite(ledRPM,LOW);
  contRPM=true;
  numPulses++;
  numPulses2++;
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
  digitalWrite(ledFunc, 0);
  digitalWrite(ledFlow, 0);
  digitalWrite(ledRPM, 0);
  digitalWrite(ledTX, 0);
  digitalWrite(ledRX, 0);

}



void loop()
{
  //digitalWrite(ledRPM,HIGH);
attachInterrupt(0, isr, RISING);    //Habilitar la interrupcion
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval) {
    // save the last time you blinked the LED
    previousMillis = currentMillis;


if (contRPM==false)
{digitalWrite(ledRPM,HIGH);}
contRPM=false;
if (contTX==true)
{contTX=false;
cambioLed(ledTX);
}
nciclo++;


    if (nciclo == (500 / interval))
    { cambioLed(ledTX);
      contTX = true;
      Serial.print("RPM,");
      Serial.print(result3);
  Serial.print(",");
  Serial.println(result2);
      
    }





if (nciclo==(1000/interval))
{
detachInterrupt(0);
  cambioLed(ledTX);
  contTX=true;
  Serial.print("RPM");
  Serial.print(",");
  if (numPulses==0)
  {result2=0;}
  else
  {result2=(1000000.0 * (float)(numPulses - 1)) / (float)(lastPulseTime - firstPulseTime);}

 result3=result2*60;
 
  Serial.print(result3);
  Serial.print(",");
  Serial.println(result2);
digitalWrite(ledRPM,HIGH);
  cambioLed(ledFunc);
  
 
numPulses2=0;
numPulses=0;
 
contador++;
nciclo=0;
}

  }
}

void cambioLed(int nLed)
{
digitalWrite(nLed,!digitalRead(nLed));
   }





