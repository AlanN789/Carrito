#include "DHT.h"

/************************** Config. comunicacion ***********************************/

#include "config.h"

/************************ Example Starts Here *******************************/
#define Analog D3
#define DHTPIN D4
#define DHTTYPE DHT11
#define echo D1
#define trigger D2
#define Motor1 D8
#define Motor3 D7
#define Motor2 D6
#define Motor4 D5
DHT dht(DHTPIN, DHTTYPE);
AdafruitIO_Feed *iniciar = io.feed("semiintegrador.onooff");
AdafruitIO_Feed *lado = io.feed("semiintegrador.izqder");
AdafruitIO_Feed *distancia = io.feed("semiintegrador.ultrasonico");
AdafruitIO_Feed *dfinal = io.feed("semiintegrador.dfinal");
AdafruitIO_Feed *temperatura = io.feed("semiintegrador.temperatura");
AdafruitIO_Feed *humedad = io.feed("semiintegrador.humedad");
AdafruitIO_Feed *velocidad = io.feed("semiintegrador.velocidad");
AdafruitIO_Feed *area = io.feed("semiintegrador.area");
int veces = 0, contador = 0;
int dis[8], disfinal[4];
boolean ini = false, der = false;
float tiempo_de_espera, distancias, h, t;
int potencia = 1023;
void setup() {
  pinMode(Analog, OUTPUT);
  pinMode(Motor1, OUTPUT);
  pinMode(Motor2, OUTPUT);
  pinMode(Motor3, OUTPUT);
  pinMode(Motor4, OUTPUT);
  pinMode (trigger, OUTPUT);
  pinMode (echo, INPUT);
  analogWrite(Analog, potencia);
  Serial.begin(115200);
  while (! Serial);
  iniciar->onMessage(handleIniciar);
  lado->onMessage(handleLado);
  velocidad->onMessage(handleVelocidad);
  Serial.print("Conectando a Adafruit IO");
  io.connect();
  while (io.status() < AIO_CONNECTED) {
    Serial.print(".");
    delay(500);
  }
  Serial.println();
  Serial.println(io.statusText());
  dht.begin();
  velocidad->get();
  iniciar->get();
  lado->get();
}

void loop() {
  delay(100);
  io.run();
  if (ini) {
    Temp();
    Ultrasonico();
    if (distancias < 30) {
      detener();
      dis[veces] = distancias;
      veces++;
      if (der) {
        Serial.println("Girando a la derecha");
        giroDer();
        delay(1000);
        detener();
        delay(1000);
        Ultrasonico();
        dis[veces] = distancias;
        veces++;
      }
      else {
        Serial.println("Girando a la izquierda");
        giroIzq();
        delay(1000);
        detener();
        delay(900);
        Ultrasonico();
        dis[veces] = distancias;
        veces++;
      }
    }
    contador++;
    if (contador == 20) {
      mandarDatos();
      contador = 0;
    }
    if (veces >= 8) {
      Serial.println("Se completo el ciclo");
      detener();
      ini = false;
      disfinal[0] = dis[0] + dis[3];
      disfinal[1] = dis[1] + dis[6];
      disfinal[2] = dis[2] + dis[5];
      disfinal[3] = dis[4] + dis[7];
      for (int i = 0; i < 4; i++) {
        Serial.print ("Distancias guardada ");
        Serial.print (i);
        Serial.print (":");
        Serial.println (disfinal[i]);
        dfinal->save(disfinal[i]);
        delay(2000);
      }
      area->save(disfinal[0] * disfinal[1]);
    }
    else
    {
      avanzar();
      Serial.print ("Avanzando ");
    }

  }
}
void Temp() {
  h = dht.readHumidity();
  t = dht.readTemperature();
  Serial.print("temperatura: ");
  Serial.print(t);
  Serial.print(" humedad: ");
  Serial.println(h);
}
void mandarDatos() {
  distancia->save(distancias);
  temperatura->save(t);
  humedad->save(h);
}
void Ultrasonico()
{
  digitalWrite (trigger, LOW);
  delayMicroseconds(2);
  digitalWrite (trigger, HIGH);
  delayMicroseconds (10);
  digitalWrite (trigger, LOW);
  tiempo_de_espera = pulseIn (echo, HIGH);
  distancias = (tiempo_de_espera / 2) / 29.15;
  Serial.print ("Distancia: ");
  Serial.print (distancias);
  Serial.println ("cm");
}
void avanzar() {
  digitalWrite (Motor3, HIGH);
  digitalWrite (Motor4, LOW);
  digitalWrite (Motor1, HIGH);
  digitalWrite (Motor2, LOW);
}
void detener() {
  digitalWrite (Motor3, LOW);
  digitalWrite (Motor4, LOW);
  digitalWrite (Motor1, LOW);
  digitalWrite (Motor2, LOW);
}
void giroDer() {
  digitalWrite (Motor3, HIGH);
  digitalWrite (Motor4, LOW);
  digitalWrite (Motor1, LOW);
  digitalWrite (Motor2, LOW);
}
void giroIzq() {
  digitalWrite (Motor3, LOW);
  digitalWrite (Motor4, LOW);
  digitalWrite (Motor1, HIGH);
  digitalWrite (Motor2, LOW);
}
void handleIniciar(AdafruitIO_Data *dataini) {

  Serial.print("received <- ");

  if (dataini->toPinLevel() == HIGH) {
    Serial.println("Iniciar");
    ini = true;
    for (int i = 0; i < 4; i++) {
      disfinal[i] = 0;
    }
    for (int i = 0; i < 8; i++) {
      dis[i] = 0;
    }
    veces = 0;
  }
  else {
    Serial.println("Detener");
    ini = false;
    detener();
  }
}
void handleLado(AdafruitIO_Data *datalado) {

  Serial.print("received <- ");

  if (datalado->toPinLevel() == HIGH) {
    Serial.println("DER");
    der = true;
  }
  else {
    Serial.println("IZQ");
    der = false;
  }
}
void handleVelocidad(AdafruitIO_Data *datavelo) {

  Serial.print("received <- ");
  Serial.println(datavelo->toInt());
  analogWrite(Analog, datavelo->toInt());
}
