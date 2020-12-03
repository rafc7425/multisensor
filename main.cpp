
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <DNSServer.h>
#include <WiFiManager.h>
#include <Arduino.h>
#include <PubSubClient.h>
#include <Servo.h>
#include <dht.h>

const char* ssid     = "Tu cola";
const char* password = "12341234";

const char *mqtt_server = "iotechs.mx";
const int mqtt_port = 1883;
const char *mqtt_user = "web_client";
const char *mqtt_pass = "12341234";

WiFiClient espClient;
PubSubClient client(espClient);


//Declaracion de variables
Servo myservo; // servo1
Servo myservo2;//servo2
Servo myservo3;//servo3
int servoPin = 5;//pin del servo1 = D1
int servo2 = 4; //pin d2
int servo3 = 0; //pin d3
int angle = 0; //inicia en 0
//---------------------//
long lastMsg = 0; //mqtt ultimo mensaje
char msg[25];// mensaje hasta 25 pocisiones [0],[1],[2]...//
//-------------------//
//temperatura//
#define DHTPIN 14 //pin D5
#define DHTTYPE DHT11 //tipo DHT11
DHT dht(DHTPIN, DHTTYPE);
float temp1;
float temp2;
/////////////////
// sensor de movimiento
const int PIRPin = 12; //pin d6 sensor de movimiento
const int LEDPin = 13;  //pin d7 led
int val = 0; // valor de movimiento
///////////////////////
//Sensor de gas
const int MQ_PIN = A0;//pin del sensor
const int buzzer = 16;//pin D0 del buzzer
const int vent = 15;//pin D8 del ventilador
/////////////////////
//relay o focos
const int RX = 1;
const int TX = 3;
//*****************************
//*** DECLARACION FUNCIONES ***
//*****************************
void setup_wifi(); // config del wifi
void callback(char* topic, byte* payload, unsigned int length); // topicos
void reconnect(); //reconexion

void setup() {
	pinMode(BUILTIN_LED, OUTPUT);// led de la placa como salida
	digitalWrite(BUILTIN_LED, LOW);
	pinMode(TX, OUTPUT);//led 1 en entrada TX
	digitalWrite(TX, LOW);//inicia apagado
	pinMode(RX, OUTPUT);//led 2 en RX
	digitalWrite(RX, LOW);//inicia apagado
	pinMode(LEDPin, OUTPUT);//pin d7 como salida
	pinMode(buzzer, OUTPUT);//pin D0 como salida
	digitalWrite(buzzer, LOW);//buzzer inicia apagado
	pinMode(vent, OUTPUT);//pin D8 como salida
	digitalWrite(vent, LOW);//ventilador apagado
	/////SERVOS///
  myservo.attach(servoPin);//servo1
	myservo2.attach(servo2); // servo 2
	myservo3.attach(servo3);//servo3
	angle = 0;//inicia servos en 0
	////TEMP//
	dht.begin(); // inicia temp y humedad

	/////////Configuracion wifi y mqtt//////////////
	//apago serial para usar pines rx y tx debuggeo usando mqtt
//	Serial.begin(115200);//inicio la comunicacion serial
	randomSeed(micros()) ;// semilla random para contar milisegundos
	setup_wifi();
	client.setServer(mqtt_server, mqtt_port);//inicio como cliente en mqtt
	client.setCallback(callback);//inicio mi conexion con topicos
}

//loop principal
void loop() {
	if (!client.connected()) {
		//si no esta conectado
		reconnect();//lo reconecto
	}
///tiempo de envio de datos //
	client.loop(); //loop de client mqtt
	long now = millis(); // variable de milisengundos
	if (now - lastMsg > 1000){ //si la resta de milisegundos es mayor a 1 segundo
		lastMsg = now; // lo reseteo a 0
///////////////////////////////////////////////
//leo si hay movimiento
 val = digitalRead(PIRPin);
 if (val == HIGH) //si está activado
 {

	 digitalWrite(LEDPin, HIGH); //activo el led
	 delay(2000);//espera 2 seg encendido
	 digitalWrite(LEDPin, LOW);// y se apaga
 }
 else{
    //si no hay movimiento se apaga
 digitalWrite(LEDPin, LOW);
}
//lectura del sensor de gas
int lectura_gas = analogRead(MQ_PIN);
float gas = lectura_gas * (5.0 / 1023.0);
if (gas >= 0.50) {
	digitalWrite(vent, HIGH);//enciende el ventilador
	for(int i=0;i<=3;i++){//suena 3 veces la alarma
		digitalWrite(buzzer,HIGH);
		delay(300);
		digitalWrite(buzzer,LOW);
		delay(300);
	}
		delay(10000);//enciende 10 seg el ventilador
		digitalWrite(vent, LOW);//se apaga
}
    //lectura del sensor de temp
   temp1 = dht.readTemperature();//lee temp
	 temp2 = dht.readHumidity();//lee humedad

	 //los envio como un arreglo [temp],[humedad],[movimiento],[gas]....
		String to_send = String(temp1) + "," + String(temp2) + "," + String(gas);//hay que cambiar los no

		to_send.toCharArray(msg, 25);//envio mensaje a mqtt

		client.publish("values", msg);//lo publico en values , valor 1,2,3,4...
	}
}




//*****************************
//***    CONEXION WIFI      ***
//*****************************
void setup_wifi(){
	delay(10);
	// Nos conectamos a nuestra red Wifi
//	Serial.println();
//	Serial.print("Conectando a ");
//	Serial.println(ssid);

	WiFi.begin(ssid, password);

	while (WiFi.status() != WL_CONNECTED) { //hago que parpadee el led mientras se conecta
    digitalWrite(BUILTIN_LED,HIGH);
	  delay(500);//enciende .5 segundos
		digitalWrite(BUILTIN_LED,LOW);
		delay(500);
//		Serial.print(".");
	}

//	Serial.println("");
//	Serial.println("Conectado a red WiFi!");
//	Serial.println("Dirección IP: ");
//	Serial.println(WiFi.localIP());
}
//escucha los topicos entrantes
//topic,payload,length
void callback(char* topic, byte* payload, unsigned int length){
	String incoming = "";//este es el topico que escucha
	//Serial.print("Mensaje recibido desde -> ");
//	Serial.print(topic);
//	Serial.println("");
	//escucha lo que hay en payload
	for (int i = 0;i<length;i++ ) { //lo convierte letra por letra en incoming
		incoming += (char)payload[i];//caracter por caracter
	}
	incoming.trim();//lo recorta donde tenga espacios ej: [luz][1]
//	Serial.println("Mensaje -> " + incoming);
//puerta abierta
  if (incoming == "open"){
  for(angle=0;angle<=180;angle+=10){
   myservo.write(angle);
  // Serial.println("servo en ");
//   Serial.println(angle);
	 delay(50);
 }
  Serial.println("puerta abierta");
 }
//puerta cerrada
  if (incoming == "close"){
  for(angle = 180;angle>=0;angle-=10){
   myservo.write(angle);
  // Serial.println("servo en ");
  // Serial.println(angle);
	 delay(50);
 }
//   Serial.println("puerta cerrada");
 }
 //cochera abierta
 if (incoming == "abierta"){
 for(angle=0;angle<=180;angle+=5){
	myservo2.write(angle);
///	Serial.println("servo en ");
//	Serial.println(angle);
	delay(50);
}
//	Serial.println("cochera abierta");
}
//cochera cerrada
if (incoming == "cerrada"){
for(angle=180;angle>=0;angle-=5){
 myservo2.write(angle);
 //Serial.println("servo en ");
// Serial.println(angle);
 delay(50);
}
// Serial.println("cochera cerrada");
}
//ventana abierta
if (incoming == "vent_abierta"){
	for(angle=0;angle<=90;angle+=5){
	 myservo3.write(angle);
//	 Serial.println("servo en ");
//	 Serial.println(angle);
	 delay(50);
}
 //Serial.println("ventana abierta");
}
//ventana cerrada
if (incoming == "vent_cerrada"){
	for(angle=90;angle>=0;angle-=5){
	 myservo3.write(angle);
//	 Serial.println("servo en ");
//	 Serial.println(angle);
	 delay(50);
}
// Serial.println("ventana cerrada");
}

//foco1
	if (incoming == "led1_on") {
		digitalWrite(BUILTIN_LED, HIGH);
		digitalWrite(RX,HIGH);
}
 if (incoming == "led1_off") {
		digitalWrite(BUILTIN_LED, LOW);
		digitalWrite(RX, LOW);
	}
//foco2
if (incoming == "led2_on") {
	digitalWrite(TX,HIGH);

}
if (incoming == "led2_off") {
	digitalWrite(TX,LOW);
}
}
void reconnect() {

	while (!client.connected()) {
//		Serial.print("Intentando conexión Mqtt...");
		// Creamos un cliente ID
		String clientId = "esp32_";
		clientId += String(random(0xffff), HEX);
		// Intentamos conectar
		if (client.connect(clientId.c_str(),mqtt_user,mqtt_pass)) {
	//		Serial.println("Conectado!");
			// Nos suscribimos
			client.subscribe("led1");
			client.subscribe("led2");
      client.subscribe("door");
			client.subscribe("ventana");
			client.subscribe("cochera");

//cambios

		} else {
	//		Serial.print("falló :( con error -> ");
	//		Serial.print(client.state());
		//	Serial.println(" Intentamos de nuevo en 5 segundos");

			delay(5000);
		}
	}
}
