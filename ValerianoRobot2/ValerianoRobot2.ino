#include "WiFi.h"
#include "AsyncUDP.h"

const char* ssid = "TP-Link_9898";
const char* password = "raspberry";

AsyncUDP udp;

const int Moto1_f = 23;
const int Moto1_r = 22;
const int Moto2_f = 19;
const int Moto2_r = 21;

const int setaDireita  = 4;
const int setaEsquerda = 5;


const int led  = LED_BUILTIN;

void stopMotor(){
  ledcWrite(0, 0);
  ledcWrite(1, 0);
  ledcWrite(2, 0);
  ledcWrite(3, 0);
}

void setMotor(int ms1, int ms2,int mv1 , int mv2) {
  int pin;
  
  pin = (ms1 == 0) ? 0 : 1;
  ledcWrite(pin, mv1);
  
  pin = (ms2 == 0) ? 2 : 3;
  ledcWrite(pin, mv2);

  digitalWrite(setaDireita, ms1);
  digitalWrite(setaEsquerda,ms2);
  
}




void setup()
{

  pinMode(Moto1_f, OUTPUT);
  pinMode(Moto1_r, OUTPUT);
  pinMode(Moto2_f, OUTPUT);
  pinMode(Moto2_r , OUTPUT);
  pinMode(setaDireita, OUTPUT);
  pinMode(setaEsquerda , OUTPUT);
  pinMode(led, OUTPUT);
  
  digitalWrite(led, 0);
  
  const int freq = 5000;
  const int ledChannel = 0;
  const int resolution = 8;
  
  ledcSetup(0,freq, resolution);//Atribuimos ao canal 0 a frequencia de 1000Hz com resolucao de 10bits.
  ledcAttachPin(Moto1_f, 0);//Atribuimos o pino 23 ao canal 0.
  ledcWrite(0, 0);//Escrevemos um duty cycle de 25% no canal 0.
  
  ledcSetup(1, freq,resolution);//Atribuimos ao canal 1 a frequencia de 1000Hz com resolucao de 10bits.
  ledcAttachPin(Moto1_r, 1);//Atribuimos o pino 22 ao canal 1.
  ledcWrite(1, 0);
  
  
  ledcSetup(2,freq, resolution);//Atribuimos ao canal 2 a frequencia de 1000Hz com resolucao de 10bits.
  ledcAttachPin(Moto2_f, 2);//Atribuimos o pino 21 ao canal 2.
  ledcWrite(2, 0);
  
  
  ledcSetup(3,freq,resolution);//Atribuimos ao canal 3 a frequencia de 1000Hz com resolucao de 10bits.
  ledcAttachPin(Moto2_r, 3);//Atribuimos o pino 19 ao canal 3.
  ledcWrite(3, 0);
  
  //Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  //Serial.println("");

  // Wait for connection
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    //Serial.print(".");
  }
  //Serial.println("");
  //Serial.print("Connected to ");
  //Serial.println(ssid);
  //Serial.print("IP address: ");
  //Serial.println(WiFi.localIP());
  
    if(udp.listen(1234)) {
        //Serial.print("UDP Listening on IP: ");
        //Serial.println(WiFi.localIP());
        udp.onPacket([](AsyncUDPPacket packet) {
            digitalWrite(led, 1);
            String str = (char*) packet.data();
            stopMotor();
            delay(150);
            //Serial.println(str);
            if (str.length()== 8){
              int m1s = str.substring(0,1).toInt();
              int m2s = str.substring(1,2).toInt();
              int m1v = str.substring(2,5).toInt();
              int m2v = str.substring(5,8).toInt();
              
              setMotor(m1s,m2s,m1v,m2v);
            }
            digitalWrite(led, (str.length()== 8) ? 0 : 1 );
            packet.printf("Got %u bytes of data", packet.length());
        });
    }
}

void loop()
{
    delay(1000);
    //Send broadcast
    udp.broadcast("Anyone here?");
}
