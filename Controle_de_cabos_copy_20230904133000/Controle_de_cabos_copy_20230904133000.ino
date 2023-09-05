//-----------------BIBLIOTECAS---------------------------
#include <esp_now.h>
#include <WiFi.h>
#include "soc/rtc.h"
#include <math.h>
#include <Arduino.h>
#include <driver/ledc.h> // Biblioteca para controlar PWM em ESP32
//-----------------PARAMETROS ENCODER--------------------
long int rotValue=0;      //contagem de pulsos do encoder 
uint8_t state=0;          //variavel do estado do encoder
int pulso_rev_enc = 60;   //pulso por revolução encoder 
//------------------SINAIS PLACA-------------------------
#define ROTARY_PINA 35    //Porta de entrada do sinal do encoder
#define ROTARY_PINB 32    //Porta de entrada do sinal do encoder
#define rele  33          //Porta de acionamento do rele de freio
#define bot_g 27          //Botão verde
#define bot_r 26          //Botão vermelho
#define STEP  23          //Pino controla os passos
#define DIR   22          //Pino controla a direção
#define ENA   21          //Pino enable
#define led   2           //Led azul da placa
//-------------VARIAVEIS DE CONTROLE--------------------
int modo = 1;             //Modo de operação do módulo
int microstepDelay=600;
float tempo_percurso=0.5;
float pwmFreq = 400;     // Frequência do PWM em Hz
float fator_freq=6.6;
const int pwmChannel = 0;     // Canal PWM (0-15)
const int pwmResolution = 8;  // Resolução do PWM (bits, 1-16) 
//---------------------ESPNOW----------------------------
bool estado;                                                    //estado de envio do dado  ESPNOW
uint8_t mestreAddress[] = {0xC4, 0xDE, 0xE2, 0x19, 0x67, 0xB4}; //Endereço mestre
esp_now_peer_info_t peerInfo;                                   //Variavel de pareamento

typedef struct Rdados{    //Estrutura dos dados recebidos pelos escravos
  int x;                  //Quantidade de pulsos que o módulo deve atingir 
};

typedef struct Edados{    //Estrutura dos dados enviados pelos escravos
  bool fim;               //Flag informando a finalização do movimento
  int id;                 //Identificação do módulo 
};

Edados conf;              //Variavel do tipo de envio dos escravos
Rdados myData;            //Variavel do tipo de recebimento dos escravos
//-------------------------------------------------------

//########################################################
//---------------------FUNÇÕES----------------------------
//########################################################

//-------------INTERRUPÇÃO DO ENCODER---------------------
portMUX_TYPE gpioMux = portMUX_INITIALIZER_UNLOCKED;

void IRAM_ATTR isrAB() {
   uint8_t s = state & 3;
  portENTER_CRITICAL_ISR(&gpioMux);
    if (digitalRead(ROTARY_PINA)) {
      s |= 4;
      }
    if (digitalRead(ROTARY_PINB)) {
      s |= 8;
      }
    
    switch (s) {
      case 0: case 5: case 10: case 15:
        break;
      case 1: case 7: case 8: case 14:
        rotValue--; break;
      case 2: case 4: case 11: case 13:
        rotValue++; break;
      case 3: case 12:
        rotValue -= 2; break;
      default:
        rotValue += 2; break;
    }
   state = (s >> 2);
   portEXIT_CRITICAL_ISR(&gpioMux);
}

//-------------ENVIO DOS DADOS POR ESPNOW-------------------

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status){
  char macStr[18];

  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);

  if(status == ESP_NOW_SEND_SUCCESS){
    estado = true;
  }
  else{
    estado = false;
    esp_err_t result1 = esp_now_send(mestreAddress, (uint8_t *) &conf, sizeof(Edados));
  }
}

//-------------RECEBIMENTO DOS DADOS POR ESPNOW-------------------

void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&myData, incomingData, sizeof(myData));
  remoto();

}

//-------------ATIVA ESPNOW-------------------

void ativar_ESPNOW() {
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    return;
  }
  setup_peer();
  esp_now_register_recv_cb(OnDataRecv);
}

//-------------REALIZA PAREAMENTO--------------

void setup_peer(){
  esp_now_register_send_cb(OnDataSent);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  memcpy(peerInfo.peer_addr, mestreAddress, 6);
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    return;
  }
}

//-------------MODO REMOTO--------------
void remoto(){

  int64_t time_fim = esp_timer_get_time()+tempo_percurso*1000000;

  pwmFreq=fator_freq*abs(myData.x-rotValue)/tempo_percurso;
  ledcDetachPin(STEP);
  ledcWriteTone(pwmChannel, pwmFreq);
  ledcAttachPin(STEP, pwmChannel);

  while(rotValue != myData.x){
    ledcWrite(pwmChannel, 128);
    if (rotValue < myData.x ){
      digitalWrite(DIR,HIGH);
    }
    else if (rotValue > myData.x){
        digitalWrite(DIR,LOW);
    }
    delay(2);

    if(time_fim>esp_timer_get_time()){
      pwmFreq=fator_freq*abs(myData.x-rotValue)/((time_fim-esp_timer_get_time())/1000000);
      if(pwmFreq>660){
        pwmFreq=660;
      }
    }

    ledcDetachPin(STEP);
    ledcWriteTone(pwmChannel, pwmFreq);
    ledcAttachPin(STEP, pwmChannel);
  }
  ledcDetachPin(STEP);
}
//----------------------------------------
void corrige(){

  pwmFreq=350;
  ledcDetachPin(STEP);
  ledcWriteTone(pwmChannel, pwmFreq);
  ledcAttachPin(STEP, pwmChannel);

  while(rotValue != myData.x){
    ledcWrite(pwmChannel, 128);
    if (rotValue < myData.x ){
      digitalWrite(DIR,HIGH);
    }
    else if (rotValue > myData.x){
        digitalWrite(DIR,LOW);
    }
    delay(2);
  }
  ledcDetachPin(STEP);
}

//----------------------------------------
void pulso(){
  digitalWrite(STEP, LOW);
  delayMicroseconds(microstepDelay);
  digitalWrite(STEP, HIGH);
  delayMicroseconds(microstepDelay);

}

//-------------SELEÇÃO DO MODO DE OPERAÇÃO-------------------

void opera(){
  delay(500);
  if(digitalRead(bot_g) == LOW && digitalRead(bot_r) == HIGH){
    switch(modo){
      case 0:
        myData.x=rotValue;
        modo = 1;
        digitalWrite(led,LOW);
        break;
      case 1:
        myData.x=rotValue;
        modo = 0;
        digitalWrite(led,HIGH);
        break;
    }
  }
}
//-------------MODO MANUAL--------------

void manual(){
  if (digitalRead(bot_g) == HIGH && digitalRead(bot_r) == HIGH){
    digitalWrite(DIR,HIGH); 
    pulso_manual();
   }
  else if (digitalRead(bot_g) == LOW && digitalRead(bot_r) == LOW){
    digitalWrite(DIR,LOW); 
    pulso_manual();
  }
}
//-------------------ROTAÇÃO DO MOTOR MANUAL-----------------------

int pulso_manual(){
  while((digitalRead(bot_g) == HIGH && digitalRead(bot_r) == HIGH) || (digitalRead(bot_g) == LOW && digitalRead(bot_r) == LOW)){
    pulso();
  } 
}
//----------------------------------------
void setup() {
  Serial.begin(115200);
  ativar_ESPNOW();                       //Ativa o ESPNOW
  esp_timer_init();                      //ativa a contagem de tempo da ESP
  conf.fim = true;                       //Inicializa a flag de termino de movimento
  conf.id = 2;                           //Numero de identificação do módulo

  pinMode(STEP, OUTPUT);
  pinMode(DIR, OUTPUT);
  pinMode(bot_g,INPUT_PULLUP);
  pinMode(bot_r,INPUT_PULLUP);
  pinMode(led,OUTPUT);
  pinMode(ROTARY_PINA, INPUT_PULLUP);
  pinMode(ROTARY_PINB, INPUT_PULLUP);
  attachInterrupt(ROTARY_PINA, isrAB, CHANGE);
  attachInterrupt(ROTARY_PINB, isrAB, CHANGE);
  pinMode (rele, OUTPUT);
  digitalWrite(rele, HIGH);
  delay(1000);                           //tempo para acionamento do rele
  ledcSetup(pwmChannel, pwmFreq, pwmResolution);
  ledcAttachPin(STEP, pwmChannel);
  ledcWrite(pwmChannel, 0);
}



void loop() {
  if(digitalRead(bot_g) == LOW && digitalRead(bot_r) == HIGH){
    opera();
  }
  switch (modo){
    case 0:
      manual();
      break;
    case 1:
      corrige();
      break;
  }
}
