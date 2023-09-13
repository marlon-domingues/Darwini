//-----------------BIBLIOTECAS---------------------------
#include <esp_now.h>
#include <WiFi.h>
#include "soc/rtc.h"
#include <math.h>
#include <Arduino.h>
#include <driver/ledc.h>     //Biblioteca para controlar PWM em ESP32
#include "HX711.h"           //biblioteca cpelula de carga
//-----------------PARAMETROS ENCODER--------------------
long int rotValue=0;          //contagem de pulsos do encoder 
uint8_t state=0;              //variavel do estado do encoder
int pulso_rev_enc = 60;       //pulso por revolução encoder 
//------------------SINAIS PLACA-------------------------
#define ROTARY_PINA 35        //Porta de entrada do sinal do encoder
#define ROTARY_PINB 32        //Porta de entrada do sinal do encoder
#define rele  33              //Porta de acionamento do rele de freio
#define bot_g 27              //Botão verde
#define bot_r 26              //Botão vermelho
#define STEP  23              //Pino controla os passos
#define DIR   22              //Pino controla a direção
#define ENA   21              //Pino enable
#define led   2               //Led azul da placa
#define DOUT_PIN 19           // Pino de dados (DOUT) do HX711 
#define SCK_PIN 18            // Pino de clock (SCK) do HX711

HX711 pesagem;                
//-------------VARIAVEIS DE CONTROLE--------------------
long peso=0;
int modo = 1;                 //Modo de operação do módulo
int microstepDelay=400;       //intervalo de tempo entre oz pulsos 
float tempo_percurso=1;       //tesmpo estimado para a finalização de um movimento
float pwmFreq = 400;          // Frequência do PWM em Hz
float fator_freq=6.6;         //relação entre a frequência do motor e do encoder
const int pwmChannel = 0;     // Canal PWM (0-15)
const int pwmResolution = 8;  // Resolução do PWM (bits, 1-16) 
int ciclos=0;                 //contador do numero de ciclos para o modo teste
int i=0;                      //variavel temporária para contagem de ciclos no teste
//---------------------ESPNOW----------------------------
bool estado;
uint8_t mestreAddress[] = {0XB0, 0XA7, 0X32, 0x14, 0XA5, 0X38}; //Endereço mestre
esp_now_peer_info_t peerInfo;                                   //Variavel de pareamento

typedef struct Rdados{        //Estrutura dos dados recebidos pelos escravos
  long int x=0;                 //Quantidade de pulsos que o módulo deve atingir 
};

Rdados myData;                //Variavel do tipo de recebimento dos escravos
//-------------------------------------------------------

//########################################################
//---------------------FUNÇÕES----------------------------
//########################################################

//-------------INTERRUPÇÃO DO ENCODER---------------------
portMUX_TYPE gpioMux = portMUX_INITIALIZER_UNLOCKED;

void IRAM_ATTR isrAB() {             //ao haver um mudança do estado do encoder aciona essa função
   uint8_t s = state & 3;
  portENTER_CRITICAL_ISR(&gpioMux);  //entra em estado crítico para não haver interrupção
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
   portEXIT_CRITICAL_ISR(&gpioMux);  //habilita a interrupção novamente
}

//-------------RECEBIMENTO DOS DADOS POR ESPNOW-------------------

void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&myData, incomingData, sizeof(myData));
  digitalWrite(led,HIGH);               //liga o led onborad
  remoto();
  digitalWrite(led,LOW);               //liga o led onborad
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
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  memcpy(peerInfo.peer_addr, mestreAddress, 6);
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    return;
  }
}

//-------------MODO REMOTO--------------
void remoto(){

  int64_t time_fim = esp_timer_get_time()+tempo_percurso*1000000; //defini o tempo em que o movimento deve terminar

  pwmFreq=fator_freq*abs(myData.x-rotValue)/tempo_percurso;       //calcula um frequencia inicial para o sinal de acionamento
  ledcDetachPin(STEP);                                            //desacopla o pino de acionamento a um canal pwm
  ledcWriteTone(pwmChannel, pwmFreq);                             //define a frequencia do canal
  ledcAttachPin(STEP, pwmChannel);                                //acopla o pino de acionamento a um canal pwm

    
    if (rotValue < myData.x ){            
      digitalWrite(DIR,HIGH);             //seta a direção de giro se for menor
      ledcWrite(pwmChannel, 128);           //liga o pwm em 50% 
      delay(2);                             //delay necessário para o pwm se ajustar
      while(rotValue < myData.x){
        if(time_fim>esp_timer_get_time()){    //se o tempo de termino ainda não tiver passado é recalculado a frequência para atender a condição
          pwmFreq=fator_freq*abs(myData.x-rotValue)/((time_fim-esp_timer_get_time())/1000000);
          if(pwmFreq>660){                    //define uma frequência máxima
            pwmFreq=660;
          }

          ledcDetachPin(STEP);                  //
          ledcWriteTone(pwmChannel, pwmFreq);   //Configura o pwm para  a nova frequencia
          ledcAttachPin(STEP, pwmChannel);      //
          delay(2);                             //delay necessário para o pwm se ajustar
        }
      }
    }
    else if (rotValue > myData.x){
        digitalWrite(DIR,LOW);            //seta a direção de giro de for maior
        ledcWrite(pwmChannel, 128);           //liga o pwm em 50% 
        delay(2);                             //delay necessário para o pwm se ajustar
        while(rotValue > myData.x){
          if(time_fim>esp_timer_get_time()){    //se o tempo de termino ainda não tiver passado é recalculado a frequência para atender a condição
            pwmFreq=fator_freq*abs(myData.x-rotValue)/((time_fim-esp_timer_get_time())/1000000);
            if(pwmFreq>660){                    //define uma frequência máxima
              pwmFreq=660;
            }

            ledcDetachPin(STEP);                  //
            ledcWriteTone(pwmChannel, pwmFreq);   //Configura o pwm para  a nova frequencia
            ledcAttachPin(STEP, pwmChannel);      //
            delay(2);                             //delay necessário para o pwm se ajustar
          }
      }
    }
  

  ledcWrite(pwmChannel, 0); 
}
//----------------------------------------
void corrige(){                           //função que corrige a posição do cbao quando estiver parado 

  if(pwmFreq!=200){
    pwmFreq=200;                            //define uma frequencia para o acionamento do motor
    ledcDetachPin(STEP);                                            //desacopla o pino de acionamento a um canal pwm
    ledcWriteTone(pwmChannel, pwmFreq);                             //define a frequencia do canal
    ledcAttachPin(STEP, pwmChannel);                                //acopla o pino de acionamento a um canal pwm
  }

  if (rotValue < myData.x ){            
    digitalWrite(DIR,HIGH);             //seta a direção de giro se for menor

    while(rotValue < myData.x ){            //permanece na condição até igualar a meta
      ledcWrite(pwmChannel, 128);           //liga o pwm em 50% 
    }
  }
  else if (rotValue > myData.x){
    digitalWrite(DIR,LOW);            //seta a direção de giro de for maior

    while(rotValue > myData.x ){            //permanece na condição até igualar a meta
      ledcWrite(pwmChannel, 128);           //liga o pwm em 50% 
    }
  }
  ledcWrite(pwmChannel, 0);           //liga o pwm em 50% 
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
  if(digitalRead(bot_g) == LOW && digitalRead(bot_r) == HIGH){  //caso os dois botoes estejam apertados ao mesmo tempo
    switch(modo){                                               //é alterado o modo de operação da máquina
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
  if (digitalRead(bot_g) == HIGH && digitalRead(bot_r) == HIGH){    //com o botão vermelho apertado gira para um lado
    digitalWrite(DIR,HIGH); 
    pulso();
   }
  else if (digitalRead(bot_g) == LOW && digitalRead(bot_r) == LOW){ //com o botão verde apertado gira para um lado
    digitalWrite(DIR,LOW); 
    pulso();
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

  pinMode(STEP, OUTPUT);                         //
  pinMode(DIR, OUTPUT);                          //
  pinMode(bot_g,INPUT_PULLUP);                   //
  pinMode(bot_r,INPUT_PULLUP);                   //
  pinMode(led,OUTPUT);                           //Define os pinos de entrada e sáida
  pinMode(ROTARY_PINA, INPUT_PULLUP);            //
  pinMode(ROTARY_PINB, INPUT_PULLUP);            //
  attachInterrupt(ROTARY_PINA, isrAB, CHANGE);   //
  attachInterrupt(ROTARY_PINB, isrAB, CHANGE);   //
  pinMode (rele, OUTPUT);

  Serial.begin(115200);                          //ativa a comunicação serial
  ativar_ESPNOW();                               //Ativa o ESPNOW
  esp_timer_init();                              //ativa a contagem de tempo da ESP

  pesagem.begin(DOUT_PIN, SCK_PIN);              //inicializa a balança

  digitalWrite(rele, HIGH);                      //aciona o rele liberando o giro do motor

  delay(1000);                                   //tempo para acionamento do rele
  
  ledcSetup(pwmChannel, pwmFreq, pwmResolution);
  ledcAttachPin(STEP, pwmChannel);
  ledcWrite(pwmChannel, 0);
}



void loop() {
  if(digitalRead(bot_g) == LOW && digitalRead(bot_r) == HIGH){
    opera();
  }

  Serial.println(modo);

  switch (modo){
    case 0:
      ledcDetachPin(STEP); 
      manual();
      break;
    case 1:
      corrige();
      break;
  }
}

/*
    case 2:                         //condição apenas para teste onde o módulo fica ciclando

      Serial.println(i);
      i++;
      digitalWrite(led,HIGH);
      while(rotValue>-240){
        digitalWrite(DIR,LOW); 
        pulso();
      }
      delay(500);
      digitalWrite(led,LOW);
      while(rotValue<-30){
        digitalWrite(DIR,HIGH); 
        pulso();
      }
      delay(500);
      break;
  */
