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
//-------------VARIAVEIS DE CONTROLE--------------------
bool flag1=false;                   //variavel para função manual 
bool flag2=false;
bool flag3=false;
int modo = 1;                 //Modo de operação do módulo
float peso=0;
float pwmFreq = 400;        // Frequência do PWM em Hz
float fator_freq=6.6;       //relação entre a frequência do motor e do encoder
#define  pwmChannel 0       // Canal PWM (0-15)
#define  pwmFreq_manual 600 //frequencia do pulso no modo manual 
#define  pwmResolution 8    // Resolução do PWM (bits, 1-16) 
#define balanca_min 0.2
#define balanca_max 10
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
//---------------------ESPNOW----------------------------
uint8_t mestreAddress[] = {0XB0, 0XA7, 0X32, 0x14, 0XA5, 0X38}; //Endereço mestre
esp_now_peer_info_t peerInfo;                                   //Variavel de pareamento

typedef struct Rdados{        //Estrutura dos dados recebidos pelos escravos
  long int x=0;                 //Quantidade de pulsos que o módulo deve atingir 
};

Rdados mot;                //Variavel do tipo de recebimento dos escravos
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

   if(!flag2){
    if(flag3){
      if(rotValue<=mot.x){
        ledcWrite(pwmChannel, 0);
      }
    }
    else{
      if(rotValue>=mot.x){
        ledcWrite(pwmChannel, 0);
      }
    }
   }
}

//-------------RECEBIMENTO DOS DADOS POR ESPNOW-------------------

void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&mot, incomingData, sizeof(mot));
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


//-------------SELEÇÃO DO MODO DE OPERAÇÃO-------------------

void opera(){
  delay(500);
  if(digitalRead(bot_g) == LOW && digitalRead(bot_r) == HIGH){  //caso os dois botoes estejam apertados ao mesmo tempo
    switch(modo){                                               //é alterado o modo de operação da máquina
      case 0:
        modo = 1;
        digitalWrite(led,LOW);
        break;
      case 1:
        modo = 0;
        digitalWrite(led,HIGH);
        break;
    }
  }
}
//-------------MODO MANUAL--------------

void manual(){
  flag1=false;
  
  while(digitalRead(bot_g) == HIGH && digitalRead(bot_r) == HIGH || digitalRead(bot_g) == LOW && digitalRead(bot_r) == LOW ){
    if(flag1==false){
      if (digitalRead(bot_g) == HIGH && digitalRead(bot_r) == HIGH){    //com o botão vermelho apertado gira para um lado
      digitalWrite(DIR,HIGH); 
      }
      else if (digitalRead(bot_g) == LOW && digitalRead(bot_r) == LOW){ //com o botão verde apertado gira para um lado
      digitalWrite(DIR,LOW); 
      }
      
      ledcDetachPin(STEP);                         //
      ledcWriteTone(pwmChannel, pwmFreq_manual);   //Configura o pwm para  a nova frequencia
      ledcAttachPin(STEP, pwmChannel);             //
      ledcWrite(pwmChannel, 128);
      flag1=true;
    }
  }
  ledcWrite(pwmChannel, 0);
  flag1=false;
  flag2=true;
}

//---------------TASK DA PESAGEM--------------------

void calc_peso(void *parameter) {
  while(true){
    peso = pesagem.get_units(3);
  }
}

//-------------CORREÇÃO DA BALANÇA-------------------

bool balanca(){
  peso = 5;        // Lê o peso da célula de carga em unidades definidas
  if (peso<=balanca_min || peso>=balanca_max){
    if(peso<=balanca_min){
      digitalWrite(DIR,LOW);
    }
    else if(peso>=balanca_max){
      digitalWrite(DIR,HIGH);
    }
  
    ledcDetachPin(STEP);                         //
    ledcWriteTone(pwmChannel, pwmFreq_manual);   //Configura o pwm para  a nova frequencia
    ledcAttachPin(STEP, pwmChannel);             //
    ledcWrite(pwmChannel, 128);
    
    flag2=true;

    return false;
  }
  else{
    return true;
  }
}

 //-------------CORRIGE A POSIÇÃO--------------

void corrige(){
  if (balanca()){
    if(flag2){
      mot.x=rotValue;
      ledcWrite(pwmChannel, 0);
      flag2 = false;
    }
    if(rotValue!=mot.x){
      if(rotValue<mot.x){
        digitalWrite(DIR,HIGH);
        flag3=true;
      }
      else if(rotValue>mot.x){
        digitalWrite(DIR,LOW);
        flag3=false;
      }
      ledcWrite(pwmChannel, 128);
    }
  }
}

//---------------SETUP--------------------------
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
  pesagem.set_scale(120653.731);                 //Calibragem da balança
  pesagem.tare();                                //Zera a balança no tare da calibragem
  xTaskCreatePinnedToCore(calc_peso, "Calculo do peso", 10000, NULL, 1, NULL, 0); // Cria a tarefa na core 0

  digitalWrite(rele, HIGH);                      //aciona o rele liberando o giro do motor

  delay(1000);                                   //tempo para acionamento do rele
  
  ledcSetup(pwmChannel, pwmFreq, pwmResolution);
  ledcAttachPin(STEP, pwmChannel);
  ledcWrite(pwmChannel, 0);
}


//---------------LOOP--------------------------
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
//---------------FIM--------------------------