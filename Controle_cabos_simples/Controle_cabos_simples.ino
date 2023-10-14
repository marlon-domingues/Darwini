//-----------------BIBLIOTECAS---------------------------
#include <esp_now.h>
#include <WiFi.h>
#include "soc/rtc.h"
#include <math.h>
#include <Arduino.h>
#include <driver/ledc.h>     //Biblioteca para controlar PWM em ESP32

//-----------------PARAMETROS ENCODER--------------------
long int rotValue=0;          //contagem de pulsos do encoder 
uint8_t state=0;              //variavel do estado do encoder

//-------------VARIAVEIS DE CONTROLE--------------------
int modo = 1;                 //Modo de operação do módulo
float pwmFreq = 400;          // Frequência do PWM que é iniciado 
float pwmFreq_lim = 1800;     //frequencia máxima do pwm 
float coeficiente_freq=5;     //relaciona o erro com a velocidade de roração
float pwmFreq_remoto = 400;

#define  pwmChannel 0         // Canal PWM (0-15)
#define  pwmFreq_manual 600   //frequencia do pulso no modo manual 
#define  pwmFreq_balanca 100  //frequencia de correção da balança
#define  pwmResolution 8      // Resolução do PWM (bits, 1-16) 

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

}

//-------------RECEBIMENTO DOS DADOS POR ESPNOW-------------------

void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&mot, incomingData, sizeof(mot));
  pwmFreq_remoto=abs(mot.x-rotValue)*coeficiente_freq;
  if(pwmFreq>pwmFreq_lim){
    pwmFreq=pwmFreq_lim;
  }
  
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
  while(digitalRead(bot_g) == HIGH && digitalRead(bot_r) == HIGH || digitalRead(bot_g) == LOW && digitalRead(bot_r) == LOW ){
      if (digitalRead(bot_g) == HIGH && digitalRead(bot_r) == HIGH){    //com o botão vermelho apertado gira para um lado
      digitalWrite(DIR,HIGH); 
      }
      else if (digitalRead(bot_g) == LOW && digitalRead(bot_r) == LOW){ //com o botão verde apertado gira para um lado
      digitalWrite(DIR,LOW); 
      }
      
      if(pwmFreq!=pwmFreq_manual){
        pwmFreq=pwmFreq_manual;
        ledcDetachPin(STEP);                         //
        ledcWriteTone(pwmChannel, pwmFreq);   //Configura o pwm para  a nova frequencia
        ledcAttachPin(STEP, pwmChannel);             //
      }

      ledcWrite(pwmChannel, 128);
      delay(10);
  }
  //ledcWrite(pwmChannel, 0);
  mot.x=rotValue;            //iguala a posição atual com a desejada para que a ferramenta não se movimente após sair do modo manual 
  delay(10);
}

//-------------CORRIGE A POSIÇÃO--------------

void Freq_PID(){
  current_time=esp_timer_get_time()/1000000;
  T=current_time-pass_time;
  if(contador_PID<3){
    erro[contador_PID]=mot.x-rotValue;
    contador_PID++;
    pwmFreq=200;
  }
   else{
    erro[0]=erro[1];
    erro[1]=erro[2];
    erro[2]=mot.x-rotValue;
    pwmFreq= pwmFreq + (kp+kd/T)*erro[2]+(-kp-2*kd/T+ki*T)*erro[1]+(kd/T)*erro[0];

    if(pwmFreq>2000){
      pwmFreq=2000;
    }
  }


  
  pass_time=esp_timer_get_time()/1000000;
}

//-------------CORRIGE A POSIÇÃO--------------

void corrige(){
  if(rotValue!=mot.x){                       //confere se precisa se movimentar 
    if(rotValue<mot.x){
      digitalWrite(DIR,HIGH);
    }
    else if(rotValue>mot.x){
      digitalWrite(DIR,LOW);
    }

  if(pwmFreq!=pwmFreq_remoto){
    pwmFreq=pwmFreq_remoto;
    ledcDetachPin(STEP);                         //
    ledcWriteTone(pwmChannel, pwmFreq);   //Configura o pwm para  a nova frequencia
    ledcAttachPin(STEP, pwmChannel);             //
  }

    ledcWrite(pwmChannel, 128);
    delay(10);
  }
  else{
    ledcWrite(pwmChannel, 0);
    delay(10);
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

  digitalWrite(rele, HIGH);                      //aciona o rele liberando o giro do motor

  delay(1000);                                   //tempo para acionamento do rele
  
  ledcSetup(pwmChannel, pwmFreq, pwmResolution);
  ledcAttachPin(STEP, pwmChannel);
  ledcWrite(pwmChannel, 0);
  delay(10);
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