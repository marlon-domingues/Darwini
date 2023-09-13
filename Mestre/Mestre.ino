//------------------BIBLIOTECAS---------------------------
#include <esp_now.h>
#include <WiFi.h>
//----------------PARAMETROS COMUNICAÇÃO------------------
uint8_t moduloAddress0[] = {0xC4, 0xDE, 0xE2, 0x19, 0x67, 0xB4};  //Endereço dos módulos
uint8_t moduloAddress1[] = {0xD4, 0xD4, 0xDA, 0x59, 0xD8, 0xC8}; 
uint8_t moduloAddress2[] = {0xCC, 0xDB, 0xA7, 0x68, 0x55, 0xB0};
uint8_t moduloAddress3[] = {0xD4, 0xD4, 0xDA, 0x5D, 0x50, 0x8C};
esp_now_peer_info_t peerInfo;                                     //Variavel de pareamento

typedef struct Edados{    //Estrutura dos dados enviados pela mestre
  long int x=0;                  //Quantidade de pulsos que o módulo deve atingir 
};

typedef struct Rdados{    //Estrutura dos dados recebidos pela mestre
  bool fim;               //Flag informando a finalização do movimento
  int id;                 //Identificação do módulo 
} Rdados;

Edados mot;               //Variavel do tipo de envio da mestre
Rdados myData;            //Variavel do tipo de recebimento da mestre
//--------------------------------------------------------

//########################################################
//---------------------FUNÇÕES----------------------------
//########################################################

//-------------ENVIO DOS DADOS POR ESPNOW-----------------

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status){
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  //*
  Serial.print(macStr);
  Serial.print(" send status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Sucesso no envio" : "Falha no envio");
  //*/
}

//-------------RECEBIMENTO DOS DADOS POR ESPNOW-------------------

void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&myData, incomingData, sizeof(myData));
  
}

//------------------REALIZA PAREAMENTO--------------------

void setup_peer(){
  esp_now_register_send_cb(OnDataSent);

  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  //registrar pareamento 0 
  memcpy(peerInfo.peer_addr, moduloAddress0, 6);
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Falha pareamento 0");
    return;
  }
  //registrar pareamento 1   
  memcpy(peerInfo.peer_addr, moduloAddress1, 6);
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Falha pareamento 1");
    return;
  }
  //registrar pareamento 2   
  memcpy(peerInfo.peer_addr, moduloAddress2, 6);
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Falha pareamento 2");
    return;
  }
  //registrar pareamento 3
  memcpy(peerInfo.peer_addr, moduloAddress3, 6);
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Falha pareamento 3");
    return;
  }
}

//-----------------ATIVA ESPNOW---------------------

void ativar_ESPNOW() {
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    return;
  }
  setup_peer();
  esp_now_register_recv_cb(OnDataRecv);
}

//-----------------ENVAR DADOS---------------------

void env(){
    esp_err_t result = esp_now_send(0, (uint8_t *) &mot, sizeof(Edados));
}
//--------------------------------------------------
void setup() {
  Serial.begin(115200);
  ativar_ESPNOW();  //Ativa o ESPNOW
  mot.x = 0;        //Posição inicial dos módulos
  esp_timer_init();                              //ativa a contagem de tempo da ESP
}

int sentido=-1;
int64_t  current_time=0;
int atualiza=0;

void loop() {
  if(Serial.available()){
    mot.x = Serial.parseInt();
    Serial.readStringUntil('\n');
    env();
  }
  /*
  if(atualiza==0){
    mot.x= mot.x+sentido*201;
    Serial.println(mot.x);
    atualiza=1;
  }

  
  if( esp_timer_get_time()-current_time>1*1000000){
    env();
    current_time=esp_timer_get_time();
    if(mot.x<-200){
      sentido=1;
    }
    else if(mot.x>-500){
      sentido=-1;
    }
    atualiza=0;
  }*/

}
