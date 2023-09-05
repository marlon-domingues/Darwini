//------------------BIBLIOTECAS---------------------------
#include <esp_now.h>
#include <WiFi.h>
//----------------PARAMETROS COMUNICAÇÃO------------------
bool pronto[4] = {true, true, true, true};  //Confirmação de funcionamento dos módulos
//----------------------ESPNOW----------------------------
uint8_t moduloAddress0[] = {0x08, 0xB6, 0x1F, 0x2A, 0xF5, 0xC0};  //Endereço dos módulos
uint8_t moduloAddress1[] = {0xD4, 0xD4, 0xDA, 0x5D, 0x50, 0x8C}; 
uint8_t moduloAddress2[] = {0xD4, 0xD4, 0xDA, 0x5C, 0xFF, 0x84};
uint8_t moduloAddress3[] = {0xD4, 0xD4, 0xDA, 0x59, 0xD8, 0xC8};
esp_now_peer_info_t peerInfo;                                     //Variavel de pareamento

typedef struct Edados{    //Estrutura dos dados enviados pela mestre
  int x;                  //Quantidade de pulsos que o módulo deve atingir 
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
  pronto[myData.id] = myData.fim;
  printar();
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

//--------------COMUNICAÇÃO PYTHON------------------

void printar(){
  Serial.println("Dados:");
  Serial.print(pronto[0]);
  Serial.print(";");
  Serial.print(pronto[1]);
  Serial.print(";");
  pronto[2]=true;
  Serial.print(pronto[2]);
  Serial.print(";");
  pronto[3]=true;
  Serial.println(pronto[3]);
}

//-----------------ENVAR DADOS---------------------

int env(){
  if (pronto[0] && pronto[1]){
    esp_err_t result = esp_now_send(0, (uint8_t *) &mot, sizeof(Edados));
  }
}
//--------------------------------------------------
void setup() {
  Serial.begin(115200);
  ativar_ESPNOW();  //Ativa o ESPNOW
  mot.x = 0;        //Posição inicial dos módulos
}

void loop() {
  if(Serial.available()){
    mot.x = Serial.parseInt();
    Serial.readStringUntil('\n');
    env();
  }
}
