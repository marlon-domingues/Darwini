//-----------------BIBLIOTECAS---------------------------
#include <esp_now.h>
#include <WiFi.h>
//-------------VARIAVEIS DE CONTROLE--------------------
int num = 0;
int dir =1;
bool pronto[4] = {true, true, true, true};
//---------------------ESPNOW----------------------------
esp_now_peer_info_t peerInfo;

uint8_t broadcastAddress1[] = {0x08, 0xB6, 0x1F, 0x2A, 0xF5, 0xC0};  
uint8_t broadcastAddress2[] = {0xD4, 0xD4, 0xDA, 0x5D, 0x50, 0x8C}; 
uint8_t broadcastAddress3[] = {0xD4, 0xD4, 0xDA, 0x5C, 0xFF, 0x84};
uint8_t broadcastAddress4[] = {0xD4, 0xD4, 0xDA, 0x59, 0xD8, 0xC8};

typedef struct Edados{
  float x;
  float y;
};

typedef struct Rdados{
  bool fim;
  int id;
} Rdados;

Edados mot;
Rdados myData;
//-------------------------------------------------------

//########################################################
//---------------------FUNÇÕES----------------------------
//########################################################

//-----------ENVIO DOS DADOS POR ESPNOW-------------------
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status){
  char macStr[18];
  //Serial.print("Enviado para: ");
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  //*
  Serial.print(macStr);
  Serial.print(" send status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Sucesso no envio" : "Falha no envio");
  //*/
}

//----------------REALIZA PAREAMENTO---------------------

void setup_peer(){
  esp_now_register_send_cb(OnDataSent);

  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  //registrar pareamento 1 
  memcpy(peerInfo.peer_addr, broadcastAddress1, 6);
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Falha pareamento 1");
    return;
  }
  //registrar pareamento 2   
  memcpy(peerInfo.peer_addr, broadcastAddress2, 6);
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Falha pareamento 2");
    return;
  }
  //registrar pareamento 3   
  memcpy(peerInfo.peer_addr, broadcastAddress3, 6);
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Falha pareamento 3");
    return;
  }
  //registrar pareamento 4
  memcpy(peerInfo.peer_addr, broadcastAddress4, 6);
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Falha pareamento 4");
    return;
  }
}

//-------------RECEBIMENTO DOS DADOS POR ESPNOW-------------------

void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&myData, incomingData, sizeof(myData));
  Serial.print("Estado: ");
  Serial.print(myData.fim);
  Serial.print(" Modulo: ");
  Serial.println(myData.id);
  pronto[myData.id] = myData.fim;
}

//-------------ENVIA OS DADOS-----------------------------------

int cont(){
  esp_err_t result = esp_now_send(0, (uint8_t *) &mot, sizeof(Edados));
  memset(pronto, false, sizeof(pronto));
}

//---------------------------------------------------------------

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  setup_peer();
  esp_now_register_recv_cb(OnDataRecv);
  mot.x == 0;
}

void loop() {
  if(pronto[0] && pronto[1] ){
    mot.x = -50;
    cont();
  }
  if(pronto[0] && pronto[1] ){
    mot.x = -10;
    cont();
  }
}
