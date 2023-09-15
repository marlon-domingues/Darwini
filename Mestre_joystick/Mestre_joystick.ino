//------------------BIBLIOTECAS---------------------------
#include <esp_now.h>
#include <WiFi.h>
//-----------------------JOYSTICK-------------------------
#define pino_x 33
#define pino_y 32
#define altura  600
#define largura 400
bool pronto[4] = {true, true, true, true};  //Confirmação de funcionamento dos módulos
int comprimento[4] = {0, 0, 0, 0};
float coord_x = 0;
float coord_y = 0;
const int coord_modulos[4][2] = {
  {0, 0},
  {largura, 0},
  {0, altura},
  {largura, altura}
};
//----------------------ESPNOW----------------------------
uint8_t moduloAddresses[4][6] = {            //Endereço dos módulos
  //{0x08, 0xB6, 0x1F, 0x2A, 0xF5, 0xC0},
  //{0x24, 0xD7, 0xEB, 0x10, 0x52, 0x78},
  {0xC4, 0xDE, 0xE2, 0x19, 0x67, 0xB4},
  {0xD4, 0xD4, 0xDA, 0x59, 0xD8, 0xC8},
  {0xCC, 0xDB, 0xA7, 0x68, 0x55, 0xB0},
  {0xD4, 0xD4, 0xDA, 0x5D, 0x50, 0x8C}
};

esp_now_peer_info_t peerInfo;                                     //Variavel de pareamento

typedef struct Edados{    //Estrutura dos dados enviados pela mestre
  int rotValue;                  //Quantidade de pulsos que o módulo deve atingir 
};

typedef struct Rdados{    //Estrutura dos dados recebidos pela mestre
  bool fim;               //Flag informando a finalização do movimento
  int pos;
  int id;                 //Identificação do módulo 
} Rdados;

Edados mot;               //Variavel do tipo de envio da mestre
Rdados myData;            //Variavel do tipo de recebimento da mestre

//-------------ENVIO DOS DADOS POR ESPNOW-----------------

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status){
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);

  if(status == ESP_NOW_SEND_SUCCESS){
    //pronto[id] = true;
  }
  else{
    //pronto[id] = false;
  }
}

//--------------------------------------------------------

//########################################################
//---------------------FUNÇÕES----------------------------
//########################################################

//
void calculo_comprimento(){
  for (int i = 0; i < 4; i++){
    //mot.rotValue = round(sqrt(pow(coord_modulos[i][1] - coord_x,2)+pow(coord_modulos[i][2] - coord_y,2)));
    mot.rotValue = round(sqrt(pow(coord_x,2)+pow(coord_y,2)));
    //comprimento[i] = mot.rotValue;
    //Serial.println(mot.rotValue);
    esp_err_t result = esp_now_send(moduloAddresses[i], (uint8_t *) &mot, sizeof(Edados));
  }
}

//-------------INTERRUPÇÃO DO TEMPO---------------------
esp_timer_handle_t timerHandle;

void IRAM_ATTR env_pos(void* arg) {
  int val_x = map(analogRead(pino_x),0,4095,100,-100);
  int val_y = map(analogRead(pino_y),0,4095,100,-100);


  if(abs(val_x) > 20 || abs(val_y) > 200){
    //int modulo = sqrt(pow(val_x,2)+pow(val_y,2));

    coord_x += val_x*10/abs(val_x);
    //coord_y += val_y*10/abs(val_y);
    
    calculo_comprimento();
  }
    
  else{
    esp_timer_stop(timerHandle);
  }
}

esp_timer_create_args_t timerArgs = {.callback = &env_pos,.name = "Enviar"};

//--------------COMUNICAÇÃO PYTHON------------------

void printar(){
  Serial.println("Dados:");
  Serial.print(pronto[0]);
  Serial.print(";");
  Serial.print(pronto[1]);
  Serial.print(";");
  Serial.print(pronto[2]);
  Serial.print(";");
  Serial.println(pronto[3]);
  Serial.print(comprimento[0]);
  Serial.print(";");
  Serial.print(comprimento[1]);
  Serial.print(";");
  Serial.print(comprimento[2]);
  Serial.print(";");
  Serial.println(comprimento[3]);
  Serial.print(int(coord_x));
  Serial.print(";");
  Serial.println(int(coord_y));  
}

//-------------RECEBIMENTO DOS DADOS POR ESPNOW-------------------

void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&myData, incomingData, sizeof(myData));
  pronto[myData.id] = myData.fim;
  //comprimento[myData.id] = myData.pos;
  //printar();
}

//------------------REALIZA PAREAMENTO--------------------

void setup_peer(){
  esp_now_register_send_cb(OnDataSent);

  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  //registrar pareamento 
  for(int i = 0; i<4; i++){
    memcpy(peerInfo.peer_addr, moduloAddresses[i], 6);
    if (esp_now_add_peer(&peerInfo) != ESP_OK){
      Serial.print("Falha pareamento ");
      Serial.println(i);
      return;
    }
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

//-------------ATIVAR INTERRUPÇÃO POR TEMPO--------------
void ativar_int_tempo(){
  
  esp_timer_create(&timerArgs, &timerHandle);
  esp_timer_start_periodic(timerHandle, 50000); // A cada 0,5 segundo
}

//---------------------------------------------------
void setup() {
  Serial.begin(115200);
  ativar_ESPNOW();  //Ativa o ESPNOW
  ativar_int_tempo();
  
  pinMode(pino_x,INPUT);
  pinMode(pino_y,INPUT);
}

void loop() {
  if(!esp_timer_is_active(timerHandle)){
    if(abs(map(analogRead(pino_x),0,4095,100,-100))>20 || abs(map(analogRead(pino_y),0,4095,100,-100))>20){
      esp_timer_start_periodic(timerHandle, 50000);
    }
  }
}
