// Código para o envio de informações para os robôs

#include <esp_now.h>
#include <esp_wifi.h>
#include <WiFi.h>

// Definindo variáveis ------------------------------------------

// MAC Adress genérico para enviar os dados no canal selecionado
uint8_t broadcast_adr1[] = {0xC0, 0x49, 0xEF, 0xE4, 0xBE, 0x48};
uint8_t broadcast_adr3[] = {0xC0, 0x49, 0xEF, 0xE4, 0xDF, 0x4C};
uint8_t broadcast_adr4[] = {0x94, 0xE6, 0x86, 0x3C, 0xCA, 0xE4};

#define CANAL 7

// Varáveis para armazenar informação recebida
const byte numChars = 64;
char receivedChars[numChars];
char tempChars[numChars];   
boolean newData = false;     
int id, count;

typedef struct struct_message{
  char message[numChars];
  } struct_message;

struct_message commands;

//Peer do ESP-NOW
esp_now_peer_info_t peerInfo;

// Tratando informação recebida via serial -----------------------

void recvWithStartEndMarkers(){
    static boolean recvInProgress = false;
    static byte ndx = 0;
    char startMarker = '<';
    char endMarker = '>';
    char in;

    while (Serial.available()){
        //  Formato da mensagem::
        //  <[id1],[v_l1],[v_a1],[id2],[v_l2],[v_a2],[id3],[v_l3],[v_a3]>
        in = Serial.read();

        if (recvInProgress == true){
            if (in != endMarker){
                receivedChars[ndx] = in;
                ndx++;
                if (ndx >= numChars){
                    ndx = numChars - 1;
                }
            }
            else{
                receivedChars[ndx] = '\0'; // onde termina a string
                recvInProgress = false;
                ndx = 0;
                newData = true;
            }
        }

        else if (in == startMarker){
            recvInProgress = true;
        }
    }
}

// Função de envio via ESP-NOW -----------------------------------

void sendData(){   
    esp_err_t message1 = esp_now_send(broadcast_adr1, (uint8_t *) &commands, sizeof(commands));
    esp_err_t message2 = esp_now_send(broadcast_adr3, (uint8_t *) &commands, sizeof(commands));
    esp_err_t message3 = esp_now_send(broadcast_adr4, (uint8_t *) &commands, sizeof(commands));
    delay(3); // esse delay é necessário para que os dados sejam enviados corretamente
}

//Setup e loop ---------------------------------------------------------------

void setup() {

  // Inicializando comunicação serial
  Serial.begin(115200);

  // Configuração ESP-NOW
  ESP_ERROR_CHECK(esp_netif_init());
  ESP_ERROR_CHECK(esp_event_loop_create_default());
  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&cfg));
  ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
  ESP_ERROR_CHECK(esp_wifi_start());
  ESP_ERROR_CHECK(esp_wifi_set_channel(CANAL, WIFI_SECOND_CHAN_NONE));
  esp_wifi_set_max_tx_power(84);


  if (esp_now_init() != ESP_OK) 
  {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  memcpy(peerInfo.peer_addr, broadcast_adr1, 6);
  if (esp_now_add_peer(&peerInfo) != ESP_OK) 
  {
    Serial.println("Failed to add peer");
    return;
  }

  memcpy(peerInfo.peer_addr, broadcast_adr3, 6);
  if (esp_now_add_peer(&peerInfo) != ESP_OK) 
  {
    Serial.println("Failed to add peer");
    return;
  }

  memcpy(peerInfo.peer_addr, broadcast_adr4, 6);
  if (esp_now_add_peer(&peerInfo) != ESP_OK) 
  {
    Serial.println("Failed to add peer");
    return;
  }
 
}

void loop() {
  recvWithStartEndMarkers();
  if (newData == true){ // Verificando se há uma nova informação recebida via serial
      strcpy(commands.message, receivedChars);
      sendData(); // Envia a nova informação via ESP-NOW
      newData = false;
  }
}