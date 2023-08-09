// Código para o recebimento de informações

#include <esp_now.h>
#include <esp_wifi.h>

// Definindo variáveis ------------------------------------------
int robot_id = 0;

int id;
int first_mark = 0, second_mark;

float v_l, v_a;

const byte numChars = 64;
char commands[numChars];
char tempChars[numChars];


typedef struct struct_message{
  char message[64];
  } struct_message;

struct_message rcv_commands;

// Atualizando dados recebidos via ESP-NOW------------------------

void OnDataRecv(const uint8_t * mac_addr, const uint8_t *incomingData, int len) {
  memcpy(&rcv_commands, incomingData, sizeof(rcv_commands));
  first_mark = millis();
  strcpy(commands, rcv_commands.message);
}

void parseData(){
    char * strtokIndx;
  
    strtokIndx = strtok(tempChars, ",");
    
    while (strtokIndx != NULL){
        id = atoi(strtokIndx);
        
        if(id == robot_id){         
          strtokIndx = strtok(NULL, ",");  
          v_l = atof(strtokIndx);       
          strtokIndx = strtok(NULL, ",");         
          v_a = atof(strtokIndx);
          strtokIndx = strtok(NULL, ","); 
       }

       else{
          strtokIndx = strtok(NULL, ",");     
          strtokIndx = strtok(NULL, ",");         
          strtokIndx = strtok(NULL, ","); 
       }
   } 
}

// Setup e loop --------------------------------------------------

void setup() {

  // Configurações comunicação ESP-NOW

  ESP_ERROR_CHECK(esp_netif_init());
  ESP_ERROR_CHECK(esp_event_loop_create_default());
  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
  ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );
  ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_STA) );
  ESP_ERROR_CHECK( esp_wifi_start());
  ESP_ERROR_CHECK( esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE));

  if (esp_now_init() != ESP_OK) 
  {
    return;
  }

  esp_now_register_recv_cb(OnDataRecv);
}

void loop() {
  second_mark = millis();

  // Necessário para proteger a informação original
  strcpy(tempChars, commands);
  parseData();

  if (second_mark - first_mark > 500) {
    v_l = 0.00;
    v_a = 0.00;
  }
}