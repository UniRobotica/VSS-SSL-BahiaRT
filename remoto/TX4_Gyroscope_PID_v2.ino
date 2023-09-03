
//ACSO-VVS-F

// Código de controle joystick via ESP-NOW do ACSO-VSSS-F com Giroscópio Acelerômetro e controle PID

// Bibliotecas
#include <esp_now.h>
#include <esp_wifi.h>
#include <WiFi.h>

// Declaração de pinos que vão conectados ao joystick
#define SWITCH_PIN 25
#define X_AXIS_PIN 32
#define Y_AXIS_PIN 33

unsigned long Timeold; // Parâmetro de tempo para cálcular envio de dados no Monitor Serial (default = 1000ms)

bool ButtonSW = false; // Armazena valor para ativação/desativação do PID

// MAC Adress para enviar os dados no canal selecionado
uint8_t broadcast_adr[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
//uint8_t broadcast_adr[] = {0x70, 0xB8, 0xF6, 0x5D, 0x76, 0x64};  // MAC Adress do MR 70:B8:F6:5D:76:64

typedef struct struct_message
{
  byte xAxisValue;  // Eixo X
  byte yAxisValue;  // Eixo Y
  byte switchPressed;  // Ativação/Desativação do PID
}struct_message;

struct_message commands;

esp_now_peer_info_t peerInfo;

//This function is used to map 0-4095 joystick value to 0-254. hence 127 is the center value which we send.
//It also adjust the deadband in joystick.
//Jotstick values range from 0-4095. But its center value is not always 2047. It is little different.
//So we need to add some deadband to center value. in our case 1800-2200. Any value in this deadband range is mapped to center 127.
int mapAndAdjustJoystickDeadBandValues(int value, bool reverse)
{
  if (value >= 2000)
  {
    value = map(value, 2000, 4095, 127, 254);
  }
  else if (value <= 1800)
  {
    value = map(value, 1800, 0, 127, 0);  
  }
  else
  {
    value = 127;
  }

  if (reverse)
  {
    value = 254 - value;
  }
  return value;
}

// Função Callback para envio dos dados
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status){
  //Serial.print("\r\nLast Packet Send Status:\t ");
  //Serial.println(status);
  //Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Message sent" : "Message failed");
}

void setup()
{
  // Inicia a Serial
  Serial.begin(115200);

  // Configuração do wifi
  ESP_ERROR_CHECK(esp_netif_init());
  ESP_ERROR_CHECK(esp_event_loop_create_default());
  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&cfg));
  ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
  ESP_ERROR_CHECK(esp_wifi_start());
  ESP_ERROR_CHECK(esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE));
  esp_wifi_set_max_tx_power(84);

  // Inicia ESP-NOW
  if (esp_now_init() != ESP_OK)
  {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  else
  {
    //Serial.println("Succes: Initialized ESP-NOW");
  }

  // Register peer
  esp_now_peer_info_t peerInfo;
  memset(&peerInfo, 0, sizeof(peerInfo));
  memcpy(peerInfo.peer_addr, broadcast_adr, 6);
  peerInfo.channel = 1;  
  peerInfo.encrypt = false;
  
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK)
  {
    Serial.println("Failed to add peer");
    return;
  }
  else
  {
    //Serial.println("Succes: Added peer");
  }

  esp_now_register_send_cb(OnDataSent);

  pinMode(SWITCH_PIN, INPUT_PULLUP);   
}
 
void loop()
{
  commands.xAxisValue = mapAndAdjustJoystickDeadBandValues(analogRead(X_AXIS_PIN), false);
  commands.yAxisValue = mapAndAdjustJoystickDeadBandValues(analogRead(Y_AXIS_PIN), false);

  if (digitalRead(SWITCH_PIN) == LOW && ButtonSW == false)
  {
    commands.switchPressed = true;
    ButtonSW = true;
    delay(200);  // Delay necessário para não falhar a atribuição do valor
  }
  else if (digitalRead(SWITCH_PIN) == LOW && ButtonSW == true)
  {
    commands.switchPressed = false;
    ButtonSW = false;
    delay(200);  // Delay necessário para não falhar a atribuição do valor
  }
  
  // Exibe valores no Monitor Serial
  //if (millis() - Timeold >= 50)
  //{
    Serial.println();
    Serial.print("Botão: ");
    Serial.print(commands.switchPressed);
    Serial.print(" | ");
    Serial.print(commands.xAxisValue);
    Serial.print(" | ");
    Serial.print("Eixo Y: ");
    Serial.print(commands.yAxisValue);
    Serial.println();
  //}
  sendData();
}

void sendData(){   
    // esse delay é necessário para que os dados sejam enviados corretamente
    esp_err_t message = esp_now_send(broadcast_adr, (uint8_t *) &commands, sizeof(commands));
    delay(50);    
}
