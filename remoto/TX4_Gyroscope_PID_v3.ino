
// ACSO-VSSS-F

// Código de controle joystick via ESP-NOW do ACSO-VSSS-F com Giroscópio Acelerômetro, controle PID e Supervisório

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
uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
//uint8_t broadcastAddress[] = {0x70, 0xB8, 0xF6, 0x5D, 0x76, 0x64};  // MAC Adress do MR 70:B8:F6:5D:76:64

// Estrutura de envio/recebimento de dados
typedef struct struct_message
{
  byte xAxisValue; // Eixo X
  byte yAxisValue; // Eixo Y
  byte switchPressed; // Ativação/Desativação do PID
  double AngleErrorSup; // Atribuição do erro de angulação que foi resultante
  double Output2Sup; // PWM do motor esquerdo
  double AngleErrorM2Sup; //  Atribuição do erro de angulação que foi atribuído motor esquerdo
  double Output1Sup; // PWM do motor direito
  double AngleErrorM1Sup; // Atribuição do erro de angulação que foi atribuído motor direito
}struct_message;

struct_message myData;

struct_message incomingReadings;

esp_now_peer_info_t peerInfo;

// Função usada para converter a escala de leitura do joystick de 0-4095 para 0-254, tendo o centro da escala em 127.
// A escala joystick é de 0-4095 e o centro seria em 2047, porém valor central costuma ser um pouco diferente disso.
// É necessário criar uma zona morta, com essa função a zona morta será entre os valores 1800-2200, atribuindo o valor de 127 para o centro do joystick.
int mapAndAdjustJoystickDeadBandValues(int value)
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
  return value;
}

// Função Callback para envio dos dados
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status)
{
  //Serial.print("\r\nLast Packet Send Status:\t ");
  //Serial.println(status);
  //Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Message sent" : "Message failed");
}
// Função Callback para recebimento dos dados
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len)
{
  memcpy(&incomingReadings, incomingData, sizeof(incomingReadings));
  //Serial.print("Bytes received: ");
  //Serial.println(len);
  incomingReadings.AngleErrorSup;
  incomingReadings.Output2Sup;
  incomingReadings.AngleErrorM2Sup;
  incomingReadings.Output1Sup;
  incomingReadings.AngleErrorM1Sup;
  //Serial.println(incomingReadings.AngleError);
  //Serial.println(incomingReadings.Output2Sup);
  //Serial.println(incomingReadings.AngleErrorM2Sup);
  //Serial.println(incomingReadings.Output1Sup);
  //Serial.println(incomingReadings.AngleErrorM1Sup);
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
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
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

  // Função de Callback para envio de dados
  esp_now_register_send_cb(OnDataSent);

  // Função de Callback para recebimento de dados
  esp_now_register_recv_cb(OnDataRecv);

  pinMode(SWITCH_PIN, INPUT_PULLUP);
}
 
void loop()
{
  myData.xAxisValue = mapAndAdjustJoystickDeadBandValues(analogRead(X_AXIS_PIN));
  myData.yAxisValue = mapAndAdjustJoystickDeadBandValues(analogRead(Y_AXIS_PIN));

  if (digitalRead(SWITCH_PIN) == LOW && ButtonSW == false)
  {
    myData.switchPressed = true;
    ButtonSW = true;
    delay(200);  // Delay necessário para não falhar a atribuição do valor
  }
  else if (digitalRead(SWITCH_PIN) == LOW && ButtonSW == true)
  {
    myData.switchPressed = false;
    ButtonSW = false;
    delay(200);  // Delay necessário para não falhar a atribuição do valor
  }

  esp_err_t result; // Declaração

  // Envia os dados via ESP-NOW
  if (millis() - Timeold >= 50)
  {
    result = esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));
    if (result == ESP_OK)
    {
    //Serial.println("Sent with success");
    }
    else
    {
    Serial.println("Error sending the data");
    }

    // Exibe valores no Monitor Serial
    Serial.print("PID: ");
    Serial.print(myData.switchPressed);
    Serial.print(" | ");
    //Serial.print("Eixo X: ");
    //Serial.print(myData.xAxisValue);
    //Serial.print(" | ");
    //Serial.print("Eixo Y: ");
    //Serial.print(myData.yAxisValue);
    //Serial.print(" | ");
    Serial.print("AngleErrorSup: ");
    Serial.print(incomingReadings.AngleErrorSup);
    Serial.print(" | ");
    Serial.print("PWM2: ");
    Serial.print(incomingReadings.Output2Sup);
    Serial.print(" | ");
    Serial.print("AngleErrorM2: ");
    Serial.print(incomingReadings.AngleErrorM2Sup);
    Serial.print(" | ");
    Serial.print("PWM1: ");
    Serial.print(incomingReadings.Output1Sup);
    Serial.print(" | ");
    Serial.print("AngleErrorM1: ");
    Serial.println(incomingReadings.AngleErrorM1Sup);

    Timeold = millis();
  }
}
