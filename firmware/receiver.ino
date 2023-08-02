#include <esp_now.h>
#include <esp_wifi.h>
#include <WiFi.h>

#define PWMA 15
#define A1 2
#define B1 4

#define PWMB 13
#define A2 12 
#define B2 14

// This is de code for the board that is in robots
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


void OnDataRecv(const uint8_t * mac_addr, const uint8_t *incomingData, int len) {
  memcpy(&rcv_commands, incomingData, sizeof(rcv_commands));
  // Update the structures with the new incoming data
  first_mark = millis();
  strcpy(commands, rcv_commands.message);
}


void setup() {
  
  // configuração de pinos
  ledcAttachPin(PWMA, 1);
  ledcAttachPin(PWMB, 2);

  ledcSetup(1, 80000, 8);
  ledcSetup(2, 80000, 8);

  //pinMode(stby, OUTPUT);
  pinMode(A1, OUTPUT);
  pinMode(A2, OUTPUT);
  pinMode(B1, OUTPUT);
  pinMode(B2, OUTPUT);
  //digitalWrite(stby, 1);
  digitalWrite(A1, 0);
  digitalWrite(A2, 0);
  digitalWrite(B1, 0);
  digitalWrite(B2, 0);

  // configurações comunicação

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

void motor_R(int speedR) { // se o valor for positivo gira para um lado e se for negativo troca o sentido
  if (speedR > 0) {
    digitalWrite(A1, 1);
    digitalWrite(B1, 0);
  } else {
    digitalWrite(A1, 0);
    digitalWrite(B1, 1);
  }
  ledcWrite(1, abs( speedR));
}


void motor_L(int speedL) {
  if (speedL > 0) {
    digitalWrite(A2, 1);
    digitalWrite(B2, 0);
  } else {
    digitalWrite(A2, 0);
    digitalWrite(B2, 1);
  }
  ledcWrite(2, abs( speedL));
}


void motors_control(float wl, float wr) {
  if (wr > 0 )
  {
    motor_R(255);
  } else if (wr < 0)
  {
    motor_R(-255);
  } else
  {
    motor_R(0);
  }

  if (wl > 0 )
  {
    motor_L(255);
  } else if (wl < 0)
  {
    motor_L(-255);
  } else
  {
    motor_L(0);
  }

}

void loop() {
  second_mark = millis();

  strcpy(tempChars, commands); // necessário para proteger a informação original
  parseData();

  if (second_mark - first_mark > 500) {
    v_l = 0.00;
    v_a = 0.00;
  }

  motors_control(v_l, v_a);
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