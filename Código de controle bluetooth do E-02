//Utilização do código com o APP Bluetooth RC Controller.
//Basta baixar da play store e instalar.
//Ao abrir o APP, clicar na engrenagem, escolher a opção "connect to car" e parear com o "E-02 BLUETOOTH".


//CÓDIGO E-02 BLUETOOTH

#include "BluetoothSerial.h"

BluetoothSerial SerialBT;

//MOTOR DIREITO
int VeloMotor1 = 15;
int Motor1A = 2;
int Motor1B = 4;

//MOTOR ESQUERDO
int VeloMotor2 = 13;
int Motor2A = 12;
int Motor2B = 14;

int velocidade1 = 55; //VELOCIDADE BAIXA
int velocidade2 = 155; //VELOCIDADE MÉDIA
int velocidade3 = 255; //VELOCIDADE ALTA

char Recebido; //ARMAZENA DADOS RECEBIDO VIA BLUETOOTH

void setup() {
  Serial.begin(115200);  //VELOCIDADE DA CONEXÃO
  SerialBT.begin("E-02 BLUETOOTH"); //NOME DA REDE BLUETOOTH
  Serial.println("Dispositivo conectado");

  pinMode(VeloMotor1, OUTPUT);
  pinMode(Motor1A, OUTPUT);
  pinMode(Motor1B, OUTPUT);
  pinMode(VeloMotor2, OUTPUT);
  pinMode(Motor2A, OUTPUT);
  pinMode(Motor2B, OUTPUT);
  
  digitalWrite(Motor1A, LOW);
  digitalWrite(Motor1B, LOW);
  digitalWrite(Motor2A, LOW);
  digitalWrite(Motor2B, LOW);
}

void loop() {
  if (!Serial.available()) {  //SE A SERIAL NÃO ESTIVER ATIVA
  digitalWrite(Motor1A, LOW);  //MANTÉM DESLIGADO OS MOTORES
  digitalWrite(Motor1B, LOW);
  digitalWrite(Motor2A, LOW);
  digitalWrite(Motor2B, LOW);
  }
  if (Serial.available()) {  //SE A SERIAL ESTIVER ATIVA
    SerialBT.write(Serial.read());  //ENVIA DADOS DO MONITOR SERIAL VIA BLUETOOTH
  }
  if (SerialBT.available()) {  //SE O BLUETOOTH ESTIVER ATIVO 
    Recebido = SerialBT.read();  //LÊ DOS DADOS RECEBIDOS VIA BLUETOOTH E ATRIBUI À VARIÁVEL "Recebido"
    SerialBT.write(Recebido);  //ESCREVE OS DADOS RECEBIDO VIA BLUETOOTH
    Serial.println(Recebido);  //PRINTA NO SERIAL MONITOR OS DADOS RECEBIDOS VIA BLUETOOTH

    //CARACTERES RECEBIDOS VIA BLUETOOTH
    if (Recebido == 'F') {
      frente();
    }
    else if (Recebido == 'I') {
      frente_direita();
    }
    else if (Recebido == 'G') {
      frente_esquerda();
    }    
    else if (Recebido == 'B') {
      tras();
    }
    else if (Recebido == 'J') {
      tras_direita();
    }
    else if (Recebido == 'H') {
      tras_esquerda();
    }
    else if (Recebido == 'L') {
      esquerda();
    } 
    else if (Recebido == 'R') {
      direita();
    }
    else {
      parado();
    }
  }
}
 
//COMANDOS DIRECIONAIS DO ROBÔ 
void frente() {
  analogWrite(VeloMotor1, velocidade2);
  digitalWrite(Motor1A, HIGH);
  digitalWrite(Motor1B, LOW);
  analogWrite(VeloMotor2, velocidade2);
  digitalWrite(Motor2A, LOW);
  digitalWrite(Motor2B, HIGH);
}
  
void frente_direita() {
  analogWrite(VeloMotor1, velocidade1);
  digitalWrite(Motor1A, HIGH);
  digitalWrite(Motor1B, LOW);
  analogWrite(VeloMotor2, velocidade2);
  digitalWrite(Motor2A, LOW);
  digitalWrite(Motor2B, HIGH);
}
  
void frente_esquerda() {
  analogWrite(VeloMotor1, velocidade2);
  digitalWrite(Motor1A, HIGH);
  digitalWrite(Motor1B, LOW);
  analogWrite(VeloMotor2, velocidade1);
  digitalWrite(Motor2A, LOW);
  digitalWrite(Motor2B, HIGH);
}
 
void tras() {
  analogWrite(VeloMotor1, velocidade2);
  digitalWrite(Motor1A, LOW);
  digitalWrite(Motor1B, HIGH);
  analogWrite(VeloMotor2, velocidade2);
  digitalWrite(Motor2A, HIGH);
  digitalWrite(Motor2B, LOW);
}
 
void tras_direita() {
  analogWrite(VeloMotor1, velocidade1);
  digitalWrite(Motor1A, LOW);
  digitalWrite(Motor1B, HIGH);
  analogWrite(VeloMotor2, velocidade2);
  digitalWrite(Motor2A, HIGH);
  digitalWrite(Motor2B, LOW);
}
 
void tras_esquerda() {
  analogWrite(VeloMotor1, velocidade2);
  digitalWrite(Motor1A, LOW);
  digitalWrite(Motor1B, HIGH);
  analogWrite(VeloMotor2, velocidade1);
  digitalWrite(Motor2A, HIGH);
  digitalWrite(Motor2B, LOW);
}
 
void esquerda() {
  analogWrite(VeloMotor1, velocidade1);
  digitalWrite(Motor1A, HIGH);
  digitalWrite(Motor1B, LOW);
  analogWrite(VeloMotor2, velocidade1);
  digitalWrite(Motor2A, HIGH);
  digitalWrite(Motor2B, LOW);
}
 
void direita() {
  analogWrite(VeloMotor1, velocidade1);
  digitalWrite(Motor1A, LOW);
  digitalWrite(Motor1B, HIGH);
  analogWrite(VeloMotor2, velocidade1);
  digitalWrite(Motor2A, LOW);
  digitalWrite(Motor2B, HIGH);
}

void parado() {
  digitalWrite(Motor1A, LOW);
  digitalWrite(Motor1B, LOW);
  digitalWrite(Motor2A, LOW);
  digitalWrite(Motor2B, LOW);
}
