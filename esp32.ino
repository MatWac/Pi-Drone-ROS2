#include <Arduino.h>
#include <string.h>

const int numMotors = 4;
int motorPins[numMotors] = {4, 5, 18, 19};  // Exemple de pins de moteur
int motorValues[numMotors]; 

const int freq = 50;
const int resolution = 12;
int PWMValues[4];

void setup() {

  for(int i = 0; i < 4; i++)
  {
    ledcSetup(i, freq, resolution);
    ledcAttachPin(motorPins[i],i);
    ledcWrite(i, 140);
    delay(2000);
  }
  
  Serial.begin(115200);
}

void loop() {
  if (Serial.available()) {
    String receivedData = Serial.readStringUntil('\n');
    parseAndApplyPWM(receivedData);
    Serial.flush();
  }
}

void parseAndApplyPWM(String data) {
  int motorIndex = 0;
  int lastIndex = 0;
  int nextIndex = 0;

  while (nextIndex != -1 && motorIndex < numMotors) {
    nextIndex = data.indexOf(',', lastIndex);
    if (nextIndex == -1) {
      motorValues[motorIndex] = data.substring(lastIndex).toInt();
    } else {
      motorValues[motorIndex] = data.substring(lastIndex, nextIndex).toInt();
    }
    lastIndex = nextIndex + 1;
    motorIndex++;
  }

  // Appliquer les valeurs PWM aux moteurs
  for (int i = 0; i < numMotors; i++) {
    ledcWrite(i, motorValues[i]);
    Serial.print(" PWM ");
    Serial.print(i);
    Serial.print(" : ");
    Serial.println(motorValues[i]);
  }
}
