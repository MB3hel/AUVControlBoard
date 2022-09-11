
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

#define LED_PIN 13
#define BNO055_SAMPLERATE_DELAY_MS (100)

Adafruit_BNO055 cb_imu = Adafruit_BNO055(-1, 0x28);

void error(){
    while(1){
        digitalWrite(LED_PIN, !digitalRead(LED_PIN));
        delay(500);
    }
}

void setup(){
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, HIGH);
    Serial.begin(115200);
    while(!Serial);
    Serial.println("SW8E Control Board Hardware Test");

    Serial.print("    BNO055 IMU..........");
    if(cb_imu.begin()){
        Serial.println("Pass");
    }else{
        Serial.println("Fail");
    }

    Serial.println("Hardware Tests Completed Successfully.");
    digitalWrite(LED_PIN, LOW);
}

void loop(){
    
}
