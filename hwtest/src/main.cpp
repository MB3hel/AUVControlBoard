
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_DotStar.h>
#include <MS5837.h>

#define LED_PIN 13
#define BNO055_SAMPLERATE_DELAY_MS (100)

Adafruit_BNO055 cb_imu = Adafruit_BNO055(-1, 0x28);
Adafruit_BMP280 cb_prs;
MS5837 cb_depth;
Adafruit_DotStar dotstar(1, 8, 6, DOTSTAR_BRG);


void error(){
    Serial.println("Test failed. Hardware tests aborted.");
    while(1){
        digitalWrite(LED_PIN, !digitalRead(LED_PIN));
        delay(500);
    }
}

void setup(){
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, HIGH);
    dotstar.begin();
    dotstar.show();
    Serial.begin(115200);
    
    // Comment out this line to avoid waiting for serial 
    // connectivity before running tests
    while(!Serial);


    Serial.println("Starting SW8 control board hardware tests");

    // Internal IMU
    Serial.print("    BNO055 IMU..........................");
    if(cb_imu.begin()){
        Serial.println("Pass");
    }else{
        Serial.println("Fail");
        error();
    }

    // Internal pressure and temperature sensor
    Serial.print("    BMP280 Pressure Sensor..............");
    if(cb_prs.begin()){
        Serial.println("Pass");
    }else{
        Serial.println("Fail");
        error();
    }

    // External depth sensor
    Serial.print("    MS5837 Pressure Sensor..............");
    if(cb_depth.init()){
        Serial.println("Pass");
    }else{
        Serial.println("Fail");
        error();
    }

    Serial.println("Hardware tests completed successfully.");
    digitalWrite(LED_PIN, LOW);
}

void loop(){
    
}
