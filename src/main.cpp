#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <SPI.h>
#include <RadioLib.h>

#define servoMin 500
#define servoMax 2400

constexpr uint8_t SDApin = 21;
constexpr uint8_t SCLpin = 22;

//UART
constexpr char deviceName[] = "wheel";
constexpr uint8_t UART_RX = 4;
constexpr uint8_t UART_TX = 5;
constexpr uint32_t UART_BAUD = 115200;
HardwareSerial espLink(1);
char recieveBuffer[64];
size_t recieveLengh = 0;
bool discardLine = false;

uint32_t lastSendTime = 0;
uint32_t sendCount = 0;

constexpr uint8_t pwm_freq = 20000;
constexpr uint8_t pwm_bits = 8;

constexpr uint8_t left_phase = 16;
constexpr uint8_t left_enable = 17;
constexpr uint8_t left_ch = 0;
constexpr uint8_t right_phase = 18;
constexpr uint8_t right_enable = 19;
constexpr uint8_t right_ch = 1;

Adafruit_PWMServoDriver pca = Adafruit_PWMServoDriver(0x40);

enum servoJoint:uint8_t{
    front = 0,
    rear = 1,
    leftfront = 2,
    rightfront = 3,
    leftrear = 4,
    rightrear = 5,
};
/*
servo→
股関節前後２個
肩１個ずつ：合計４個

*/
void servo_move(servoJoint joint,int angle){
    int pulseUs = map(constrain(angle,0,180),0,180,servoMin,servoMax);
    pca.writeMicroseconds(joint,pulseUs);
}

class MotorDrive {
public:
    uint8_t phasePin;
    uint8_t enablePin;
    uint8_t pwmch;

    void setup() {
        digitalWrite(enablePin,LOW);
        digitalWrite(phasePin,OUTPUT);
        pinMode(enablePin,OUTPUT);
        pinMode(phasePin,OUTPUT);

        ledcSetup(pwmch,pwm_freq,pwm_bits);
        ledcWrite(pwmch,0);
        ledcAttachPin(enablePin,pwmch);
    }

    void drive(uint8_t phasePin,uint8_t pwmch,int val) {
        val = constrain(val,-255,255);
        digitalWrite(phasePin,val < 0 ? HIGH : LOW);
        ledcWrite(pwmch,abs(val));           
    }
};

MotorDrive leftmotor{left_phase,left_enable,left_ch};
MotorDrive rightmotor{right_phase,right_enable,right_ch};

void recieveData(){
    while(espLink.available() > 0){
        char c = static_cast<char>(espLink.read());
        if(c=='\r') continue;

        if(c == '\n'){
            if(!discardLine && recieveLengh > 0){
                recieveBuffer[recieveLengh] = '\0';
                Serial.print("recieved:");
                Serial.println(recieveBuffer);
            }
            recieveLengh = 0;
            discardLine = false;
            continue;
        }
        if(discardLine) continue;

        if(recieveLengh < sizeof(recieveBuffer) - 1){
            recieveBuffer[recieveLengh++] = c;
        } else{
            recieveLengh = 0;
            discardLine = true;
        }
    }
}

void setup(){
    Serial.begin(115200);
    Wire.begin(SDApin,SCLpin);
    espLink.begin(UART_BAUD,SERIAL_8N1,UART_RX,UART_TX);
    Serial.print("UART begin");
    Serial.println(deviceName);

    if(!pca.begin()){
        Serial.println("PCA9685 not found");
        while(true){
            delay(1000);
        }
    }
    pca.setPWMFreq(50);
    delay(1000);

    leftmotor.setup();
    rightmotor.setup();
}

void loop(){

}