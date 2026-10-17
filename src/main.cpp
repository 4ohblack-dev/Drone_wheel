#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <SPI.h>
#include <RadioLib.h>
#include<ControlPacket.h>
#include<string.h>

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
uint8_t receiveBuffer[sizeof(ControlPacket)];
size_t receiveLength = 0;
bool validCommandReceived = false;
uint32_t lastReceiveTime = 0;

constexpr uint32_t pwm_freq = 20000;
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
        digitalWrite(phasePin,LOW);
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

void handlePacket(const ControlPacket& packet) {
    if (packet.leftPower < -255 || packet.leftPower > 255 ||
        packet.rightPower < -255 || packet.rightPower > 255) {
        return;
    }

    for (uint8_t i = 0; i < 6; i++) {
        if (packet.servoAngle[i] > 180) {
            return;
        }
    }

    validCommandReceived = true;
    lastReceiveTime = millis();

    Serial.printf(
        "受信 #%u : left=%d right=%d front=%u\n",
        static_cast<unsigned int>(packet.sequence),
        static_cast<int>(packet.leftPower),
        static_cast<int>(packet.rightPower),
        static_cast<unsigned int>(packet.servoAngle[0])
    );

    // 通信確認後、ここにモーター・サーボ制御を追加する
}

void receiveData() {
    while (espLink.available() > 0) {
        receiveBuffer[receiveLength++] =
            static_cast<uint8_t>(espLink.read());

        if (receiveLength < sizeof(ControlPacket)) {
            continue;
        }

        ControlPacket packet;
        memcpy(&packet, receiveBuffer, sizeof(packet));

        uint16_t expectedCRC = calculateCRC(
            receiveBuffer,
            offsetof(ControlPacket, crc)
        );

        if (packet.start1 == 0xAA &&
            packet.start2 == 0x55 &&
            packet.crc == expectedCRC) {

            handlePacket(packet);
            receiveLength = 0;
        } else {
            // データがずれた場合、1バイトずらして再確認
            memmove(
                receiveBuffer,
                receiveBuffer + 1,
                sizeof(receiveBuffer) - 1
            );

            receiveLength = sizeof(receiveBuffer) - 1;
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
    receiveData();

    if(!validCommandReceived||millis()-lastReceiveTime >=300){
        ledcWrite(left_ch,0);
        ledcWrite(right_ch,0);
    }
}