#ifndef SERVO_CLASS_H_
#define SERVO_CLASS_H_

#include "ESP32_Servo.h"

class ServoClass{
    const uint8_t pin;
    const int8_t dir;
    const int zero;
    int min = -90;
    int max = 90;
    Servo servo;

public:
    ServoClass(uint8_t pin, int8_t dir, int zero) 
        : pin(pin), dir(dir), zero(zero){}

    void Init(int min,int max){
        servo.attach(pin,544,2400);
        this->min=min;
        this->max=max;
    }
    void SetRadian(float angle){
        constexpr float pi = 3.1415f;
        SetDegree(angle*180/pi);
    }
    void SetDegree(float angle){
        if(angle > max) angle = max;
        if(angle < min) angle = min;
        servo.write((angle + zero)*dir + 90);
    }
};

#endif
