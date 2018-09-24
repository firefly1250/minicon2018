#ifndef UNDERCARRIAGE_H_
#define UNDERCARRIAGE_H_

#include "leg.h"
#include "inverse_kinematics.h"

class Undercarriage{
    Leg legs[4];    

public:
    Undercarriage () :
        legs{
            Leg(ServoClass(2,-1,-27),ServoClass(19,-1,0),ServoClass(22,1,-20),pi/4),//1
            Leg(ServoClass(15,-1,-45),ServoClass(18,-1,5),ServoClass(1,1,0),pi*3/4),//2
            Leg(ServoClass(16,-1,25),ServoClass(17,-1,0),ServoClass(3,1,0),pi*5/4),//3
            Leg(ServoClass(4,-1,0),ServoClass(21,-1,-10),ServoClass(23,1,0),pi*7/4)//4
        }
        {}

    void Setup(){
        for(int i=0;i<4;i++) legs[i].Init();
    }

    void Loop(const uint8_t* const received) {
        static unsigned long long t=0;
        static bool s=false;
        
        const int period = received[3]*10;
        const int z_const = -received[2];
        const float vx = ((int)received[0]-127)/127.0f*20;
        const float vy = ((int)received[1]-127)/127.0f*20;
        const float omega = ((int)received[4]-127)/127.0f*pi/9;
        
        float x[4],y[4],z[4];

        if(omega == 0){
            for(int i=0;i<4;i++){
                if(period==1500){
                    x[i] = -vx;
                    y[i] = -vy;
                }
                else{
                    const int8_t sign = i % 2 == 0 ? 1 : -1;
                    x[i] = sign*vx*cos(t*(2*pi)/(2*period));
                    y[i] = -sign*vy*cos(t*(2*pi)/(2*period));
                }
            }
        }
        else{
            constexpr float L = 133.086;
            for(int i=0;i<4;i++){
                const float alpha = pi/4*(2*i+1);
                if(period==1500){
                    x[i] = L*cos(omega+alpha)-L*cos(alpha);
                    y[i] = L*sin(omega+alpha)-L*sin(alpha);
                }
                else{
                    const float theta = omega*cos(t*(2*pi)/(2*period));
                    const int8_t sign = i % 2 == 0 ? 1 : -1;
                    x[i] = L*cos(sign*theta+alpha)-L*cos(alpha);
                    y[i] = L*sin(sign*theta+alpha)-L*sin(alpha);
                }
            }
        }

        if((vx==0 && vy == 0 && omega==0) || period==1500){
            for(int i=0;i<4;i++) z[i]=z_const;
        }
        else{
            z[0]=z[2] = s ? z_const :-30*cos(t*(2*pi)/period)+z_const+30;
            z[1]=z[3] = !s ? z_const :-30*cos(t*(2*pi)/period)+z_const+30;
        }
        
        for(int i=0;i<4;i++) legs[i].SetXYZ(x[i],y[i],z[i]);
        
        t+=10;
        if(t % period == 0) s = !s;
    }

    void InverseKinematicsTest() {
        static unsigned long long t=0;
        constexpr int period = 5000;
        float x = 25*cos(t*(2*pi)/period)+95;
        float z = 25*sin(t*(2*pi)/period)-60;
        float t1,t2;
        std::tie(t1,t2)=InverseKinematics(x,z);
        legs[0].SetHip(t1);
        legs[0].SetKnee(t2+pi/2);
        delay(10);
        t+=10;
    }

    
    void Spin(){
        static unsigned long long t=0;
        static bool s=false;
        const int period = 300;
        float theta = -20*cos(t*(2*pi)/period);
        float yaw = -20*cos(t*(2*pi)/(2*period));
        if(s){
            legs[0].SetHipDegree(theta);
            legs[1].SetHipDegree(-20);
            legs[2].SetHipDegree(theta);
            legs[3].SetHipDegree(-20);    
        }
        else{
            legs[0].SetHipDegree(-20);
            legs[1].SetHipDegree(theta);
            legs[2].SetHipDegree(-20);
            legs[3].SetHipDegree(theta);
        }
        legs[0].SetYawDegree(yaw);
        legs[1].SetYawDegree(-yaw);
        legs[2].SetYawDegree(yaw);
        legs[3].SetYawDegree(-yaw);

        t += 10;
        if(t % period == 0){
            s = !s;
        }
        delay(10);
    }

    void AdjustZero(){
        if(Serial.available()<5) return;
        
        char l = Serial.read();
        if(!(l >='1' && l <= '4')) return;

        char p = Serial.read();      
        if(!(p=='y' || p=='h' || p=='k')) return;

        char s = Serial.read();      
        if(!(s=='+' || s=='-')) return;
        
        char n1 = Serial.read();
        if(!(n1 >='0' && n1 <= '9')) return;
        
        char n2 = Serial.read();
        if(!(n2 >='0' && n2 <= '9')) return;

        int n = ((n1-48)*10 + (n2-48)) * (s=='+' ? 1 : -1);

        if(p=='y') legs[l-48-1].SetYaw(n);
        if(p=='h') legs[l-48-1].SetHip(n);
        if(p=='k') legs[l-48-1].SetKnee(n);

        Serial.print(l);
        Serial.print(p);
        Serial.println(n);
    }

};

#endif
