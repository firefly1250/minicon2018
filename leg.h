#ifndef LEG_H_
#define LEG_H_

#include "servoclass.h"
#include "inverse_kinematics.h"

class Leg{
    static constexpr float l1 = 39.8;
    static constexpr float l2 = 82;
    static constexpr float l3 = 92.03 + 1.2;

    const float psi;

    ServoClass yaw;
    ServoClass hip;
    ServoClass knee;

public:
    Leg(ServoClass yaw,ServoClass hip,ServoClass knee,float psi)
        : yaw(yaw), hip(hip), knee(knee), psi(psi){}

    void Init(){
        yaw.Init(-40,40);
        hip.Init(-90,90);
        knee.Init(-90,90);
        yaw.SetDegree(0);
        hip.SetDegree(0);
        knee.SetDegree(0);
    }
 
    void SetXYZ(float x,float y,float z){
        constexpr float L = 133.086;
        const float xm = cos(psi)*x + sin(psi)* y + L;
        const float ym = -sin(psi)*x + cos(psi)* y + 0;
        
        constexpr float l0 = 39.8;
        const float r = sqrt(xm*xm+ym*ym);
        const float theta = atan2(ym,xm);
  
        float t1,t2,t3;
        std::tie(t1,t2)=InverseKinematics(r-l0,z);

        const float yaw = theta * 2;
        const float hip = t1;
        const float knee = t2+pi/2;
        Set(yaw,hip,knee);
    }

    void Set(float yaw_rad,float hip_rad,float knee_rad){
        SetYaw(yaw_rad);
        SetHip(hip_rad);
        SetKnee(knee_rad);
    }

    void SetYaw(float rad){
        yaw.SetRadian(rad);
    }
    void SetHip(float rad){
        hip.SetRadian(rad);
    }
    void SetKnee(float rad){
        knee.SetRadian(rad);
    }
    void SetYawDegree(float deg){
        yaw.SetDegree(deg);
    }
    void SetHipDegree(float deg){
        hip.SetDegree(deg);
    }
    void SetKneeDegree(float deg){
        knee.SetDegree(deg);
    }
    void SetDegree(float yaw_deg, float hip_deg, float knee_deg){
        yaw.SetDegree(yaw_deg);
        hip.SetDegree(hip_deg);
        knee.SetDegree(knee_deg);
    }
};

#endif
