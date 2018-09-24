#ifndef INVERSE_KINEMATICS_H_
#define INVERSE_KINEMATICS_H_

#include <complex>
#include <tuple>

using  cp= std::complex<float>;

constexpr float pi = 3.1415f;
constexpr float deg2rad(float deg){return deg/180.0f*pi;}
constexpr float rad2deg(float rad){return rad*180.0f/pi;}

std::tuple<float,float> InverseKinematics(const float x, const float z){
  constexpr float l1=82;
  constexpr float l2=72;
  constexpr float l3=81;
  constexpr float l4=12.55;
  constexpr float l5=47.381;
  constexpr float psi = deg2rad(7.28);

  const cp s1 = sqrt(cp(-(- l1*l1 + 2*l1*l2 - l2*l2 + x*x + z*z)*(l1*l1 + 2*l1*l2 + l2*l2 - x*x - z*z),0));
  const float t11= real(-log((cp(l1*l1 - l2*l2 + x*x + z*z ,0)-s1)/cp(2*l1*x, -2* l1*z))*cp(0,1));
  const float t12= real(-log((cp(l1*l1 - l2*l2 + x*x + z*z ,0)+s1)/cp(2*l1*x, -2* l1*z))*cp(0,1));
  const float t1 = fabs(t11) < fabs(t12) ? t11 : t12;

  const float phi = fabs(acos((l1*l1 + l2*l2 - x*x - z*z)/(2*l1*l2)));

  const float X=x+l5*cos(t1+phi+psi);
  const float Z=z+l5*sin(t1+phi+psi);

  const cp s2  = sqrt(cp(-(- X*X - Z*Z + l3*l3 + 2*l3*l4 + l4*l4)*(X*X + Z*Z - l3*l3 + 2*l3*l4 - l4*l4),0));
  const float t21 = real(-log((cp(X*X + Z*Z - l3*l3 + l4*l4,0)-s2)/cp(2*X*l4 ,- 2*Z*l4))*cp(0,1));
  const float t22 = real(-log((cp(X*X + Z*Z - l3*l3 + l4*l4,0)+s2)/cp(2*X*l4 ,- 2*Z*l4))*cp(0,1));
  const float t2 = fabs(-pi/2-t21) < fabs(-pi/2-t22) ? t21 : t22;
  
  return std::make_tuple(t1,t2);
}

#endif