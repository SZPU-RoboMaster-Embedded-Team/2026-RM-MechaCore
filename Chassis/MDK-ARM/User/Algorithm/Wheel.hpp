#pragma once
#include "stdxxx.hpp"

class Wheel_Basic
{
public:
  virtual void UpDate(float vx, float vy, float vw, float MaxSpeed) = 0;
};

class Mecanum : public Wheel_Basic
{
private:
  float kx = 0, ky = 0;

public:
  float speed[4];

  void UpDate(float vx, float vy, float vw, float MaxSpeed);
};

class SG : public Wheel_Basic
{
private:
  float _speed[4];

public:
  float speed[4];
  float angle[4];
  void UpDate(float vx, float vy, float vw, float MaxSpeed);
};

// 外部接口
template <class T>
class Wheel_t
{
public:
  T WheelType;
};
