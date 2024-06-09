#pragma once

#include "util/metrics.h"
template <typename T = float> class RectangleIntegral {
public:
  explicit RectangleIntegral(T inital) : m_integral(inital) {}

  void reset(T value) {
    m_integral = value;
  }

  inline void integrate(T v, Time dt){
    m_integral += v * static_cast<float>(dt);
  }

  inline T get(){
    return m_integral;
  }

private:
  T m_integral;
};
