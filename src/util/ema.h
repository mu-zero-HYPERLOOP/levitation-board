#pragma once

template<typename base_t = float>
class ExponentialMovingAverage {
public:
  ExponentialMovingAverage(float alpha = 1.0f, base_t inital = base_t())
      : m_alpha(alpha), m_one_minus_alpha(1.0 - m_alpha), m_val(inital) {}

  void push(base_t x) { m_val = x * m_alpha + (m_one_minus_alpha)*m_val; }

  void push(base_t x) volatile { m_val = x * m_alpha + (m_one_minus_alpha)*m_val; }

  base_t get() { return m_val; }
  base_t get() volatile{ return m_val; }

  void set_alpha(float alpha){
    m_alpha = alpha;
    m_one_minus_alpha = 1.0f - alpha;
  }

  void reset(base_t value){
    m_val = value;
  }

private:
  float m_alpha;
  float m_one_minus_alpha;
  base_t m_val;
};
