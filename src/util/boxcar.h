#pragma once
#include <array>
#include <cstddef>
#include <cstdlib>
#include <vector>

/// Cool name for a simple moving average filter, where all weights are the same!
template <typename base_t, size_t N> struct BoxcarFilter {
  static_assert(N > 0);

public:
  explicit BoxcarFilter(const base_t &inital)
      : m_values(), m_sum(N * inital), m_idx(0) {
    for (unsigned int i = 0; i < N; ++i) {
      m_values[i] = inital;
    }
  }

  void push(const base_t &v) {
    m_sum -= m_values[m_idx];
    m_values[m_idx] = v;
    m_sum += m_values[m_idx];
    m_avg = m_sum / static_cast<float>(N);
    m_idx = (m_idx + 1) % N;
  }

  void reset(const base_t& v) {
    // NOTE: This is a horrible impl.
    for(size_t i = 0; i < N; ++i){
      push(v);
    }

  }

  base_t &get() {return m_avg;}

  constexpr size_t size() {
    return N;
  }

private:
  std::array<base_t, N> m_values;
  base_t m_sum;
  size_t m_idx;
  base_t m_avg;
};



template<typename base_t> struct DynamicBoxcar{

public:
  explicit DynamicBoxcar(const base_t &inital, const size_t size)
  {
    m_size = size;
    for (unsigned int i = 0; i < size; ++i) {
      m_values[i] = inital;
    }
    m_sum = inital * size;
    m_avg = inital;
    m_idx = 0;
  }

  void push(const base_t &v) {
    m_sum -= m_values[m_idx];
    m_values[m_idx] = v;
    m_sum += m_values[m_idx];
    m_avg = m_sum / static_cast<float>(m_size);
    m_idx = (m_idx + 1) % m_size;
  }

  void reset(const base_t& v, const size_t size) {
    m_size = size;
    // NOTE: This is a horrible impl.
    for (unsigned int i = 0; i < m_size; ++i){
      m_values[i] = v;
    }
    m_sum = v * m_size;
    m_avg = v;
    m_idx = 0;
  }

  base_t &get() {return m_avg;}

  size_t size() {
    return m_size;
  }

private:
  base_t m_values[1000];
  size_t m_size;
  base_t m_sum;
  size_t m_idx;
  base_t m_avg;
};
