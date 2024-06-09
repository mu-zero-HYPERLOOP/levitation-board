#include "airgap_transition.h"
#include "canzero/canzero.h"
#include "util/metrics.h"
#include "util/timestamp.h"
#include <cmath>

static float sigmoid(float x) { return 1.0f / (1.0f + std::exp(-x)); }

struct SigmoidAnimator {
private:
  Distance m_start_airgap;
  Distance m_end_airgap;
  Timestamp m_start_timestamp;
  Duration m_duration;

public:
  void reset(const Distance &airgap) {
    m_start_airgap = airgap;
    m_end_airgap = airgap;
    m_start_timestamp = Timestamp::now();
    m_duration = 0_s;
  }

  void transition_to(const Distance &target, const Duration &duration) {
    if ((target - m_end_airgap).abs() > 0.1_mm) {
      m_start_airgap = current();
      m_end_airgap = target;
      m_start_timestamp = Timestamp::now();
      m_duration = duration;
    }
  }

  Distance current() {
    if (m_duration.as_us() == 0){
      m_start_airgap = m_end_airgap;
      return m_end_airgap;
    }
    const auto now = Timestamp::now();
    const Duration time_since_start = now - m_start_timestamp;
    float alpha = time_since_start.as_us() / (float)m_duration.as_us();
    if (alpha > 1.0f) {
      // transition done!
      m_start_airgap = m_end_airgap;
      return m_end_airgap;
    }
    // map [0,1] -> [-5,5]
    const float x = alpha * 10.0f - 5.0f;
    const float y = sigmoid(x);
    // linear interpolation between start and end.
    return m_start_airgap * (1.0f - y) + m_end_airgap * y;
  }

  bool in_transition() const {
    const auto now = Timestamp::now();
    const Duration time_since_start = now - m_start_timestamp;
    return time_since_start <= m_duration;
  }
};

static Distance grounded_left;
static Distance grounded_right;

static SigmoidAnimator left_animator;
static SigmoidAnimator right_animator;

void airgap_transition::begin() {}

void airgap_transition::calibrate() {
  grounded_left = Distance(canzero_get_airgap_left() * 1e-3);
  grounded_right = Distance(canzero_get_airgap_right() * 1e-3);
  canzero_set_target_airgap_left(grounded_left / 1_mm);
  canzero_set_target_airgap_right(grounded_right / 1_mm);

  left_animator.reset(grounded_left);
  right_animator.reset(grounded_right);
}

void airgap_transition::transition_to_ground(const Time &duration) {
  left_animator.transition_to(grounded_left, duration);
  right_animator.transition_to(grounded_right, duration);
}

void airgap_transition::transition_to(const Distance &airgap,
                                      const Time &duration) {
  left_animator.transition_to(airgap, duration);
  right_animator.transition_to(airgap, duration);
}

bool airgap_transition::done() {
  return !left_animator.in_transition() && !right_animator.in_transition();
}

void airgap_transition::update() {
  const Distance target_left = left_animator.current();
  canzero_set_target_airgap_left(target_left / 1_mm);

  const Distance target_right = right_animator.current();
  canzero_set_target_airgap_right(target_right / 1_mm);
}
