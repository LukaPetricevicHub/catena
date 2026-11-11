// Small deterministic PRNG (xorshift32). The platform feeds fresh entropy every step.
#ifndef CATENA_RNG_H
#define CATENA_RNG_H

#include <stdint.h>

namespace catena {

class Rng {
 public:
  void seed(uint32_t s) { state_ = s ? s : 0x9E3779B9u; }
  void mix(uint32_t bits) {
    state_ ^= bits + 0x9E3779B9u + (state_ << 6) + (state_ >> 2);
    if (!state_) state_ = 0x9E3779B9u;
  }
  uint32_t next() {
    state_ ^= state_ << 13;
    state_ ^= state_ >> 17;
    state_ ^= state_ << 5;
    return state_;
  }
  // Uniform in [0, 1).
  float uniform() { return static_cast<float>(next() >> 8) * (1.0f / 16777216.0f); }
  float uniform(float lo, float hi) { return lo + (hi - lo) * uniform(); }

 private:
  uint32_t state_ = 0x9E3779B9u;
};

}  // namespace catena

#endif
