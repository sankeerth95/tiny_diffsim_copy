#ifndef PENDULUM__DEF_H_
#define PENDULUM__DEF_H_
#include "utils/differentiation.hpp"
#include "dynamics/forward_dynamics.hpp"
#include "dynamics/integrator.hpp"
#include "multi_body.hpp"

#include "utils/pendulum.hpp"
#include "world.hpp"


const int kNumLinks = 1;
const int kNumTimesteps = 100;

const bool kLossOnQ = true;
const bool kLossOnQd = false;
const bool kLossOnQdd = false;

std::vector<std::vector<double>> groundtruth_states;

template <typename Algebra>
struct Pendulum {
  using Scalar = typename Algebra::Scalar;
  using Vector3 = typename Algebra::Vector3;

  Scalar dt{Algebra::from_double(0.01)};

  // dimensionality of the parameter vector (important for some AD libraries)
  static const int kDim = kNumLinks;

  void rollout(const std::vector<Scalar>& params,
               std::vector<std::vector<Scalar>>* output_states) const {
    tds::World<Algebra> world;
    tds::MultiBody<Algebra> &pendulum = *world.create_multi_body();
    init_compound_pendulum(pendulum, world, kNumLinks, params);
    std::vector<Scalar> state(3 * kNumLinks);
    for (int t = 0; t < kNumTimesteps; ++t) {
      // populate state with q, qd, qdd
      int i = 0;
      for (int j = 0; j < kNumLinks; ++j, ++i) {
        state[i] = pendulum.q(j);
      }
      for (int j = 0; j < kNumLinks; ++j, ++i) {
        state[i] = pendulum.qd(j);
      }
      for (int j = 0; j < kNumLinks; ++j, ++i) {
        state[i] = pendulum.qdd(j);
      }
      output_states->push_back(state);

      // simulate and integrate
      tds::forward_dynamics(pendulum, world.get_gravity());
      tds::integrate_euler(pendulum, dt);
    }
  }

  // call operator that computes the cost; to be executed by the optimizer
  Scalar operator()(const std::vector<Scalar>& params) const {
    assert(!groundtruth_states.empty());

    Scalar cost = Algebra::zero();
    std::vector<std::vector<Scalar>> our_states;
    rollout(params, &our_states);
    for (int t = 0; t < kNumTimesteps; ++t) {
      const auto& true_state = groundtruth_states[t];
      const auto& our_state = our_states[t];
      // compute squared L2 norm over state difference
      if constexpr (kLossOnQ) {
        for (int j = 0; j < kNumLinks; ++j) {
          Scalar diff = Algebra::from_double(true_state[j]) - our_state[j];
          cost += diff * diff;
        }
      }
      if constexpr (kLossOnQd) {
        for (int j = 0; j < kNumLinks; ++j) {
          Scalar diff =
              Algebra::from_double(true_state[j + 3]) - our_state[j + 3];
          cost += diff * diff;
        }
      }
      if constexpr (kLossOnQdd) {
        for (int j = 0; j < kNumLinks; ++j) {
          Scalar diff =
              Algebra::from_double(true_state[j + 6]) - our_state[j + 6];
          cost += diff * diff;
        }
      }
    }
    cost /= kNumTimesteps;
    return cost;
  }
};

#endif // PENDULUM__DEF_H_

