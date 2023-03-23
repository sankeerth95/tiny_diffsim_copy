#include <chrono>

#include "utils/differentiation.hpp"
#include "dynamics/forward_dynamics.hpp"
#include "dynamics/integrator.hpp"
#include "multi_body.hpp"
#include "utils/pendulum.hpp"
#include "world.hpp"
#include "../examples/environments/ant_environment.h"




// non thread safe static timer helper functions
using TimerVar = std::chrono::time_point<std::chrono::high_resolution_clock>;
struct TimerHelper {
    static TimerVar start_time;
    static TimerVar end_time;
    static std::string name;
public:
    static void start_timer(std::string name){
        start_time = std::chrono::high_resolution_clock::now();
    }

    static void end_and_print(){
        end_time = std::chrono::high_resolution_clock::now();
        auto elapsed_ms = end_time - start_time;
        // end_time - start_time;
        std::cout << "Elapsed time: "  << name << " " << std::chrono::duration_cast<std::chrono::nanoseconds>(elapsed_ms).count() << "ns\n";
    }
};


TimerVar TimerHelper::start_time;
TimerVar TimerHelper::end_time;
std::string TimerHelper::name;

// template<typename Algebra>
// struct ThreadVectorizedEnv{
//   using Scalar = typename Algebra::Scalar;
//   using Vector3 = typename Algebra::Vector3;

//   void rollout(const std::vector<Scalar>& params,
//                std::vector<std::vector<Scalar>>* output_states) const {

//     // initialize a world with the type of algebra
//     tds::World<Algebra> world;
//     tds::MultiBody<Algebra> &pendulum = *world.create_multi_body();

//     // all the initialization happens here
//     init_compound_pendulum(pendulum, world, kNumLinks, params);
//     std::vector<Scalar> state(3 * kNumLinks);
//     for (int t = 0; t < kNumTimesteps; ++t) {
//       // populate state with q, qd, qdd
//       int i = 0;
//       for (int j = 0; j < kNumLinks; ++j, ++i) {
//         state[i] = pendulum.q(j);
//       }
//       for (int j = 0; j < kNumLinks; ++j, ++i) {
//         state[i] = pendulum.qd(j);
//       }
//       for (int j = 0; j < kNumLinks; ++j, ++i) {
//         state[i] = pendulum.qdd(j);
//       }
//       output_states->push_back(state);

//       // simulate and integrate
//       tds::forward_dynamics(pendulum, world.get_gravity());
//       tds::integrate_euler(pendulum, dt);
//     }
//   }
// };




int main(int argc, char* argv[]) {
  // first generate a groundtruth trajectory from a pendulum with all link
  // lengths = 1.5
  Simulation<tds::EigenAlgebra> sim;


  std::vector<double> true_params(kNumLinks, 1.5);
  sim.rollout(true_params, &groundtruth_states);

  // run gradient descent
  double learning_rate = 0.3;

  typedef tds::GradientFunctional<tds::DIFF_CPPAD_CODEGEN_AUTO, Simulation> GradFun;
  TimerHelper::start_timer("Compilation");
  GradFun::Compile();
  TimerHelper::end_and_print();

  // typedef tds::GradientFunctional<tds::DIFF_CPPAD_AUTO, Simulation> GradFun;

  std::vector<double> our_params(kNumLinks, 1.0);  // initial guess

  GradFun diffsim;
  for (int i = 0; i < 100; ++i) {
      printf("Iteration %i - cost: %.5f\n", i, diffsim.value(our_params));

      TimerHelper::start_timer("gradfun");
      const auto& grad = diffsim.gradient(our_params);
      TimerHelper::end_and_print();

      for (int j = 0; j < kNumLinks; ++j) {
        our_params[j] -= learning_rate * grad[j];
      }
  }
  printf("Estimated parameters (should be [");
  for (int j = 0; j < kNumLinks; ++j) {
    printf(" %.2f", true_params[j]);
  }
  printf(" ]):\n");
  for (int j = 0; j < kNumLinks; ++j) {
    printf("\t%.5f", our_params[j]);
  }
  printf("\n");

  return EXIT_SUCCESS;
}







