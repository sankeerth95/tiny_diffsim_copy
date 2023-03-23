#include <chrono>
#include "utils/differentiation.hpp"
#include "pendulum_simobj.h"
#include "my_timer.h"
#include "world.hpp"


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







