#include <iostream>
#include <vector>
#include "utils/differentiation.hpp"
#include "my_timer.h"

template<typename Algebra>
struct Simulation{
    using Scalar = typename Algebra::Scalar;
    // size of v
    static const int kDim = 10000;
    Scalar operator ()(const std::vector<Scalar>& v) const  {
        Scalar sum = v[0];
        std::vector<Scalar> u(kDim/2);
        for(int i = 0; i < kDim/2; i++){
            u[i] += v[2*i]*v[2*i+1];
        }
        for(int i = 0; i < kDim/2-1; i++){
            sum += u[i]*u[i+1];
        }
        return sum;
    }
};

int main(int argc, char* argv[]) {
  // first generate a groundtruth trajectory from a pendulum with all link
  // lengths = 1.5
  Simulation<tds::EigenAlgebra> sim;

  std::vector<double> our_params(10000, 1.7);  // initial guess
    double x = 0;
  TimerHelper::start_timer("double forward pass");
  for(int i = 0; i < 1000; ++i)
      x += sim(our_params);
  TimerHelper::end_and_print();
    std::cout <<x <<std::endl;
//   std::vector<double> true_params(kNumLinks, 1.5);
//   sim.rollout(true_params, &groundtruth_states);

  // run gradient descent

  typedef tds::GradientFunctional<tds::DIFF_CPPAD_CODEGEN_AUTO, Simulation> GradFun;
  GradFun::Compile();

//   typedef tds::GradientFunctional<tds::DIFF_CPPAD_AUTO, Simulation> GradFun;

  GradFun diffsim;

// 
    TimerHelper::start_timer("AD forward pass");
    for(int i = 0; i < 1000; ++i)
        x += diffsim.value(our_params);
    TimerHelper::end_and_print();
    std::cout <<x <<std::endl;

    TimerHelper::start_timer("AD backward pass");
    for(int i = 0; i < 1000; ++i)
        diffsim.gradient(our_params);
    TimerHelper::end_and_print();


    TimerHelper::start_timer("double forward pass");
    for(int i = 0; i < 1000; ++i)
        x += sim(our_params);
    TimerHelper::end_and_print();
    std::cout <<x <<std::endl;


    TimerHelper::start_timer("AD forward pass");
    for(int i = 0; i < 1000; ++i)
        x += diffsim.value(our_params);
    TimerHelper::end_and_print();
    std::cout <<x <<std::endl;

    return EXIT_SUCCESS;
}

