#include <iostream>
#include <vector>
#include "utils/differentiation.hpp"
#include "my_timer.h"


#define KDIM 1000
#define KDepth 10





#define MEASURE_ITERS 1000

template<typename Algebra>
struct Simulation{
    using Scalar = typename Algebra::Scalar;
    // size of v
    static const int kDim = KDIM;
    Scalar operator ()(const std::vector<Scalar>& v) const  {
        Scalar sum = Algebra::zero();//v[0];

        std::vector<Scalar> u(kDim, Algebra::zero());
        for(int i = 0; i < KDepth; i++){
            if( i == 0)
            {
                for(int i = 0; i < kDim-1; i++){
                    u[i] += v[i]*v[i+1];
                }
                u[kDim-1] += v[kDim-1];
            } else {
                for(int i = 0; i < kDim-1; i++){
                    u[i] += u[i]*u[i+1];
                }
                u[kDim-1] += u[kDim-1];

            }
        }      
        for(int i = 0; i < kDim; i++){
            sum += u[i];
        }
        return sum;
    }
};

template<typename Algebra>
struct SimulationWide{
    using Scalar = typename Algebra::Scalar;
    // size of v
    static const int kDim = KDIM;
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




__attribute__((noinline))
std::vector<double> backward_manual_wide(std::vector<double>& v){//}, std::vector<double>& jacc){
    
    std::vector<double> jac(v.size(), 0.0);
    
    for(int i = 2; i < KDIM-2; i+=2){
        jac[i] = v[i+1]*v[i+2]*v[i+3] + v[i-2]*v[i-1]*v[i+1]; 
        jac[i+1] = v[i-2]*v[i-1]*v[i] + v[i]*v[i+2]*v[i+3]; 
    }

    return jac;
}




// sum = v[0]*v[1]*v[2]*v[3] + v[2]*v[3]*v[4]*v[5] + ... v[];
__attribute__((noinline))
std::vector<double> backward_manual(std::vector<double>& v){//}, std::vector<double>& jacc){
    
    std::vector<double> jac(v.size(), 0.0);
    
    for(int i = 2; i < KDIM-2; i+=2){
        jac[i] = v[i+1]*v[i+2]*v[i+3] + v[i-2]*v[i-1]*v[i+1]; 
        jac[i+1] = v[i-2]*v[i-1]*v[i] + v[i]*v[i+2]*v[i+3]; 
    }

    return jac;
}



int main(int argc, char* argv[]) {
  // first generate a groundtruth trajectory from a pendulum with all link
  // lengths = 1.5
  Simulation<tds::EigenAlgebra> sim;

  std::vector<double> our_params(KDIM, 1.7);  // initial guess
    double x = 0;
  TimerHelper::start_timer("double forward pass");
  for(int i = 0; i < 1000; ++i)
      x += sim(our_params);
  TimerHelper::end_and_print();
    std::cout <<x <<std::endl;
//   std::vector<double> true_params(kNumLinks, 1.5);
//   sim.rollout(true_params, &groundtruth_states);

  // run gradient descent

    typedef tds::GradientFunctional<tds::DIFF_CPPAD_AUTO, Simulation> GradFun;
    // typedef tds::GradientFunctional<tds::DIFF_CPPAD_CODEGEN_AUTO, Simulation> GradFun;
    // GradFun::Compile();

//   typedef tds::GradientFunctional<tds::DIFF_CPPAD_AUTO, Simulation> GradFun;

    GradFun diffsim;

// 
    TimerHelper::start_timer("AD forward pass");
    for(int i = 0; i < MEASURE_ITERS; ++i)
        x += diffsim.value(our_params);
    TimerHelper::end_and_print();
    std::cout <<x <<std::endl;



    TimerHelper::start_timer("double forward pass");
    for(int i = 0; i < MEASURE_ITERS; ++i)
        x += sim(our_params);
    TimerHelper::end_and_print();
    std::cout <<x <<std::endl;


    TimerHelper::start_timer("AD forward pass");
    for(int i = 0; i < MEASURE_ITERS; ++i)
        x += diffsim.value(our_params);
    TimerHelper::end_and_print();
    std::cout <<x <<std::endl;

    TimerHelper::start_timer("AD forward pass");
    for(int i = 0; i < MEASURE_ITERS; ++i)
        x += diffsim.value(our_params);
    TimerHelper::end_and_print();
    std::cout <<x <<std::endl;

    std::vector<double> jac(our_params.size(), 0.0);
    x = 0.0;
    TimerHelper::start_timer("backward manual");
    for(int i = 0; i < MEASURE_ITERS; ++i){
        auto jac_c = backward_manual(our_params);//, jac);
        for(int j = 0; j < jac_c.size(); j++)
            x += jac_c[j];
    }
    TimerHelper::end_and_print();
    std::cout <<x <<std::endl;


    x = 0.0;
    TimerHelper::start_timer("AD backward pass");
    for(int i = 0; i < MEASURE_ITERS; ++i){
        auto jac_c = diffsim.gradient(our_params);
        printf("jac, v size = %d, %d\n", jac_c.size(), our_params.size());
        for(int j = 0; j < jac_c.size(); j++)
            x += jac_c[j];
    }
    TimerHelper::end_and_print();
    std::cout <<x <<std::endl;



    x = 0.0;
    TimerHelper::start_timer("AD backward pass");
    for(int i = 0; i < MEASURE_ITERS; ++i){
        auto jac_c = diffsim.gradient(our_params);
        for(int j = 0; j < jac_c.size(); j++)
            x += jac_c[j];
    }
    TimerHelper::end_and_print();
    std::cout <<x <<std::endl;



    x = 0.0;
    TimerHelper::start_timer("backward manual");
    for(int i = 0; i < MEASURE_ITERS; ++i){
        auto jac_c = backward_manual(our_params);//, jac);
        for(int j = 0; j < jac_c.size(); j++)
            x += jac_c[j];
    }
    TimerHelper::end_and_print();
    std::cout <<x <<std::endl;



    return EXIT_SUCCESS;
}

