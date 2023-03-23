#ifndef ANT_SIMOBJ_H
#define ANT_SIMOBJ_H

#include "../examples/environments/ant_environment.h"



template <typename Algebra>
struct AntBaseSim {

  using Scalar = typename Algebra::Scalar;
 
  std::vector<Scalar> operator() (const std::vector<Scalar>& v) const {
    return v;
  }

};

template<typename Algebra>
struct AntContactDiffSim : public AntContactSimulation<Algebra> {

  using Scalar = typename Algebra::Scalar;
  static const int kDim = 28;

  AntContactDiffSim(){

  }

  Scalar operator() (const std::vector<Scalar>& v) {

    auto sum = Algebra::zero();  
    for(Scalar s : AntContactSimulation<Algebra>::operator()(v)){
      sum += s;
    }
    return sum;
  }

};




#endif // ANT_SIMOBJ_H

