#include <iostream>
#include "utils/differentiation.hpp"

#include "urdf/urdf_parser.hpp"
#include "urdf/urdf_to_multi_body.hpp"
#include "urdf/urdf_cache.hpp"

// #include "utils.h"


template <typename Algebra>
struct LaikagoSimWrapper {
    using Scalar = typename Algebra::Scalar;
    tds::World<Algebra> world_;
    tds::UrdfCache<Algebra> cache_;
    tds::MultiBody<Algebra> *mb_system;

    int output_dim;
    int n_timesteps;
    float dt = 1.0f/60.0f;

    LaikagoSimWrapper(){
        std::cout << "LaikagoSimWrapper" << std::endl;

        bool is_floating = true;
        mb_system = cache_.construct("laikago/laikago.urdf", world_, false, is_floating);

    }


    ~LaikagoSimWrapper(){
        // delete mb_system;
    }

    // trace the operation here, and compile it later..... long trace operations?????
    std::vector<Scalar> operator() (const std::vector<Scalar>& v) const{

        // set state
        for(int i = 0; i < system->dof(); i++) mb_system->q_[i] = v[i];
        for(int i = 0; i < system->dof_qd(); i++) mb_system->qd_[i] = v[i + system->dof()];

        std::vector<Scalar> res(output_dim);
        for(int i = 0; i < n_timesteps; i++) {
            int qd_offset = system->is_floating() ? 6 : 0;

            qstd::vector<double> q_targets;
            q_targets.resize(system->tau.size());


            // compute torque
            {
                mb_system->tau[0] = 0;
            }


            // update state 
            // forward_dynamics
            tds::forward_dynamics(*mb_system, world.get_gravity()); // robotics specific thing - 
            mb_system->clear_forces();

            // semi implicit euler - update vel, check system state, update pos
            tds::integrate_euler_qdd(*mb_system, dt);
            world.step(dt);
            tds::integrate_euler(*mb_system, dt);



            // set state to output result
            for(int i = 0; i < system->dof(); i++) res[i] = mb_system->q_[i];

        }



        return v;
    }
};


int main(){

    LaikagoSimWrapper<tds::EigenAlgebra> sim;

    std::vector<double> v = {1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20};

    auto x = sim(v);

    return 0;
}




