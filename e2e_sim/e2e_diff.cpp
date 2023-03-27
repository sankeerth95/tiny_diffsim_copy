#include <iostream>
#include <vector>

#include "utils/differentiation.hpp"
#include "utils/file_utils.hpp"

#include "urdf/urdf_parser.hpp"
#include "urdf/urdf_to_multi_body.hpp"

#include "math/tiny/tiny_double_utils.h"
#include "math/tiny/tiny_algebra.hpp"

#include "visualizer/meshcat/meshcat_urdf_visualizer.h"
#include "pendulum_def.h"


typedef TinyAlgebra<double, ::TINY::DoubleUtils> MyAlgebra;

template<typename Algebra>
struct LaikagoSimEnv{
    using Scalar = typename Algebra::Scalar;
    using Vector3 = typename Algebra::Vector3;
    using Quaternion = typename Algebra::Quaternion;

    tds::World<Algebra> world;
    std::vector<tds::MultiBody<Algebra>*> mb;

    // render a plane
    tds::UrdfParser<Algebra> parser;
    std::vector<tds::UrdfStructures<Algebra> > urdf_structures;
    MeshcatUrdfVisualizer<Algebra> meshcat_viz;

    // private
    std::string plane_file_name;
    std::string laikago_file_name;


    Scalar const dt = Algebra::from_double(0.01);

    LaikagoSimEnv(): mb(2), urdf_structures(2){
        set_ground();
        set_robot_instance();
        meshcat_viz.delete_all();
    }

    void set_render_scene(){
        set_render_ground();
        set_render_robot_instance();
    }

    void update_render(){
        meshcat_viz.render();
        // meshcat_viz.sync_visual_transforms(mb[0]);
        // meshcat_viz.sync_visual_transforms(mb[1]);
    }


    void set_robot_instance(){

        mb[1] = world.create_multi_body();
        mb[1]->set_floating_base(true);

        std::string laikago_file_name;
        tds::FileUtils::find_file("laikago/laikago.urdf", laikago_file_name);
        urdf_structures[1] = parser.load_urdf(laikago_file_name);
        tds::UrdfToMultiBody<Algebra>::convert_to_multi_body(urdf_structures[1], world, *mb[1], 0);

        mb[1]->initialize();
        double knee_angle = -0.5;
        double abduction_angle = 0.2;
        std::vector<double> initial_poses = {
            0., 0., 0., 1., 0, 0, 0, 1.5,
            abduction_angle, 0., knee_angle, abduction_angle, 0., knee_angle,
            abduction_angle, 0., knee_angle, abduction_angle, 0., knee_angle,
        };

        for (int i = 0; i < 19; i++) {
            mb[1]->q_[i] = initial_poses[i];
        }
        // mb[1]->set_position(Vector3(1.5, -2.0, 0.5));
        // mb[1]->set_orientation(Quaternion(0.0, 0.0, 0.0, 1.0));
    }

    void set_ground(){
        mb[0] = world.create_multi_body();
        mb[0]->set_floating_base(false);
        tds::FileUtils::find_file("plane_implicit.urdf", plane_file_name);
        urdf_structures[0] = parser.load_urdf(plane_file_name);
        tds::UrdfToMultiBody<Algebra>::convert_to_multi_body(urdf_structures[0], world, *mb[0],0);
    }

    void set_render_ground(){
        char plane_search_path[TINY_MAX_EXE_PATH_LEN];
        tds::FileUtils::extract_path(plane_file_name.c_str(), plane_search_path, TINY_MAX_EXE_PATH_LEN);
        meshcat_viz.m_path_prefix = plane_search_path;
        meshcat_viz.convert_visuals(urdf_structures[0], "checker_purple.png", mb[0]);
    }


    void set_render_robot_instance(){
        char laikago_search_path[TINY_MAX_EXE_PATH_LEN];
        tds::FileUtils::extract_path(laikago_file_name.c_str(), laikago_search_path, TINY_MAX_EXE_PATH_LEN);
        meshcat_viz.m_path_prefix = laikago_search_path;
        meshcat_viz.convert_visuals(urdf_structures[1], "laikago_tex.jpg", mb[1]);
    }


    static const int kDim = 10;
    Scalar operator()(const std::vector<Scalar>& v) {

        tds::forward_dynamics(*mb[1], world.get_gravity());
        tds::integrate_euler_qdd(*mb[1], dt);
        world.step(dt);
        tds::integrate_euler(*mb[1], dt);
 
        // 
        return v[0] + v[1];
    }
};


int main(){

    auto sim = LaikagoSimEnv<tds::EigenAlgebra>();

    typedef tds::GradientFunctional<tds::DIFF_CPPAD_CODEGEN_AUTO, LaikagoSimEnv> GradFunc;
    // return 0;
    GradFunc::Compile();
    GradFunc diffsim;

    bool done = false;
    while(!done){

        std::vector<double> params(10, 1.0);

        // sim
        sim(params);

        sim.update_render();

        // // diffsim
        diffsim.value(params);
        // // TODO: backward

        diffsim.gradient(params);


    }


    return 0;
}
