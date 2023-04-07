#include <iostream>
#include <vector>

#include "utils/differentiation.hpp"
#include "utils/file_utils.hpp"

#include "urdf/urdf_parser.hpp"
#include "urdf/urdf_to_multi_body.hpp"

#include "math/tiny/tiny_double_utils.h"
#include "math/tiny/tiny_algebra.hpp"

// #include "visualizer/meshcat/meshcat_urdf_visualizer.h"
#include "opengl_urdf_visualizer.h"
// #include "tiny_visual_instance_generator.h"
#include "pendulum_def.h"

std::string LAIKAGO_URDF_NAME="laikago/laikago_toes.urdf";

typedef TinyAlgebra<double, ::TINY::DoubleUtils> MyAlgebra;

template<typename Algebra>
struct Visualizer_stub{
    // typename Algebra::MultiBody;

    void delete_all(){ }
    void render(){ }
    void sync_visual_transforms(tds::MultiBody<Algebra>* mb){}
    void convert_visuals(tds::UrdfStructures<Algebra>& urdf_structures, std::string texture_file_name, tds::MultiBody<Algebra>* mb){}

    std::string m_path_prefix;

};



template<typename Algebra>
struct LaikagoSimEnv{
    using Scalar = typename Algebra::Scalar;
    using Vector3 = typename Algebra::Vector3;
    using Quaternion = typename Algebra::Quaternion;
    using Visualizer = Visualizer_stub<Algebra>;
    // using Visualizer = OpenGLUrdfVisualizer<Algebra>;

    tds::World<Algebra> world;
    std::vector<tds::MultiBody<Algebra>*> mb;

    // render a plane
    tds::UrdfParser<Algebra> parser;
    std::vector<tds::UrdfStructures<Algebra> > urdf_structures;
    // MeshcatUrdfVisualizer<Algebra> visualizer;
    Visualizer visualizer;

    // private
    std::string plane_file_name;
    std::string laikago_file_name;


    Scalar const dt = Algebra::from_double(0.01);

    LaikagoSimEnv(): mb(2), urdf_structures(2){
        set_ground();
        set_robot_instance();
        visualizer.delete_all();
        set_render_scene();
    }

    void set_render_scene(){
        set_render_ground();
        set_render_robot_instance();
    }

    void update_render(){
        visualizer.render();
        // visualizer.sync_visual_transforms(mb[0]);
        visualizer.sync_visual_transforms(mb[1]);
    }


    void set_robot_instance(){

        mb[1] = world.create_multi_body();
        mb[1]->set_floating_base(true);

        tds::FileUtils::find_file(LAIKAGO_URDF_NAME, laikago_file_name);
        urdf_structures[1] = parser.load_urdf(laikago_file_name);
        tds::UrdfToMultiBody<Algebra>::convert_to_multi_body(urdf_structures[1], world, *mb[1], 0);

        mb[1]->initialize();
        double knee_angle = -0.5;
        double abduction_angle = 0.2;
        std::vector<double> initial_poses = {
            0., 0., 0., 1., 0, 0, 0, 0.52,
            abduction_angle, 0., knee_angle, abduction_angle, 0., knee_angle,
            abduction_angle, 0., knee_angle, abduction_angle, 0., knee_angle,
        };

        for (int i = 0; i < 19; i++) {
            mb[1]->q_[i] = initial_poses[i];
        }
        // mb[1]->set_position(Vector3(0, 0, 0.52)); # Equivalent to setting q_[4..6]
        // mb[1]->set_orientation(Quaternion(0.0, 0.0, 0.0, 1.0)); # Equivalent to setting q_[0..3]
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
        visualizer.m_path_prefix = plane_search_path;
        visualizer.convert_visuals(urdf_structures[0], "checker_purple.png", mb[0]);
    }


    void set_render_robot_instance(){
        char laikago_search_path[TINY_MAX_EXE_PATH_LEN];
        tds::FileUtils::extract_path(laikago_file_name.c_str(), laikago_search_path, TINY_MAX_EXE_PATH_LEN);
        visualizer.m_path_prefix = laikago_search_path;
        visualizer.convert_visuals(urdf_structures[1], "laikago_tex.jpg", mb[1]);
    }


    static const int qDim = 1;
    static const int qdDim = 1;
    static const int kDim = qDim + qdDim;

    Scalar operator()(const std::vector<Scalar>& v) {

        for(int i = 0; i < qDim; i++){
            mb[1]->q(i) = v[i];
        }
        for(int i = 0; i < qDim; i++){
            mb[1]->qd(i) = v[qDim+i];
        }

        tds::forward_dynamics(*mb[1], world.get_gravity());
        tds::integrate_euler_qdd(*mb[1], dt);
        world.step(dt);
        tds::integrate_euler(*mb[1], dt);

        Scalar res  = Algebra::zero();
        for(int i = 0; i < qDim; i++){
            res += mb[1]->q(i);
        }
        
        for(int i = 0; i < qdDim; i++){
            res += mb[1]->qd(i);
        }
        return res;
    }
};

void wrapper(std::vector<double>& v, LaikagoSimEnv<tds::EigenAlgebra>& sim) {
    sim(v);
}

extern void __enzyme_autodiff(void*, std::vector<double>&, LaikagoSimEnv<tds::EigenAlgebra>&);


int main(){

    auto sim = LaikagoSimEnv<tds::EigenAlgebra>();

    typedef tds::GradientFunctional<tds::DIFF_ENZYME, LaikagoSimEnv> GradFunc;
    // typedef tds::GradientFunctional<tds::DIFF_CPPAD_AUTO, LaikagoSimEnv> GradFunc;
    // typedef tds::GradientFunctional<tds::DIFF_CPPAD_CODEGEN_AUTO, LaikagoSimEnv> GradFunc;
    // GradFunc::Compile();

    GradFunc diffsim;

    bool done = false;
    while(!done){
        std::vector<double> v(LaikagoSimEnv<tds::EigenAlgebra>::kDim, 1.0);
        // sim
        sim(v);
        sim.update_render();

        // diffsim.value(v);
        // diffsim.gradient(v);
        __enzyme_autodiff((void *) wrapper, v, sim);

    }
    return 0;
}



// print out 
// adj_f1()
// adj_f2()
// .... 

