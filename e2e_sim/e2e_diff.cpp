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


template<typename Algebra>
struct Simulator{
    using Scalar = typename Algebra::Scalar;

    static const int kDim = 10;
    Scalar operator()(const std::vector<Scalar>& v) const {
        // . .. 
        return v[0] + v[1];
    }
};


// prepare a world
template<typename Algebra>
auto prepare(){
    // return Simulator<Algebra>();
    return Pendulum<Algebra>();

}

template<typename Algebra>
auto init_visualize(){
    MeshcatUrdfVisualizer<Algebra> meshcat_viz;
    meshcat_viz.delete_all();

    tds::World<Algebra> world;

    // render a plane
    {
        tds::MultiBody<Algebra>& plane_mb = *world.create_multi_body();
        plane_mb.set_floating_base(false);

        tds::UrdfParser<Algebra> parser;
        std::string plane_file_name;
        tds::FileUtils::find_file("plane_implicit.urdf", plane_file_name);    
        tds::UrdfStructures<Algebra> plane_urdf_structures = parser.load_urdf(plane_file_name);
        tds::UrdfToMultiBody<Algebra>::convert_to_multi_body(plane_urdf_structures, world, plane_mb,0);

        char plane_search_path[TINY_MAX_EXE_PATH_LEN];
        tds::FileUtils::extract_path(plane_file_name.c_str(), plane_search_path, TINY_MAX_EXE_PATH_LEN);
        meshcat_viz.m_path_prefix = plane_search_path;

        std::string texture_path = "checker_purple.png";
        meshcat_viz.convert_visuals(plane_urdf_structures, texture_path,&plane_mb);
    }

    // render an object
    {
        tds::MultiBody<Algebra>& obj_mb = *world.create_multi_body();
        obj_mb.set_floating_base(true);

        tds::UrdfParser<Algebra> parser;
        std::string obj_file_name;
        tds::FileUtils::find_file("laikago/laikago_toes_zup.urdf", obj_file_name);
        tds::UrdfStructures<Algebra> obj_urdf_structures = parser.load_urdf(obj_file_name);
        tds::UrdfToMultiBody<Algebra>::convert_to_multi_body(obj_urdf_structures, world, obj_mb, 0);

        char obj_search_path[TINY_MAX_EXE_PATH_LEN];
        tds::FileUtils::extract_path(obj_file_name.c_str(), obj_search_path, TINY_MAX_EXE_PATH_LEN);
        meshcat_viz.m_path_prefix = obj_search_path;

        obj_mb.initialize();
        std::string texture_path = "laikago_tex.jpg";
        meshcat_viz.convert_visuals(obj_urdf_structures, texture_path, &obj_mb);

        using Vector3 = typename Algebra::Vector3;
        using Quaternion = typename Algebra::Quaternion;

        double knee_angle = -0.5;
        double abduction_angle = 0.2;
        std::vector<double> initial_poses = {
            0., 0., 0., 1., 0, 0, 0, 1.5,
            abduction_angle, 0., knee_angle, abduction_angle, 0., knee_angle,
            abduction_angle, 0., knee_angle, abduction_angle, 0., knee_angle,
        };
        for (int i = 0; i < 19; i++) {
            obj_mb.q_[i] = initial_poses[i];
        }
        obj_mb.set_position(Vector3(1.5, -2.0, 0.5));
        obj_mb.set_orientation(Quaternion(0.0, 0.0, 0.0, 1.0));
    }

    return meshcat_viz;
}

template<typename Algebra>
void render(MeshcatUrdfVisualizer<Algebra>& meshcat_viz){
    meshcat_viz.render();
}



int main(){

    auto sim = prepare<tds::EigenAlgebra>();

    bool done = false;
    
    // initalize meshcat
    auto meshcat_viz = init_visualize<tds::EigenAlgebra>();

    while(!done){

        std::vector<double> params(10, 1.0);
        sim(params);


        // TODO: backward
        // ..


        render<tds::EigenAlgebra>(meshcat_viz);
    }


    // {
    //     typedef tds::GradientFunctional<tds::DIFF_CPPAD_CODEGEN_AUTO, Simulator> GradFunc;

    //     GradFunc::Compile();
    //     GradFunc diffsim;

    //     // // initialize simulation env - 
    //     // // auto world = prepare();

    //     // std::vector<double> our_params(10, 1.0);

    //     // // forward pass 
    //     diffsim.value(params);

    //     // // backward function - graident using cppad
    //     // // cppad code ... 
    //     diffsim.gradient(params);
    // }


    // modified codegen process - efficient ad code
    //...

    return 0;
}
