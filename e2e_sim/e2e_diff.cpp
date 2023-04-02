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
struct LaikagoSimEnv{
    using Scalar = typename Algebra::Scalar;
    using Vector3 = typename Algebra::Vector3;
    using Quaternion = typename Algebra::Quaternion;
    using TVisualLinkInfo = typename OpenGLUrdfVisualizer<Algebra>::TinyVisualLinkInfo;

    tds::World<Algebra> world;
    std::vector<tds::MultiBody<Algebra>*> mb;

    // render a plane
    tds::UrdfParser<Algebra> parser;
    std::vector<tds::UrdfStructures<Algebra> > urdf_structures;
    // MeshcatUrdfVisualizer<Algebra> visualizer;
    OpenGLUrdfVisualizer<Algebra> visualizer;

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
        set_opengl_visualizer();
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

    void set_opengl_visualizer(){
        int num_total_threads = 1;
        std::vector<int> visual_instances;
        std::vector<int> num_instances;
        int num_base_instances = 0;
        
        for (int t = 0;t< num_total_threads;t++)
        {
            ::TINY::TinyVector3f pos(0, 0, 0);
            ::TINY::TinyQuaternionf orn(0, 0, 0, 1);
            ::TINY::TinyVector3f scaling(1, 1, 1);
            int uid = urdf_structures[1].base_links[0].urdf_visual_shapes[0].visual_shape_uid;
            TVisualLinkInfo& vis_link = visualizer.m_b2vis[uid];
            int instance = -1;
            int num_instances_per_link = 0;
            for (int v = 0; v < vis_link.visual_shape_uids.size(); v++)
            {
                int sphere_shape = vis_link.visual_shape_uids[v];
                ::TINY::TinyVector3f color(1, 1, 1);
                //visualizer.m_b2vis
                instance = visualizer.m_opengl_app.m_renderer->register_graphics_instance(
                    sphere_shape, pos, orn, color, scaling);
                visual_instances.push_back(instance);
                num_instances_per_link++;
                mb[1]->visual_instance_uids().push_back(instance);
            }
            num_base_instances = num_instances_per_link;

            for (int i = 0; i < mb[1]->num_links(); ++i) {
                
                int uid = urdf_structures[1].links[i].urdf_visual_shapes[0].visual_shape_uid;
                TVisualLinkInfo& vis_link = visualizer.m_b2vis[uid];
                int instance = -1;
                int num_instances_per_link = 0;
                for (int v = 0; v < vis_link.visual_shape_uids.size(); v++)
                {
                    int sphere_shape = vis_link.visual_shape_uids[v];
                    ::TINY::TinyVector3f color(1, 1, 1);
                    //visualizer.m_b2vis
                    instance = visualizer.m_opengl_app.m_renderer->register_graphics_instance(
                        sphere_shape, pos, orn, color, scaling);
                    visual_instances.push_back(instance);
                    num_instances_per_link++;

                    mb[1]->links_[i].visual_instance_uids.push_back(instance);
                }
                num_instances.push_back(num_instances_per_link);
            }
        }
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

    // typedef tds::GradientFunctional<tds::DIFF_CPPAD_CODEGEN_AUTO, LaikagoSimEnv> GradFunc;
    // return 0;
    // GradFunc::Compile();
    // GradFunc diffsim;

    bool done = false;
    while(!done){

        std::vector<double> params(10, 1.0);

        // sim
        sim(params);

        sim.update_render();

        // // // diffsim
        // diffsim.value(params);
        // // // TODO: backward

        // diffsim.gradient(params);


    }


    return 0;
}
