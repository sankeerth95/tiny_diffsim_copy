#include <chrono>
#include <vector>
#include "utils/differentiation.hpp"

#include "ant_simobj.h"
#include "my_timer.h"

#include "math/tiny/tiny_double_utils.h"
#include "math/tiny/tiny_algebra.hpp"


#include "utils/file_utils.hpp"
#include "urdf/urdf_parser.hpp"
#include "urdf/urdf_to_multi_body.hpp"

// meshcat visualizer
#include "visualizer/meshcat/meshcat_urdf_visualizer.h"

typedef double TinyDualScalar;
typedef double MyScalar;
typedef ::TINY::DoubleUtils MyTinyConstants;
typedef TinyAlgebra<double, MyTinyConstants> MyAlgebra;


int main(int argc, char* argv[]) {

  // TinyRendererUrdfVisualizer<MyAlgebra> meshcat_viz;
  MeshcatUrdfVisualizer<MyAlgebra>  meshcat_viz;
  meshcat_viz.delete_all();
  tds::World<MyAlgebra> world;
  tds::MultiBody<MyAlgebra>& plane_mb = *world.create_multi_body();
  plane_mb.set_floating_base(false);
  
  {
    tds::UrdfParser<MyAlgebra> parser;
    std::string plane_file_name;
    tds::FileUtils::find_file("plane_implicit.urdf", plane_file_name);    
    tds::UrdfStructures<MyAlgebra> plane_urdf_structures = parser.load_urdf(plane_file_name);
    tds::UrdfToMultiBody<MyAlgebra>::convert_to_multi_body(plane_urdf_structures, world, plane_mb,0);


    std::string texture_path = "checker_purple.png";
    char plane_search_path[TINY_MAX_EXE_PATH_LEN];
    tds::FileUtils::extract_path(plane_file_name.c_str(), plane_search_path, TINY_MAX_EXE_PATH_LEN);
    meshcat_viz.m_path_prefix = plane_search_path;
    meshcat_viz.convert_visuals(plane_urdf_structures, texture_path,&plane_mb);
  }



  {
    std::string urdf_name = "gym/ant_org_xyz_xyzrot.urdf";//ant_org.urdf";
    std::string m_urdf_filename;
    tds::FileUtils::find_file(urdf_name, m_urdf_filename);

   
    tds::UrdfCache<MyAlgebra> cache;
    bool is_floating = false;
    tds::MultiBody<MyAlgebra>* mb_ = cache.construct(m_urdf_filename, world, false, is_floating);
    mb_->base_X_world().set_identity();

    // world.default_friction = 1;
    // world.get_mb_constraint_solver()->keep_all_points_ = true;
    // world.get_mb_constraint_solver()->pgs_iterations_ = 1;
    
    // Scalar combinedContactStiffness ( 1.);
    // Scalar combinedContactDamping ( .1);

    // Scalar denom = (dt * combinedContactStiffness + combinedContactDamping);
    // if (denom < 1e-6)
		// {
    //   denom = 1e-6;
    // }
    // Scalar cfm = Scalar(1) / denom;
    // Scalar erp = (dt * combinedContactStiffness) / denom;

    // world.get_mb_constraint_solver()->cfm_ = cfm;
    // world.get_mb_constraint_solver()->erp_ = erp;

    // for (const auto link : *mb_) {
    // //just copy the link world transform. Still have to multiple with visual transform for each instance.
    //     if (link.X_visuals.size())
    //     {
    //         num_visual_links_++;
    //     }
    // }

    // std::string texture_path = "checker_purple.png";
    // meshcat_viz.convert_visuals(cache.retrieve(urdf_name), texture_path, mb_);
  }




  // meshcat_viz.convert_visuals(urdf_structures, texture_path,&mb);
  bool done = false;
  while(!done){
  
    // meshcat_viz.sync_visual_transforms2(env.contact_sim.mb_,env.sim_state_with_graphics, env.contact_sim.input_dim());
    meshcat_viz.render();
  }

  // differentiable part

  // typedef tds::GradientFunctional<tds::DIFF_CPPAD_CODEGEN_AUTO, AntContactDiffSim> GradFunc;
  // GradFunc::Compile();
  // // typedef tds::GradientFunctional<tds::DIFF_CPPAD_AUTO, AntContactDiffSim> GradFunc;
  // GradFunc diffsim;
  // std::vector<double> our_params(AntContactDiffSim::kDim, 1.0);  // initial guess
  // diffsim.value(our_params);
  // diffsim.gradient(our_params);

  return EXIT_SUCCESS;
}






