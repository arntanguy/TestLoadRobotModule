#pragma once
#include <mc_rbdyn/PlanarSurface.h>
#include <mc_rbdyn/RobotModule.h>
#include <RBDyn/parsers/urdf.h>
#include <SpaceVecAlg/SpaceVecAlg>
// For unique_path only
#include <boost/filesystem.hpp>
namespace bfs = boost::filesystem;
#include <filesystem>
namespace fs = std::filesystem;
#include <mc_rbdyn/Surface.h>

#include <mc_rtc/visual_utils.h> // for makeVisualSphere...


namespace mc_rbdyn
{

std::string make_temporary_path(const std::string & prefix)
{
  auto tmp = fs::temp_directory_path();
  auto pattern = fmt::format("{}-%%%%-%%%%-%%%%-%%%%", prefix);
  // std::filesystem does not have a unique_path function in c++17
  // keep boost around for now
  auto out = tmp / bfs::unique_path(pattern).string();
  fs::create_directories(out);
  return out.string();
}

std::string surfaceToXML(const Surface & surface)
{
  const auto & X_b_s = surface.X_b_s();
  Eigen::Vector3d xyz = X_b_s.translation();
  Eigen::Vector3d rpy = mc_rbdyn::rpyFromMat(X_b_s.rotation().transpose()); // Implement or use existing

  if(surface.type() == "planar")
  {
    auto & planarSurface = static_cast<const mc_rbdyn::PlanarSurface &>(surface);
  std::string xml = fmt::format(
    "<planar_surface name=\"{}\" link=\"{}\">\n"
    "  <origin rpy=\"{:.8f} {:.8f} {:.8f}\" xyz=\"{:.8f} {:.8f} {:.8f}\" />\n"
    "  <points>\n",
    surface.name(), surface.bodyName(),
    rpy.x(), rpy.y(), rpy.z(),
    xyz.x(), xyz.y(), xyz.z()
  );

  for(const auto & pt : planarSurface.planarPoints())
  {
    xml += fmt::format("    <point xy=\"{:.8f} {:.8f}\" />\n", pt.first, pt.second);
  }
  xml += "  </points>\n";

  if(!surface.materialName().empty())
  {
    xml += fmt::format("  <material name=\"{}\" />\n", surface.materialName());
  }

  xml += "</planar_surface>\n";
  return xml;
  }
  mc_rtc::log::error_and_throw("surfaceToXML: Unsupported surface type {}", surface.type());
}

std::string surfacesToXML(const std::string & robotName, const std::vector<std::shared_ptr<Surface>> & surfaces)
{
  std::string xml = fmt::format("<?xml version=\"1.0\" ?>\n<robot name=\"{}\">\n", robotName);
  for(const auto & s : surfaces)
  {
    xml += surfaceToXML(*s);
  }
  xml += "</robot>\n";
  return xml;
}


std::vector<std::shared_ptr<Surface>> genSurfacesFromVisual(const rbd::parsers::Visual & visual)
{
  struct SurfaceGen
  {
    std::string name;
    Eigen::Vector3d direction;
  };
  auto generators = std::vector<SurfaceGen>{
    {"Front", {1, 0, 0}},
    {"Back", {-1, 0, 0}},
    {"Left", {0, 1, 0}},
    {"Right", {0, -1, 0}},
    {"Top", {0, 0, 1}},
    {"Bottom", {0, 0, -1}}
  };

  auto surfaces = std::vector<std::shared_ptr<Surface>>{};
  surfaces.reserve(generators.size());

  double d = 0;
  if(visual.geometry.type == rbd::parsers::Geometry::SPHERE)
  {
    auto geom = mc_rtc::details::getVisualGeometry<rbd::parsers::Geometry::SPHERE>(const_cast<rbd::parsers::Visual&>(visual));
    d = geom.radius;
  }

  for(const auto & [name, dir] : generators)
  {

    Eigen::Vector3d z_axis = dir.normalized();
    Eigen::Vector3d temp = (std::abs(z_axis.x()) < 0.99) ? Eigen::Vector3d::UnitX() : Eigen::Vector3d::UnitY();
    Eigen::Vector3d x_axis = temp.cross(z_axis).normalized();
    Eigen::Vector3d y_axis = z_axis.cross(x_axis).normalized();

    Eigen::Matrix3d rotation;
    rotation.col(0) = x_axis;
    rotation.col(1) = y_axis;
    rotation.col(2) = z_axis; // z is aligned with dir

    surfaces.emplace_back(std::make_shared<mc_rbdyn::PlanarSurface>
       (
          name,
          visual.name, // bodyName
          sva::PTransformd(rotation, dir * d), // X_b_s

          "plastic", // materialName
          std::vector<std::pair<double, double>>{
          {-d, -d},
          {d, -d},
          {d, d},
          {-d, d}
          }
        ));
  }

  return surfaces;
}

inline RobotModulePtr robotModuleFromVisual(
    const std::string & name,
    const rbd::parsers::Visual & visual,
    bool isFixed = true
    )
{
  auto pr = rbd::parsers::ParserResult{};
  pr.visual[name] = { visual };
  pr.collision[name] = { visual };

  auto rbInertia = sva::RBInertiad{};
  rbd::Body body(rbInertia, name);
  pr.mbg.addBody(body);

  pr.mb = pr.mbg.makeMultiBody(name, isFixed);
  pr.mbc = rbd::MultiBodyConfig(pr.mb);
  pr.mbc.zero(pr.mb);

  // Create the robot module
  auto rmPtr = std::make_shared<RobotModule>(name, pr);
  auto & rm = *rmPtr;

  // generate surfaces
  auto surfaces = genSurfacesFromVisual(visual);

  // Module is created, but we still need to export it for other tools (visualization, ...)
  auto saveModule = [&]()
  {
    auto path = make_temporary_path(name);

    auto urdf_path = (fs::path(path) / "urdf" / (name + ".urdf")).string();
    auto urdf_dir = fs::path(urdf_path).parent_path();
    if(!fs::exists(urdf_dir)) { fs::create_directories(urdf_dir); }

    auto rsdf_dir = fs::path(path) / "rsdf";
    if(!fs::exists(rsdf_dir)) { fs::create_directories(rsdf_dir); }
    auto surfaces_xml = surfacesToXML(name, surfaces);
    {
      auto rsdf_path = (rsdf_dir / (name + ".rsdf")).string();
      std::ofstream ofs(rsdf_path);
      ofs << surfaces_xml;
    }


    std::ofstream ofs(urdf_path);
    ofs << rbd::parsers::to_urdf(pr);

    // Generate a module file so that the generated RobotModule can be re-used
    // XXX: currently devices are not saved so strictly speaking the generated yaml robot
    // won't be exactly identical to the one generated here
    {
      std::string module_yaml = fmt::format("{}/{}.yaml", path, name);
      rm.path = path;
      rm.urdf_path = urdf_path;
      rm.rsdf_dir = rsdf_dir.string();
      rm._parameters = {"json", module_yaml};
      rm._canonicalParameters = rm._parameters;

      auto yaml = mc_rtc::ConfigurationLoader<mc_rbdyn::RobotModule>::save(rm, false, {}, rm.mb.joint(0).dof() == 0);
      yaml.save(module_yaml);
      mc_rtc::log::info("Result module for {} in: {}", name, module_yaml);
    }
  };
  saveModule();

  return rmPtr;
}
}
