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

/*
 * Computes the inertia of a box-shaped rigid body.
 *
 * Assumptions:
 * - The box is a solid, homogeneous object.
 * - The mass is uniformly distributed.
 * - The inertia is computed about the center of mass.
 * - The box is aligned with the coordinate axes.
 *
 * @param mass Mass of the box.
 * @param size Dimensions of the box (length, width, height).
 * @return Inertia of the box as sva::RBInertiad.
 */
sva::RBInertiad computeBoxInertia(double mass, const Eigen::Vector3d & size)
{
  /* Compute principal moments of inertia for a box:
     I_xx = (1/12) * mass * (size.y^2 + size.z^2)
     I_yy = (1/12) * mass * (size.x^2 + size.z^2)
     I_zz = (1/12) * mass * (size.x^2 + size.y^2)
     Construct the inertia matrix as a diagonal matrix */
  double I_xx = (1.0 / 12.0) * mass * (size.y() * size.y() + size.z() * size.z());
  double I_yy = (1.0 / 12.0) * mass * (size.x() * size.x() + size.z() * size.z());
  double I_zz = (1.0 / 12.0) * mass * (size.x() * size.x() + size.y() * size.y());
  Eigen::Matrix3d inertia = Eigen::Matrix3d::Zero();
  inertia(0, 0) = I_xx;
  inertia(1, 1) = I_yy;
  inertia(2, 2) = I_zz;
  return sva::RBInertiad(mass, Eigen::Vector3d::Zero(), inertia);
}

/*
 * Computes the inertia of a sphere-shaped rigid body.
 *
 * Assumptions:
 * - The sphere is solid and homogeneous.
 * - The mass is uniformly distributed.
 * - The inertia is computed about the center of mass.
 *
 * @param mass Mass of the sphere.
 * @param radius Radius of the sphere.
 * @return Inertia of the sphere as sva::RBInertiad.
 */
sva::RBInertiad computeSphereInertia(double mass, double radius)
{
  return sva::RBInertiad(mass, Eigen::Vector3d::Zero(), (2. / 5.) * mass * radius * radius * Eigen::Matrix3d::Identity());
}

/**
 * @brief Computes the inertia of a solid cylinder.
 *
 * This function calculates the rotational inertia tensor for a cylinder given its mass, radius, and length.
 *
 * @param mass Mass of the cylinder.
 * @param radius Radius of the cylinder.
 * @param length Length of the cylinder.
 * @return sva::RBInertiad Inertia of the cylinder.
 */
sva::RBInertiad computeCylinderInertia(double mass, double radius, double length)
{
  double I_xx = (1.0 / 12.0) * mass * (3 * radius * radius + length * length);
  double I_yy = I_xx;
  double I_zz = 0.5 * mass * radius * radius;
  Eigen::Matrix3d inertia = Eigen::Matrix3d::Zero();
  inertia(0, 0) = I_xx;
  inertia(1, 1) = I_yy;
  inertia(2, 2) = I_zz;
  return sva::RBInertiad(mass, Eigen::Vector3d::Zero(), inertia);
}

/**
 * @brief Computes the inertia from a visual geometry description.
 *
 * This function determines the inertia based on the type of geometry (box, sphere, etc.) and its parameters.
 *
 * @param visual The visual geometry description.
 * @param mass The mass of the object.
 * @return sva::RBInertiad The computed inertia.
 * @throws Throws if the geometry type is unsupported.
 */
sva::RBInertiad computeInertiaFromVisual(const rbd::parsers::Visual & visual, double mass)
{
  switch(visual.geometry.type)
  {
    case rbd::parsers::Geometry::BOX:
      {
        auto geom = mc_rtc::details::getVisualGeometry<rbd::parsers::Geometry::BOX>(const_cast<rbd::parsers::Visual&>(visual));
        return computeBoxInertia(mass, geom.size);
      }
      break;
    case rbd::parsers::Geometry::SPHERE:
      {
        auto geom = mc_rtc::details::getVisualGeometry<rbd::parsers::Geometry::SPHERE>(const_cast<rbd::parsers::Visual&>(visual));
        return computeSphereInertia(mass, geom.radius);
      }
      break;
    case rbd::parsers::Geometry::CYLINDER:
      {
        auto geom = mc_rtc::details::getVisualGeometry<rbd::parsers::Geometry::CYLINDER>(const_cast<rbd::parsers::Visual&>(visual));
        return computeCylinderInertia(mass, geom.radius, geom.length);
      }
      break;
    default:
      mc_rtc::log::error_and_throw("computeIntertiaFromVisual: Unsupported geometry type {}", visual.geometry.type);
  }
}


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
  Eigen::Vector3d rpy; // orientation in radians
  Eigen::Vector3d rpyFlipped; // orientation in radians
};

auto generators = std::vector<SurfaceGen>{
  {"Front",   {1, 0, 0}, {0, M_PI_2, 0}, {0, M_PI + M_PI_2, 0}},
  {"Back",    {-1, 0, 0}, {0, -M_PI_2, 0}, {0, M_PI - M_PI_2, 0}},
  {"Left",    {0, 1, 0}, {-M_PI_2, 0, 0}, {M_PI - M_PI_2, 0, 0}},
  {"Right",   {0, -1, 0}, {M_PI_2, 0, 0}, {M_PI + M_PI_2, 0, 0}},
  {"Top",     {0, 0, 1}, {0, 0, 0}, {M_PI, 0, 0}},
  {"Bottom",  {0, 0, -1}, {M_PI, 0, 0}, {2*M_PI, 0, 0}}
};

  auto surfaces = std::vector<std::shared_ptr<Surface>>{};
  surfaces.reserve(generators.size());

  for(const auto & [name, dir, rpyExterior, rpyInterior] : generators)
  {


    Eigen::Matrix3d rotation = mc_rbdyn::rpyToMat(rpyExterior).inverse();
    Eigen::Matrix3d rotationFlipped = mc_rbdyn::rpyToMat(rpyInterior).inverse();

    if(visual.geometry.type == rbd::parsers::Geometry::SPHERE)
    {
      auto geom = mc_rtc::details::getVisualGeometry<rbd::parsers::Geometry::SPHERE>(const_cast<rbd::parsers::Visual&>(visual));
      auto d = geom.radius;
      surfaces.emplace_back(std::make_shared<mc_rbdyn::PlanarSurface>
         (
            name + "_exterior",
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
      surfaces.emplace_back(surfaces.back()->copy());
      surfaces.back()->name(name + "_interior");
      surfaces.back()->X_b_s(sva::PTransformd{rotationFlipped, dir * d});
    }
    else if(visual.geometry.type == rbd::parsers::Geometry::BOX)
    {
      auto geom = mc_rtc::details::getVisualGeometry<rbd::parsers::Geometry::BOX>(const_cast<rbd::parsers::Visual&>(visual));
      const auto & size = geom.size;
      double hx = size.x() / 2.0;
      double hy = size.y() / 2.0;
      double hz = size.z() / 2.0;

      std::vector<std::pair<double, double>> points;
      if(std::abs(dir.x()) == 1) // Front/Back face (YZ plane)
      {
        points = {
          {-hz, -hy},
          {hz, -hy},
          {hz, hy},
          {-hz, hy}
        };
      }
      else if(std::abs(dir.y()) == 1) // Left/Right face (XZ plane)
      {
        points = {
          {-hx, -hz},
          {hx, -hz},
          {hx, hz},
          {-hx, hz}
        };
      }
      else if(std::abs(dir.z()) == 1) // Top/Bottom face (XY plane)
      {
        points = {
          {-hx, -hy},
          {hx, -hy},
          {hx, hy},
          {-hx, hy}
        };
      }

      surfaces.emplace_back(std::make_shared<mc_rbdyn::PlanarSurface>
          (
           name + "_exterior",
           visual.name, // bodyName
           sva::PTransformd(rotation, dir * (std::abs(dir.x()) ? hx : std::abs(dir.y()) ? hy : hz)), // X_b_s

           "plastic", // materialName
           points
          ));
      surfaces.emplace_back(surfaces.back()->copy());
      surfaces.back()->name(name + "_interior");
      surfaces.back()->X_b_s(sva::PTransformd{rotationFlipped, dir * (std::abs(dir.x()) ? hx : std::abs(dir.y()) ? hy : hz)});

    }
  }

  return surfaces;
}

inline RobotModulePtr robotModuleFromVisual(
    const std::string & name,
    const rbd::parsers::Visual & visual,
    const sva::RBInertiad & inertia,
    bool isFixed = true
    )
{
  auto pr = rbd::parsers::ParserResult{};
  pr.visual[name] = { visual };
  pr.collision[name] = { visual };

  rbd::Body body(inertia, name);
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

inline RobotModulePtr robotModuleFromVisual(
    const std::string & name,
    const rbd::parsers::Visual & visual,
    double mass,
    bool isFixed = true
    )
{
    return robotModuleFromVisual(name, visual, computeInertiaFromVisual(visual, mass), isFixed);
}


inline RobotModulePtr robotModuleFromVisualConfig(const std::string & name,
    mc_rtc::Configuration config)
{
  double mass = 0;
  std::optional<sva::RBInertiad> inertia = std::nullopt;
  if(auto inertiaC = config.find("inertia"))
  {
    if(auto mass_ = inertiaC->find("mass"))
    {
      mass = *mass_;
    }
    else
    {
      mc_rtc::log::error_and_throw("robotModuleFromVisualConfig: inertia provided but no mass");
    }

    if(auto spatialMomentum = inertiaC->find("momentum"))
    {
      if(auto inertia_ = inertiaC->find("inertia"))
      {
        inertia = sva::RBInertiad(mass, *spatialMomentum, *inertia_);
      }
      else
      {
        mc_rtc::log::error_and_throw("robotModuleFromVisualConfig: momentum provided but no inertia matrix");
      }
    }
  }

  auto visual = static_cast<rbd::parsers::Visual>(config);
  if(inertia)
  {
    return robotModuleFromVisual(name, visual, *inertia, config("fixed", false));
  }
  else
  {
    return robotModuleFromVisual(name, visual, mass, config("fixed", false));
  }
}

}
