#include <ignition/plugin/Register.hh>
#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/EntityComponentManager.hh>
#include <ignition/gazebo/EventManager.hh>
#include <ignition/gazebo/Types.hh>
#include <ignition/gazebo/components/Pose.hh>
#include <ignition/gazebo/components/Name.hh>
// #include <ignition/gazebo/components/WorldPoseCmd.hh>
#include <ignition/gazebo/components/PoseCmd.hh> 
#include <ignition/math/Vector3.hh>
#include <ignition/math/Quaternion.hh>
#include <ignition/math/Helpers.hh>

#include <sdf/Element.hh>
#include <yaml-cpp/yaml.h>

#include <unordered_map>
#include <vector>
#include <string>
#include <optional>
#include <iostream>
#include <cmath>
#include <chrono>
#include <cstdlib>
#include <filesystem>
namespace fs = std::filesystem;

namespace ig = ignition;
namespace gzsim = ignition::gazebo;

namespace ignition::gazebo::systems
{

struct ActorTrack
{
  gzsim::Entity entity{gzsim::kNullEntity};
  std::string name;

  std::vector<ig::math::Vector3d> wps;
  std::size_t idx{0};

  double speed{1.0};
  double tol{0.30};
  bool loop{true};

  bool resolved{false};
};

class ActorWaypointFollowerPlugin
  : public gzsim::System,
    public gzsim::ISystemConfigure,
    public gzsim::ISystemPreUpdate
{
public:

    static std::string ExpandEnv(const std::string &in) {
        std::string out; out.reserve(in.size());
        for (size_t i=0;i<in.size();) {
            if (in[i]=='$' && i+1<in.size() && in[i+1]=='{') {
            size_t j = in.find('}', i+2);
            if (j!=std::string::npos) {
                std::string var = in.substr(i+2, j-(i+2));
                const char* val = std::getenv(var.c_str());
                if (val) out += val;
                i = j+1; 
                continue;
            }
            }
            out += in[i++];
        }
        return out;
    }

  void Configure(const gzsim::Entity &,
                 const std::shared_ptr<const sdf::Element> &_sdf,
                 gzsim::EntityComponentManager &,
                 gzsim::EventManager &) override
  {
    if (_sdf->HasElement("yaml_file"))
      this->yamlFile = _sdf->Get<std::string>("yaml_file");
    if (_sdf->HasElement("default_speed"))
      this->defaultSpeed = _sdf->Get<double>("default_speed");
    if (_sdf->HasElement("default_tolerance"))
      this->defaultTol = _sdf->Get<double>("default_tolerance");
    if (_sdf->HasElement("z_override"))
      this->zOverride = _sdf->Get<double>("z_override");

    // NUEVO: expandir ${VAR} y comprobar existencia
    this->yamlFile = ExpandEnv(this->yamlFile);
    std::cout << "[ActorWaypointFollower] YAML expandido a: " << this->yamlFile << std::endl;
    if (!fs::exists(this->yamlFile)) {
        std::cerr << "[ActorWaypointFollower] YAML no encontrado en: " << this->yamlFile << std::endl;
    }

    try
    {
      YAML::Node root = YAML::LoadFile(this->yamlFile);
      auto actors = root["actors"];
      if (!actors || !actors.IsSequence())
      {
        std::cerr << "[ActorWaypointFollower] 'actors' debe ser una secuencia en "
                  << this->yamlFile << std::endl;
        return;
      }

      for (const auto &node : actors)
      {
        ActorTrack tr;
        tr.name  = node["name"].as<std::string>();
        tr.speed = node["speed"] ? node["speed"].as<double>() : this->defaultSpeed;
        tr.tol   = node["tolerance"] ? node["tolerance"].as<double>() : this->defaultTol;
        tr.loop  = node["loop"] ? node["loop"].as<bool>() : true;

        auto wps = node["waypoints"];
        if (!wps || !wps.IsSequence() || wps.size() == 0)
        {
          std::cerr << "[ActorWaypointFollower] Actor '" << tr.name
                    << "' no tiene waypoints válidos.\n";
          continue;
        }

        for (const auto &wp : wps)
        {
          double x=0, y=0, z=0;
          if (wp.IsSequence() && wp.size() >= 3)
          {
            x = wp[0].as<double>();
            y = wp[1].as<double>();
            z = wp[2].as<double>();
          }
          else if (wp.IsMap())
          {
            x = wp["x"].as<double>();
            y = wp["y"].as<double>();
            z = wp["z"].as<double>();
          }
          else
          {
            std::cerr << "[ActorWaypointFollower] Waypoint inválido en actor "
                      << tr.name << "\n";
            continue;
          }

          if (this->zOverride.has_value())
            z = this->zOverride.value();

          tr.wps.emplace_back(x, y, z);
        }

        if (!tr.wps.empty())
          this->tracks.emplace(tr.name, std::move(tr));
      }

      std::cout << "[ActorWaypointFollower] Cargados " << this->tracks.size()
                << " actores desde " << this->yamlFile << std::endl;
    }
    catch (const std::exception &e)
    {
      std::cerr << "[ActorWaypointFollower] Error leyendo '" << this->yamlFile
                << "': " << e.what() << std::endl;
    }
  }

  void PreUpdate(const gzsim::UpdateInfo &_info,
                 gzsim::EntityComponentManager &_ecm) override
  {
    if (_info.paused) return;

    const double dt = std::chrono::duration<double>(_info.dt).count();
    if (dt <= 0.0) return;

    // Resolver entidades por nombre (una sola vez por actor)
    for (auto &kv : this->tracks)
    {
      auto &tr = kv.second;
      if (!tr.resolved)
      {
        auto ents = _ecm.EntitiesByComponents(gzsim::components::Name(tr.name));
        if (!ents.empty())
        {
          tr.entity = *ents.begin();
          tr.resolved = true;
          // if (!_ecm.Component<gzsim::components::WorldPoseCmd>(tr.entity))
          //   _ecm.CreateComponent(tr.entity, gzsim::components::WorldPoseCmd(ig::math::Pose3d()));
        }
      }
    }

    for (auto &kv : this->tracks)
    {
      auto &tr = kv.second;
      if (!tr.resolved || tr.entity == gzsim::kNullEntity || tr.wps.empty())
        continue;

      auto poseComp = _ecm.Component<gzsim::components::Pose>(tr.entity);
      if (!poseComp) continue;

      ig::math::Pose3d pose = poseComp->Data();
      const ig::math::Vector3d tgt = tr.wps[tr.idx];

      ig::math::Vector3d to = tgt - pose.Pos();
      ig::math::Vector2d to2D(to.X(), to.Y());
      const double dist2D = to2D.Length();

      static double acc = 0.0;
      acc += dt;
      if (acc >= 0.2) {
      std::cout << "[AWF] " << tr.name
          << " -> target idx " << tr.idx
          << " pos=(" << pose.Pos().X() << "," << pose.Pos().Y() << ")"
          << " tgt=(" << tgt.X() << "," << tgt.Y() << ")"
          << " dist=" << dist2D
          << std::endl;
          acc = 0.0; }

      // ¿Llegó?
      if (dist2D <= tr.tol)
      {
        if (tr.idx + 1 < tr.wps.size()) tr.idx++;
        else if (tr.loop) tr.idx = 0;
        else
        {
          // Mantener orientación final y posición
          const double yaw = std::atan2(to.Y(), to.X());
          ig::math::Quaterniond q(0, 0, yaw);
          ig::math::Pose3d hold(tgt.X(), tgt.Y(), tgt.Z(), 0, 0, q.Yaw());
          // _ecm.SetComponentData<gzsim::components::WorldPoseCmd>(tr.entity, hold);
          _ecm.SetComponentData<gzsim::components::Pose>(tr.entity, hold);
          continue;
        }
      }

      // Siguiente target (por si cambió)
      const ig::math::Vector3d t = tr.wps[tr.idx];
      ig::math::Vector3d dir = t - pose.Pos();
      ig::math::Vector2d dir2D(dir.X(), dir.Y());
      const double d = dir2D.Length();
      if (d < 1e-6) continue;

      ig::math::Vector2d step2D = dir2D / d * tr.speed * dt;
      if (step2D.Length() > d) step2D = dir2D;

      const double newX = pose.Pos().X() + step2D.X();
      const double newY = pose.Pos().Y() + step2D.Y();
      const double newZ = this->zOverride.has_value() ? this->zOverride.value() : pose.Pos().Z();

      const double yaw = std::atan2(dir.Y(), dir.X());
      ig::math::Quaterniond q(0, 0, yaw);

      ig::math::Pose3d newPose(newX, newY, newZ, 0, 0, q.Yaw());
      // _ecm.SetComponentData<gzsim::components::WorldPoseCmd>(tr.entity, newPose);
      _ecm.SetComponentData<gzsim::components::Pose>(tr.entity, newPose);
    }
  }

private:
  std::string yamlFile{"config/actor_routes.yaml"};
  double defaultSpeed{1.0};
  double defaultTol{0.30};
  std::optional<double> zOverride{std::nullopt};

  std::unordered_map<std::string, ActorTrack> tracks;
};

} // namespace ignition::gazebo::systems

IGNITION_ADD_PLUGIN(ignition::gazebo::systems::ActorWaypointFollowerPlugin,
                    ignition::gazebo::System,
                    ignition::gazebo::ISystemConfigure,
                    ignition::gazebo::ISystemPreUpdate)

IGNITION_ADD_PLUGIN_ALIAS(ignition::gazebo::systems::ActorWaypointFollowerPlugin,
                          "ignition::gazebo::systems::ActorWaypointFollowerPlugin")