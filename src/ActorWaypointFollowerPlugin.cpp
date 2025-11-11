#include <ignition/plugin/Register.hh>
#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/EntityComponentManager.hh>
#include <ignition/gazebo/EventManager.hh>
#include <ignition/gazebo/Types.hh>
#include <ignition/gazebo/components/Pose.hh>
#include <ignition/gazebo/components/PoseCmd.hh>
#include <ignition/gazebo/components/Name.hh>
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
namespace gzcomp = gz::sim::components;
namespace ignition::gazebo::systems
{

  struct Waypoint {
    ig::math::Vector3d pos;   // x,y,z
    bool hasYaw{false};
    double yaw{0.0};          // rad
  };

  struct ActorTrack
  {
    gzsim::Entity entity{gzsim::kNullEntity};
    std::string name;
    std::vector<Waypoint> wps;
    std::size_t idx{0};
    double speed{1.0};
    double tol{0.10};
    bool loop{true};
    bool resolved{false}; //Indicates whether the plugin has already found and associated the actor's name
  };

class ActorWaypointFollowerPlugin : public gzsim::System, public gzsim::ISystemConfigure, public gzsim::ISystemPreUpdate
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

    // Checking if YAML file exist
    this->yamlFile = ExpandEnv(this->yamlFile);
    std::cout << "[ActorWaypointFollower] YAML expanded to: " << this->yamlFile << std::endl;
    if (!fs::exists(this->yamlFile)) {
        std::cerr << "[ActorWaypointFollower] YAML was not found in: " << this->yamlFile << std::endl;
    }

    // Parse YAML
    try
    {
      YAML::Node root = YAML::LoadFile(this->yamlFile);
      auto actors = root["actors"];
      if (!actors || !actors.IsSequence())
      {
        std::cerr << "[ActorWaypointFollower] 'actors' must be a sequence defined in " << this->yamlFile << std::endl;
        return;
      }
std::cout << "1 actors.size(): " << actors.size() << std::endl; //For debug
      for (const auto &node : actors) // Iteration over each actor defined
      {
        ActorTrack tr;
        tr.name  = node["name"].as<std::string>();
        tr.speed = node["speed"] ? node["speed"].as<double>() : this->defaultSpeed;
        tr.tol   = node["tolerance"] ? node["tolerance"].as<double>() : this->defaultTol;
        tr.loop  = node["loop"] ? node["loop"].as<bool>() : true;
        auto wps = node["waypoints"];
std::cout << "2 name: " << tr.name << std::endl << "wp: " << wps << std::endl; //For debug

        if (!wps || !wps.IsSequence() || wps.size() == 0)
        {
          std::cerr << "[ActorWaypointFollower] Actor '" << tr.name << "' doesnt have valid waypoint.\n";
          continue;
        }
std::cout << "3 wps.size(): " << wps.size() << std::endl; //For debug
        for (const auto &wp : wps)
        {
          Waypoint wpt;
//           if (wp.IsSequence()) {
// std::cout << "4.1 wp: " << wp << " , wp.IsSequence(): " << wp.IsSequence() << std::endl; //For debug
//             if (wp.size() == 3) {
//               wpt.pos.Set(wp[0].as<double>(), wp[1].as<double>(), wp[2].as<double>());
//             } 
//             else if (wp.size() >= 4) {
//               double yaw_deg = wp[3].as<double>();
//               wpt.hasYaw = true;
//               wpt.yaw = yaw_deg * M_PI / 180.0;  // deg -> rad
//             }
//             else {
//               std::cerr << "[ActorWaypointFollower] Waypoint secuencia con <3 valores en actor "
//                         << tr.name << "\n";
//               continue;
//             }
// std::cout << "4.2 wp.size() : " << wp.size()  << " , wpt.pos: " << wpt.pos << std::endl; //For debug
//           } 
          // else if (wp.IsMap()) {
          if (wp.IsMap()) {
std::cout << "5.1 wp: " << wp << " , wp.IsMap(): " << wp.IsMap() << std::endl; //For debug
            double x = wp["x"].as<double>();
            double y = wp["y"].as<double>();
            double z = wp["z"].as<double>();
            double roll = wp["roll"].as<double>();
            double pitch = wp["pitch"].as<double>();
            double yaw   = wp["yaw"].as<double>();
            wpt.pos.Set(x, y, z);
            wpt.hasYaw = true;
            wpt.yaw = yaw * M_PI / 180.0;
            // if (wp["yaw"]) {
            //   double yaw_deg = wp["yaw"].as<double>();
            //   wpt.hasYaw = true;
            //   wpt.yaw = yaw_deg * M_PI / 180.0;
            // }
          } 
          else {
            std::cerr << "[ActorWaypointFollower] Invalid actor Waypoint " << tr.name << "\n";
            continue;
          }

          if (this->zOverride.has_value())
            wpt.pos.Z() = this->zOverride.value();

          tr.wps.emplace_back(std::move(wpt));
        }

        if (!tr.wps.empty())
          this->tracks.emplace(tr.name, std::move(tr));
      }

      



// =========================================================================
// BLOQUE DE CÓDIGO PARA RECORRER E IMPRIMIR TODO EL MAPA
// =========================================================================
std::cout << "\n--- Inicio de Impresión de Todas las Rutas ---" << std::endl;

// 'it' es el nombre que le damos al par clave-valor en cada iteración.
// Es de tipo std::pair<const std::string, ActorTrack>
for (const auto& it : this->tracks) 
{
    // Acceder a la CLAVE (Nombre del actor)
    const std::string& actor_name = it.first; 
    
    // Acceder al VALOR (Estructura ActorTrack)
    const ActorTrack& track_data = it.second;

    std::cout << "Actor: " << actor_name << std::endl;
    std::cout << "  - Velocidad (m/s): " << track_data.speed << std::endl;
    std::cout << "  - Tolerancia (m): " << track_data.tol << std::endl;
    std::cout << "  - Bucle (Loop): " << (track_data.loop ? "Sí" : "No") << std::endl;
    std::cout << "  - Waypoints Cargados: " << track_data.wps.size() << std::endl;
    std::cout << "  - Waypoint Actual (idx): " << track_data.idx << std::endl;

    // Opcional: Imprimir los detalles de cada waypoint
    std::cout << "  --- Waypoints ---" << std::endl;
    for (size_t i = 0; i < track_data.wps.size(); ++i)
    {
        const Waypoint& wp = track_data.wps[i];
        std::cout << "    [" << i << "] Pos: (" 
                  << wp.pos.X() << ", " 
                  << wp.pos.Y() << ", " 
                  << wp.pos.Z() << ")";
        
        if (wp.hasYaw) {
            // Imprimimos el yaw en grados para facilitar la lectura (convirtiendo de rad a deg)
            std::cout << ", Yaw (deg): " << wp.yaw * 180.0 / M_PI; 
        }
        std::cout << std::endl;
    }
}
std::cout << "--- Fin de Impresión de Rutas ---" << std::endl;
// =========================================================================



      std::cout << "[ActorWaypointFollower] loaded " << this->tracks.size()<< " actors from " << this->yamlFile << std::endl;
    }
    catch (const std::exception &e)
    {
      std::cerr << "[ActorWaypointFollower] Error reading '" << this->yamlFile << "': " << e.what() << std::endl;
    }
  }

  // This method runs in every simulation cycle. It's the plugin's motion engine.
  void PreUpdate(const gzsim::UpdateInfo &_info, gzsim::EntityComponentManager &_ecm) override
  {
    
    if (_info.paused) return; // This ensures that the movement logic is executed only while the simulation is running 

    // Calculates the elapsed time (dt) in seconds since the last update — a key factor for velocity-based motion.
    const double dt = std::chrono::duration<double>(_info.dt).count();
    if (dt <= 0.0) return;

    // Resolve entities by name (only once per actor)
    for (auto &kv : this->tracks) // kv is the actor entity
    {
      auto &tr = kv.second; // Access to ActorTrack structure from tracks variable (unordered_map <std::string, ActorTrack> tracks)
      if (!tr.resolved)
      {
        auto ents = _ecm.EntitiesByComponents(gzsim::components::Name(tr.name));
        if (!ents.empty())
        {
          tr.entity = *ents.begin(); //assigns the unique entity identifier of Gazebo to the tr.entity member of the ActorTrack structure.
          tr.resolved = true;
          if (!_ecm.Component<gzcomp::WorldPoseCmd>(tr.entity)) // Check if <gzcomp::WorldPoseCmd> exist
          {
            _ecm.CreateComponent(tr.entity, gzcomp::WorldPoseCmd(ig::math::Pose3d()));
          }
        }
      }
    }

    for (auto &kv : this->tracks) // iterates over all registered actors
    {
      auto &tr = kv.second;
      if (!tr.resolved || tr.entity == gzsim::kNullEntity || tr.wps.empty())
        continue;

      auto poseComp = _ecm.Component<gzcomp::WorldPoseCmd>(tr.entity); // Access the actor's current Pose component.
      if (!poseComp) continue;
      ig::math::Pose3d pose = poseComp->Data();

      // Target Reading
      const Waypoint &tgtWP = tr.wps[tr.idx];
      const ig::math::Vector3d tgt = tgtWP.pos;

      ig::math::Vector3d to = tgt - pose.Pos();
      ig::math::Vector2d to2D(to.X(), to.Y());
      const double dist2D = to2D.Length();

      // ¿Actor Arrive to position?
      if (dist2D <= tr.tol) // Target distance
      {
        double yawHold = tgtWP.hasYaw ? tgtWP.yaw : pose.Rot().Yaw();

        ig::math::Pose3d hold(tgt.X(), tgt.Y(), tgt.Z(), 0, 0, yawHold);
        _ecm.SetComponentData<gzcomp::WorldPoseCmd>(tr.entity, hold);

        if (tr.idx + 1 < tr.wps.size()) 
          tr.idx++;
        else if (tr.loop) 
          tr.idx = 0;
        else continue;
      }

      // Compute step for movement
      const Waypoint &nextWP = tr.wps[tr.idx];
      ig::math::Vector3d dir = nextWP.pos - pose.Pos();
      ig::math::Vector2d dir2D(dir.X(), dir.Y());
      const double d = dir2D.Length();
      if (d < 1e-6) 
        continue;
      ig::math::Vector2d step2D = dir2D / d * tr.speed * dt;
      if (step2D.Length() > d) 
        step2D = dir2D;
      
      // To apply pose command
      const double newX = pose.Pos().X() + step2D.X();
      const double newY = pose.Pos().Y() + step2D.Y();
      const double newZ = this->zOverride.has_value() ? this->zOverride.value() : pose.Pos().Z();



      //Code Snippet to debug Actor position:
      static double acc = 0.0;
      acc += dt;
      if (acc >= 0.1) {
      std::cout << "[AWF] " << tr.name
          << " -> target idx " << tr.idx
          << " pos=(" << pose.Pos().X() << "," << pose.Pos().Y() << ")"
          << " tgt=(" << tgt.X() << "," << tgt.Y() << ")"
          << " dist=" << dist2D
          << " step2D=" << step2D
          << " next=(" << newX << "," << newY << "," << newZ << ")"
          << std::endl;
          acc = 0.0; }




      // A) Mirar hacia el movimiento:
      double yawMove = std::atan2(dir.Y(), dir.X());

      // B) O forzar el heading que trae el waypoint destino (si lo quieres rígido):
      // double yawDesired = nextWP.hasYaw ? nextWP.yaw : yawMove;

      ig::math::Pose3d newPose(newX, newY, newZ, 0, 0, yawMove);
      _ecm.SetComponentData<gzcomp::WorldPoseCmd>(tr.entity, newPose);
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