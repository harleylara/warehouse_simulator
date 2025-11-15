#include <ignition/plugin/Register.hh>
#include <gz/sim/System.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/EventManager.hh>
#include <gz/sim/Types.hh>

#include <gz/sim/components/Actor.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Pose.hh>

#include <gz/math/Vector3.hh>
#include <gz/math/Quaternion.hh>
#include <gz/math/Angle.hh>
#include <yaml-cpp/yaml.h>

#include <string>
#include <vector>
#include <queue>
#include <optional>
#include <mutex>
#include <filesystem>
#include <chrono>
#include <iostream>
#include <cmath>
#include <cstdlib>
#include <algorithm>                 // std::clamp
#include <ignition/math/Helpers.hh>  // IGN_DTOR (si lo usas)

namespace fs = std::filesystem;

namespace ignition::gazebo::systems
{

// Waypoint compacto: X, Y, Z opcional, Yaw en rad y flag.
struct Waypoint
{
  double x{0}, y{0}, z{0};
  double yaw{0};      // rad
  bool hasYaw{false};
};

class ActorWaypointFollowerPlugin final
  : public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPreUpdate
{
public:
  ActorWaypointFollowerPlugin() = default;

  // --- Utilidad: expandir ${VAR} en rutas ---
  static std::string ExpandEnv(const std::string &in)
  {
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

  // --- Configure: leer SDF, preparar componentes y cargar YAML ---
void Configure(const gz::sim::Entity &_entity,
               const std::shared_ptr<const sdf::Element> &_sdf,
               gz::sim::EntityComponentManager &_ecm,
               gz::sim::EventManager &) override
{
  this->actorEntity_ = _entity;

  // 1) Verificar que la entidad es un Actor
  auto actorComp = _ecm.Component<gz::sim::components::Actor>(this->actorEntity_);
  if (!actorComp)
  {
    std::cerr << "[ActorWaypointFollowerPlugin] Entity " << _entity << " is not an actor.\n";
    return;
  }

  // 2) Leer parámetros SDF
  if (_sdf->HasElement("follow_mode"))        this->followMode_       = _sdf->Get<std::string>("follow_mode");
  if (_sdf->HasElement("linear_velocity"))    this->linVelocity_      = _sdf->Get<double>("linear_velocity");
  if (_sdf->HasElement("angular_velocity"))   this->angVelocity_      = _sdf->Get<double>("angular_velocity");
  if (_sdf->HasElement("linear_tolerance"))   this->linTolerance_     = _sdf->Get<double>("linear_tolerance");
  if (_sdf->HasElement("angular_tolerance"))  this->angTolerance_     = _sdf->Get<double>("angular_tolerance"); // aquí asumo rad
  if (_sdf->HasElement("animation_factor"))   this->animationFactor_  = _sdf->Get<double>("animation_factor");
  if (_sdf->HasElement("default_rotation"))   this->defaultRotation_  = _sdf->Get<double>("default_rotation");  // rad
  if (_sdf->HasElement("yaml_file"))          this->yamlFile_         = _sdf->Get<std::string>("yaml_file");
  if (_sdf->HasElement("actor_name"))         this->actorNameOverride_= _sdf->Get<std::string>("actor_name");

  this->yamlFile_ = ExpandEnv(this->yamlFile_);
  if (!this->yamlFile_.empty() && !fs::exists(this->yamlFile_))
    std::cerr << "[ActorWaypointFollowerPlugin] YAML not found at: " << this->yamlFile_ << "\n";

  // 3) Nombre del actor en la escena
  std::string sceneActorName = "(unknown)";
  if (auto nameComp = _ecm.Component<gz::sim::components::Name>(this->actorEntity_))
    sceneActorName = nameComp->Data();

  // 4) Selección de animación: <animation> del SDF o primera anim disponible
  std::string animationName;
  if (_sdf->HasElement("animation"))
  {
    animationName = _sdf->Get<std::string>("animation");
  }
  else
  {
    if (actorComp->Data().AnimationCount() < 1)
    {
      std::cerr << "[ActorWaypointFollowerPlugin] Actor has no animations.\n";
      return;
    }
    animationName = actorComp->Data().AnimationByIndex(0)->Name();
  }
  if (animationName.empty())
  {
    std::cerr << "[ActorWaypointFollowerPlugin] Empty animation name.\n";
    return;
  }

  if (!_ecm.Component<gz::sim::components::AnimationName>(this->actorEntity_))
    _ecm.CreateComponent(this->actorEntity_, gz::sim::components::AnimationName(animationName));
  else
    *_ecm.Component<gz::sim::components::AnimationName>(this->actorEntity_) = gz::sim::components::AnimationName(animationName);
  _ecm.SetChanged(this->actorEntity_, gz::sim::components::AnimationName::typeId,
                  gz::sim::ComponentState::OneTimeChange);

  if (!_ecm.Component<gz::sim::components::AnimationTime>(this->actorEntity_))
    _ecm.CreateComponent(this->actorEntity_, gz::sim::components::AnimationTime());

  // 5) Asegurar que existe Pose (aún no la tocaremos hasta conocer el primer waypoint)
  gz::math::Pose3d initialPose = gz::math::Pose3d::Zero;
  if (auto poseComp = _ecm.Component<gz::sim::components::Pose>(this->actorEntity_))
  {
    initialPose = poseComp->Data();
  }
  else
  {
    _ecm.CreateComponent(this->actorEntity_, gz::sim::components::Pose(gz::math::Pose3d::Zero));
  }

  // 6) Cargar YAML y seleccionar la ruta del actor
  if (!this->yamlFile_.empty())
  {
    try
    {
      YAML::Node root = YAML::LoadFile(this->yamlFile_);
      auto actors = root["actors"];
      if (!actors || !actors.IsSequence())
      {
        std::cerr << "[ActorWaypointFollowerPlugin] 'actors' must be a sequence in " << this->yamlFile_ << "\n";
      }
      else
      {
        const std::string targetName = this->actorNameOverride_.empty() ? sceneActorName : this->actorNameOverride_;
        bool found = false;

        for (const auto &node : actors)
        {
          if (!node["name"]) continue;
          const std::string nm = node["name"].as<std::string>();
          if (nm != targetName) continue;

          found = true;

          if (node["speed"])              this->linVelocity_  = node["speed"].as<double>();
          if (node["tolerance"])          this->linTolerance_ = node["tolerance"].as<double>();
          if (node["linear_tolerance"])   this->linTolerance_ = node["linear_tolerance"].as<double>();
          if (node["angular_tolerance"])  this->angTolerance_ = IGN_DTOR(node["angular_tolerance"].as<double>()); // deg → rad
          if (node["loop"])               this->loop_         = node["loop"].as<bool>();

          auto wps = node["waypoints"];
          if (!wps || !wps.IsSequence() || wps.size() == 0)
          {
            std::cerr << "[ActorWaypointFollowerPlugin] Actor '" << nm << "' has no valid waypoints.\n";
            break;
          }

          this->targetPoses_.clear();
          this->targetPoses_.reserve(wps.size());

          for (const auto &wp : wps)
          {
            if (!wp.IsMap()) continue;
            Waypoint w;
            w.x = wp["x"].as<double>();
            w.y = wp["y"].as<double>();
            w.z = wp["z"] ? wp["z"].as<double>() : initialPose.Pos().Z();

            if (wp["yaw"])
            {
              w.yaw = wp["yaw"].as<double>() * M_PI / 180.0; // deg → rad
              w.hasYaw = true;
            }
            this->targetPoses_.push_back(w);
          }

          // Debug resumen
          std::cout << "[AWF] Actor '" << targetName << "' -> " << this->targetPoses_.size()
                    << " waypoints, lin_vel=" << this->linVelocity_
                    << ", lin_tol=" << this->linTolerance_
                    << ", ang_tol=" << this->angTolerance_
                    << ", loop=" << std::boolalpha << this->loop_ << std::noboolalpha << "\n";

          for (size_t i = 0; i < this->targetPoses_.size(); ++i)
          {
            const auto &w = this->targetPoses_[i];
            if(print_debug){ 
              std::cout << "  [wp " << i << "] x=" << w.x << " y=" << w.y << " z=" << w.z
                        << (w.hasYaw ? (std::string(" yaw_deg=") + std::to_string(w.yaw * 180.0 / M_PI)) : " (sin yaw)") << "\n";
            }
          }

          // 7) Teleport inicial:
          //    - Pose: fija Z y yaw=0 (sin rotación) y X=Y=0 para que TrajectoryPose no se rote
          //    - TrajectoryPose: pone X,Y,Yaw del primer waypoint (Z=0 por convención de TrajectoryPose)
          if (!this->targetPoses_.empty())
          {
            const auto &w0 = this->targetPoses_.front();
            const double yaw0 = w0.hasYaw ? w0.yaw : this->defaultRotation_;

            // Pose → sólo Z y yaw=0
            if (auto poseComp = _ecm.Component<gz::sim::components::Pose>(this->actorEntity_))
            {
              auto p = poseComp->Data();
              p.Pos().X(0); p.Pos().Y(0);
              p.Pos().Z(w0.z);
              p.Rot() = gz::math::Quaterniond::Identity; // yaw=0
              *poseComp = gz::sim::components::Pose(p);
              _ecm.SetChanged(this->actorEntity_, gz::sim::components::Pose::typeId,
                              gz::sim::ComponentState::OneTimeChange);
            }
            else
            {
              gz::math::Pose3d p(0, 0, w0.z, 0, 0, 0); // yaw=0
              _ecm.CreateComponent(this->actorEntity_, gz::sim::components::Pose(p));
              _ecm.SetChanged(this->actorEntity_, gz::sim::components::Pose::typeId,
                              gz::sim::ComponentState::OneTimeChange);
            }

            // TrajectoryPose → X,Y,yaw del primer WP (Z=0)
            gz::math::Pose3d tp(w0.x, w0.y, 0, 0, 0, yaw0);
            if (auto tpComp = _ecm.Component<gz::sim::components::TrajectoryPose>(this->actorEntity_))
              *tpComp = gz::sim::components::TrajectoryPose(tp);
            else
              _ecm.CreateComponent(this->actorEntity_, gz::sim::components::TrajectoryPose(tp));
            _ecm.SetChanged(this->actorEntity_, gz::sim::components::TrajectoryPose::typeId,
                            gz::sim::ComponentState::OneTimeChange);

            // Índice inicial de la ruta: si hay más de un waypoint, arrancamos hacia el 2º
            this->idx_ = (this->targetPoses_.size() > 1) ? 1 : 0;
            
            if(print_debug){
              std::cout << "[AWF] Teleported to first waypoint: ("
                        << w0.x << "," << w0.y << "," << w0.z
                        << "), yaw=" << yaw0
                        << ". Starting idx=" << this->idx_ << "\n";
            }
          }

          break; // ya encontramos el actor objetivo
        }

        if (!found)
        { 
          std::cerr << "[ActorWaypointFollowerPlugin] No actor named '"
                    << (this->actorNameOverride_.empty()? sceneActorName : this->actorNameOverride_)
                    << "' in YAML file '" << this->yamlFile_ << "'.\n";
        }
        else
        {
          this->pathCompletedLogged_ = false;
          if(print_debug){
            std::cout << "[ActorWaypointFollowerPlugin] Loaded " << this->targetPoses_.size()
                      << " waypoints for '"
                      << (this->actorNameOverride_.empty()? sceneActorName : this->actorNameOverride_)
                      << "' from " << this->yamlFile_ << "\n";
          }
        }
      }
    }
    catch (const std::exception &e)
    {
      std::cerr << "[ActorWaypointFollowerPlugin] Error reading YAML '" << this->yamlFile_
                << "': " << e.what() << "\n";
    }
  }

  // 8) Inicializa tiempos internos
  this->lastUpdate_ = std::chrono::steady_clock::duration::zero();
}

// --- PreUpdate: lógica de orientación + traslación y avance de animación ---
void PreUpdate(const gz::sim::UpdateInfo &_info,
               gz::sim::EntityComponentManager &_ecm) override
{
  if (_info.paused)
    return;

  // Forzamos modo path
  if (this->followMode_ != "path")
    this->followMode_ = "path";

  auto trajPoseComp =
      _ecm.Component<gz::sim::components::TrajectoryPose>(this->actorEntity_);
  if (!trajPoseComp)
  {
    std::cout << "[AWF][WARN] TrajectoryPose missing; actor will not move via trajectory. Waiting...\n";
    return;
  }

  // dt en segundos (double)
  const double dt =
      std::chrono::duration_cast<std::chrono::duration<double>>(_info.dt).count();
  if (dt <= 0.0)
    return;

  const double dt_clamped = std::min(dt, 0.2); // máx 0.2 s por tick

  auto currentPose = trajPoseComp->Data();
  gz::math::Pose3d newPose = currentPose;
  double distanceTraveled = 0.0;

  // Sin ruta cargada o terminada
  if (this->targetPoses_.empty() ||
      this->idx_ >= static_cast<int>(this->targetPoses_.size()))
  {
    return;
  }

  // Waypoint objetivo actual
  const Waypoint &wp = this->targetPoses_[this->idx_];

  // Vector 2D hacia el target (posición)
  gz::math::Vector2d target2d(wp.x, wp.y);
  gz::math::Vector2d current2d(currentPose.Pos().X(), currentPose.Pos().Y());
  gz::math::Vector2d delta = target2d - current2d;
  double L = delta.Length();

  // ¿Es un waypoint "solo de giro"? (misma posición)
  const double posEps = 1e-3;
  bool yawOnly = (L < posEps) && wp.hasYaw;

  // --- orientación deseada ---
  double yawNow = currentPose.Rot().Euler().Z();
  double yawDesired = 0.0;

  if (yawOnly)
  {
    // Si el waypoint sólo cambia yaw, el deseado es el yaw del wp
    yawDesired = wp.yaw;
  }
  else
  {
    // Para waypoints con cambio de posición, mirar hacia el punto
    yawDesired = std::atan2(delta.Y(), delta.X());
  }

  auto wrapPi = [](double a)
  {
    while (a > M_PI)  a -= 2.0 * M_PI;
    while (a <= -M_PI) a += 2.0 * M_PI;
    return a;
  };
  double yawDiff = wrapPi(yawDesired - yawNow);

  // Paso angular limitado por dt y angVelocity_
  const double dt_s = dt_clamped;
  double angStep = this->angVelocity_ * dt_s;
  double yawStep = std::clamp(yawDiff, -angStep, angStep);
  double newYaw  = yawNow + yawStep;

  // --- avance lineal ---
  gz::math::Vector2d step = gz::math::Vector2d::Zero;

  if (!yawOnly && L > 1e-6)
  {
    // Escala de avance según alineación (0.1..1.0)
    double align = std::cos(std::min(std::abs(yawDiff), M_PI)); // 1=mirando, -1=de espaldas
    align = std::max(align, 0.1);                               // nunca 0

    auto smooth = [](double x)
    {
      return x * x * (3 - 2 * x); // smoothstep
    };
    double gain = smooth((align - 0.1) / 0.9); // [0.1,1] → [0,1]

    step = (delta / L) * (this->linVelocity_ * (0.3 + 0.7 * gain)) * dt_s;
    if (step.Length() > L)
      step = delta; // no pasarse

    newPose.Pos().X() += step.X();
    newPose.Pos().Y() += step.Y();
    distanceTraveled = step.Length();
  }
  else
  {
    // Waypoint solo de giro o L≈0 → no movemos X/Y
    newPose.Pos().X() = currentPose.Pos().X();
    newPose.Pos().Y() = currentPose.Pos().Y();
  }

  // Z del waypoint actual
  newPose.Pos().Z(wp.z);

  // Rotación provisional
  newPose.Rot() = gz::math::Quaterniond(0, 0, newYaw);

  // --- criterios de llegada ---
  bool posOk = (L < this->linTolerance_);
  bool yawOk = (!wp.hasYaw) || (std::abs(yawDiff) < this->angTolerance_);

  bool reached = false;

  if (yawOnly)
  {
    // Waypoint de giro en el sitio: sólo importa el yaw
    if (yawOk)
      reached = true;
  }
  else
  {
    // Waypoint de traslación
    if (posOk)
    {
      // Si tiene yaw, podemos exigir también yawOk para avanzar "fino"
      if (!wp.hasYaw || yawOk)
        reached = true;
    }
  }

  if (reached)
  {
    // Ajustamos yaw EXACTO al del waypoint si viene especificado
    if (wp.hasYaw)
      newPose.Rot() = gz::math::Quaterniond(0, 0, wp.yaw);

    if (this->idx_ < static_cast<int>(this->targetPoses_.size()) - 1)
    {
      if (print_debug)
      {
        std::cout << "[AWF] Reached wp " << this->idx_
                  << " (posOk=" << posOk << ", yawOk=" << yawOk << "). Next -> "
                  << (this->idx_ + 1) << std::endl;
      }
      this->idx_++;
    }
    else
    {
      // Último waypoint
      if (!this->pathCompletedLogged_)
      {
        std::cout << "[ActorWaypointFollowerPlugin] Path completed.\n";
        this->pathCompletedLogged_ = true;
      }

      if (this->loop_)
      {
        if (print_debug)
          std::cout << "[AWF] Looping path -> wp 0\n";
        this->idx_ = 0;
      }
      else
      {
        // Mantener pose final y salir
        *trajPoseComp = gz::sim::components::TrajectoryPose(newPose);
        _ecm.SetChanged(this->actorEntity_,
                        gz::sim::components::TrajectoryPose::typeId,
                        gz::sim::ComponentState::OneTimeChange);
        return;
      }
    }
  }

  if (print_debug)
  {
    std::cout << "[AWF] idx=" << this->idx_
              << " pos=(" << currentPose.Pos().X() << "," << currentPose.Pos().Y()
              << "," << currentPose.Pos().Z() << ")"
              << " -> new=(" << newPose.Pos().X() << "," << newPose.Pos().Y()
              << "," << newPose.Pos().Z() << ")"
              << " yawNow=" << yawNow
              << " yawDes=" << yawDesired
              << " yawDiff=" << yawDiff
              << " L=" << L
              << " yawOnly=" << std::boolalpha << yawOnly << std::noboolalpha
              << std::endl;
  }

  if (this->linVelocity_ <= 1e-6)
    std::cout << "[AWF][WARN] linear_velocity ~ 0, no avanzará.\n";
  if (this->angVelocity_ <= 1e-6)
    std::cout << "[AWF][WARN] angular_velocity ~ 0, no podrá orientar.\n";

  // Aplicar TrajectoryPose
  *trajPoseComp = gz::sim::components::TrajectoryPose(newPose);
  _ecm.SetChanged(this->actorEntity_,
                  gz::sim::components::TrajectoryPose::typeId,
                  gz::sim::ComponentState::OneTimeChange);

  // Avance de animación proporcional a la distancia recorrida
  if (distanceTraveled > 1e-5)
  {
    auto animTimeComp =
        _ecm.Component<gz::sim::components::AnimationTime>(this->actorEntity_);
    if (animTimeComp)
    {
      auto animTime =
          animTimeComp->Data() +
          std::chrono::duration_cast<std::chrono::steady_clock::duration>(
              std::chrono::duration<double>(distanceTraveled *
                                            this->animationFactor_));
      *animTimeComp = gz::sim::components::AnimationTime(animTime);
      _ecm.SetChanged(this->actorEntity_,
                      gz::sim::components::AnimationTime::typeId,
                      gz::sim::ComponentState::OneTimeChange);
    }
  }
}

static bool ShouldPrint(const gz::sim::UpdateInfo &_info, double hz = 10.0)
{
  static std::chrono::steady_clock::duration lastPrint{std::chrono::steady_clock::duration::zero()};
  const auto period = std::chrono::duration<double>(1.0 / hz);
  if (_info.simTime - lastPrint >= std::chrono::duration_cast<std::chrono::steady_clock::duration>(period))
  {
    lastPrint = _info.simTime;
    return true;
  }
  return false;
}

private:
  // --- Estado ---
  gz::sim::Entity actorEntity_{gz::sim::kNullEntity};

  // Parámetros "estilo plugin oficial"
  std::string followMode_{"path"}; // nos centramos en "path"
  double linVelocity_{1.0};        // m/s
  double angVelocity_{IGN_DTOR(90)}; // rad/s (p.ej. 90°/s)
  double linTolerance_{0.10};      // m
  double angTolerance_{IGN_DTOR(6)}; // rad (p.ej. 6°)
  double animationFactor_{4.0};
  double defaultRotation_{M_PI/2.0};

  // YAML
  std::string yamlFile_{"config/actor_routes.yaml"};
  std::string actorNameOverride_{};
  bool loop_{true};
  bool print_debug{false};

  // Ruta actual
  std::vector<Waypoint> targetPoses_;
  int idx_{0};
  bool pathCompletedLogged_{false};

  // Tiempos
  std::chrono::steady_clock::duration lastUpdate_{std::chrono::steady_clock::duration::zero()};

  // Mutex reservado (si más adelante agregas subscripciones/colas)
  std::mutex mutex_;
};

// --- Registro del plugin ---
} // namespace gazebo_yaml_actor

IGNITION_ADD_PLUGIN(ignition::gazebo::systems::ActorWaypointFollowerPlugin,
                    ignition::gazebo::System,
                    ignition::gazebo::ISystemConfigure,
                    ignition::gazebo::ISystemPreUpdate)

IGNITION_ADD_PLUGIN_ALIAS(ignition::gazebo::systems::ActorWaypointFollowerPlugin,
                          "ignition::gazebo::systems::ActorWaypointFollowerPlugin")
