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
  void Configure(const gz::sim::Entity &_entity, const std::shared_ptr<const sdf::Element> &_sdf,
               gz::sim::EntityComponentManager &_ecm, gz::sim::EventManager &) override
  {
    this->actorEntity_ = _entity;

    // Verificar Actor
    auto actorComp = _ecm.Component<gz::sim::components::Actor>(this->actorEntity_);
    if (!actorComp)
    {
      std::cerr << "[ActorWaypointFollowerPlugin] Entity " << _entity << " is not an actor.\n";
      return;
    }

    // Leer parámetros SDF
    if (_sdf->HasElement("follow_mode"))       this->followMode_      = _sdf->Get<std::string>("follow_mode");
    if (_sdf->HasElement("linear_velocity"))   this->linVelocity_     = _sdf->Get<double>("linear_velocity");
    if (_sdf->HasElement("angular_velocity"))  this->angVelocity_     = _sdf->Get<double>("angular_velocity");
    if (_sdf->HasElement("linear_tolerance"))  this->linTolerance_    = _sdf->Get<double>("linear_tolerance");
    if (_sdf->HasElement("angular_tolerance")) this->angTolerance_    = _sdf->Get<double>("angular_tolerance");
    if (_sdf->HasElement("animation_factor"))  this->animationFactor_ = _sdf->Get<double>("animation_factor");
    if (_sdf->HasElement("default_rotation"))  this->defaultRotation_ = _sdf->Get<double>("default_rotation");
    if (_sdf->HasElement("yaml_file"))         this->yamlFile_        = _sdf->Get<std::string>("yaml_file");
    if (_sdf->HasElement("actor_name"))        this->actorNameOverride_= _sdf->Get<std::string>("actor_name");

    this->yamlFile_ = ExpandEnv(this->yamlFile_);
    if (!this->yamlFile_.empty() && !fs::exists(this->yamlFile_))
      std::cerr << "[ActorWaypointFollowerPlugin] YAML not found at: " << this->yamlFile_ << "\n";

    // Nombre en escena
    std::string sceneActorName = "(unknown)";
    if (auto nameComp = _ecm.Component<gz::sim::components::Name>(this->actorEntity_))
      sceneActorName = nameComp->Data();

    // Animación: usar <animation> si está; si no, la primera
    std::string animationName;
    if (_sdf->HasElement("animation")) animationName = _sdf->Get<std::string>("animation");
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

    // Componentes de animación
    if (!_ecm.Component<gz::sim::components::AnimationName>(this->actorEntity_))
      _ecm.CreateComponent(this->actorEntity_, gz::sim::components::AnimationName(animationName));
    else
      *_ecm.Component<gz::sim::components::AnimationName>(this->actorEntity_) = gz::sim::components::AnimationName(animationName);
    _ecm.SetChanged(this->actorEntity_, gz::sim::components::AnimationName::typeId, gz::sim::ComponentState::OneTimeChange);

    if (!_ecm.Component<gz::sim::components::AnimationTime>(this->actorEntity_))
      _ecm.CreateComponent(this->actorEntity_, gz::sim::components::AnimationTime());

    // Pose inicial de referencia
    gz::math::Pose3d initialPose = gz::math::Pose3d::Zero;
    if (auto poseComp = _ecm.Component<gz::sim::components::Pose>(this->actorEntity_))
    {
      initialPose = poseComp->Data();
      auto p = initialPose; p.Pos().X(0); p.Pos().Y(0); // convención: X,Y=0 en Pose
      *poseComp = gz::sim::components::Pose(p);
    }
    else
    {
      _ecm.CreateComponent(this->actorEntity_, gz::sim::components::Pose(gz::math::Pose3d::Zero));
    }

    // TrajectoryPose para mover X-Y
    if (!_ecm.Component<gz::sim::components::TrajectoryPose>(this->actorEntity_))
    {
      auto tp = initialPose; tp.Pos().Z(0);
      _ecm.CreateComponent(this->actorEntity_, gz::sim::components::TrajectoryPose(tp));
    }

    // --- Cargar YAML y ruta del actor ---
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
            if (node["speed"])             this->linVelocity_  = node["speed"].as<double>();
            if (node["tolerance"])         this->linTolerance_ = node["tolerance"].as<double>();
            if (node["linear_tolerance"])  this->linTolerance_ = node["linear_tolerance"].as<double>();
            if (node["angular_tolerance"]) this->angTolerance_ = IGN_DTOR(node["angular_tolerance"].as<double>());
            if (node["loop"])              this->loop_         = node["loop"].as<bool>();

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
              if (wp["yaw"]) { w.yaw = wp["yaw"].as<double>() * M_PI / 180.0; w.hasYaw = true; }
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
              std::cout << "  [wp " << i << "] x=" << w.x << " y=" << w.y << " z=" << w.z
                        << (w.hasYaw ? (std::string(" yaw_deg=") + std::to_string(w.yaw * 180.0 / M_PI)) : " (sin yaw)") << "\n";
            }

            // --- Teleport al primer waypoint ---
            if (!this->targetPoses_.empty())
            {
              const auto &w0 = this->targetPoses_.front();

              // Pose: X=Y=0, Z desde YAML, yaw inicial (YAW del wp o default)
              if (auto poseComp = _ecm.Component<gz::sim::components::Pose>(this->actorEntity_))
              {
                auto p = poseComp->Data();
                p.Pos().X(0); p.Pos().Y(0); p.Pos().Z(w0.z);
                const double yaw0 = w0.hasYaw ? w0.yaw : this->defaultRotation_;
                p.Rot() = gz::math::Quaterniond(0, 0, yaw0);
                *poseComp = gz::sim::components::Pose(p);
                _ecm.SetChanged(this->actorEntity_, gz::sim::components::Pose::typeId,
                                gz::sim::ComponentState::OneTimeChange);
              }
              else
              {
                gz::math::Pose3d p(0, 0, w0.z, 0, 0, w0.hasYaw ? w0.yaw : this->defaultRotation_);
                _ecm.CreateComponent(this->actorEntity_, gz::sim::components::Pose(p));
                _ecm.SetChanged(this->actorEntity_, gz::sim::components::Pose::typeId,
                                gz::sim::ComponentState::OneTimeChange);
              }

              // TrajectoryPose: X/Y del primer waypoint, Z=0 (convención)
              gz::math::Pose3d tp(w0.x, w0.y, 0, 0, 0, w0.hasYaw ? w0.yaw : this->defaultRotation_);
              if (auto tpComp = _ecm.Component<gz::sim::components::TrajectoryPose>(this->actorEntity_))
                *tpComp = gz::sim::components::TrajectoryPose(tp);
              else
                _ecm.CreateComponent(this->actorEntity_, gz::sim::components::TrajectoryPose(tp));
              _ecm.SetChanged(this->actorEntity_, gz::sim::components::TrajectoryPose::typeId,
                              gz::sim::ComponentState::OneTimeChange);

              // Arrancar desde el segundo waypoint (si existe)
              this->idx_ = (this->targetPoses_.size() > 1) ? 1 : 0;

              std::cout << "[AWF] Teleported to first waypoint: (" << w0.x << "," << w0.y << "," << w0.z
                        << "), yaw=" << (w0.hasYaw ? w0.yaw : this->defaultRotation_)
                        << ". Starting idx=" << this->idx_ << "\n";
            }

            break; // ya encontramos al actor objetivo
          }

          if (!found)
          {
            std::cerr << "[ActorWaypointFollowerPlugin] No actor named '" << (this->actorNameOverride_.empty()? sceneActorName : this->actorNameOverride_)
                      << "' in YAML file '" << this->yamlFile_ << "'.\n";
          }
          else
          {
            this->pathCompletedLogged_ = false;
            std::cout << "[ActorWaypointFollowerPlugin] Loaded " << this->targetPoses_.size()
                      << " waypoints for '" << (this->actorNameOverride_.empty()? sceneActorName : this->actorNameOverride_)
                      << "' from " << this->yamlFile_ << "\n";
          }
        }
      }
      catch (const std::exception &e)
      {
        std::cerr << "[ActorWaypointFollowerPlugin] Error reading YAML '" << this->yamlFile_
                  << "': " << e.what() << "\n";
      }
    }

    this->lastUpdate_ = std::chrono::steady_clock::duration::zero();
  }


  // --- PreUpdate: lógica de orientación + traslación y avance de animación ---
  void PreUpdate(const gz::sim::UpdateInfo &_info, gz::sim::EntityComponentManager &_ecm) override
  {
    if (_info.paused) return;

    if (this->followMode_ != "path") {
      // Soporta "velocity" como el oficial si quisieras extender;
      // aquí nos centramos en "path" (YAML).
      this->followMode_ = "path";
    }

    auto trajPoseComp = _ecm.Component<gz::sim::components::TrajectoryPose>(this->actorEntity_);
    if (!trajPoseComp) { // aún no inicializado
      std::cout << "[AWF][WARN] TrajectoryPose missing; actor will not move via trajectory. Waiting...\n";
      return;
    }

    // dt en segundos (double)
    const double dt = std::chrono::duration_cast<std::chrono::duration<double>>(_info.dt).count();
    // Opcional: clamp por seguridad ante saltos grandes (resets/pausas)
    if (dt <= 0.0) return;
    const double dt_clamped = std::min(dt, 0.2); // máx 0.2 s por tick

    auto currentPose = trajPoseComp->Data();
    gz::math::Pose3d newPose = currentPose;
    double distanceTraveled = 0.0;

    // Sin ruta cargada o terminada
    if (this->targetPoses_.empty() || this->idx_ >= static_cast<int>(this->targetPoses_.size())) {
      return;
    }

    // Waypoint objetivo
    const Waypoint &tgt = this->targetPoses_[this->idx_];

    // Vector 2D hacia el target
    gz::math::Vector2d target2d(tgt.x, tgt.y);
    gz::math::Vector2d current2d(currentPose.Pos().X(), currentPose.Pos().Y());
    gz::math::Vector2d to = target2d - current2d;
    double dist = to.Length();

    // ¿Llegamos?
    if (dist < this->linTolerance_) {
      // Orientación final si viene especificada
      if (tgt.hasYaw) {
        newPose.Rot() = gz::math::Quaterniond(0, 0, tgt.yaw);
      }
      // Siguiente
      if (this->idx_ < static_cast<int>(this->targetPoses_.size()) - 1) {
        this->idx_++;
        std::cout << "[AWF] approaching final waypoint #" << this->idx_ << std::endl;
        // No mover aún; aplicamos abajo un paso nulo este tick
        to = gz::math::Vector2d::Zero;
      } else {
        if (!this->pathCompletedLogged_) {
          std::cout << "[ActorWaypointFollowerPlugin] Path completed.\n";
          this->pathCompletedLogged_ = true;
        }
        if (this->loop_) {
          this->idx_ = 0;
        } else {
          // Mantener pose; salir
          *trajPoseComp = gz::sim::components::TrajectoryPose(newPose);
          _ecm.SetChanged(this->actorEntity_, gz::sim::components::TrajectoryPose::typeId, gz::sim::ComponentState::OneTimeChange);
          return;
        }
      }
      // Dentro del bloque if (dist < this->linTolerance_) { ... }
          std::cout << "[AWF] Reached wp " << this->idx_
          << " (dist=" << dist << "). "
          << (this->idx_ < static_cast<int>(this->targetPoses_.size()) - 1 ? "Next → " + std::to_string(this->idx_+1)
                                                                            : (this->loop_ ? "Looping → 0" : "Path end"))
          << std::endl;
    }

    // Recalcular si cambiamos idx
    const Waypoint &next = this->targetPoses_[this->idx_];
    gz::math::Vector2d dir(next.x - currentPose.Pos().X(), next.y - currentPose.Pos().Y());
    double L = dir.Length();

    // --- orientación deseada ---
    double yawNow    = currentPose.Rot().Euler().Z();
    double yawDesired= std::atan2(dir.Y(), dir.X());

    // wrap a [-pi, pi)
    auto wrapPi = [](double a){
      while (a >  M_PI) a -= 2*M_PI;
      while (a <= -M_PI) a += 2*M_PI;
      return a;
    };
    double yawDiff = wrapPi(yawDesired - yawNow);

    // rotar con límite por tick
    const double dt_s = dt_clamped; // asumiendo que arriba ya calculaste dt_clamped (en segundos)
    double angStep = this->angVelocity_ * dt_s;
    double yawStep = std::clamp(yawDiff, -angStep, angStep);
    double newYaw  = yawNow + yawStep;

    // Escala de avance según alineación (0.1..1.0)
    double align = std::cos(std::min(std::abs(yawDiff), M_PI));  // 1=mirando al target, -1=de espaldas
    align = std::max(align, 0.1);                                // nunca 0: siempre empujamos algo

    // Si quieres “afilar” el empuje, usa una curva suave:
    auto smooth = [](double x){ return x*x*(3 - 2*x); };        // smoothstep
    double gain = smooth((align - 0.1) / 0.9);                  // mapea [0.1,1] → [0,1]

    // Paso lineal (¡sin gatear por headingOk!)
    gz::math::Vector2d step = gz::math::Vector2d::Zero;
    if (L > 1e-6)
    {
      step = (dir / L) * (this->linVelocity_ * (0.3 + 0.7*gain)) * dt_s;
      if (step.Length() > L) step = dir; // no pasarse
      newPose.Pos().X() += step.X();
      newPose.Pos().Y() += step.Y();
      distanceTraveled = step.Length();
    }

    // Z del waypoint actual
    newPose.Pos().Z(next.z);

    // Rotación final: si el waypoint trae yaw, cuando estemos razonablemente alineados
    bool almostAligned = std::abs(yawDiff) < this->angTolerance_;
    newPose.Rot() = gz::math::Quaterniond(0, 0, (almostAligned && next.hasYaw) ? next.yaw : newYaw);

    // LOG útil
    if(print_debug){
      std::cout << "[AWF] idx=" << this->idx_
                << " cur=(" << currentPose.Pos().X() << "," << currentPose.Pos().Y() << "," << currentPose.Pos().Z() << ")"
                << " yawNow=" << yawNow
                << " tgt=(" << next.x << "," << next.y << "," << next.z << ")"
                << " yawDes=" << yawDesired
                << " dist=" << L
                << " yawDiff=" << yawDiff
              << std::endl;
    }

    if (this->linVelocity_ <= 1e-6)
      std::cout << "[AWF][WARN] linear_velocity ~ 0, no avanzará.\n";
    if (this->angVelocity_ <= 1e-6)
      std::cout << "[AWF][WARN] angular_velocity ~ 0, no podrá orientar.\n";
    

    // Aplicar
    *trajPoseComp = gz::sim::components::TrajectoryPose(newPose);
    _ecm.SetChanged(this->actorEntity_, gz::sim::components::TrajectoryPose::typeId, gz::sim::ComponentState::OneTimeChange);

    // Avance de animación proporcional a la distancia (como el oficial)
    if (distanceTraveled > 1e-5) {
      auto animTimeComp = _ecm.Component<gz::sim::components::AnimationTime>(this->actorEntity_);
      if (animTimeComp) {
        auto animTime = animTimeComp->Data() + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            std::chrono::duration<double>(distanceTraveled * this->animationFactor_));
        *animTimeComp = gz::sim::components::AnimationTime(animTime);
        _ecm.SetChanged(this->actorEntity_, gz::sim::components::AnimationTime::typeId, gz::sim::ComponentState::OneTimeChange);
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
