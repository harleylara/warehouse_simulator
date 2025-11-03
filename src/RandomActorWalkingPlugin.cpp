#include <algorithm>               // std::clamp
#include <cmath>                   // std::cos, std::sin, std::atan2
#include <random>
#include <string>

#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/Actor.hh>
#include <ignition/gazebo/Util.hh>

// Componentes para búsqueda por nombre y tipo Actor (ECS)
#include <ignition/gazebo/components/Name.hh>
#include <ignition/gazebo/components/Actor.hh>

#include <ignition/math/Vector3.hh>
#include <ignition/math/Angle.hh>

#include <ignition/plugin/Register.hh>

// Aliases para evitar ambigüedades
namespace ig   = ignition::gazebo;
namespace igc  = ignition::gazebo::components;
namespace math = ignition::math;

class RandomActorWalkingPlugin
  : public ig::System,
    public ig::ISystemConfigure,
    public ig::ISystemPreUpdate
{
public:
  void Configure(const ig::Entity &entity,
                 const std::shared_ptr<const sdf::Element> &sdf,
                 ig::EntityComponentManager &,
                 ig::EventManager &) override
  {
    this->worldEntity = entity;

    // Parámetros
    this->actorName = sdf->Get<std::string>("actor_name", "actor_walking").first;

    auto p = sdf->Get<double>("ws_min_x", -10.0); this->minX = p.first;
    p      = sdf->Get<double>("ws_max_x",  10.0); this->maxX = p.first;
    p      = sdf->Get<double>("ws_min_y", -10.0); this->minY = p.first;
    p      = sdf->Get<double>("ws_max_y",  10.0); this->maxY = p.first;

    this->speed     = sdf->Get<double>("speed", 1.0).first;        // m/s
    this->turnRate  = sdf->Get<double>("turn_rate", 1.5).first;    // rad/s
    this->zHeight   = sdf->Get<double>("z_height", 1.0).first;     // altura fija

    // Posición inicial por parámetro (si no está, usa centro del workspace)
    this->startProvided = sdf->HasElement("start_x") && sdf->HasElement("start_y");
    this->startX  = sdf->Get<double>("start_x",
                     0.5*(this->minX + this->maxX)).first;
    this->startY  = sdf->Get<double>("start_y",
                     0.5*(this->minY + this->maxY)).first;
    this->startYaw = sdf->Get<double>("start_yaw", 0.0).first;     // rad

    // Clamp preventivo por si pasan algo fuera de rango
    this->startX = std::clamp(this->startX, this->minX, this->maxX);
    this->startY = std::clamp(this->startY, this->minY, this->maxY);

    unsigned seed = sdf->Get<unsigned>("seed", 12345u).first;
    this->rng.seed(seed);
  }

  void PreUpdate(const ig::UpdateInfo &info,
                 ig::EntityComponentManager &ecm) override
  {
    if (info.paused)
      return;

    // Buscar el actor una sola vez cuando ya exista en el ECM
    if (this->actorEntity == ig::kNullEntity)
    {
      ecm.Each<igc::Actor, igc::Name>(
        [&](const ig::Entity &e,
            const igc::Actor *,
            const igc::Name *n) -> bool
        {
          if (n && n->Data() == this->actorName)
          {
            this->actorEntity = e;
            igndbg << "[RandomActorWalkingPlugin] Encontré el actor: "
                   << this->actorName << " (entity " << e << ")\n";

            // Pose inicial desde parámetros, con Z fija y yaw dado
            ig::Actor act(e);
            math::Vector3d startPos(this->startX, this->startY, this->zHeight);
            math::Quaterniond startRot(0, 0, this->startYaw);
            math::Pose3d initPose(startPos, startRot);
            act.SetTrajectoryPose(ecm, initPose);

            // Primer objetivo aleatorio
            this->PickNewGoal(true);
            return false; // parar búsqueda
          }
          return true; // seguir buscando
        });

      // Aún no existe este frame
      if (this->actorEntity == ig::kNullEntity)
        return;
    }

    // Mover al actor
    ig::Actor act(this->actorEntity);
    auto poseOpt = act.WorldPose(ecm);
    if (!poseOpt) return;

    auto pose = *poseOpt;
    const math::Vector3d p = pose.Pos();

    // Elegir nuevo objetivo si hemos llegado al actual
    if (p.Distance(this->goal) < 0.25)
      this->PickNewGoal(false);

    // Dirección hacia el objetivo (solo X-Y)
    math::Vector3d dir = this->goal - p;
    dir.Z(0);
    if (dir.Length() > 1e-6)
      dir.Normalize();

    // Control de orientación (suavizado)
    double yaw    = pose.Rot().Yaw();
    double yawDes = std::atan2(dir.Y(), dir.X());

    math::Angle delta(yawDes - yaw);
    delta.Normalize();
    double dyaw = delta.Radian();

    double stepYaw = std::clamp(dyaw,
                                -this->turnRate * info.dt.count(),
                                 this->turnRate * info.dt.count());
    yaw += stepYaw;

    // Avance lineal
    math::Vector3d vel(this->speed * std::cos(yaw),
                       this->speed * std::sin(yaw), 0);

    // Integración con Z fija
    math::Vector3d newPos = p + vel * info.dt.count();
    newPos.Z(this->zHeight);

    // (Opcional) mantener dentro del rectángulo de trabajo:
    newPos.X(std::clamp(newPos.X(), this->minX, this->maxX));
    newPos.Y(std::clamp(newPos.Y(), this->minY, this->maxY));

    // Aplicar pose
    math::Pose3d newPose(newPos, math::Quaterniond(0, 0, yaw));
    act.SetTrajectoryPose(ecm, newPose);
  }

private:
  void PickNewGoal(bool first)
  {
    std::uniform_real_distribution<double> rx(this->minX, this->maxX);
    std::uniform_real_distribution<double> ry(this->minY, this->maxY);

    // Objetivo aleatorio en X-Y, con Z fija
    this->goal.Set(rx(this->rng), ry(this->rng), this->zHeight);

    if (!first)
    {
      igndbg << "[RandomActorWalkingPlugin] Nuevo objetivo: "
             << this->goal << "\n";
    }
  }

private:
  ig::Entity worldEntity{ig::kNullEntity};
  ig::Entity actorEntity{ig::kNullEntity};
  std::string actorName{"actor_walking"};

  // Workspace
  double minX{-10}, maxX{10}, minY{-10}, maxY{10};

  // Cinemática simple
  double speed{1.0};
  double turnRate{1.5};
  double zHeight{1.0};                // Altura fija

  // Start pose parametrizable
  bool   startProvided{false};
  double startX{0.0}, startY{0.0}, startYaw{0.0};

  // Estado interno
  math::Vector3d goal{0,0,1};
  std::mt19937 rng;
};

// Registro del plugin (Fortress / Ignition)
IGNITION_ADD_PLUGIN(RandomActorWalkingPlugin,
                    ignition::gazebo::System,
                    ignition::gazebo::ISystemConfigure,
                    ignition::gazebo::ISystemPreUpdate)

IGNITION_ADD_PLUGIN_ALIAS(RandomActorWalkingPlugin,
                          "gz::sim::systems::RandomActorWalkingPlugin")
