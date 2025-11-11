#include <ignition/plugin/Register.hh>
#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/components/Name.hh>
#include <ignition/gazebo/components/Model.hh>
#include <ignition/gazebo/components/Pose.hh>
#include <ignition/gazebo/components/LinearVelocity.hh>
#include <ignition/gazebo/components/AngularVelocity.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Angle.hh>

#include <random>
#include <string>
#include <cmath>
#include <algorithm>

namespace igz = ignition::gazebo;
namespace igm = ignition::math;

class RandomRobotMovementPlugin
  : public igz::System,
    public igz::ISystemConfigure,
    public igz::ISystemPreUpdate
{
public:
  void Configure(const igz::Entity &/*_entity*/,
                 const std::shared_ptr<const sdf::Element> &_sdf,
                 igz::EntityComponentManager &/*_ecm*/,
                 igz::EventManager &/*_eventMgr*/) override
  {
    this->modelName = _sdf->Get<std::string>("model_name", "robot").first;

    auto p_min_x = _sdf->Get<double>("ws_min_x", -10.0); this->minX = p_min_x.first;
    auto p_max_x = _sdf->Get<double>("ws_max_x",  10.0); this->maxX = p_max_x.first;
    auto p_min_y = _sdf->Get<double>("ws_min_y", -10.0); this->minY = p_min_y.first;
    auto p_max_y = _sdf->Get<double>("ws_max_y",  10.0); this->maxY = p_max_y.first;

    this->speed = _sdf->Get<double>("speed", 1.0).first;
    this->turnRate = _sdf->Get<double>("turn_rate", 0.5).first;

    unsigned seed = _sdf->Get<unsigned>("seed", 12345u).first;
    this->rng.seed(seed);
  }

  void PreUpdate(const igz::UpdateInfo &_info,
                 igz::EntityComponentManager &_ecm) override
  {
    if (_info.paused)
      return;

    if (this->modelEntity == igz::kNullEntity)
    {
      this->modelEntity = _ecm.EntityByComponents(igz::components::Model(),
                                                  igz::components::Name(this->modelName));
      if (this->modelEntity == igz::kNullEntity)
        return;
      
      // Pick the first goal once the model is found
      this->PickNewGoal(_ecm);
    }

    auto poseComp = _ecm.Component<igz::components::Pose>(this->modelEntity);
    if (!poseComp)
      return;

    const igm::Pose3d &pose = poseComp->Data();
    const igm::Vector3d &pos = pose.Pos();

    if (pos.Distance(this->goal) < 0.5)
    {
      this->PickNewGoal(_ecm);
    }

    igm::Vector3d dir = this->goal - pos;
    dir.Z(0);

    double yaw = pose.Rot().Yaw();
    double targetYaw = std::atan2(dir.Y(), dir.X());

    igm::Angle delta(targetYaw - yaw);
    delta.Normalize();
    double turn = delta.Radian();

    igm::Vector3d linearVel;
    igm::Vector3d angularVel;

    if (std::abs(turn) > 0.1)
    {
      // Turn in place
      angularVel.Z(std::clamp(turn, -this->turnRate, this->turnRate));
      linearVel.X(0);
    }
    else
    {
      // Move forward
      angularVel.Z(0);
      linearVel.X(this->speed);
    }
    
    // Rotate linear velocity to world frame
    linearVel = pose.Rot() * linearVel;

    // Set velocities
    this->SetLinearVelocity(_ecm, linearVel);
    this->SetAngularVelocity(_ecm, angularVel);
  }

private:
  void PickNewGoal(igz::EntityComponentManager &_ecm)
  {
    std::uniform_real_distribution<double> rx(this->minX, this->maxX);
    std::uniform_real_distribution<double> ry(this->minY, this->maxY);
    
    auto poseComp = _ecm.Component<igz::components::Pose>(this->modelEntity);
    if (!poseComp) return;

    this->goal.Set(rx(this->rng), ry(this->rng), poseComp->Data().Pos().Z());
  }

  void SetLinearVelocity(igz::EntityComponentManager &_ecm, const igm::Vector3d &_vel)
  {
    auto velComp = _ecm.Component<igz::components::LinearVelocity>(this->modelEntity);
    if (!velComp)
    {
      _ecm.CreateComponent(this->modelEntity, igz::components::LinearVelocity(_vel));
    }
    else
    {
      velComp->Data() = _vel;
    }
  }

  void SetAngularVelocity(igz::EntityComponentManager &_ecm, const igm::Vector3d &_vel)
  {
    auto velComp = _ecm.Component<igz::components::AngularVelocity>(this->modelEntity);
    if (!velComp)
    {
      _ecm.CreateComponent(this->modelEntity, igz::components::AngularVelocity(_vel));
    }
    else
    {
      velComp->Data() = _vel;
    }
  }

private:
  igz::Entity modelEntity{igz::kNullEntity};
  std::string modelName;
  double minX, maxX, minY, maxY;
  double speed;
  double turnRate;
  igm::Vector3d goal;
  std::mt19937 rng;
};

IGNITION_ADD_PLUGIN(RandomRobotMovementPlugin,
                    igz::System,
                    igz::ISystemConfigure,
                    igz::ISystemPreUpdate)

IGNITION_ADD_PLUGIN_ALIAS(RandomRobotMovementPlugin,
                          "gz::sim::systems::RandomRobotMovementPlugin")
