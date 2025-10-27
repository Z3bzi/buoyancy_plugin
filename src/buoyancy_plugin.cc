#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/common/Console.hh>
#include <functional>
#include <vector>
#include <ignition/math/Helpers.hh>
#include <ignition/math/Vector3.hh>

using namespace gazebo;

class BuoyancyPlugin : public ModelPlugin {
public:
  struct LinkCfg {
    physics::LinkPtr link;
    double A_wp{0.0};
    double height{0.0};
    ignition::math::Vector3d cobRel{0,0,0};
    double linD{0.0};
    double angD{0.0};
    double kRight{0.0};
  };

  void Load(physics::ModelPtr model, sdf::ElementPtr sdf) override {
    model_ = model;
    world_ = model_->GetWorld();

    rho_   = sdf->Get<double>("fluid_density", 997.0).first;
    zSurf_ = sdf->Get<double>("surface_z", 0.0).first;
    g_     = 9.80665;

    gzmsg << "[BuoyancyPlugin] Available links:" << std::endl;
    for (const auto &link : model_->GetLinks()) {
      if (link)
        gzmsg << "  - '" << link->GetName() << "' (canonical="
              << (link->GetId() == model_->GetLink()->GetId() ? "true" : "false") << ")"
              << std::endl;
    }

    gzmsg << "[BuoyancyPlugin] fluid_density=" << rho_ << ", surface_z=" << zSurf_ << std::endl;

    if (sdf->HasElement("link")) {
      auto l = sdf->GetElement("link");
      while (l) {
        LinkCfg cfg;
        const auto linkName = l->Get<std::string>("name");
        cfg.link   = model_->GetLink(linkName);
        if (!cfg.link) {
          cfg.link = model_->GetLink(model_->GetName() + "::" + linkName);
        }
        if (!cfg.link) {
          gzerr << "[BuoyancyPlugin] Unable to find link '" << l->Get<std::string>("name")
                << "' on model '" << model_->GetName() << "'" << std::endl;
          l = l->GetNextElement("link");
          continue;
        }
        cfg.A_wp   = l->Get<double>("waterplane_area", 0.0).first;
        cfg.height = l->Get<double>("height", 0.0).first;
        cfg.linD   = l->Get<double>("linear_damping", 0.0).first;
        cfg.angD   = l->Get<double>("angular_damping", 0.0).first;
        cfg.kRight = l->Get<double>("k_righting", 0.0).first;

        if (l->HasElement("cob")) {
          auto c = l->GetElement("cob");
          cfg.cobRel = ignition::math::Vector3d(
            c->Get<double>("x", 0).first,
            c->Get<double>("y", 0).first,
            c->Get<double>("z", 0).first);
        }

        links_.push_back(cfg);
        gzmsg << "[BuoyancyPlugin] link='" << cfg.link->GetName()
              << "' waterplane_area=" << cfg.A_wp
              << " height=" << cfg.height
              << " linear_damping=" << cfg.linD
              << " angular_damping=" << cfg.angD
              << " k_righting=" << cfg.kRight
              << " cob=" << cfg.cobRel << std::endl;
        l = l->GetNextElement("link");
      }
    }

    updateConn_ = event::Events::ConnectWorldUpdateBegin(
      std::bind(&BuoyancyPlugin::OnUpdate, this));
  }

  void OnUpdate() {
    for (auto &cfg : links_) {
      if (!cfg.link) continue;
      auto pose = cfg.link->WorldCoGPose();
      double z = pose.Pos().Z();
      double sub = ignition::math::clamp(zSurf_ - z, 0.0, cfg.height);
      double Fb = rho_ * g_ * cfg.A_wp * sub;

      if (Fb > 0) {
        ignition::math::Vector3d cobWorld =
          pose.Pos() + pose.Rot().RotateVector(cfg.cobRel);
        cfg.link->AddForceAtWorldPosition({0,0,Fb}, cobWorld);
      }

      cfg.link->AddForce(-cfg.linD * cfg.link->WorldLinearVel());
      cfg.link->AddTorque(-cfg.angD * cfg.link->WorldAngularVel());

      if (cfg.kRight > 0) {
        auto R = pose.Rot().Euler();
        cfg.link->AddTorque({-cfg.kRight * R.X(), -cfg.kRight * R.Y(), 0});
      }
    }
  }

private:
  physics::ModelPtr model_;
  physics::WorldPtr world_;
  event::ConnectionPtr updateConn_;
  std::vector<LinkCfg> links_;
  double rho_{997.0}, zSurf_{0.0}, g_{9.80665};
};

GZ_REGISTER_MODEL_PLUGIN(BuoyancyPlugin)
