#include "gazebo/msgs/msgs.hh"
#include <gazebo_gpsdeniedenv_plugin.h>

using namespace std;

namespace gazebo {
GZ_REGISTER_MODEL_PLUGIN(GpsDeniedEnv)

GpsDeniedEnv::GpsDeniedEnv() : ModelPlugin()
{
}

GpsDeniedEnv::~GpsDeniedEnv()
{
}

void GpsDeniedEnv::Load(physics::ModelPtr parent, sdf::ElementPtr sdf)
{
    _model = parent;
    _prev_t = 0.0;

    _namespace.clear();
    if (sdf->HasElement("robotNamespace")) {
        _namespace = sdf->GetElement("robotNamespace")->Get<std::string>();
    } else {
        gzerr << "[gazebo_gps_plugin] Please specify a robotNamespace.\n";
    }

    _node = transport::NodePtr(new gazebo::transport::Node());
    _node->Init(_namespace);

    _gpsdenied_pos_pub = _node->Advertise<msgs::Vector3d>("~/gps_denied_pos");
    // _gpsdenied_pos_pub = _node->Advertise<sensor_msgs::msgs::GpsDenied>("~/gps_denied_pos");


    // Listen to the update event. This event is broadcast every
    // simulation iteration.
    this->updateConnection = event::Events::ConnectWorldUpdateBegin(
            std::bind(&GpsDeniedEnv::OnUpdate, this));
}

void GpsDeniedEnv::OnUpdate()
{
        double t = common::Time::GetWallTime().Double();

        if ( t - _prev_t > 1.0 ) {
            double rand_x = RandomFloat(-1.0, 1.0) * 1;
            double rand_y = RandomFloat(-1.0, 1.0) * 1;
            double rand_z = RandomFloat(-1.0, 1.0) * 1;

            randWalkPose += ignition::math::Vector3d(rand_x, rand_y, rand_z);
            // printf("%f : %f, %f, %f\n", t, randWalkPose.X(), randWalkPose.Y(), randWalkPose.Z() );
            _model->SetLinearVel(randWalkPose);

            _prev_t  = t;
        }


        msgs::Vector3d pos;
        ignition::math::Vector3d model_pos = _model->WorldPose().Pos();
        pos.set_x(model_pos.X());
        pos.set_y(model_pos.Y());
        pos.set_z(360);
        _gpsdenied_pos_pub->Publish(pos);

        // sensor_msgs::msgs::GpsDenied msg;
        // ignition::math::Vector3d model_pos = _model->WorldPose().Pos();
        // msg.set_x(model_pos.X());
        // msg.set_y(model_pos.Y());
        // msg.set_z(model_pos.Z());
        // _gpsdenied_pos_pub->Publish(msg);
}

float GpsDeniedEnv::RandomFloat(float a, float b)
{
        float random = ((float) rand()) / (float) RAND_MAX;
        float diff = b - a;
        float r = random * diff;
        return a + r;
}
} // namespace gazebo
