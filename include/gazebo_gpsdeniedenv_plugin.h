/*
 * Copyright (C) 2012 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#include <ignition/math/Quaternion.hh>
#include <ignition/math/Vector3.hh>

#include "gazebo/common/common.hh"
#include "gazebo/gazebo.hh"
#include "gazebo/common/Animation.hh"
#include "gazebo/common/CommonTypes.hh"
#include "gazebo/common/KeyFrame.hh"
#include "gazebo/gazebo.hh"
#include "gazebo/physics/Model.hh"
#include "gazebo/msgs/msgs.hh"

#include <GpsDenied.pb.h>

namespace gazebo
{
typedef const boost::shared_ptr<const sensor_msgs::msgs::GpsDenied> GpsDeniedPtr;

class GpsDeniedEnv : public ModelPlugin
{

public:
    GpsDeniedEnv();
    virtual ~GpsDeniedEnv();

protected:
    void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/);

private:
    void OnUpdate();
    float RandomFloat(float a, float b);

private:
    // Pointer to the model
    physics::ModelPtr _model;
    transport::NodePtr _node;
    std::string _namespace;

    transport::PublisherPtr _gpsdenied_pos_pub;
    ignition::math::Vector3d randWalkPose;
    private: double _prev_t;
    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;
};
}

