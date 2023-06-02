/*
 * Copyright 2020 Naval Postgraduate School
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


#include <uuv_kinematics_ros_plugin/UUVKinematicsROSPlugin.hh>

#include <gazebo/physics/Base.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/rendering/rendering.hh>

namespace uuv_kinematics_ros
{
/////////////////////////////////////////////////
UUVKinematicsROSPlugin::UUVKinematicsROSPlugin()
{
}

/////////////////////////////////////////////////
UUVKinematicsROSPlugin::~UUVKinematicsROSPlugin()
{
  this->rosNode->shutdown();
  this->updateConnection.reset();
}

/////////////////////////////////////////////////
void UUVKinematicsROSPlugin::Load(gazebo::physics::ModelPtr _model,
                             sdf::ElementPtr _sdf)
{
  if (!ros::isInitialized())
  {
    gzerr << "Not loading plugin since ROS has not been "
          << "properly initialized.  Try starting gazebo with ros plugin:\n"
          << "  gazebo -s libgazebo_ros_api_plugin.so\n";
    return;
  }

  GZ_ASSERT(_model != NULL, "Invalid model pointer");
  GZ_ASSERT(_sdf != NULL, "Invalid SDF element pointer");

  this->model = _model;
  // this->gzNode = gazebo::transport::NodePtr(new gazebo::transport::Node());
  this->rosNode.reset(new ros::NodeHandle(""));

  // Read link names from SDF configuration
  if (_sdf->HasElement("base_link_name"))
  {
    this->base_link_name = _sdf->Get<std::string>("base_link_name");
  }
  else
  {
    this->base_link_name = "base_link";  // defult name
  }

  // Subscribe Pose from outside
  ros::SubscribeOptions so =
  ros::SubscribeOptions::create<geometry_msgs::Pose>(
      this->model->GetName() + "/pose",
      1,
      boost::bind(&UUVKinematicsROSPlugin::ConveyPose, this, _1),
      ros::VoidPtr(), &this->poseSubQueue);
  this->poseSubscriber = this->rosNode->subscribe(so);
  // Spin up the queue helper thread.
  this->poseSubQueueThread = std::thread(std::bind(
      &UUVKinematicsROSPlugin::poseSubThread, this));

  // Coordinate transform functions : base_link
  this->nedTransform["base_link"].header.frame_id = this->model->GetName() + "/base_link";
  this->nedTransform["base_link"].child_frame_id = this->model->GetName() + "/base_link_ned";
  this->nedTransform["base_link"].transform.translation.x = 0;
  this->nedTransform["base_link"].transform.translation.y = 0;
  this->nedTransform["base_link"].transform.translation.z = 0;
  tf2::Quaternion quat;
  quat.setRPY(M_PI, 0, 0);
  this->nedTransform["base_link"].transform.rotation.x = quat.x();
  this->nedTransform["base_link"].transform.rotation.y = quat.y();
  this->nedTransform["base_link"].transform.rotation.z = quat.z();
  this->nedTransform["base_link"].transform.rotation.w = quat.w();

  // Connect the update event callback
  this->Connect();

  // Initiated
  gzmsg << std::endl;
  gzmsg << "###############################################" << std::endl;
  gzmsg << "#######  UUV KINEMATICS CONTROL PLUGIN  #######" << std::endl;
  gzmsg << "###############################################" << std::endl;
  gzmsg << "Vehicle Model name      : " << this->model->GetName() << std::endl;
  gzmsg << "Vehicle Base_link name  : " << this->base_link_name << std::endl;
  gzmsg << "--------------------------------------------------" << std::endl;
  gzmsg << "Pose Topic (ROS geometry_msgs/Vector3 type)" << std::endl;
  gzmsg << ":\t" + this->model->GetName() + "/pose" << std::endl;
  gzmsg << "##################################################" << std::endl;
  gzmsg << std::endl;

  // Get initial position
  this->initPosition = this->model->WorldPose().Pos();
  this->initOrientation = this->model->WorldPose().Rot();
}

/////////////////////////////////////////////////
void UUVKinematicsROSPlugin::Connect()
{
  // Connect the update event
  this->updateConnection = gazebo::event::Events::ConnectWorldUpdateBegin(
      boost::bind(&UUVKinematicsROSPlugin::Update, this, _1));
}

/////////////////////////////////////////////////
void UUVKinematicsROSPlugin::Init()
{
  // Nothing here
}

/////////////////////////////////////////////////
void UUVKinematicsROSPlugin::Reset()
{
  // Nothing here
}

/////////////////////////////////////////////////
void UUVKinematicsROSPlugin::Update(const gazebo::common::UpdateInfo &)
{
  // Update time
  this->time = this->model->GetWorld()->SimTime();

  if (ros::Time::now() != this->nedTransform["base_link"].header.stamp)
  {
    this->nedTransform["base_link"].header.stamp = ros::Time::now();
    this->tfBroadcaster["base_link"].sendTransform(this->nedTransform["base_link"]);
  }
}

/////////////////////////////////////////////////
void UUVKinematicsROSPlugin::poseSubThread()
{
  static const double timeout = 0.01;
  while (this->rosNode->ok())
  {
    this->poseSubQueue.callAvailable(ros::WallDuration(timeout));
  }
}

/////////////////////////////////////////////////
void UUVKinematicsROSPlugin::ConveyPose(
  const geometry_msgs::Pose::ConstPtr &_msg)
{
  // Convey Pose to functions
  this->ConveyKinematicsPose(_msg);
}

/////////////////////////////////////////////////
void UUVKinematicsROSPlugin::ConveyKinematicsPose(const geometry_msgs::Pose::ConstPtr &_msg)
{
  ignition::math::Vector3d targetPosition;
  targetPosition.X() = _msg->position.x;
  targetPosition.Y() = _msg->position.y;
  targetPosition.Z() = _msg->position.z;

  ignition::math::Pose3d targetPose;
  targetPose.Pos().X() = this->initPosition.X() + _msg->position.x;
  targetPose.Pos().Y() = this->initPosition.Y() + _msg->position.y;
  targetPose.Pos().Z() = this->initPosition.Z() + _msg->position.z;

  ignition::math::Quaternion<double> targetQuaternion;
  targetQuaternion.X() = _msg->orientation.x;
  targetQuaternion.Y() = _msg->orientation.y;
  targetQuaternion.Z() = _msg->orientation.z;
  targetQuaternion.W() = _msg->orientation.w;
  ignition::math::Vector3d targetEulerOrientation(this->initOrientation.Euler() + targetQuaternion.Euler());
  targetPose.Rot() = targetQuaternion.EulerToQuaternion(targetEulerOrientation);

  this->model->SetWorldPose(targetPose);
  this->link = this->model->GetLink(this->model->GetName() + "/" + this->base_link_name);
  this->link->ResetPhysicsStates();

}

GZ_REGISTER_MODEL_PLUGIN(UUVKinematicsROSPlugin)
}
