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

/// \file Kinematics ROS plugin for a ROS node

#ifndef __UUV_KINEMATICS_ROS_PLUGIN_HH__
#define __UUV_KINEMATICS_ROS_PLUGIN_HH__

#include <ros/ros.h>
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include <gazebo_msgs/ModelState.h>
#include <gazebo_msgs/SetModelState.h>
#include <gazebo_msgs/GetModelState.h>
#include <gazebo_msgs/LinkState.h>
#include <gazebo_msgs/SetLinkState.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>

#include <boost/scoped_ptr.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/PhysicsEngine.hh>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

#include <map>
#include <string>
#include <vector>
#include <thread>

namespace uuv_kinematics_ros
{
  class UUVKinematicsROSPlugin : public gazebo::ModelPlugin
  {
    /// \brief Constructor
    public: UUVKinematicsROSPlugin();

    /// \brief Destructor
    public: virtual ~UUVKinematicsROSPlugin();

    /// \brief Load module and read parameters from SDF.
    public: void Load(gazebo::physics::ModelPtr _model,
                      sdf::ElementPtr _sdf);

    /// \brief Initialize Module.
    public: virtual void Init();

    /// \brief Reset Module.
    public: virtual void Reset();

    /// \brief Update the simulation state.
    /// \param[in] _info Information used in the update event.
    public: virtual void Update(const gazebo::common::UpdateInfo &);

    /// \brief Connects the update event callback
    protected: virtual void Connect();

    /// \brief Update event
    protected: gazebo::event::ConnectionPtr updateConnection;

    /// \brief Convey model state from gazebo topic to outside
    protected: virtual void ConveyPose(const geometry_msgs::Pose::ConstPtr &_msg);

    /// \brief Convey model state from gazebo topic to outside (model)
    protected: virtual void ConveyKinematicsPose(const geometry_msgs::Pose::ConstPtr &_msg);

    /// \brief Pointer to the model structure
    protected: gazebo::physics::ModelPtr model;

    /// \brief Pointer to this ROS node's handle.
    private: boost::scoped_ptr<ros::NodeHandle> rosNode;

    /// \brief ROS Subscribers from outside
    private: ros::Subscriber poseSubscriber;

    /// \brief A ROS callbackqueue that helps process messages
    private: ros::CallbackQueue poseSubQueue;

    /// \brief A thread the keeps running the rosQueue
    private: std::thread poseSubQueueThread;

    /// \brief ROS helper function that processes messages
    private: void poseSubThread();

    /// \brief Time at gazebo simulation
    protected: gazebo::common::Time time;
    
    /// \brief Pointer to the base_link
    protected: gazebo::physics::LinkPtr link;

    /// \brief Base link name
    protected: std::string base_link_name;

    /// \brief Model State
    protected: gazebo_msgs::ModelState modelState;

    /// \brief Model Pose
    protected: ignition::math::Vector3<double> initPosition;
    protected: ignition::math::Quaternion<double> initOrientation;

    private: std::map<std::string, geometry_msgs::TransformStamped> nedTransform;
    private: std::map<std::string, tf2_ros::TransformBroadcaster> tfBroadcaster;
  };
}

#endif  // __UUV_KINEMATICS_ROS_PLUGIN_HH__
