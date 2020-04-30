#ifndef _REXARM_PLUGIN_HH_
#define _REXARM_PLUGIN_HH_

#include <boost/shared_ptr.hpp>

#include <ignition/math/Pose3.hh>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

#include <rexarm_poses.pb.h>

namespace gazebo
{
    typedef const boost::shared_ptr<const rexarm_poses_msgs::msgs::RexarmPoses> 
        ConstRexarmPosesPtr;

    class RexarmPlugin : public ModelPlugin
    {
        public: RexarmPlugin() {}
        
        public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
        {
            // Safety check
            if (_model->GetJointCount() == 0)
            {
                std::cerr << "Invalid joint count, rexarm plugin not loaded\n";
                return;
            }

            // Store the pointer to the model
            this->model = _model;
            
            // Get the joints
            this->base_joint = _model->GetJoint("workspace::rexarm_gripper::rexarm::base_joint");
            this->shoulder_joint = _model->GetJoint("workspace::rexarm_gripper::rexarm::shoulder_joint");
            this->elbow_joint = _model->GetJoint("workspace::rexarm_gripper::rexarm::elbow_joint");
            this->wrist_joint = _model->GetJoint("workspace::rexarm_gripper::rexarm::wrist_joint");
            this->gp1_joint = _model->GetJoint("workspace::rexarm_gripper::gripper::joint1_1a");
            this->gp2_joint = _model->GetJoint("workspace::rexarm_gripper::gripper::joint2_1a");

            // Get the end_effector link
            this->end_effector = _model->GetLink("workspace::end_effector");
            
            // Setup a P-controller, with a gain of 0.1.
            this->base_pid = common::PID(1, 0, 0);
            this->shoulder_pid = common::PID(1, 0, 0);
            this->elbow_pid = common::PID(1, 0, 0);
            this->wrist_pid = common::PID(1, 0, 0);

            // Apply the P-controller to the joint.
            this->model->GetJointController()->SetPositionPID(
                    "workspace::rexarm_gripper::rexarm::base_joint", this->base_pid);
            this->model->GetJointController()->SetPositionPID(
                    "workspace::rexarm_gripper::rexarm::shoulder_joint", this->shoulder_pid);
            this->model->GetJointController()->SetPositionPID(
                    "workspace::rexarm_gripper::rexarm::elbow_joint", this->elbow_pid);
            this->model->GetJointController()->SetPositionPID(
                    "workspace::rexarm_gripper::rexarm::wrist_joint", this->wrist_pid);

            // Listen to the simulation update event.
            this->updateConnection = event::Events::ConnectWorldUpdateBegin(
                                     std::bind(&RexarmPlugin::OnUpdate, this));
            // Create the node
            this->node = transport::NodePtr(new transport::Node());
            this->node->Init(this->model->GetWorld()->Name());
            
            // Create a topic name
            std::string status_topic = "~/workspace/rexarm/poses";
            std::string cmd_topic = "~/workspace/rexarm/joint_cmd";

            // Listen to client's joint command request
            this->sub = this->node->Subscribe(cmd_topic, &RexarmPlugin::SetJointPositions, this);
            
            // Publish joint states while receiving simulation update
            this->pub = this->node->Advertise<rexarm_poses_msgs::msgs::RexarmPoses>(status_topic);
            std::cout << this->model->GetGripperCount() << std::endl;
            if (this->model->GetGripperCount() == 1)
                std::cout << this->model->GetGripper(0)->IsAttached() << std::endl;
        }

        // Called by the world update start event
        public: void OnUpdate()
        {
            rexarm_poses_msgs::msgs::RexarmPoses cmd;
            
            // Fill in the link msg
            gazebo::msgs::Pose *msg_ee_pose = cmd.mutable_end_effector_pose();
            gazebo::msgs::Set(msg_ee_pose, this->end_effector->WorldPose());
            // gzmsg << cmd.end_effector_pose().position().x() << std::endl; 
            
            // Fill in the joints msgs
            cmd.set_base_joint_pos(this->base_joint->Position());
            cmd.set_shoulder_joint_pos(this->shoulder_joint->Position());
            cmd.set_elbow_joint_pos(this->elbow_joint->Position());
            cmd.set_wrist_joint_pos( this->wrist_joint->Position());
        
            this->pub->Publish(cmd);
        }
    
        // Set rexarm joint positions
        public: void SetJointPositions(ConstRexarmPosesPtr &joint_cmd)
        {
            this->base_joint->SetPosition(0, joint_cmd->base_joint_pos());
            this->shoulder_joint->SetPosition(0, joint_cmd->shoulder_joint_pos());
            this->elbow_joint->SetPosition(0, joint_cmd->elbow_joint_pos());
            this->wrist_joint->SetPosition(0, joint_cmd->wrist_joint_pos());
            this->gp1_joint->SetPosition(0, joint_cmd->gp1_joint_pos());
            this->gp2_joint->SetPosition(0, joint_cmd->gp2_joint_pos());
        }
           
        // Pointer to the model.
        private: physics::ModelPtr model;

        // Pointer to the end-effeecotr link.
        private: physics::LinkPtr end_effector;
        
        // Pointer to the joints.
        private: physics::JointPtr base_joint, shoulder_joint, elbow_joint, wrist_joint,
                                   gp1_joint, gp2_joint;

        // A PID controller for the joint.
        private: common::PID base_pid, shoulder_pid, elbow_pid, wrist_pid;
        
        // Pointer to the update event connection
        private: event::ConnectionPtr updateConnection;
        
        // A node used for transport
        private: transport::NodePtr node;

        // A subscriber listens to client's request.
        private: transport::SubscriberPtr sub;

        // A publisher sends the target joint postions.
        private: transport::PublisherPtr pub;
    };

    // Register this plugin with the simulator
    GZ_REGISTER_MODEL_PLUGIN(RexarmPlugin)
}
#endif
