#ifndef _TRJMARKER_PLUGIN_HH_
#define _TRJMARKER_PLUGIN_HH_

#include <boost/shared_ptr.hpp>

#include <ignition/math/Vector3.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/rendering/rendering.hh>

#include <rexarm_poses.pb.h>

namespace gazebo
{
    typedef const boost::shared_ptr<const rexarm_poses_msgs::msgs::RexarmPoses> 
        ConstRexarmPosesPtr;

    class TrjmarkerPlugin : public VisualPlugin
    {
        public:

            TrjmarkerPlugin() {}

            virtual void Load(rendering::VisualPtr _visual, sdf::ElementPtr _sdf)
            {
                this->visual = _visual;
                
                // Pointer used to update the trajectory
                this->trj = this->visual->CreateDynamicLine(rendering::RENDERING_LINE_STRIP);
                
                // Trajectory is invisible by default
                this->trj->setVisible(false);
                
                // Initialize the end-effector position for rendering
                this->ee_pos = ignition::math::Vector3d(0, 0, 0.35);
                    
                // Create the node
                this->node = transport::NodePtr(new transport::Node());
                this->node->Init(this->visual->Name());
                
                // Listen to the render update event.
                this->updateConnection = event::Events::ConnectRender(
                                         std::bind(&TrjmarkerPlugin::OnRenderUpdate, this));

                // Listen to the end-effector pose update
                this->sub_poses = this->node->Subscribe("/gazebo/default/workspace/rexarm/poses",
                                                        &TrjmarkerPlugin::UpdateEEPosition,
                                                        this);
                // Listen to the trajectory command
                this->sub_trjcmd = this->node->Subscribe("~/trj_cmd",
                                                         &TrjmarkerPlugin::ClearTrajectory,
                                                         this);
            }
            
            // Update the end-effector position
            void UpdateEEPosition(ConstRexarmPosesPtr& poses)
            {
                ee_pos.X(0.01*poses->end_effector_pose().position().x());
                ee_pos.Y(0.01*poses->end_effector_pose().position().y());
                ee_pos.Z(poses->end_effector_pose().position().z());
            }
            
            // Update the trajectory
            void OnRenderUpdate()
            {
                this->trj->AddPoint(this->ee_pos, 
                                    ignition::math::Color::Green);
                this->trj->setMaterial("Gazebo/Green");
                this->trj->setVisibilityFlags(GZ_VISIBILITY_GUI);
            }

            // Clear/Show/Hide the trajectory
            void ClearTrajectory(ConstIntPtr& value)
            {
                switch(value->data())
                {
                    case 1:
                        this->trj->Clear();
                        this->trj->Update();
                        break;

                    case 2:
                        this->trj->setVisible(true);
                        break;

                    case 3:
                        this->trj->setVisible(false);
                        break;
                }
            }


        private:

            // The visual pointer used to visualize the end-effector trajectory
            rendering::VisualPtr visual;

            // End-effector trajectory
            rendering::DynamicLines *trj;
            
            // Vector used to store the updated end-effector position
            ignition::math::Vector3d ee_pos;

            // Pointer to the update event connection
            event::ConnectionPtr updateConnection;

            // A node used for transport
            transport::NodePtr node;

            // A subscriber listens to rexarm joint positions update
            transport::SubscriberPtr sub_poses;

            // A subscriber listens to the command of clearing the trajectory
            transport::SubscriberPtr sub_trjcmd;
    };

    // Register this plugin within the simulator
    GZ_REGISTER_VISUAL_PLUGIN(TrjmarkerPlugin)
}
#endif
