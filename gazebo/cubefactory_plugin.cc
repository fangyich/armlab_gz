#ifndef _CUBEFACTORY_PLUGIN_HH_
#define _CUBEFACTORY_PLUGIN_HH_

#include <ignition/math/Pose3.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>

#include <cube_info.pb.h>

namespace gazebo
{
    class CubeFactoryPlugin : public WorldPlugin
    {
        typedef const boost::shared_ptr<const cube_info_msgs::msgs::CubeInfo> 
        CubeInfoPtr;

        public:

            CubeFactoryPlugin()
            {
                // Initialize the sdf template of the cube
                this->cubeSDF.SetFromString(
                    "<sdf version ='1.5'>\
                        <model name ='cube_proto'>\
                            <pose>0 0 0 0 0 0</pose>\
                            <link name ='link'>\
                                <inertial>\
                                    <mass>10e-3</mass>\
                                    <inertia>\
                                        <ixx>6.048e-7</ixx>\
                                        <ixy>0</ixy>\
                                        <ixz>0</ixz>\
                                        <iyy>6.048e-7</iyy>\
                                        <iyz>0</iyz>\
                                        <izz>6.048e-7</izz>\
                                    </inertia>\
                                </inertial>\
                                <collision name ='collision'>\
                                    <geometry>\
                                        <box><size>0.01905 0.01905 0.01905</size></box>\
                                    </geometry>\
                                </collision>\
                                <visual name ='visual'>\
                                    <geometry>\
                                        <box><size>0.01905 0.01905 0.01905</size></box>\
                                    </geometry>\
                                    <material>\
                                        <script>\
                                            <uri>file://media/materials/scripts/gazebo.material</uri>\
                                            <name>Gazebo/Grey</name>\
                                        </script>\
                                    </material>\
                                </visual>\
                            </link>\
                        </model>\
                    </sdf>");
            }

            void Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf)
            {
                // Store the world pointer
                this->parent = _parent;

                // Listen to the simulation update event
                this->updateConnection = event::Events::ConnectWorldUpdateBegin(
                                         std::bind(&CubeFactoryPlugin::OnUpdate, 
                                         this));

                // Create a new transport node
                this->node = transport::NodePtr(new transport::Node());
                this->node->Init(_parent->Name());

                // Listen to the cube spawning command
                this->sub = this->node->Subscribe("~/workspace/cube_cmd",
                                                  &CubeFactoryPlugin::CubeCommand,
                                                  this);
                // Publish cubes' pose data while receiving simulation update
                this->pub = this->node->Advertise<cube_info_msgs::msgs::CubeInfo>(
                                        "~/workspace/cube_info");
            }

            // Publish cubes' pose info
            void OnUpdate()
            {
                for (int i = 0; i < this->cube_id.size(); i++)
                {
                    physics::ModelPtr p = this->parent->ModelByName(cube_id[i]);
                    if (p != NULL)
                    {
                        cube_info_msgs::msgs::CubeInfo info;
                        // Fill in the id
                        info.set_id(cube_id[i]);
                        // Fill in the pose msg
                        gazebo::msgs::Pose *msg_cube_pose = info.mutable_cube_pose();
                        gazebo::msgs::Set(msg_cube_pose, p->WorldPose());
                        // Publish the cube info
                        this->pub->Publish(info);
                    }
                }
            }

            // Process the cube generation/information request
            void CubeCommand(ConstIntPtr& value)
            {
                switch(value->data())
                {
                    // Cube generation
                    case 1:
                        std::string new_id =  "cube"+std::to_string(this->cube_id.size()); 

                        // Assign the color and the position to the cube randomly
                        std::string color = this->cube_color[std::rand()%7];
                        double x = -0.3+std::rand()%7/10.0;
                        double y = -0.3+std::rand()%7/10.0;
                        double z = 0.1;
                        std::string pose = std::to_string(x)+" "+std::to_string(y)+" "+std::to_string(z)+
                                           " "+"0 0 0";

                        // Modify the sdf template
                        sdf::ElementPtr model = this->cubeSDF.Root()->GetElement("model");
                        model->GetAttribute("name")->SetFromString(new_id);
                        model->GetElement("pose")->Set(pose);
                        model->GetElement("link")->GetElement("visual")->GetElement("material")
                             ->GetElement("script")->GetElement("name")->Set(color);
                        
                        // Insert the cube to the world
                        this->parent->InsertModelSDF(this->cubeSDF);
                        
                        // Record the newly generated cube's id
                        //this->cubeSDF.PrintValues();
                        //std::cout << "get cube command" << std::endl;
                        this->cube_id.push_back(new_id);
                        break;    
                }
            }

        private:

            // The world pointer
            physics::WorldPtr parent;

            // A node used for transport
            transport::NodePtr node;

            // A subscriber listens to the cube spawning command
            transport::SubscriberPtr sub;
            
            // A publisher sends the newly added cube's info
            transport::PublisherPtr pub;
            
            // Pointer to the update event connection
            event::ConnectionPtr updateConnection;
            
            // The sdf template of the cube
            sdf::SDF cubeSDF;

            // List of available colors
            std::string cube_color[7] = {"Gazebo/Red", "Gazebo/Blue", "Gazebo/Green", "Gazebo/Yellow", 
             "Gazebo/Black", "Gazebo/Purple","Gazebo/Orange"};

            std::vector<std::string> cube_id;
    };

    // Register this plugin with the simulator
    GZ_REGISTER_WORLD_PLUGIN(CubeFactoryPlugin)
}
#endif
