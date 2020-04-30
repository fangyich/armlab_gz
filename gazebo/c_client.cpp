#include <gazebo/gazebo_config.h>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/gazebo_client.hh>
#include <ignition/math/Pose3.hh>

#include <iostream>
#include <string>
#include <sstream>
#include <queue>
#include <map>

#include <boost/chrono.hpp>
#include <boost/atomic.hpp>
#include <boost/asio.hpp>
#include <boost/asio/signal_set.hpp>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>

#include <rexarm_poses.pb.h>
#include <cube_info.pb.h>


using boost::asio::ip::tcp;

typedef const boost::shared_ptr<const rexarm_poses_msgs::msgs::RexarmPoses> ConstRexarmPosesPtr;
typedef const boost::shared_ptr<const cube_info_msgs::msgs::CubeInfo> ConstCubeInfoPtr;

class GzState
{
    public:

        GzState (gazebo::transport::NodePtr& node):
            _c_node(node)
        {
            // Subscribe to the pose update topic of the rexarm
            _pose_sub = _c_node->Subscribe("~/workspace/rexarm/poses", 
                                           &GzState::UpdateSimStates, this);
            // Subscribe to the cube info update topic
            _cube_sub = _c_node->Subscribe("~/workspace/cube_info", 
                                           &GzState::UpdateCubeInfo, this);
        }

        // Return rexarm poses in string
        std::string get_state_msg ()
        {
            return _state_msg;
        }

        // Return cube info in string
        std::string get_cube_info ()
        {
            std::string info = "";
            for(std::map<std::string,gazebo::msgs::Pose>::iterator it=_cube_list.begin(); 
                it!=_cube_list.end(); ++it)
            {
                ignition::math::Quaterniond qt = gazebo::msgs::ConvertIgn((it->second).orientation());
                info += it->first + "," + 
                        std::to_string((it->second).position().x()) + "," +
                        std::to_string((it->second).position().y()) + "," +
                        std::to_string((it->second).position().z()) + "," +
                        std::to_string(qt.Yaw()) + "," +
                        std::to_string(qt.Pitch()) + "," +
                        std::to_string(qt.Roll()) + "\n" ;
            }
            return info;
        }

    private:

        // Gazebo transport node & subscribers
        gazebo::transport::NodePtr& _c_node;
        gazebo::transport::SubscriberPtr _pose_sub, _cube_sub; 
        // rexarm poses
        std::string _state_msg;
        // cube info
        std::map<std::string, gazebo::msgs::Pose> _cube_list;

        void UpdateSimStates (ConstRexarmPosesPtr& joint_pos)
        {   
            _state_msg = std::to_string(joint_pos->end_effector_pose().position().x()) + "," +
                         std::to_string(joint_pos->end_effector_pose().position().y()) + "," +
                         std::to_string(joint_pos->end_effector_pose().position().z()) + "," + 
                         std::to_string(joint_pos->base_joint_pos()) + "," + 
                         std::to_string(joint_pos->shoulder_joint_pos()) + "," +
                         std::to_string(joint_pos->elbow_joint_pos()) + "," + 
                         std::to_string(joint_pos->wrist_joint_pos());
        }

        void UpdateCubeInfo (ConstCubeInfoPtr& info)
        {
            _cube_list[info->id()] = info->cube_pose();
        }
};

class Request
{
    public:

        Request (boost::asio::io_service& ios, 
                 gazebo::transport::NodePtr& node,
                 GzState& states):
            _sock(ios),
            _node(node),
            _states(states) {}
        
        void Response ()
        { 
            parse_message();
            process_request(std::stoi(_items[0]));
        }

        void Stop ()
        {
            try
            {
                _sock.close();
            }
            catch (boost::system::error_code& e)
            {
	        	std::cerr << e.message() << std::endl;
            }
        }

        tcp::socket& sock ()
        { 
            return _sock;
        }
    
    private:
        // TCP socket
        tcp::socket _sock;
        // Gazebo transport node
        gazebo::transport::NodePtr& _node;
        // Arm & environment data from Gazebo
        GzState& _states;
        // Parsed request
        std::vector<std::string> _items;

        void parse_message ()
        {
            // retrieve data from the socket
		    boost::asio::streambuf buf;
		    boost::asio::read_until(_sock, buf, '\n');
		    std::string msg = boost::asio::buffer_cast<const char*>(buf.data());

            // parse the message
            std::stringstream stream(msg);
            std::string tmp;
            while (getline(stream, tmp, ','))
                _items.push_back(tmp);
	    
        }

        void process_request (int type)
        {
            switch (type)
            {
                case 1:
                    send_joint_command();
                    break;
                case 2:
                    send_states();
                    break;
                case 3:
                    send_trj_command();
                    break;
                case 4:
                    send_cube_command();
                    break;
                case 5:
                    send_cube_info();
                    break;
                default:
                    break;
            }
        }

        void send_joint_command ()
        {
            // Create a publisher
            gazebo::transport::PublisherPtr pub;
            pub = _node->Advertise<rexarm_poses_msgs::msgs::RexarmPoses>(
                    "~/workspace/rexarm/joint_cmd");
            // Create a joint command message
            rexarm_poses_msgs::msgs::RexarmPoses cmd;
            cmd.set_base_joint_pos(std::stod(_items[1]));
            cmd.set_shoulder_joint_pos(std::stod(_items[2]));
            cmd.set_elbow_joint_pos(std::stod(_items[3]));
            cmd.set_wrist_joint_pos(std::stod(_items[4]));
            cmd.set_gp1_joint_pos(std::stod(_items[5]));
			cmd.set_gp2_joint_pos(std::stod(_items[6]));
            pub->Publish(cmd);
        }

        void send_states ()
        {
            try
            {
                boost::asio::write(_sock, boost::asio::buffer(_states.get_state_msg()));
            }
            catch (boost::system::system_error& e)
            {
                std::cerr << "[ERROR] send failed: " << e.what() << std::endl;
            }
        }
        
        void send_trj_command ()
        {
            // Create a publisher
            gazebo::transport::PublisherPtr pub;
            pub = _node->Advertise<gazebo::msgs::Int>(
                    "/gazebo/ground_plane::link::visual/trj_cmd");
            // Create a trajectory command message
            gazebo::msgs::Int cmd;
            cmd.set_data(std::stoi(_items[1]));
            pub->Publish(cmd);
        }

        void send_cube_command ()
        {
            // Create a publisher
            gazebo::transport::PublisherPtr pub;
            pub = _node->Advertise<gazebo::msgs::Int>(
                    "~/workspace/cube_cmd");
            // Create a trajectory command message
            gazebo::msgs::Int cmd;
            cmd.set_data(std::stoi(_items[1]));
            pub->Publish(cmd);
        }

        void send_cube_info ()
        {
            try
            {
                boost::asio::write(_sock, boost::asio::buffer(_states.get_cube_info()));
            }
            catch (boost::system::system_error& e)
            {
                std::cerr << "[ERROR] send failed: " << e.what() << std::endl;
            }
        }
};

class TCPServer
{
    public:
        
        TCPServer (int service_port,  
                   boost::asio::io_service &srv_ios, 
                   gazebo::transport::NodePtr &node,
                   GzState& state):
            _srv_ios(srv_ios),
            _acceptor(srv_ios, tcp::endpoint(tcp::v4(), service_port)),
            _node(node),
            _gz_state(state),
            m_stop(false)
        {}
        
        void Start ()
        {
    	    _threads.create_thread(boost::bind(&TCPServer::accept_thread, this));
            _threads.create_thread(boost::bind(&TCPServer::process_thread, this));        
        }

        void Stop ()
        {
            m_stop.store(true);
            std::cout << "Receive interrupt, closing the tcp server..." << std::endl;
        }

    public:
        
        typedef boost::shared_ptr<Request> ReqPtr;

        
    private:
        
        boost::asio::io_service& _srv_ios;
        tcp::acceptor _acceptor;
        gazebo::transport::NodePtr& _node;
        GzState& _gz_state;
        std::atomic<bool> m_stop;
        //gazebo::transport::PublisherPtr _pub;
        
        boost::thread_group _threads;
        boost::mutex _qlock;
        std::queue<ReqPtr> _reqQueue;

        void accept_thread()
        {
            std::cout << "Accept thread created" << std::endl;
            while(!m_stop.load())
            {
                // create a pointer fro the object of new request
                ReqPtr req(new Request(_srv_ios, _node, _gz_state));
		        // create a socket and wait for connection
		        _acceptor.accept(req->sock());
		        // enqueue the reuqest
                boost::mutex::scoped_lock lk(_qlock);
                _reqQueue.push(req);
            }
        }

        void process_thread()
        {
            std::cout << "Process thread created" << std::endl;
            while(!m_stop.load())
            {
                boost::this_thread::sleep_for(boost::chrono::milliseconds(1));
                boost::mutex::scoped_lock lk(_qlock);
                if (!_reqQueue.empty())
                {
                    ReqPtr p = _reqQueue.front();
                    _reqQueue.pop();
                    p->Response();
                }
            }
        }
};


int main (int _argc, char **_argv)
{
    int service_port = std::stoi(_argv[1]);  
    // open the port
	boost::asio::io_service io_service;  // to client
    boost::asio::signal_set signal_set(io_service, SIGTERM, SIGINT);
    signal_set.async_wait(
        [&io_service](
           const boost::system::error_code& error,
           int signal_number)
        {
            std::cout << "Got signal " << signal_number << "; "
                "stopping io_service." << std::endl;
            io_service.stop();
        }
    );        
    // Load gazebo as a client
    gazebo::client::setup();

    // Create our node for communication
    gazebo::transport::NodePtr node(new gazebo::transport::Node());
    node->Init();
    try
    {
        // Start the client 
        GzState states(node);
        // Start the server
        TCPServer server(service_port, io_service, node, states);
        server.Start();
        
        std::cout << "Interface is running!" << std::endl;
        // Wait for the interrupts
        io_service.run();
        // Clean up
        server.Stop();
  		gazebo::client::shutdown();
    }
    catch (std::exception& e)
    {
        std::cerr << e.what() << std::endl;
    }

    return 0; 
}
