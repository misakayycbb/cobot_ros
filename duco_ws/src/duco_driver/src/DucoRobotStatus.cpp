
#include "duco_driver/DucoCobot.h"
#include "duco_driver/DucoRobotStatus.h"

#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int32.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32MultiArray.h>

#include <sensor_msgs/JointState.h>
#include <control_msgs/FollowJointTrajectoryFeedback.h>
#include <industrial_msgs/RobotStatus.h>

#include "duco_msgs/DucoIOState.h"
#include "duco_msgs/DucoRobotState.h"



namespace duco_driver
{

    using namespace DucoRPC;
    unsigned int DEFAULT_AXIS = 6;
    DucoRobotStatus::DucoRobotStatus()
    {
        std::string s;
        ros::param::get("/duco_driver/server_host", s); //The server_host should be corresponding to the robot controller setup.
        server_host_ = (s=="")? "127.0.0.1" : s;

        joint_names_ .push_back("joint_1");
        joint_names_ .push_back("joint_2");
        joint_names_ .push_back("joint_3");
        joint_names_ .push_back("joint_4");
        joint_names_ .push_back("joint_5");
        joint_names_ .push_back("joint_6");

        duco_robot_ = new DucoRPC::DucoCobot(server_host_, 7003);

        std::string def_joint_names[] = {"joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"};
        std::vector<std::string> jointNames;
        ros::param::get("controller_joint_names",jointNames);

        std::remove(jointNames.begin(), jointNames.end(), std::string());

        if(  jointNames.size()>0 )
        {
            for (size_t i = 0; i < jointNames.size(); i++)
            {
                joint_names_[i] = jointNames[i];
                std::cout <<" joint_names_ " << joint_names_ [i]<< std::endl;
            }
            axis_number_ = jointNames.size();
        }
        else
        {
            axis_number_ = 6;
        }

        for(uint ii=0;  ii!=axis_number_;  ++ii)
        {
            current_joints_.push_back(0);
        }

        /*通知MoveGroup机器人的状态*/
        robot_status_pub_ = nh_.advertise<industrial_msgs::RobotStatus>("robot_status", 100);
        joint_states_pub_ = nh_.advertise<sensor_msgs::JointState>("joint_states", 300);
        joint_feedback_pub_ = nh_.advertise<control_msgs::FollowJointTrajectoryFeedback>("feedback_states", 100);

        /*通知客户端机器人的状态*/
        robot_state_pub_ = nh_.advertise<duco_msgs::DucoRobotState> ("/duco_cobot/robot_state",30);
        robot_io_pub_ = nh_.advertise<duco_msgs::DucoIOState>("/duco_cobot/io_state", 30);
        rib_pub_ = nh_.advertise<std_msgs::Int32MultiArray>("/duco_cobot/rib_status", 100);
        connect_pub_ = nh_.advertise<std_msgs::UInt8>("connect", 100);

    }

    DucoRobotStatus::~DucoRobotStatus()
    {
        duco_robot_->close();
    }

    void DucoRobotStatus::getRobotStatus()
    {
        std::vector<double> robot_joint_value;
        //获取机器人控制器状态数据并转发给ROS
        if( controller_connected_flag_ )
        {
            ROS_INFO("timerCallback before robot_status_pub_ ");
            /** Query the states of robot joints **/
            {
                /** this method upates the joint states more quickly **/
                duco_robot_->get_actual_joints_position(robot_joint_value);

               ROS_INFO("duco_robot_->get_actual_joints_position robot_joint_value %d", static_cast<int>(robot_joint_value.size()) );

                if( robot_joint_value.size()>0 )
                {
                    current_joints_.clear();
                    std::vector<double> joints;
                    for(int i = 0; i < robot_joint_value.size(); i++)
                    {
                        joints.push_back( robot_joint_value[i] );
                        current_joints_.push_back(robot_joint_value[i]);
                    }

                    ROS_INFO("duco_robot_->robotmoving before " );
                    /** Get the buff size of thr rib **/
                    start_move_ = duco_robot_->robotmoving();

                    RobotStatusList  status;
                    // this is controlled by Robot Controller
                    duco_robot_->getRobotStatus(status);            
                    // publish robot_status information to the controller action server.
                    robot_status_.mode.val            = (int8_t)status.robotState;
                    robot_status_.e_stopped.val       = (int8_t)(status.emcStopSignal );
                    robot_status_.drives_powered.val  = (int8_t)1;
                    robot_status_.motion_possible.val = (int)(!start_move_);
                    robot_status_.in_motion.val       = (int)start_move_;
                    robot_status_.in_error.val        = (int)0;   //used for protective stop.
                    robot_status_.error_code          = (int32_t)status.robotError;

                    emergency_stopped_ = status.emcStopSignal;

                    uint count = status.jointExpectPosition.size()>DEFAULT_AXIS ? DEFAULT_AXIS : status.jointExpectPosition.size();
                    
                    ROS_INFO("duco_robot_->getRobotStatus count %d",count);

                    for( uint ii =0 ; ii!=count; ++ii )
                    {
                        ROS_INFO("duco_robot_->getRobotStatus ii %d jointExpectPosition size %d", ii , static_cast<int>(robot_state_.joint_expect_position.size()));
                        ROS_INFO("status.jointExpectPosition ii %d jointExpectPosition %.6f",  ii, status.jointExpectPosition[ii] );
                        ROS_INFO("robot_state_.joint_expect_position  ii %d robot_state_.joint_expect_position %.6f", ii , robot_state_.joint_expect_position[ii]);
           
                        robot_state_.joint_expect_position[ii] = status.jointExpectPosition[ii];
                        robot_state_.joint_expect_velocity[ii] = status.jointExpectVelocity[ii];
                        robot_state_.joint_expect_accelera[ii] = status.jointExpectAccelera[ii];
                        robot_state_.joint_actual_position[ii] = status.jointActualPosition[ii];
                        robot_state_.joint_actual_velocity[ii] = status.jointActualVelocity[ii];
                        robot_state_.joint_actual_accelera[ii] = status.jointActualAccelera[ii];
                        robot_state_.joint_actual_current[ii] = status.jointActualCurrent[ii];

                        robot_state_.cart_expect_position[ii] = status.cartExpectPosition[ii];
                        robot_state_.cart_expect_velocity[ii] = status.cartExpectVelocity[ii];
                        robot_state_.cart_expect_accelera[ii] = status.cartExpectAccelera[ii];
                        robot_state_.cart_actual_position[ii]  = status.cartActualPosition[ii];
                        robot_state_.cart_actual_velocity[ii] = status.cartActualAccelera[ii];
                        robot_state_.cart_actual_accelera[ii] = status.cartActualAccelera[ii];


                        //robot_state_.slave_ready[ii] = status.slaveReady[ii];



                        //robot_state_.joint_temperature[ii] = status.jointTemperature[ii];
                        //robot_state_.driver_temperature[ii] = status.driverTemperature[ii];

                        ROS_INFO("duco_robot_->getRobotStatus Leave ii %d", ii );

                    }

                    robot_state_.collision = status.collision;
                    robot_state_.collision_axis = status.collisionAxis;

                    robot_state_.emc_stop_signal = status.emcStopSignal;
                    robot_state_.robot_state = status.robotState;
                    robot_state_.robot_error = status.robotError;

                    ROS_INFO("duco_robot_->getRobotStatus ");
                    
                }
                else
                {
                    /** Here we check the connection to satisfy the ROS-I specification **/
                    /** Try to connect with the robot again **/
                    ROS_ERROR("Cann't connect to the robot controller!");
                }

                //publish the rib_status to the controller simulator
                //rib_status_.data[0] = buf_queue_.size();
                rib_status_.data.resize(3);
                rib_status_.data[0] = 0;
                rib_status_.data[1] = 0;
                rib_status_.data[2] = controller_connected_flag_;
        
                ROS_INFO("robot_status_pub_  before ");

                robot_status_pub_.publish(robot_status_);
                rib_pub_.publish(rib_status_);
                robot_state_pub_.publish(robot_state_);
                
                ROS_INFO("robot_state_pub_  publish ");

            }

            ROS_INFO("RobotStatusReply before joint_states_pub_");

            {
                //joint_states_pub_
                //joint_feedback_pub_
                sensor_msgs::JointState joint_state;
                control_msgs::FollowJointTrajectoryFeedback joint_feedback;

                joint_state.header.stamp = ros::Time::now();
                joint_state.name.resize(axis_number_);
                joint_feedback.joint_names.resize(axis_number_);
                joint_state.position.resize(axis_number_);
                joint_feedback.actual.positions.resize(axis_number_);
                for(int i = 0; i<axis_number_; i++)
                {
                    joint_state.name[i] = joint_names_[i];
                    if(controller_connected_flag_)
                        joint_state.position[i] = robot_joint_value[i];
                    else
                        joint_state.position[i] = current_joints_[i];

                    joint_feedback.joint_names[i] = joint_names_[i];
                    joint_feedback.actual.positions[i] = joint_state.position[i];
                }

                joint_states_pub_.publish(joint_state);
                joint_feedback_pub_.publish(joint_feedback);
            }//joint_states_pub_   joint_feedback_pub_

            ROS_INFO("RobotStatusReply before io_pub_");
            {
                //IO Msg
                int ret = 0;

                IOStatusList          robot_io;
                duco_msgs::DucoIOState io_msg;
                /** robot control board IO **/
                if( duco_robot_ )
                    duco_robot_->getRobotIOStatus( robot_io );

                /** F1-F6 are reserved. **/
                for (unsigned int i = 0; i < robot_io.digitalInputs.size(); i++)
                {
                    duco_msgs::DucoDigital digo;
                    digo.pin = i;
                    digo.flag = 0;
                    digo.state = robot_io.digitalInputs[i];
                    io_msg.digital_in_states.push_back(digo);
                }

                for (unsigned int i = 0; i < robot_io.digitalOutputs.size(); i++)
                {
                    duco_msgs::DucoDigital dogo;
                    dogo.pin = i;
                    dogo.flag = 0;
                    dogo.state = robot_io.digitalOutputs[i];

                    ROS_INFO("robot_io.digitalOutputs[%d] value %d", i, static_cast<int> (dogo.state));

                    io_msg.digital_out_states.push_back(dogo);      
                }

                for (unsigned int i = 0; i < robot_io.analogCurrentOutputs.size(); i++)
                {
                    duco_msgs::DucoAnalog ai;
                    ai.pin = 0;
                    ai.state = robot_io.analogCurrentOutputs[0];
                    io_msg.analog_in_states.push_back(ai);
                }

                for (unsigned int i = 0; i < robot_io.analogVoltageOutputs.size(); i++)
                {
                    duco_msgs::DucoAnalog ao;
                    ao.pin = 0;
                    ao.state = robot_io.analogVoltageOutputs[0];
                    io_msg.analog_out_states.push_back(ao);
                }

                // robot tool IO
                for (unsigned int i = 0; i < robot_io.toolIOIn.size(); i++)
                {
                    duco_msgs::DucoDigital digo;
                    digo.pin = i;
                    digo.state = robot_io.toolIOIn[i];
                    digo.flag = 1;
                    io_msg.tool_in_states.push_back(digo);
                }

                for (unsigned int i = 0; i < robot_io.toolIOOut.size(); i++)
                {
                    duco_msgs::DucoDigital dogo;
                    dogo.pin = i;
                    dogo.state = robot_io.toolIOOut[i];

                    ROS_INFO("robot_io.toolIOOut[%d] value %d", i, static_cast<int> (dogo.state));

                    dogo.flag = 1;
                    io_msg.tool_out_states.push_back(dogo);
                }

                io_msg.stamp = "The current IO states: " + std::to_string(ros::Time::now().toSec()) + "s";
                robot_io_pub_.publish(io_msg);

            } //IO Msg
        }
        else
        {
            //lianjie buchenggong 
            /** maintain the ros-controller states from the ros environment **/
            //setCurrentPosition(target_point_);      //return back immediately
        }
    }

    bool DucoRobotStatus::connectToRobotController()
    {
        uint connects = 0;

        bool result = false;
        if( duco_robot_->open() != 0)
        {
            ROS_INFO("duco_robot_->open!=0");
            while ( connects <5&&duco_robot_->open() != 0 )
            {
                duco_robot_->open();
                connects++;

                #ifdef _WIN32
                        Sleep(1000);
                #else
                        usleep(10);
                #endif    
            }

            if( connects<5 )
            {
                controller_connected_flag_ = true;
                result = true;
            }
            else
            {
                result = false;
                controller_connected_flag_ = false;
            }
        }
        else
        {
            ROS_INFO("duco_robot_->open==0");
            result = true;
            controller_connected_flag_ = true;
        }

        return result;
    }

    void DucoRobotStatus::run()
    {
        ROS_INFO("Start the Robot Status!");
        /** connect to the robot controller **/
        if ( !controller_connected_flag_  )
        {
            if( connectToRobotController() )
            {
                /** Switches to ros-controller **/
                    ROS_INFO("Switches to ros-controller successfully");
            }
            else
            {
                    ROS_WARN("Failed to switch to ros-controller, the robot is still controlled by the robot controller!");
            }
        }

        if( controller_connected_flag_  )
        {
            std_msgs::UInt8 aa;
            aa.data = 1;
            connect_pub_.publish(aa);
        }
        else
        {
            std_msgs::UInt8 aa;
            aa.data = 0;
            connect_pub_.publish(aa);
        }

        getRobotStatus();

        ROS_INFO("run leave ");
    }
      
}

