/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2017-2018, SIASUN Robotics
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *       * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *       * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *       * Neither the name of the Southwest Research Institute, nor the names
 *       of its contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "duco_driver/DucoDriver.h"
#include "duco_driver/DucoCobot.h"
#include "duco_driver/DucoRobotInterface.h"

namespace duco_driver {

using namespace DucoRPC;
const unsigned int  IOMAXNUM = 16;
const unsigned int  TOOLIOMAXNUM = 2;


DucoDriver::DucoDriver( )
    :emergency_stopped_(false)
    ,protective_stopped_(false)
    ,normal_stopped_(false)
    ,controller_connected_flag_(false)
    ,start_move_(false)
    ,joint_receive_finish_(false)
    ,collision_class_(6)
{

    joint_names_ .push_back("joint_1");
    joint_names_ .push_back("joint_2");
    joint_names_ .push_back("joint_3");
    joint_names_ .push_back("joint_4");
    joint_names_ .push_back("joint_5");
    joint_names_ .push_back("joint_6");

    std::string speedStr;
    ros::param::get("/duco_driver/speed",  speedStr);

    if( speedStr=="" )
    {
        robot_speed_ = 0.5;
        ROS_INFO("robot_speed_1 %.6f",robot_speed_);
    }
    else
    {
         robot_speed_ = stod(speedStr);
        ROS_INFO("robot_speed_1 %.6f",robot_speed_);

        if(  robot_speed_< 1e-10 ||  robot_speed_ > 1 )
        {
            robot_speed_ = 0.5;
        }
    }
   
    ROS_INFO("robot_speed_2 %.6f",robot_speed_);

    std::vector<std::string> jointNames;
    ros::param::get("controller_joint_names",jointNames);

    std::remove(jointNames.begin(), jointNames.end(), std::string());

    if(  jointNames.size()>0 )
    {
        for (size_t i = 0; i < jointNames.size(); i++)
        {
            joint_names_[i] = jointNames[i];
             ROS_INFO("joint_names_ %s ",joint_names_ [i].c_str() );
        }
    }

    std::string s;
    ros::param::get("/duco_driver/server_host", s); //The server_host should be corresponding to the robot controller setup.
    server_host_ = (s=="")? "127.0.0.1" : s;
    duco_robot_ = new DucoRPC::DucoCobot(server_host_, 7003);
    duco_robot_->open();


    ik_srv_ = nh_.advertiseService("/duco_cobot/get_ik",&DucoDriver::getIK, this);
    fk_srv_ = nh_.advertiseService("/duco_cobot/get_fk",&DucoDriver::getFK, this);
    io_status_srv_= nh_.advertiseService("/duco_cobot/io_states_srv",&DucoDriver::ioServices,this);
    duco_command_srv_= nh_.advertiseService("/duco_cobot/duco_command_srv",&DucoDriver::doRobotCommand,this);
    duco_states_srv_= nh_.advertiseService("/duco_cobot/robot_states_srv",&DucoDriver::getRobotStates,this);

    /** subscribe topics **/
     /*订阅来自DucoRobotStatus的消息*/
    io_states_subs_ = nh_.subscribe("/duco_cobot/io_state", 10, &DucoDriver::ioStateCallback, this);
    robot_states_subs_ = nh_.subscribe("/duco_cobot/robot_state", 10, &DucoDriver::robotStateCallback, this);

    /*订阅来自MoveGroup的消息*/
    robot_status_subs_ = nh_.subscribe("robot_status", 10, &DucoDriver::robotStatusCallback, this);
    trajectory_execution_subs_ = nh_.subscribe("trajectory_execution_event", 10, &DucoDriver::trajectoryExecutionCallback,this);
    moveit_controller_subs_ = nh_.subscribe("joint_path_command", 2000, &DucoDriver::moveItPosCallback,this);


}

DucoDriver::~DucoDriver()
{
    duco_robot_->close();
}

bool DucoDriver::connectToRobotController()
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
            ROS_INFO("duco_robot_->open==0" );
            result = true;
            controller_connected_flag_ = true;
        }

        
        if(  controller_connected_flag_ )
        {
            ROS_INFO("controller_connected_flag_");

             duco_robot_->power_on(true);

            usleep(1000000);

             duco_robot_->enable( true );

             usleep(1000000);

            ROS_INFO("get actual joints position");

            std::vector<double> robot_joint_value;
            duco_robot_->get_actual_joints_position(robot_joint_value);

            double v = 0.1;
            double a = 1;
            double  r = 0;
            double  block = true;

            duco_robot_->movej2(robot_joint_value, v, a, r, block);
        }

        return result;
}

bool DucoDriver::ioServices(duco_msgs::DucoIOStates::Request& req,duco_msgs::DucoIOStates::Response &res)
{
    //general IO
    std::lock_guard<std::mutex> lock( io_mutex_ );
    if( IOOper::kGetIO==req.ioOper )
    {
        if( kStandardDI == req.type )
        {
            ROS_INFO("DucoDriver::ioServices kGetIO kStandardDI req.port  %d", static_cast<int>(req.port) );
            if( req.port>IOMAXNUM || req.port<1 )
                return false;
            ROS_INFO("DucoDriver::ioServices kGetIO kStandardDI value %d",  static_cast<int> (io_state_.digital_in_states[req.port-1] .state)  );
            res.result = io_state_.digital_in_states[req.port-1] .state;
            return true;
        }
        else if( kStandardDO == req.type )
        {
            ROS_INFO("DucoDriver::ioServices kGetIO kStandardDO req.port %d", static_cast<int>(req.port) );
            if( req.port>IOMAXNUM || req.port<1 )
                return false;
            ROS_INFO("DucoDriver::ioServices kGetIO kStandardDO value %d", static_cast<int>(req.port) );
            res.result = io_state_.digital_out_states[req.port-1] .state;
            return true;
        }
        else if( kToolDI == req.type ) 
        {
            ROS_INFO("DucoDriver::ioServices kGetIO kToolDI req.port %d", static_cast<int>(req.port)) ;
            if( req.port>TOOLIOMAXNUM || req.port<1 )
                return false;

            ROS_INFO("DucoDriver::ioServices kGetIO kToolDI  value %d", static_cast<int> (io_state_.tool_in_states[req.port-1] .state)) ;
            res.result = io_state_.tool_in_states[req.port-1] .state;
            return true;
        }
        else if( kToolDO == req.type )
        {
            ROS_INFO("DucoDriver::ioServices kGetIO kToolDO req.port %d", static_cast<int>(req.port)) ;
            if( req.port>IOMAXNUM || req.port<1 )
                return false;
            ROS_INFO("DucoDriver::ioServices kGetIO kToolDO  req.port value %d", static_cast<int> (io_state_.tool_out_states[req.port-1] .state) ) ;
            res.result = io_state_.tool_out_states[req.port-1] .state;
            return true;
        }
    }
    else if(  IOOper::kSetIO==req.ioOper )
    {
        if( kStandardDO == req.type )
        {
            ROS_INFO("DucoDriver::ioServices kSetIO kStandardDO req.port  %d", static_cast<int>(req.port)) ;
            if( req.port>IOMAXNUM || req.port<1 )
                return false;

            ROS_INFO("DucoDriver::ioServices kSetIO kStandardDO value %d", static_cast<int>(req.value)) ;    
            int result = duco_robot_->set_standard_digital_out(req.port, req.value, true);
            if( result == 0 )
            {
                res.result = true;
            }
            else
            {
               res.result = false;
            }
            return true;
        }
        else if( kToolDO == req.type )
        {
            ROS_INFO("DucoDriver::ioServices kSetIO kToolDO req.port  %d", static_cast<int>(req.port)) ;
            if( req.port>IOMAXNUM || req.port<1 )
                return false;
            ROS_INFO("DucoDriver::ioServices kSetIO kToolDO value %d", static_cast<int>(req.value)) ;    
            int result = duco_robot_->set_tool_digital_out(req.port, req.value, true);
            if( result == 0 )
            {
                res.result = true;
            }
            else
            {
               res.result = false;
            }
            return true;
        }
    }
}

bool DucoDriver::doRobotCommand(duco_msgs::DucoCommand::Request &req,duco_msgs::DucoCommand::Response &res)
{
    if( controller_connected_flag_ )
    {
        switch (req.commandType) 
        {
            case kSystemCommand:
            {
                switch (req.operateType)
                {
                    case kSystemPowerOn: //POWER ON
                    {
                        res.state = duco_robot_->power_on(true);
                        break;
                    }
                    case kSystemEnable: //ENABLE
                    {
                        res.state = duco_robot_->enable(true);
                        break;
                    }
                    case kSystemDisable: //DISABLE
                    {
                        res.state = duco_robot_->disable(true);
                        break;
                    }
                    case kSystemPowerOff: //POWER OFF
                    {
                        res.state = duco_robot_->power_off(true);
                        break;
                    }
                }
                break;
            }
            case kMotionCommand:
            {
                switch (req.operateType) 
                {
                    case kMotionPause:
                    {
                        res.state = duco_robot_->pause(true);
                        break;
                    }
                    case kMotionStop:
                    {
                        res.state = duco_robot_->stop(true);
                        break;
                    }
                    case kMotionResume:
                    {
                        res.state = duco_robot_->resume(true);
                        break;
                    }
                }
                break;
            }
            case kCollisionCommand:
            {
                switch (req.operateType)
                {
                    //duco_robot_->collision_detect
                }
                break;
            }
            case kProgramCommand:
            {
                res.state = duco_robot_->run_program(req.program, true);
                break;
            }
            case kScriptCommad:
            {
                res.state = duco_robot_->run_program(req.script, true);
                break;
            }
        }
    }
    usleep(500000);
    return true;
}

bool DucoDriver::getRobotStates(duco_msgs::DucoRobotStates::Request &req,duco_msgs::DucoRobotStates::Response &res)
{
    res.robot_states = robot_state_;
    return true;
}

bool DucoDriver::setRobotJointsByMoveIt()
{
    bool ret = true;

    // First check if the buf_queue_ is Empty
    if(!joint_track_.empty()&& joint_receive_finish_ )
    {   
        joint_receive_finish_ = false;
        if(  2==joint_track_.size() )
        {
            std::vector<double> joint2;
            for (size_t i = 0; i < 6; i++)
            {
                double tmp = (joint_track_[0][i] + joint_track_[1][i])/2; 
                joint2.push_back(tmp);
            }
            std::vector<double>joint1 = joint_track_[0];
            std::vector<double>joint3 = joint_track_[1];

            joint_track_.clear();
            joint_track_.push_back(joint1);
            joint_track_.push_back(joint2);
            joint_track_.push_back(joint3);
        }
     
        if( controller_connected_flag_ )      // actually no need this judgment
        {
                duco_robot_->trackEnqueue( joint_track_, true);
                if( !start_move_ )
                {
                    ROS_INFO("Start Move robot_speed_ %.6f", robot_speed_);    
                    duco_robot_->trackJointMotion(robot_speed_, 10,false);
                }
                joint_track_.clear();
        }
    }
    return ret;
}


void DucoDriver::moveItPosCallback(const trajectory_msgs::JointTrajectory::ConstPtr &msg)
{
    if(msg->points.size() == 0 )
    {
        ROS_INFO("DucoDriver::moveItPosCallback msg is empty");    
    }
    else
    {
        if( controller_connected_flag_ )
        {
            if( start_move_ )
            {
                ROS_INFO("DucoDriver::moveItPosCallback the robot is moving just wait");    
                usleep( 4000 );
                if( start_move_  )
                {
                    ROS_INFO("DucoDriver::moveItPosCallback the robot is still move");  
                    for(int i = 0; i < msg->points.size(); i++)
                    {
                        // first remaps point to controller joint order, the add the point to the controller.
                        trajectory_msgs::JointTrajectoryPoint point = msg->points[i];
                        std::vector<double> joints;
                        for(int j = 0; j < 6; j++)
                        {
                            for(int k = 0; k < 6; k++)
                            {
                                if(joint_names_[j] == msg->joint_names[k])
                                {
                                    joints.push_back(msg->points[i].positions[k]);
                                    break;
                                }
                            }
                        }
                        ROS_INFO("DucoDriver::moving joints i %d %.6f %.6f %.6f  %.6f %.6f %.6f ", i,  joints[0], joints[1], joints[2], joints[3], joints[4], joints[5]);  
                        joints.clear();
                        return ;
                    }
                }
            }
        }

        ROS_INFO("DucoDriver::moveItPosCallback size %d", static_cast<int>(msg->points.size()) );
        for(int i = 0; i < msg->points.size(); i++)
        {
            // first remaps point to controller joint order, the add the point to the controller.
            trajectory_msgs::JointTrajectoryPoint point = msg->points[i];
            std::vector<double> joints;
            for(int j = 0; j < 6; j++)
            {
                for(int k = 0; k < 6; k++)
                {
                    if(joint_names_[j] == msg->joint_names[k])
                    {
                        joints.push_back(msg->points[i].positions[k]);
                        break;
                    }
                }
            }

             ROS_INFO("DucoDriver::moveItPosCallback joints i %d %.6f %.6f %.6f  %.6f %.6f %.6f ", i,  joints[0], joints[1], joints[2], joints[3], joints[4], joints[5]);  
            joint_track_.push_back(joints) ;
        }
        joint_receive_finish_ = true;
    }
}

void DucoDriver::ioStateCallback(const duco_msgs::DucoIOState::ConstPtr& msg )
{
    std::lock_guard<std::mutex> lock( io_mutex_ );

#if 0
    io_state_.digital_in_states.clear();
    for (unsigned int i = 0; i < msg->digital_in_states.size(); i++)
    {
        duco_msgs::DucoDigital digo;
        digo = msg->digital_in_states[i];
         io_state_.digital_in_states.push_back(digo);
    }

    io_state_.digital_out_states.clear();
    for (unsigned int i = 0; i < msg->digital_out_states.size(); i++)
    {
        duco_msgs::DucoDigital dogo;
        dogo = msg->digital_out_states[i];
        io_state_.digital_out_states.push_back(dogo);
        //std::cout <<"DucoDriver::ioStateCallback digital_out_states [" << i <<"] value " << static_cast<int>(dogo.state)  << std::endl;
    }

    io_state_.analog_in_states.clear();
    for (unsigned int i = 0; i < msg->analog_in_states.size(); i++)
    {
        duco_msgs::DucoAnalog ai;
        ai = msg->analog_in_states[i];
        io_state_.analog_in_states.push_back(ai);
    }

    io_state_.analog_out_states.clear();
    for (unsigned int i = 0; i < msg->analog_out_states.size(); i++)
    {
        duco_msgs::DucoAnalog ao;
        ao = msg->analog_out_states[i];
        io_state_.analog_out_states.push_back(ao);
    }

    // robot tool IO
    io_state_.tool_in_states.clear();
    for (unsigned int i = 0; i < msg->tool_in_states.size(); i++)
    {
        duco_msgs::DucoDigital di;
        di = msg->tool_in_states[i];
        io_state_.tool_in_states.push_back(di);
    }

    io_state_.tool_out_states.clear();
    for (unsigned int i = 0; i < msg->tool_out_states.size(); i++)
    {
        duco_msgs::DucoDigital dogo;
        dogo = msg->tool_out_states[i];
        io_state_.tool_out_states.push_back(dogo);
        //std::cout <<"DucoDriver::ioStateCallback tool_out_states [" << i <<"] value " << static_cast<int>(dogo.state) << std::endl;
    }
#endif
    io_state_ = *msg;
}

void DucoDriver::robotStateCallback(const duco_msgs::DucoRobotState::ConstPtr& msg )
{
    std::lock_guard<std::mutex> lock( robot_status_mutex_ );
    robot_state_ = *msg;
}

void DucoDriver::robotStatusCallback(const industrial_msgs::RobotStatus::ConstPtr &msg)
{
     robot_status_ = *msg;
     start_move_ = robot_status_.in_motion.val;
     emergency_stopped_ = robot_status_.e_stopped.val;
}

void DucoDriver::trajectoryExecutionCallback(const std_msgs::String::ConstPtr &msg)
{
    if(msg->data == "stop")
    {
        ROS_INFO("trajectory execution status: stop");
        normal_stopped_ = true;
    }
}

void DucoDriver::updateControlStatus()
{
    if( !controller_connected_flag_ )
    {
        connectToRobotController();
    }

    if( controller_connected_flag_ )
    {
        setRobotJointsByMoveIt();
    }

    if( emergency_stopped_ )
    {
        ROS_INFO("DucoDriver::updateControlStatus emergency_stopped_  %d" ,  emergency_stopped_);
        //clear the buffer, there will be a jerk
        joint_track_.clear();
    }
    else if( protective_stopped_ || normal_stopped_ )
    {
        ROS_INFO("DucoDriver::updateControlStatus emergency_stopped_  %d" ,  normal_stopped_);

        if( controller_connected_flag_ )
        {
            duco_robot_->stop(true);
        }

        joint_track_.clear();
        //clear the flag
        if(normal_stopped_)
        {
            normal_stopped_ = false;
            protective_stopped_ = false;
        }
    }
}

bool DucoDriver::getFK(duco_msgs::DucoGetFKRequest& req, duco_msgs::DucoGetFKResponse& resp)
{
    resp.success = true;

    std::vector<double> cartesian;
    std::vector<double>     joint;
    std::vector<double>      tool;
    std::vector<double>      wobj;

    joint.push_back(req.joint[0]);
    joint.push_back(req.joint[1]);
    joint.push_back(req.joint[2]);
    joint.push_back(req.joint[3]);
    joint.push_back(req.joint[4]);
    joint.push_back(req.joint[5]);

    if( duco_robot_ )
    {
        duco_robot_->get_tcp_pose(tool);
        duco_robot_->get_wobj(wobj);
        duco_robot_->cal_fkine(cartesian, joint, tool, wobj);
    }

    resp.pos.push_back(cartesian[0]);
    resp.pos.push_back(cartesian[1]);
    resp.pos.push_back(cartesian[2]);
    resp.rpy.push_back(cartesian[3]);
    resp.rpy.push_back(cartesian[4]);
    resp.rpy.push_back(cartesian[5]);

    return resp.success;
}

bool DucoDriver::getIK(duco_msgs::DucoGetIKRequest& req, duco_msgs::DucoGetIKResponse& resp)
{
    resp.success = false;

    std::vector<double>     joint_ref;
    joint_ref.push_back(req.ref_joint[0]);
    joint_ref.push_back(req.ref_joint[1]);
    joint_ref.push_back(req.ref_joint[2]);
    joint_ref.push_back(req.ref_joint[3]);
    joint_ref.push_back(req.ref_joint[4]);
    joint_ref.push_back(req.ref_joint[5]);

    std::vector<double>      tool;
    std::vector<double>      wobj;

    if( duco_robot_ )
    {
        duco_robot_->get_tcp_pose(tool);
        duco_robot_->get_wobj(wobj);
    }


    std::vector<double>      cartesian;
    cartesian.push_back(req.pos[0]);
    cartesian.push_back(req.pos[1]);
    cartesian.push_back(req.pos[2]);
    cartesian.push_back(req.rpy[0]);
    cartesian.push_back(req.rpy[1]);
    cartesian.push_back(req.rpy[2]);

    std::vector<double>     joint_result;

    if( duco_robot_ )
        duco_robot_->cal_ikine(joint_result, cartesian, joint_ref, tool, wobj);

    resp.joint.push_back(joint_result[0]);
    resp.joint.push_back(joint_result[1]);
    resp.joint.push_back(joint_result[2]);
    resp.joint.push_back(joint_result[3]);
    resp.joint.push_back(joint_result[4]);
    resp.joint.push_back(joint_result[5]);

    resp.success = true;

    return resp.success;
}

}

