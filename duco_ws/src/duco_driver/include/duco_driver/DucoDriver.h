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
#ifndef DUCODRIVER_H
#define DUCODRIVER_H

#include <string>
#include <sys/timeb.h>
#include <queue>
#include <mutex>
#include <vector>

#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int32.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32MultiArray.h>

#include <duco_msgs/DucoIOState.h>
#include <duco_msgs/DucoIOStates.h>
#include <duco_msgs/DucoIOStatesRequest.h>
#include <duco_msgs/DucoIOStatesResponse.h>

#include <duco_msgs/DucoCommand.h>
#include <duco_msgs/DucoCommandRequest.h>
#include <duco_msgs/DucoCommandResponse.h>

#include <duco_msgs/DucoRobotState.h>
#include <duco_msgs/DucoRobotStates.h>
#include <duco_msgs/DucoRobotStatesRequest.h>
#include <duco_msgs/DucoRobotStatesResponse.h>

#include <duco_msgs/DucoGetFK.h>
#include <duco_msgs/DucoGetFKRequest.h>
#include <duco_msgs/DucoGetFKResponse.h>
#include <duco_msgs/DucoGetIK.h>
#include <duco_msgs/DucoGetIKRequest.h>
#include <duco_msgs/DucoGetIKResponse.h>

#include <duco_msgs/DucoDigital.h>
#include <duco_msgs/DucoAnalog.h>

#include <industrial_msgs/RobotStatus.h>
#include "sensor_msgs/JointState.h"
#include <control_msgs/FollowJointTrajectoryFeedback.h>
#include <trajectory_msgs/JointTrajectory.h>
#include "duco_driver/DucoCobot.h"

namespace DucoRPC
{
    class DucoCobot;
};


namespace duco_driver
{
    class DucoDriver
    {
        /*定时器频率*/
        const int TIMER_SPAN_ = 50;
        /*关节比较阈值*/
        const double THRESHHOLD = 0.000001;

        public:
            
            /*构造函数*/
            DucoDriver();
            
            /*析构函数*/
            ~DucoDriver();

            /*更新机器人控制状态，比如机器人是否启动运行*/
            void updateControlStatus();

        protected:

            /*RPC连接服务*/
            bool  connectToRobotController();

             /*根据MoveIt下发机器人关节数据*/
            bool setRobotJointsByMoveIt();

        private:
            /*机器人正解函数服务*/
            bool getFK(duco_msgs::DucoGetFKRequest& req, duco_msgs::DucoGetFKResponse& resp);
            
            /*机器人逆解函数服务*/
            bool getIK(duco_msgs::DucoGetIKRequest& req, duco_msgs::DucoGetIKResponse& resp);

             /*获取机器人IO服务*/
            bool ioServices(duco_msgs::DucoIOStates::Request& req,duco_msgs::DucoIOStates::Response &res);

            /*执行外部接口命令服务*/
            bool doRobotCommand(duco_msgs::DucoCommand::Request &req,duco_msgs::DucoCommand::Response &res);

            /*获取当前机器人状态服务*/
            bool getRobotStates(duco_msgs::DucoRobotStates::Request &req,duco_msgs::DucoRobotStates::Response &res);

        private:
            /* 获取机器人当前IO状态 */
            void ioStateCallback(const duco_msgs::DucoIOState::ConstPtr& msg );
            
            /* 获取机器人当前位置状态 */
            void robotStateCallback(const duco_msgs::DucoRobotState::ConstPtr& msg );

            /* 获取来自MoveGroup的机器人状态数据 */
            void robotStatusCallback(const industrial_msgs::RobotStatus::ConstPtr &msg);
            
            /*根据MoveIt规划执行*/
            void moveItPosCallback(const trajectory_msgs::JointTrajectory::ConstPtr &msg);

            /*监控机器人停止信号*/
            void trajectoryExecutionCallback(const std_msgs::String::ConstPtr &msg);

        private://服务节点
            /*ik server*/
            ros::ServiceServer ik_srv_;

            /*fk server*/
            ros::ServiceServer fk_srv_;

            /*io status server*/
            ros::ServiceServer io_status_srv_;

            /*robot status server*/
            ros::ServiceServer duco_states_srv_;

            /*robot command server*/
            ros::ServiceServer duco_command_srv_;

       private://订阅节点
            /*moveAPI_cmd 机器人 通过RPC控制 订阅节点*/
            ros::Subscriber moveAPI_subs_;

            /*moveItController_cmd 机器人 通过Moveit控制 订阅节点*/
            ros::Subscriber moveit_controller_subs_;

            /*trajectory_execution_event 机器人 停止信号 订阅节点*/
            ros::Subscriber trajectory_execution_subs_;

            /*io_states_subs_ 机器人IO 订阅节点*/
            ros::Subscriber io_states_subs_;

            /*robot_states_subs_ 机器人状态 订阅节点*/
            ros::Subscriber robot_states_subs_;

            /*订阅来自MoveGroup的机器人状态*/
            ros::Subscriber robot_status_subs_;

            double                   robot_speed_;

        private:
            /*当前机器人轴的数量*/
            int axis_number_;

            /*互*/
            std::mutex     io_mutex_;

            std::mutex     robot_status_mutex_;

            bool                  joint_receive_finish_ ;

            /*主机IP地址*/
            std::string server_host_;

            /*碰撞等级*/
            int collision_class_;

            /*机器人RPC接口*/
            DucoRPC::DucoCobot       *duco_robot_ = nullptr;

            std::vector<std::vector<double> > joint_track_;

             /*机器人IO状态*/
            duco_msgs::DucoIOState  io_state_;

            /*机器人状态*/
            duco_msgs::DucoRobotState  robot_state_;

            /*机器人状态*/
            industrial_msgs::RobotStatus robot_status_;

             /*机器人急停标志*/
            bool emergency_stopped_;

            /*机器人保护停止标志*/
            bool protective_stopped_;

            /*机器人正常停止规划标志 OTG*/
            bool normal_stopped_;

            /*当前机器人是否连接标志*/
            bool controller_connected_flag_;

            /*当前机器人是否启动运行标志*/
            bool start_move_;

            /*机器人关节名称*/
            std::vector<std::string>     joint_names_;

            /*ROS 节点句柄*/
            ros::NodeHandle nh_;
 
    };
}

#endif /* DUCODRIVER_H */
