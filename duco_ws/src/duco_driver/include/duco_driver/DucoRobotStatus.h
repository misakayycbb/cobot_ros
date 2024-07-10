#ifndef DUCOROBOTSTATUS_H
#define DUCOROBOTSTATUS_H

//#include "DucoCobot.h"

#include <ros/ros.h>
#include <std_msgs/Int32MultiArray.h>
#include <industrial_msgs/RobotStatus.h>
#include <duco_msgs/DucoRobotState.h>
#include <duco_msgs/DucoIOState.h>

namespace DucoRPC
{
    class DucoCobot;
};

namespace duco_driver
{
    class DucoRobotStatus
    {
        const unsigned int TIMER_SPAN_ = 50;
    public :
        DucoRobotStatus();

        ~DucoRobotStatus();

        /*运行*/
        void        run();
        
    protected:
        /*连接机器人*/
        bool connectToRobotController();

        /*定时器信号，用于反馈机器人当前状态*/
        void getRobotStatus();

    private:  

        /*机器人IP地址*/
        std::string                             server_host_;

        /*定时器操作指针*/
        ros::Timer timer_;

        /*机器人当前位置，用于失去连接备份*/
        std::vector<double>  current_joints_;

          /*RPC连接标志*/
        bool                       controller_connected_flag_;
        
        /*机器人开始运动标志*/
        bool                       start_move_;

        /*机器人急停标志*/
        bool                        emergency_stopped_;

        /*机器人轴名称*/
        std::vector<std::string>  joint_names_;
        
        /*机器人轴数量*/
        unsigned int                       axis_number_;

         /* /duco_driver/rib_status 机器人状态发布节点 */
        ros::Publisher  rib_pub_;

        /*用于发布给MoveGroup机器人robot_status 机器人状态*/
        ros::Publisher robot_status_pub_;

        /*用于发布给MoveGroup机器人关节状态*/
        ros::Publisher  joint_states_pub_;

        /*用于发布给MoveGroup机器人关节数据*/
        ros::Publisher  joint_feedback_pub_;

        /*/duco_driver/io_states 机器人控制IO状态 发布节点*/
        ros::Publisher  connect_pub_;

        /*用于发布给客户端机器人状态*/
        ros::Publisher  robot_state_pub_;
        
        /*用于发布给客户端机器人IO状态*/
        ros::Publisher  robot_io_pub_;

        /*rib_status_*/
        std_msgs::Int32MultiArray rib_status_;

        /*机器人状态*/
        industrial_msgs::RobotStatus robot_status_;

         /*用于反馈给客户端机器人数据*/
        duco_msgs::DucoRobotState    robot_state_;

         /*用于反馈给客户端机器人IO数据*/
        duco_msgs::DucoIOState            io_state;

        /*ROS 节点句柄*/
        ros::NodeHandle nh_;

        /*机器人RPC接口*/
        DucoRPC::DucoCobot       *duco_robot_ = nullptr;
    };

}

#endif 