/*
* Software License Agreement (BSD License)
*
* Copyright (c) 2011, Southwest Research Institute
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* 	* Redistributions of source code must retain the above copyright
* 	notice, this list of conditions and the following disclaimer.
* 	* Redistributions in binary form must reproduce the above copyright
* 	notice, this list of conditions and the following disclaimer in the
* 	documentation and/or other materials provided with the distribution.
* 	* Neither the name of the Southwest Research Institute, nor the names
*	of its contributors may be used to endorse or promote products derived
*	from this software without specific prior written permission.
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

#ifndef DUCOROBOTINTERFACE_H
#define DUCOROBOTINTERFACE_H

#include "ros/ros.h"
#include "duco_msgs/DucoRobotState.h"

#include <string>
/**
 * \brief User interface for duco robot．
 * User can use this interface to control the robot in ROS.
 * The premise is that duco_driver (server) should be opened
 *
 */
namespace duco_driver
{
/*一些操作定义*/
typedef enum
{
    kSystemCommand = 0,
    kMotionCommand,
    kCollisionCommand,
    kProgramCommand,
    kScriptCommad
}CommandType;

typedef enum
{
    kSystemPowerOn = 0,
    kSystemEnable,
    kSystemDisable,
    kSystemPowerOff
}SystemOper;

typedef enum
{
    kMotionStop = 0,
    kMotionPause,
    kMotionResume
}MotionOper;

typedef enum
{
    kSetIO = 0,
    kGetIO
}IOOper;

typedef enum
{
    kStandardDI = 0,
    kStandardDO,
    kToolDI,
    kToolDO
}IOType;

class DucoRobotInterface
{
public:

    /**
   * \brief Constructor
   */
    DucoRobotInterface();
    
    /**
   * \brief Destructor
   */
    virtual ~DucoRobotInterface(){}

    /**
   * \brief  robot Power On
   * \return true on success, false otherwise
   */
    bool powerOn();


    /**
   * \brief robot Power Off
   * \return true on success, false otherwise
   */
    bool powerOff();

    /**
   * \brief enable robot control
   * \return true on success, false otherwise
   */
    bool enable();

    /**
   * \brief Disable robot control
   * \return true on success, false otherwise
   */
    bool disable();

    /**
   * \brief Pause the robot
   *
   * \return true on success, false otherwise
   */
    bool pause();

    /**
   * \brief Resume the robot
   *
   * \return true on success, false otherwise
   */
    bool resume();

    /**
   * \brief Stop the robot
   *
   * \return true on success, false otherwise
   */
    bool stop();
    /**
   * \brief Robot collision detection config
   *
   * \param col_on true:on false:off
   * \param sensitivity collision detection sensitivity(1~100)
   * \return true on success, false otherwise
   */
    bool collisionControl(bool col_on,int sensitivity);

    /**
   * \brief Run an existing program　in the robot controller
   *
   * \param name the name of the program(example:demo.jspf)
   * \return true on success, false otherwise
   */
    bool runProgram(std::string name);

    /**
   * \brief run a script
   * \param  script the name of the script(example:demo.script)
   * \return true on success, false otherwise
   */
    bool script(std::string script);

    /**
   * \brief Get robot status
   * \param status robot status(reference　duco_msgs)
   * \return true on success, false otherwise
   */
    bool getRobotStatus(duco_msgs::DucoRobotState &state);

    /**
   * \brief Get control cabinet io input
   *
   * \param portnum port number 1~8
   * \param val 0/1
   * \return true on success, false otherwise
   */
    bool ioIn(IOType type, int portnum,bool &value);

    /**
   * \brief set control cabinet io output
   *
   * \param portnum port number 1~8
   * \param val 0/1
   * \return true on success, false otherwise
   */
    bool ioOut(IOType type, int  portnum, bool val);

private:
    //Can not be assigned and copied
    DucoRobotInterface(const DucoRobotInterface &){}
    DucoRobotInterface& operator=(const DucoRobotInterface & ){}
    ros::NodeHandle nh_;

    //io client(match robot_driver server)
    ros::ServiceClient io_status_srv_;

    //robot_status client(match robot_driver server)
    ros::ServiceClient robot_status_srv_;

    //robot command client(match robot_driver server)
    ros::ServiceClient robot_command_srv_;

};


} //DucoRobotInterface




#endif
