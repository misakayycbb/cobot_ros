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

#include "duco_driver/DucoRobotInterface.h"
#include "duco_msgs/DucoCommand.h"
#include "duco_msgs/DucoIOStates.h"
#include "duco_msgs/DucoRobotStates.h"


namespace duco_driver
{
DucoRobotInterface::DucoRobotInterface()
{
    io_status_srv_ = nh_.serviceClient<duco_msgs::DucoIOStates>("/duco_cobot/io_states_srv");
    robot_status_srv_ = nh_.serviceClient<duco_msgs::DucoRobotStates>("/duco_cobot/robot_states_srv");
    robot_command_srv_ = nh_.serviceClient<duco_msgs::DucoCommand>("/duco_cobot/duco_command_srv");
}
bool DucoRobotInterface::powerOn()
{
    duco_msgs::DucoCommand srv;
    srv.request.commandType = CommandType::kSystemCommand;
    srv.request.operateType = SystemOper::kSystemPowerOn;
    return robot_command_srv_.call(srv);
}

bool DucoRobotInterface::powerOff()
{
    duco_msgs::DucoCommand srv;
    srv.request.commandType = CommandType::kSystemCommand;
    srv.request.operateType = SystemOper::kSystemPowerOff;
    return robot_command_srv_.call(srv);

}

bool DucoRobotInterface::enable()
{
    duco_msgs::DucoCommand srv;
    srv.request.commandType = CommandType::kSystemCommand;
    srv.request.operateType = SystemOper::kSystemEnable;
    return robot_command_srv_.call(srv);
}

bool DucoRobotInterface::disable()
{
    duco_msgs::DucoCommand srv;
    srv.request.commandType = CommandType::kSystemCommand;
    srv.request.operateType = SystemOper::kSystemDisable;
    return robot_command_srv_.call(srv);

}

bool DucoRobotInterface::collisionControl(bool col_on,int sensitivity)
{
    duco_msgs::DucoCommand srv;
    srv.request.commandType = CommandType::kCollisionCommand;
    srv.request.operateType = 0;
    srv.request.col_on = col_on;
    srv.request.sensitivity = sensitivity;
    return robot_command_srv_.call(srv);

}

bool DucoRobotInterface::pause()
{
    duco_msgs::DucoCommand srv;
    srv.request.commandType = CommandType::kMotionCommand;
    srv.request.operateType = MotionOper::kMotionPause;
    return robot_command_srv_.call(srv);
}

bool DucoRobotInterface::resume()
{
    duco_msgs::DucoCommand srv;
    srv.request.commandType = CommandType::kMotionCommand;
    srv.request.operateType = MotionOper::kMotionResume;
    return robot_command_srv_.call(srv);
}

bool DucoRobotInterface::stop()
{
    duco_msgs::DucoCommand srv;
    srv.request.commandType = CommandType::kMotionCommand;
    srv.request.operateType = MotionOper::kMotionStop;
    return robot_command_srv_.call(srv);
}

bool DucoRobotInterface::runProgram(std::string name)
{
    duco_msgs::DucoCommand srv;
    srv.request.commandType = CommandType::kProgramCommand;
    srv.request.operateType = 0;
    srv.request.program = name;
    return robot_command_srv_.call(srv);
}

bool DucoRobotInterface::script(std::string script)
{
    duco_msgs::DucoCommand srv;
    srv.request.commandType = CommandType::kScriptCommad;
    srv.request.operateType = 0;
    srv.request.script = script;
    return robot_command_srv_.call(srv);

}

bool DucoRobotInterface::getRobotStatus(duco_msgs::DucoRobotState &status)
{
    duco_msgs::DucoRobotStates srv;
    if(!robot_command_srv_.call(srv))
        return false;
    status=srv.response.robot_states;
    return true;
}

bool DucoRobotInterface::ioOut(IOType type, int  portnum, bool  val)
{
    duco_msgs::DucoIOStates srv;
    srv.request.ioOper = IOOper::kSetIO;
    srv.request.type = type;
    srv.request.port = portnum;
    srv.request.value = val;
    return io_status_srv_.call(srv);

}

bool DucoRobotInterface::ioIn(IOType type, int portnum,bool &value)
{
    duco_msgs::DucoIOStates srv;
    srv.request.ioOper = kGetIO;
    srv.request.type = type;
    srv.request.port = portnum;
    if(!io_status_srv_.call(srv))
        return false;
    value=srv.response.result;
    ROS_INFO("DucoRobotInterface::ioIn value ");
    return true;

}

}
