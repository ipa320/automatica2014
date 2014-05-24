/*!
 *****************************************************************
 * \file
 *
 * \note
 *   Copyright (c) 2014 \n
 *   Fraunhofer Institute for Manufacturing Engineering
 *   and Automation (IPA) \n\n
 *
 *****************************************************************
 *
 * \note
 *   Project name: care-o-bot
 * \note
 *   ROS stack name: cob_driver
 * \note
 *   ROS package name: cob_phidgets
 *
 * \author
 *   Author: Thiago de Freitas, mail: tdf@ipa.fhg.de
 * \author
 *   Supervised by: Thiago de Freitas, mail: tdf@ipa.fhg.de
 *
 * \date Date of creation: 23.05.2014
 *
 * \brief
 *   this package integrates phidget boards into the ros system
 *
 *****************************************************************
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     - Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer. \n
 *     - Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution. \n
 *     - Neither the name of the Fraunhofer Institute for Manufacturing
 *       Engineering and Automation (IPA) nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission. \n
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License LGPL as
 * published by the Free Software Foundation, either version 3 of the
 * License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License LGPL for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License LGPL along with this program.
 * If not, see <http://www.gnu.org/licenses/>.
 *
 ****************************************************************/

#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "sensor_msgs/Range.h"
#include "cob_srvs/SetString.h"

#include <libphidgets/phidget21.h>
#include <sstream>

std::string led_mode = "off";

class Button
{
	ros::NodeHandle n_;
	ros::Publisher pub_button_;
	int id_, filter_size_;
    bool val_;
	std::string frame_id_;
public:

    Button(const std::string &fr_id, const int id) :
			id_(id), frame_id_(fr_id)
	{
		char buffer[256];
		sprintf(buffer, "button_%d", id);
        pub_button_ = n_.advertise<std_msgs::Bool>(buffer, 0);
	}

	int getId() const
	{
		return id_;
	}

    void update(const bool v)
	{
        val_ = v;
	}

    bool getVal()
    {
        return val_;
    }

    std::string getFrameID()
    {
        return frame_id_;
    }
};

void display_generic_properties(CPhidgetHandle phid)
{
    int sernum, version;
    const char *deviceptr, *label;
    CPhidget_getDeviceType(phid, &deviceptr);
    CPhidget_getSerialNumber(phid, &sernum);
    CPhidget_getDeviceVersion(phid, &version);
    CPhidget_getDeviceLabel(phid, &label);

    ROS_INFO("%s", deviceptr);
    ROS_INFO("Version: %8d SerialNumber: %10d", version, sernum);
    ROS_INFO("Label: %s", label);
    return;
}

int IFK_AttachHandler(CPhidgetHandle IFK, void *userptr)
{
    //CPhidgetInterfaceKit_setSensorChangeTrigger((CPhidgetInterfaceKitHandle)IFK, 0, 0);
    //printf("Attach handler ran!\n");
    return 0;
}

std::vector<Button>* g_buttons;
std::map<std::string, int> buttons_map;

int CCONV InputChangeHandler(CPhidgetInterfaceKitHandle IFK, void *userptr,
        int Index, int State)
{

    led_mode = "off";
    int count = 0;
    
    g_buttons = (std::vector<Button>*) userptr;
    for (size_t i = 0; i < g_buttons->size(); i++)
        if ((*g_buttons)[i].getId() == Index)
        {
            (*g_buttons)[i].update(State);
            if(State==1 && count==0)
                led_mode = "on";
            
        }
        
       

    return 0;
}



bool update_button_state(cob_srvs::SetString::Request  &req,
         cob_srvs::SetString::Response &res)
{

  if(buttons_map.find( req.data ) != buttons_map.end())
  {
    //ROS_INFO("Setting button state to: %d", (*g_buttons)[buttons_map[req.data]].getVal());
    if((*g_buttons)[buttons_map[req.data]].getVal())
    {
      res.errorMessage.data = ("Button ON");
      
     }

    else
      res.errorMessage.data = "Button OFF";
  }
  else
  {
      res.errorMessage.data = ("The requested button does not exist.");
  }

  res.success = true;



  return true;
}




int main(int argc, char **argv)
{
	ros::init(argc, argv, "cob_phidgets");


	ros::NodeHandle nh_("~");
    std::vector<Button> g_buttons;

    if (nh_.hasParam("buttons"))
	{
		XmlRpc::XmlRpcValue v;
        nh_.param("buttons", v, v);
		for (int i = 0; i < v.size(); i++)
		{
			ROS_ASSERT(v[i].size()>=2);

			int id = v[i][0];
			std::string fr_id = v[i][1];


            g_buttons.push_back(Button(fr_id, id));
            buttons_map[fr_id] = i;
		}
	}
	else
	{
        ROS_ERROR("Parameter buttons not set, shutting down node...");
		nh_.shutdown();
		return false;
	}

	ros::Rate loop_rate(10);

	//init and open phidget
    int numInputs, numOutputs, numbuttons;
	int err;

    /* This adds the Server to be called to get the actual state of the buttons */
    ros::ServiceServer service = nh_.advertiseService("button_state",update_button_state);
    ros::ServiceClient client = nh_.serviceClient<cob_srvs::SetString>("led_control/set_mode");
    cob_srvs::SetString setStr;
    

	CPhidgetInterfaceKitHandle IFK = 0;
	CPhidget_enableLogging(PHIDGET_LOG_VERBOSE, NULL);
	CPhidgetInterfaceKit_create(&IFK);
    CPhidgetInterfaceKit_set_OnInputChange_Handler(IFK,
            InputChangeHandler, (void*) &g_buttons);
	CPhidget_set_OnAttach_Handler((CPhidgetHandle) IFK, IFK_AttachHandler,
			NULL);
	//opening phidget
	CPhidget_open((CPhidgetHandle) IFK, -1);

	std::string prev_led_mode;
	//wait 5 seconds for attachment
	ROS_INFO("waiting for phidgets attachement...");
	if ((err = CPhidget_waitForAttachment((CPhidgetHandle) IFK, 10000))
			!= EPHIDGET_OK)
	{
		const char *errStr;
		CPhidget_getErrorDescription(err, &errStr);
		ROS_ERROR("Error waiting for attachment: (%d): %s", err, errStr);
		goto exit;
	}
	ROS_INFO("... attached");

	display_generic_properties((CPhidgetHandle) IFK);
	CPhidgetInterfaceKit_getOutputCount((CPhidgetInterfaceKitHandle) IFK,
			&numOutputs);
	CPhidgetInterfaceKit_getInputCount((CPhidgetInterfaceKitHandle) IFK,
			&numInputs);
	CPhidgetInterfaceKit_getSensorCount((CPhidgetInterfaceKitHandle) IFK,
            &numbuttons);
	//CPhidgetInterfaceKit_setOutputState((CPhidgetInterfaceKitHandle)IFK, 0, 1);
	ROS_INFO(
            "buttons:%d Inputs:%d Outputs:%d", numbuttons, numInputs, numOutputs);

	while (ros::ok())
    {
        ros::spin();
		loop_rate.sleep();
		/*
        setStr.request.data = led_mode;

        if(led_mode != prev_led_mode)
        {
            if (client.call(setStr))
            {
                std::string error_msg = setStr.response.errorMessage.data;
                ROS_INFO("Message to Button: %s", error_msg.c_str());
            }
            else
            {
                ROS_ERROR("Failed to call service button_state");
                return 1;
            }
        }

        prev_led_mode = led_mode;
        */
	}

	exit: CPhidget_close((CPhidgetHandle) IFK);
	CPhidget_delete((CPhidgetHandle) IFK);

	return 0;
}
