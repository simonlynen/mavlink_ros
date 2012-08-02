/*
 * params.h
 *
 *  Created on: Aug 2, 2012
 *      Author: slynen
 */

#ifndef PARAMS_H_
#define PARAMS_H_

#include <string.h>
#include <ros/ros.h>
#include <vector>
#include <boost/algorithm/string.hpp>
#include <iostream>
#include <fstream>

namespace mavlink_ros{

class FixParams
{
public:
	bool send_raw_mavlink;
	

	FixParams(){
		readParams();
	}

	void readParams(){

		ros::NodeHandle nh("~");
		nh.getParam("send_raw_mavlink", send_raw_mavlink);
	}

};

}

#endif /* PARAMS_H_ */

