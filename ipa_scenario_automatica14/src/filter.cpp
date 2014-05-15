/*!
 *****************************************************************
 * \file
 *
 * \note
 *   Copyright (c) 2012 \n
 *   Fraunhofer Institute for Manufacturing Engineering
 *   and Automation (IPA) \n\n
 *
 *****************************************************************
 *
 * \note
 *  Project name: TODO FILL IN PROJECT NAME HERE
 * \note
 *  ROS stack name: TODO FILL IN STACK NAME HERE
 * \note
 *  ROS package name: TODO FILL IN PACKAGE NAME HERE
 *
 * \author
 *  Author: TODO FILL IN AUTHOR NAME HERE
 * \author
 *  Supervised by: TODO FILL IN CO-AUTHOR NAME(S) HERE
 *
 * \date Date of creation: TODO FILL IN DATE HERE
 *
 * \brief
 *
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

// ROS includes
#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>

#include <cob_3d_mapping_msgs/ShapeArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>

class As_Node
{
protected:
  ros::NodeHandle n_;
public:
  As_Node(): n_("~") {
  }

  virtual ~As_Node() {}

  virtual void onInit()=0;

  void start() {

  }
};

class Filter_Node : public As_Node
{
  ros::Subscriber shapes_sub_;
  ros::Publisher  shapes_pub_, shapes_all_pub_, marker_pub_, marker_workspace_pub_;

  //parameters
  double angle_tolerance_, distance_tolerance_;
  int min_weight_, max_weight_;
  pcl::PointCloud<pcl::PointXYZ> workspace_;
  
  bool plane_present_;
  Eigen::Vector3f plane_n_;
  float plane_d_;
  int plane_samples_;
  
public:
  // Constructor
  Filter_Node() : plane_present_(false), plane_n_(Eigen::Vector3f::Zero()), plane_d_(0), plane_samples_(0)
  {
  }

  virtual ~Filter_Node()
  {}

  void onInit() {
    this->start();

    ros::NodeHandle *n=&(this->n_);
    shapes_sub_ = this->n_.subscribe("/shapes_array", 1, &Filter_Node::shapesSubCallback, this);
    shapes_all_pub_ = n->advertise<cob_3d_mapping_msgs::ShapeArray>("/shapes_array_all_out", 1);
    shapes_pub_ = n->advertise<cob_3d_mapping_msgs::ShapeArray>("/shapes_array_obj_out", 1);
    marker_pub_ = n->advertise<visualization_msgs::Marker>("marker_dbg",1);
    marker_workspace_pub_ = n->advertise<visualization_msgs::Marker>("marker_workspace",1);

	angle_tolerance_ = 0.05;
	distance_tolerance_ = 0.03;
	min_weight_ = 0;
	max_weight_ = std::numeric_limits<int>::max();
	plane_samples_ = 5;
	
    this->n_.getParam("angle_tolerance", angle_tolerance_);
    this->n_.getParam("distance_tolerance", distance_tolerance_);
    
    this->n_.getParam("min_weight", min_weight_);
    this->n_.getParam("max_weight", max_weight_);
    
    this->n_.getParam("samples", plane_samples_);
    
    //get params for each measurement
	XmlRpc::XmlRpcValue workspace, ground;
	workspace_.clear();
	if(this->n_.getParam("workspace",workspace))
	{
		if(workspace.getType() != XmlRpc::XmlRpcValue::TypeArray || workspace.size()%2!=0) ROS_ERROR("workspace is a list of 2D-points!");
		else {
			for(int i=0; i<workspace.size(); i+=2) {
				pcl::PointXYZ pt;
				pt.x = (double)workspace[i];
				pt.y = (double)workspace[i+1];
				workspace_.push_back(pt);
			}
		}
	}
	
	if(this->n_.getParam("ground",ground))
	{
		if(ground.getType() != XmlRpc::XmlRpcValue::TypeArray || ground.size()!=4) ROS_ERROR("ground is a list of 4 values!");
		else {
			plane_n_(0) = (double)ground[0];
			plane_n_(1) = (double)ground[1];
			plane_n_(2) = (double)ground[2];
			plane_d_ = (double)ground[3];
			
			plane_present_ = true;
		}
	}
    
    ROS_DEBUG("config\ntolerance: %f %f\nweight: %d %d",
		angle_tolerance_, distance_tolerance_,
		min_weight_, max_weight_);
    
    angle_tolerance_ = std::cos(angle_tolerance_);
  }

	bool detect_ground(const cob_3d_mapping_msgs::ShapeArray &s) {
		if(s.shapes.size()<1 || plane_present_) return false;
		
		size_t m = 0;
		for(size_t i=1; i<s.shapes.size(); i++)
			if(s.shapes[i].weight>s.shapes[m].weight)
				m=i;

		Eigen::Vector3f n;
		for(int j=0; j<3; j++)
			n(j) = s.shapes[m].params[j];
		n.normalize();
		
		plane_n_ += n;
		plane_d_ += s.shapes[m].params[3];
		
		--plane_samples_;
		plane_present_ = (plane_samples_==0);
		
		if(plane_present_) {
			const float l = plane_n_.norm();
			plane_n_/=l;
			plane_d_/=l;
			
			ROS_INFO("found ground plane: %f %f %f %f",
				plane_n_(0), plane_n_(1), plane_n_(2), plane_d_);
		}
		
		return plane_present_;
	}

  void
  shapesSubCallback(cob_3d_mapping_msgs::ShapeArray s)
  {
	publishWorkspace(s.header);
	
	if(s.shapes.size()<1) return;
	
	if(!plane_present_ && !detect_ground(s)) return;

	cob_3d_mapping_msgs::ShapeArray objs;
	for(size_t i=0; i<s.shapes.size(); i++) {
		Eigen::Vector3f n;
		for(int j=0; j<3; j++)
			n(j) = s.shapes[i].params[j];
		n.normalize();
		
		Eigen::Vector3f center;
		center(0) = s.shapes[i].pose.position.x;
		center(1) = s.shapes[i].pose.position.y;
		center(2) = s.shapes[i].pose.position.z;
		pcl::PointXYZ pt;
		pt.x = center(0);
		pt.y = center(1);
		
		Eigen::Quaternionf q;
		q.x() = s.shapes[i].pose.orientation.x;
		q.y() = s.shapes[i].pose.orientation.y;
		q.z() = s.shapes[i].pose.orientation.z;
		q.w() = s.shapes[i].pose.orientation.w;
		
		//unset, otherwise visualization will be messed up...
		s.shapes[i].pose = geometry_msgs::Pose();
		//s.shapes[i].params[3] = -s.shapes[i].params[3];
		
		const float d = center.dot(n);

		const bool crit = std::abs(n.dot(plane_n_)) < angle_tolerance_ ||
			plane_d_-d < distance_tolerance_ ||
			s.shapes[i].weight < min_weight_ ||
			s.shapes[i].weight > max_weight_ ||
			(workspace_.size()>0 && !pcl::isXYPointIn2DXYPolygon(pt, workspace_));
		ROS_DEBUG("criteria %s --> %f>=%f  %f>=%f %d<=%f<=%d",
			crit?"No ":"Yes",
			std::abs(n.dot(plane_n_)), angle_tolerance_,
			plane_d_-d, distance_tolerance_,
			min_weight_, s.shapes[i].weight, max_weight_);

		if( crit ) continue;

		objs.shapes.push_back(s.shapes[i]);
		
		publishMarker(center, q.toRotationMatrix().col(1),s.header, i);
	}
	
	shapes_pub_.publish(objs);
	shapes_all_pub_.publish(s);
  }
  
  void
  publishWorkspace(const std_msgs::Header& header)
  {
    visualization_msgs::Marker marker_centroid;
    marker_centroid.type = visualization_msgs::Marker::LINE_STRIP;
    marker_centroid.header = header;
    marker_centroid.id = 0;
    marker_centroid.action = visualization_msgs::Marker::ADD;
    
    for(size_t i=0; i<workspace_.size()+1; i++) {
		geometry_msgs::Point pt;
		pt.x = workspace_[i%workspace_.size()].x;
		pt.y = workspace_[i%workspace_.size()].y;
		pt.z = -(pt.x*plane_n_(0)+pt.y*plane_n_(1)-plane_d_)/plane_n_(2) -0.05;
		marker_centroid.points.push_back(pt);
	}
    
    marker_centroid.pose.position.x = 0;
    marker_centroid.pose.position.y = 0;
    marker_centroid.pose.position.z = 0;
    
    marker_centroid.color.r = 1.0;
    marker_centroid.color.g = 0.843137255;
    marker_centroid.color.b = 0.0;
    marker_centroid.color.a = 1.0;
    
    marker_centroid.scale.x = 0.025;
    marker_workspace_pub_.publish(marker_centroid);
  }
  
  void
  publishMarker(const Eigen::Vector3f &centroid, const Eigen::Vector3f &normal, const std_msgs::Header& header, const int id)
  {
    visualization_msgs::Marker marker_centroid;
    marker_centroid.type = visualization_msgs::Marker::ARROW;
    marker_centroid.header = header;
    marker_centroid.id = id;
    marker_centroid.action = visualization_msgs::Marker::ADD;
    geometry_msgs::Point pt;
    pt.x = centroid(0);
    pt.y = centroid(1);
    pt.z = centroid(2);
    marker_centroid.points.push_back(pt);
    pt.x = centroid(0) + normal(0);
    pt.y = centroid(1) + normal(1);
    pt.z = centroid(2) + normal(2);
    marker_centroid.points.push_back(pt);
    marker_centroid.pose.position.x = 0;
    marker_centroid.pose.position.y = 0;
    marker_centroid.pose.position.z = 0;
    marker_centroid.color.b = 1.0;
    marker_centroid.color.a = 1.0;
    marker_centroid.scale.x = 0.05;
    marker_centroid.scale.y = 0.05;
    marker_centroid.scale.z = 0.05;
    marker_pub_.publish(marker_centroid);
  }
};


int main(int argc, char **argv) {
  ros::init(argc, argv, "qppf_filter");
  Filter_Node fn;

  fn.onInit();

  ros::spin();

  return 0;
}


