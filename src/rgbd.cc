/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>

// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>

#include <opencv2/core/core.hpp>

#include "include/System.h"

using namespace std;

class ImageGrabber
{
public:
    typedef ORB_SLAM2::Tracking::eTrackingState eTrackingState;
    typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

    ImageGrabber(ORB_SLAM2::System* pSLAM, ros::NodeHandle nh);

    /**
     *
     * @brief
     */
    void GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD);
    
    /**
     *
     * @brief
     */
    tf::StampedTransform broadcastTfs(cv::Mat T_cam_world);

    /**
     *
     * @brief 
     */
    void publishNavMsg(tf::StampedTransform current_world_cam);

    /**
     *
     * @brief
     */
    void publishPointCloud();

protected:
    ORB_SLAM2::System* mpSLAM;

    ros::NodeHandle mNodeHandler;
    ros::Publisher mOdomPublisher;
    ros::Publisher mPointCloudPublisher;
    
    tf::TransformBroadcaster mtfBroadcaster;
    tf::TransformListener mtfListener;
    /**! Transformation used to setup the transform of ORBSLAM, for better visualization **/
    tf::Transform mTransform_odom_world;

    tf::StampedTransform mStampedTransform_world_cam;
    
    std::string mStrWorldFrameId;
    std::string mStrCameraFrameId;
    std::string mStrBaseLinkFrameId;
    std::string mStrOdomFrameId;

    // the transforms published by camera driver
    std::string mStrCameraLinkId;
    std::string mStrCameraOpticalFrameId;

    eTrackingState mLastTrackingState;

    double mnPosePositionVariance;
    double mnPoseOrientationVariance;
    double mnTwistPositionVariance;
    double mnTwistOrientationVariance;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "RGBD");
    ros::start();

    if(argc != 3)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM2 RGBD path_to_vocabulary path_to_settings" << endl;        
        ros::shutdown();
        return 1;
    }    

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::RGBD,true);

    ros::NodeHandle nh("~");
    ImageGrabber igb(&SLAM, nh);

    message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh, "/camera/rgb/image_raw", 1);
    message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, "/camera/depth_registered/image_raw", 1);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), rgb_sub,depth_sub);
    sync.registerCallback(boost::bind(&ImageGrabber::GrabRGBD,&igb,_1,_2));

    ros::spin();

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    ros::shutdown();

    return 0;
}
 
ImageGrabber::ImageGrabber(ORB_SLAM2::System* pSLAM, ros::NodeHandle nh) : 
    mpSLAM(pSLAM), mNodeHandler(nh), mLastTrackingState(eTrackingState::SYSTEM_NOT_READY)
{
    ros::param::param<std::string>("~world_frame",      mStrWorldFrameId,       "orb_world");
    ros::param::param<std::string>("~camera_frame",     mStrCameraFrameId,      "orb_camera");
    ros::param::param<std::string>("~baselink_frame",   mStrBaseLinkFrameId,    "base_link");
    ros::param::param<std::string>("~odom_frame",       mStrOdomFrameId,        "wheelodom");

    ros::param::param<std::string>("~cameralink_frame",         mStrCameraLinkId,           "camera_link");
    ros::param::param<std::string>("~cameraopticallink_frame",  mStrCameraOpticalFrameId,   "camera_rgb_optical_frame");

    ros::param::param<double>("~pose_position_variance",        mnPosePositionVariance,     999.0);
    ros::param::param<double>("~pose_orientation_variance",     mnPoseOrientationVariance,  999.0);
    ros::param::param<double>("~twist_position_variance",       mnPosePositionVariance,     99999999.0);
    ros::param::param<double>("~twist_orientation_variance",    mnPoseOrientationVariance,  99999999.0);

    mOdomPublisher = mNodeHandler.advertise<nav_msgs::Odometry>("visual_odom", 10);
    mPointCloudPublisher = mNodeHandler.advertise<PointCloud> ("pointcloud", 1);
    mTransform_odom_world.setIdentity();
}

void ImageGrabber::GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptrRGB;
    try
    {
        cv_ptrRGB = cv_bridge::toCvShare(msgRGB);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv_bridge::CvImageConstPtr cv_ptrD;
    try
    {
        cv_ptrD = cv_bridge::toCvShare(msgD);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv::Mat T_cam_world = mpSLAM->TrackRGBD(cv_ptrRGB->image,cv_ptrD->image,cv_ptrRGB->header.stamp.toSec());
    auto currentTrackingState = (eTrackingState)mpSLAM->GetTrackingState();
    
    if (currentTrackingState == eTrackingState::LOST)
    {
        mpSLAM->Reset();
        return;
    }
    if (currentTrackingState != eTrackingState::OK)
    {
        return;
    }

    auto stampedTransform_world_cam = broadcastTfs(T_cam_world);
        
    // publish the odom topic
    if (mLastTrackingState == eTrackingState::OK && currentTrackingState == eTrackingState::OK)
    {
        publishNavMsg(stampedTransform_world_cam);
        // publishPointCloud();
    }

    mStampedTransform_world_cam = stampedTransform_world_cam;
    mLastTrackingState = currentTrackingState;
}

tf::StampedTransform ImageGrabber::broadcastTfs(cv::Mat T_camoptical_worldoptical)
{
    tf::Transform transform_camoptical_worldoptical;

    tf::StampedTransform transform_camlink_camoptical;
    transform_camlink_camoptical.setIdentity();

    if (!T_camoptical_worldoptical.empty())
    {
        transform_camoptical_worldoptical.setOrigin(tf::Vector3(
            T_camoptical_worldoptical.at<float>(0, 3), T_camoptical_worldoptical.at<float>(1, 3), T_camoptical_worldoptical.at<float>(2, 3)
        ));
        transform_camoptical_worldoptical.setBasis(tf::Matrix3x3(
           T_camoptical_worldoptical.at<float>(0, 0), T_camoptical_worldoptical.at<float>(0, 1), T_camoptical_worldoptical.at<float>(0, 2),
           T_camoptical_worldoptical.at<float>(1, 0), T_camoptical_worldoptical.at<float>(1, 1), T_camoptical_worldoptical.at<float>(1, 2),
           T_camoptical_worldoptical.at<float>(2, 0), T_camoptical_worldoptical.at<float>(2, 1), T_camoptical_worldoptical.at<float>(2, 2)   
        ));
    }

    try
    {
        mtfListener.lookupTransform(mStrCameraLinkId, mStrCameraOpticalFrameId, 
                                    ros::Time(0), transform_camlink_camoptical);
        
    }
    catch (tf::TransformException ex)
    {

        ROS_ERROR("%s", ex.what());
    }

    // when there is transition from not okay to okay, get the current odometry tf to set the coordinate frame of the map
    auto currentTrackingState = (eTrackingState)mpSLAM->GetTrackingState();
    // if (mLastTrackingState != eTrackingState::OK && currentTrackingState == eTrackingState::OK)
    // {
    //     tf::StampedTransform transform_odom_camoptical;
    //     transform_odom_camoptical.setIdentity();

    //     try
    //     {
    //         mtfListener.waitForTransform(mStrOdomFrameId, mStrCameraOpticalFrameId,
    //                                   ros::Time::now(), ros::Duration(1.0));
    //         mtfListener.lookupTransform(mStrOdomFrameId, mStrCameraOpticalFrameId,
    //                                     ros::Time(0), transform_odom_camoptical);
    //     }
    //     catch (tf::TransformException ex)
    //     {

    //         ROS_ERROR("%s", ex.what());
    //     }

    //     mTransform_odom_world = transform_odom_camoptical * transform_camoptical_worldoptical * transform_camlink_camoptical.inverse();
    // }

    tf::Transform transform_world_camlink = transform_camlink_camoptical * transform_camoptical_worldoptical.inverse() * 
                                            transform_camlink_camoptical.inverse();
    tf::StampedTransform stampedTransform_world_cam = tf::StampedTransform(
        transform_world_camlink, ros::Time::now(), mStrWorldFrameId, mStrCameraFrameId
    );
    // mtfBroadcaster.sendTransform(stampedTransform_world_cam);

    // mtfBroadcaster.sendTransform(tf::StampedTransform(
    //     transform_camlink_camoptical.inverse(), ros::Time::now(), mStrWorldFrameId, mStrWorldFrameId + std::string("_optical")
    // ));

    // don't link the two orb frames and wheel odom frames, this is trouble some
    // mtfBroadcaster.sendTransform(tf::StampedTransform(
    //     mTransform_odom_world, ros::Time::now(), mStrOdomFrameId, mStrWorldFrameId
    // ));

    return stampedTransform_world_cam;
}

void ImageGrabber::publishNavMsg(tf::StampedTransform stampedTransform_world_cam)
{
    nav_msgs::Odometry odom_msg;
    odom_msg.header.stamp = ros::Time::now();
    odom_msg.header.frame_id = mStrCameraFrameId;
    odom_msg.child_frame_id = mStrBaseLinkFrameId;
    
    auto position = stampedTransform_world_cam.getOrigin();
    auto orientation = stampedTransform_world_cam.getRotation();
    odom_msg.pose.pose.position.x = position.getX();
    odom_msg.pose.pose.position.y = position.getY();
    odom_msg.pose.pose.position.z = position.getZ(); 

    odom_msg.pose.pose.orientation.x = orientation.getX();
    odom_msg.pose.pose.orientation.y = orientation.getY();
    odom_msg.pose.pose.orientation.z = orientation.getZ();
    odom_msg.pose.pose.orientation.w = orientation.getW();

    // TODO: covariances

    // relative transform between last and current
    double dt = stampedTransform_world_cam.stamp_.toSec() - mStampedTransform_world_cam.stamp_.toSec();

    tf::Transform relative_transform = mStampedTransform_world_cam.inverse() * stampedTransform_world_cam;
    auto linear_velocity = relative_transform.getOrigin()/dt;
    
    geometry_msgs::Vector3 angular_velocity;
    tf::Matrix3x3(relative_transform.getRotation()).getRPY(
        angular_velocity.x,
        angular_velocity.y,
        angular_velocity.z
    );
    angular_velocity.x /= dt;
    angular_velocity.y /= dt;
    angular_velocity.z /= dt;

    odom_msg.twist.twist.linear.x = linear_velocity.getX();
    odom_msg.twist.twist.linear.y = linear_velocity.getY();
    odom_msg.twist.twist.linear.z = linear_velocity.getZ();

    odom_msg.twist.twist.angular.x = angular_velocity.x;
    odom_msg.twist.twist.angular.y = angular_velocity.y;
    odom_msg.twist.twist.angular.z = angular_velocity.z;

    // covariances
    odom_msg.pose.covariance[0]     = mnPosePositionVariance;
    odom_msg.pose.covariance[7]     = mnPosePositionVariance;
    odom_msg.pose.covariance[14]    = mnPosePositionVariance;

    odom_msg.pose.covariance[21]    = mnPoseOrientationVariance;
    odom_msg.pose.covariance[28]    = mnPoseOrientationVariance;
    odom_msg.pose.covariance[35]    = mnPoseOrientationVariance;

    odom_msg.twist.covariance[0]    = mnTwistPositionVariance;
    odom_msg.twist.covariance[7]    = mnTwistPositionVariance;
    odom_msg.twist.covariance[14]   = mnTwistPositionVariance;

    odom_msg.twist.covariance[21]   = mnTwistOrientationVariance;
    odom_msg.twist.covariance[28]   = mnTwistOrientationVariance;
    odom_msg.twist.covariance[35]   = mnTwistOrientationVariance;

    mOdomPublisher.publish(odom_msg);
}


void ImageGrabber::publishPointCloud()
{
    try 
    {
        PointCloud::Ptr msg(new PointCloud);
        msg->header.frame_id = mStrWorldFrameId + std::string("_optical");
        pcl_conversions::toPCL(ros::Time::now(), msg->header.stamp);
        msg->height = msg->width = 1;

        auto mapPoints = mpSLAM->GetTrackedMapPoints();
        for (auto &point: mapPoints)
        {
            if (!point)
            {
                continue;
            }
            auto coord = point->GetWorldPos();
            
            msg->points.push_back(pcl::PointXYZ(
                coord.at<float>(0),
                coord.at<float>(1),
                coord.at<float>(2)
            ));
        }
        ROS_DEBUG("publishing %d clouds", msg->points.size());
        mPointCloudPublisher.publish(msg);
    }
    catch(std::runtime_error& ex) 
    {
        ROS_ERROR("Exception: [%s]", ex.what());
    }

}