#pragma once

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <sensor_msgs/point_cloud_conversion.h>

namespace cloud_features {
    class CloudFeaturesNodelet : public nodelet::Nodelet {
        public:
             CloudFeaturesNodelet() = default;
            ~CloudFeaturesNodelet() = default;

        private:
            virtual void onInit();

        protected:
            void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);

            // ROS 
            ros::NodeHandle             nh;
            ros::NodeHandle             nhPriv;
            ros::Time                   ot;

            ros::Publisher              cloudPub;
            ros::Subscriber             cloudSub;

            std::string                 cloudInTopic;
            std::string                 cloudOutTopic;

            float                       cloudVerticalFOV;
            float                       cloudHorizontalFOV;
            int                         cloudRing;
            int                         cloudHz;
            int                         cloudHorizontalPoints;
            float                       cloudTimeInc;

            bool                        first;

            sensor_msgs::PointCloud2    cloudOut;
    };
}
