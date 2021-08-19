#include "cloud_features/nodelet/cloud_features_nodelet.hpp"
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(cloud_features::CloudFeaturesNodelet, nodelet::Nodelet)
using namespace cloud_features;

void 
CloudFeaturesNodelet::onInit()
{
    nh      = getMTNodeHandle();
    nhPriv  = getMTPrivateNodeHandle();

    nhPriv.param<std::string>("cloud_in_topic", cloudInTopic, "/os0/points");
    nhPriv.param<std::string>("cloud_out_topic", cloudOutTopic, "/cloud_out");
    nhPriv.param<float>("vertical_fov", cloudVerticalFOV, M_PI_2);
    nhPriv.param<float>("horizontal_fov", cloudHorizontalFOV, 2*M_PI);
    nhPriv.param<int>("channels", cloudRing, 64);
    nhPriv.param<int>("hz", cloudHz, 10);
    nhPriv.param<int>("horizontal_points", cloudHorizontalPoints, 1024);
    
    
    // init cloud
    sensor_msgs::PointCloud2Modifier modifier(cloudOut);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    cloudOut.point_step = addPointField(cloudOut, "intensity", 1, sensor_msgs::PointField::FLOAT32, cloudOut.point_step);        
    cloudOut.point_step = addPointField(cloudOut, "ring", 1, sensor_msgs::PointField::UINT16, cloudOut.point_step);
    cloudOut.point_step = addPointField(cloudOut, "time", 1, sensor_msgs::PointField::FLOAT32, cloudOut.point_step);
    cloudTimeInc        = (1.0f/cloudHz)/cloudHorizontalPoints;

    cloudPub    = nh.advertise<sensor_msgs::PointCloud2>(cloudOutTopic, 1);
    cloudSub    = nh.subscribe(cloudInTopic, 1, &CloudFeaturesNodelet::cloudCallback, this); 

    ros::spin();
}

void 
CloudFeaturesNodelet::cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{   
    // copy header
    cloudOut.header = msg->header;

    cloudOut.width          = msg->width;
    cloudOut.height         = msg->height;
    cloudOut.is_bigendian   = false;
    cloudOut.is_dense       = true;

    cloudOut.row_step = cloudOut.width * cloudOut.point_step;
    cloudOut.data.resize(cloudOut.height * cloudOut.row_step);
            
    sensor_msgs::PointCloud2Iterator<float>     iter_x(cloudOut, "x");
    sensor_msgs::PointCloud2Iterator<float>     iter_y(cloudOut, "y");
    sensor_msgs::PointCloud2Iterator<float>     iter_z(cloudOut, "z");
    sensor_msgs::PointCloud2Iterator<float>     *iter_intensity, *iter_time;
    sensor_msgs::PointCloud2Iterator<uint16_t>  *iter_ring;
            
    iter_intensity  = new sensor_msgs::PointCloud2Iterator<float>(cloudOut, "intensity");
    iter_ring       = new sensor_msgs::PointCloud2Iterator<uint16_t>(cloudOut, "ring");
    iter_time       = new sensor_msgs::PointCloud2Iterator<float>(cloudOut, "time");

    // convert to point cloud 1
    sensor_msgs::PointCloud tmp;
    sensor_msgs::convertPointCloud2ToPointCloud(*msg, tmp);

    int channel_i = -1;
    for (int i = 0; i < tmp.channels.size(); ++i) {
        if (tmp.channels[i].name == "intensity") {
            channel_i   = i;
            break;
        }
    }

    for(int i = 0 ; i < tmp.points.size(); i++) {
        *iter_x     = tmp.points[i].x;  ++iter_x; 
        *iter_y     = tmp.points[i].y;  ++iter_y; 
        *iter_z     = tmp.points[i].z;  ++iter_z; 
        
        // i
        if (channel_i != -1) {
            **iter_intensity = tmp.channels[channel_i].values[i]; 
            ++(*iter_intensity); 
        }
        
        float r         = std::sqrt(std::pow(tmp.points[i].x, 2) + std::pow(tmp.points[i].y, 2) + std::pow(tmp.points[i].z, 2));
        float h_angle   = std::acos(tmp.points[i].x / std::sqrt(std::pow(tmp.points[i].x, 2) + std::pow(tmp.points[i].y, 2))) * (tmp.points[i].y < 0 ? -1 : 1);
        float v_angle   = std::atan(tmp.points[i].z / std::sqrt(std::pow(tmp.points[i].x, 2) + std::pow(tmp.points[i].y, 2)));

        // ring
        **iter_ring     = (cloudRing - 1) - uint16_t((v_angle + cloudVerticalFOV/2.0f) / (cloudVerticalFOV / cloudRing));
        ++(*iter_ring);
        
        // time
        **iter_time     = cloudTimeInc * ((h_angle + cloudHorizontalFOV/2.0f) * cloudHorizontalPoints) / cloudHorizontalFOV; 
        ++(*iter_time);  
        
    }
    cloudPub.publish(cloudOut);
}
