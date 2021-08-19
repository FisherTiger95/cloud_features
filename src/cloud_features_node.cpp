#include <nodelet/loader.h>
#include <ros/ros.h>

int main (int argc, char** argv) {
    ros::init(argc, argv, "cloud_features_node");

    //Lancia il nodelet come se fosse un nodo
    nodelet::Loader nodelet;
    nodelet::M_string remap(ros::names::getRemappings());
    nodelet::V_string nargv;
    nodelet.load(ros::this_node::getName(), "cloud_features/CloudFeaturesNodelet", remap, nargv);
    ros::spin();

    return 0;
}
