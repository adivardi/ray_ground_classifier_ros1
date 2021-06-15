#include <ray_ground_classifier_nodes/ray_ground_classifier_cloud_node.hpp>

#include <ros/init.h>
#include <ros/node_handle.h>

int
main(int argc, char** argv)
{
  ros::init(argc, argv, "ray_ground_classifier");
  ros::NodeHandle nh;

  autoware::perception::filters::ray_ground_classifier_nodes::RayGroundClassifierCloudNode node(nh);
  ros::spin();

  return 0;
}
