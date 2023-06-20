#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

ros::Publisher pub;

void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg)
{
  // Convert sensor_msgs::PointCloud2 to pcl::PointCloud<pcl::PointXYZRGB>
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::fromROSMsg(*cloud_msg, *cloud);

  // Preprocessing: Filter out points outside a specified range
  pcl::PassThrough<pcl::PointXYZRGB> pass;
  pass.setInputCloud(cloud);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(0.0, 20.0); //range 
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  pass.filter(*filtered_cloud);

  // Ground plane elimination using RANSAC
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inlier_indices(new pcl::PointIndices);
  pcl::SACSegmentation<pcl::PointXYZRGB> seg;
  seg.setInputCloud(filtered_cloud);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(0.02);
  seg.segment(*inlier_indices, *coefficients);

  // Extract objects (non-ground points) from the filtered cloud
  pcl::ExtractIndices<pcl::PointXYZRGB> extract;
  extract.setInputCloud(filtered_cloud);
  extract.setIndices(inlier_indices);
  extract.setNegative(true);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr segmented_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  extract.filter(*segmented_cloud);

  // Cluster extraction
  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
  ec.setClusterTolerance(0.04); // Adjust the tolerance as needed
  ec.setMinClusterSize(200);    // Adjust the minimum cluster size as needed
  ec.setMaxClusterSize(50000);  // Adjust the maximum cluster size as needed
  ec.setInputCloud(segmented_cloud);
  ec.extract(cluster_indices);

  // Colour each segmented cluster object differently
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  int cluster_id = 0;
  for (const auto& indices : cluster_indices)
  {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZRGB>);
    for (const auto& index : indices.indices)
    {
      pcl::PointXYZRGB point;
      point.x = segmented_cloud->points[index].x;
      point.y = segmented_cloud->points[index].y;
      point.z = segmented_cloud->points[index].z;
      point.r = 255 * (cluster_id % 3 == 0);
      point.g = 255 * (cluster_id % 3 == 1);
      point.b = 255 * (cluster_id % 3 == 2);
      cluster->push_back(point);
    }
    *colored_cloud += *cluster;
    cluster_id++;
  }

  // Convert pcl::PointCloud<pcl::PointXYZRGB> to sensor_msgs::PointCloud2
  sensor_msgs::PointCloud2 output_cloud;
  pcl::toROSMsg(*colored_cloud, output_cloud);
  output_cloud.header = cloud_msg->header;
  pub.publish(output_cloud);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pclonly_node");
  ros::NodeHandle nh;
  pub = nh.advertise<sensor_msgs::PointCloud2>("output_cloud", 1);

  //input point cloud topic from kinect
  ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>("/kinect/depth/points", 1, cloudCallback);

  ros::spin();

  return 0;
}
