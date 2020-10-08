#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/passthrough.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <pcl/sample_consensus/sac_model_cylinder.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <iostream>
#include <thread>
#include <pcl/console/parse.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/common/centroid.h>
#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/surface/gp3.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/vtk_io.h>
#include <pcl/PolygonMesh.h>
#include <pcl_msgs/PolygonMesh.h>
#include <geometry_msgs/TransformStamped.h>
#include <ros/package.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <shape_msgs/Mesh.h>
#include <pcl/io/io.h>
#include <pcl_ros/publisher.h>
#include <ros/publisher.h>
#include <string>
#include <gravity/gravity_nodeConfig.h>
#include "sensor_msgs/Imu.h"
#include <math.h>
#include <visualization_msgs/Marker.h>
#include <pcl/filters/statistical_outlier_removal.h>

using namespace std;
class GravityNode
{
public:
    GravityNode()
        : private_nh("~")
    {
        g_init = false;
        gravity_marker_pub = nh_.advertise<visualization_msgs::Marker>("imu_out", 1);
        gravitya_marker_pub = nh_.advertise<visualization_msgs::Marker>("imu_average", 1);
        gravityf_marker_pub = nh_.advertise<visualization_msgs::Marker>("imu_filtered", 1);
        config_server_.setCallback(boost::bind(&GravityNode::dynReconfCallback, this, _1, _2));
        sub_kalman = nh_.subscribe("filtered_imu", 1, &GravityNode::kalmanCallback, this);
        sub_imu = nh_.subscribe("imu_data", 1, &GravityNode::imuCallback, this);
        //g_pub = private_nh.advertise<geometry_msgs::Vector3>("gravity_vector", 1);
        ros::NodeHandle private_nh("~");
        private_nh.param("frame_id", tf_frame, std::string("pico_zense_depth_frame"));
    }
    ~GravityNode() {}

    void
    dynReconfCallback(gravity::gravity_nodeConfig &config, uint32_t level)
    {
        gravity_damp = config.gravity_damp;
        lean_tolerance = config.lean_tolerance * PI / 180;
    }

    void
    kalmanCallback(const sensor_msgs::Imu::ConstPtr &msg)
    {
        gravityk[0] = msg->linear_acceleration.x;
        gravityk[1] = msg->linear_acceleration.y;
        gravityk[2] = msg->linear_acceleration.z;
        gravityk.normalize();
        visualization_msgs::Marker gravity_marker;
        uint32_t shape = visualization_msgs::Marker::ARROW;
        gravity_marker.ns = "basic_shapes";
        gravity_marker.id = 1;
        gravity_marker.type = shape;
        gravity_marker.action = visualization_msgs::Marker::ADD;
        Eigen::Vector3f centroid = Eigen::Vector3f::Zero();
        geometry_msgs::Point pt;
        pt.x = centroid(0);
        pt.y = centroid(1);
        pt.z = centroid(2);
        gravity_marker.points.push_back(pt);
        pt.x = centroid(0) + gravityk(0);
        pt.y = centroid(1) + gravityk(1);
        pt.z = centroid(2) + gravityk(2);
        gravity_marker.points.push_back(pt);
        gravity_marker.scale.x = 0.05;
        gravity_marker.scale.y = 0.1;
        gravity_marker.scale.z = 0.05;
        gravity_marker.color.r = 0.0f;
        gravity_marker.color.g = 1.0f;
        gravity_marker.color.b = 0.0f;
        gravity_marker.color.a = 0.5;
        gravity_marker.lifetime = ros::Duration();
        gravity_marker.header.frame_id = tf_frame;
        gravity_marker.header.stamp = ros::Time::now();
        gravityf_marker_pub.publish(gravity_marker);
    }

    void
    gravityDamper(const sensor_msgs::Imu::ConstPtr &msg)
    {
        gravity(0) = msg->linear_acceleration.x;
        gravity(1) = msg->linear_acceleration.y;
        gravity(2) = msg->linear_acceleration.z;
        for (int i = 0; i < gravity_damp - 1; i++)
        {
            gravity(0) += gravity_p[i][0];
            gravity(1) += gravity_p[i][1];
            gravity(2) += gravity_p[i][2];
            if (i > 0)
            {
                gravity_p[i][0] = gravity_p[i - 1][0];
                gravity_p[i][1] = gravity_p[i - 1][1];
                gravity_p[i][2] = gravity_p[i - 1][2];
            }
        }
        gravity(0) /= gravity_damp;
        gravity(1) /= gravity_damp;
        gravity(2) /= gravity_damp;
        gravity.normalize();
        gravity_p[0][0] = msg->linear_acceleration.x;
        gravity_p[0][1] = msg->linear_acceleration.y;
        gravity_p[0][2] = msg->linear_acceleration.z;
    }

    void
    imuCallback(const sensor_msgs::Imu::ConstPtr &msg)
    {
        if (!g_init)
        {
            gravity0[0] = msg->linear_acceleration.x;
            gravity0[1] = msg->linear_acceleration.y;
            gravity0[2] = msg->linear_acceleration.z;
            gravity0.normalize();
            for (int i = 0; i < 100; i++)
            {
                gravity_p[i][0] = msg->linear_acceleration.x;
                gravity_p[i][1] = msg->linear_acceleration.y;
                gravity_p[i][2] = msg->linear_acceleration.z;
            }

            g_init = true;
        }
        gravityDamper(msg);

        visualization_msgs::Marker gravity_marker;
        uint32_t shape = visualization_msgs::Marker::ARROW;
        gravity_marker.ns = "basic_shapes";
        gravity_marker.id = 1;
        gravity_marker.type = shape;
        gravity_marker.action = visualization_msgs::Marker::ADD;
        Eigen::Vector3f centroid = Eigen::Vector3f::Zero();
        geometry_msgs::Point pt;
        pt.x = centroid(0);
        pt.y = centroid(1);
        pt.z = centroid(2);
        gravity_marker.points.push_back(pt);
        pt.x = centroid(0) + gravity(0);
        pt.y = centroid(1) + gravity(1);
        pt.z = centroid(2) + gravity(2);
        gravity_marker.points.push_back(pt);
        gravity_marker.scale.x = 0.05;
        gravity_marker.scale.y = 0.1;
        gravity_marker.scale.z = 0.05;
        gravity_marker.color.r = 1.0f;
        gravity_marker.color.g = 0.0f;
        gravity_marker.color.b = 0.0f;
        gravity_marker.color.a = 0.5;
        gravity_marker.lifetime = ros::Duration();
        gravity_marker.header.frame_id = tf_frame;
        gravity_marker.header.stamp = ros::Time::now();
        gravitya_marker_pub.publish(gravity_marker);

        gravitya(0) = msg->linear_acceleration.x;
        gravitya(1) = msg->linear_acceleration.y;
        gravitya(2) = msg->linear_acceleration.z;
        gravitya.normalize();
        visualization_msgs::Marker gravity_marker2;
        gravity_marker2.ns = "basic_shapes";
        gravity_marker2.id = 1;
        gravity_marker2.type = shape;
        gravity_marker2.action = visualization_msgs::Marker::ADD;
        pt.x = centroid(0);
        pt.y = centroid(1);
        pt.z = centroid(2);
        gravity_marker2.points.push_back(pt);
        pt.x = centroid(0) + gravitya(0);
        pt.y = centroid(1) + gravitya(1);
        pt.z = centroid(2) + gravitya(2);
        gravity_marker2.points.push_back(pt);
        gravity_marker2.scale.x = 0.05;
        gravity_marker2.scale.y = 0.1;
        gravity_marker2.scale.z = 0.05;
        gravity_marker2.color.r = 0.0f;
        gravity_marker2.color.g = 0.0f;
        gravity_marker2.color.b = 1.0f;
        gravity_marker2.color.a = 0.5;
        gravity_marker2.lifetime = ros::Duration();
        gravity_marker2.header.frame_id = tf_frame;
        gravity_marker2.header.stamp = ros::Time::now();
        gravity_marker_pub.publish(gravity_marker2);
    }

private:
    double PI = 3.14159265;
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh;
    std::string tf_frame = "pico_zense_depth_frame";
    ros::Subscriber sub_imu;
    ros::Subscriber sub_kalman;
    ros::Publisher gravity_marker_pub;
    ros::Publisher gravitya_marker_pub;
    ros::Publisher gravityf_marker_pub;
    ros::Publisher g_pub;
    dynamic_reconfigure::Server<gravity::gravity_nodeConfig> config_server_;
    Eigen::Vector3f gravity, gravity0, gravityk, gravitya;
    int gravity_damp = 5;
    double gravity_p[100][3];
    bool g_init;
    double lean_tolerance;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "gravity_node");
    GravityNode gn;
    ros::spin();
}