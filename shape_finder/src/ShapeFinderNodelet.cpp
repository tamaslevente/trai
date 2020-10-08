#include "includes.h"

namespace shape_finder
{
    class ShapeFinderNodelet : public nodelet::Nodelet
    {

    private:
        double PI = 3.14159265;
        ros::NodeHandle nh_;
        ros::NodeHandle private_nh;
        std::string tf_frame = "pico_zense_depth_frame";
        float leaf_size = 0.02f;
        float th = 0.6;
        size_t big_plane_size = 1000;
        size_t min_cloud_size = 100;
        ros::Subscriber sub_;
        ros::Subscriber sub_g;
        ros::Publisher normal_marker_pub;
        pcl_ros::Publisher<sensor_msgs::PointCloud2> pub_;
        dynamic_reconfigure::Server<shape_finder::shape_finder_nodeletConfig> config_server_;
        Eigen::Vector3f gravity;
        double hv_tolerance;
        double NormalDistanceWeightP;
        int MaxIterationsP;
        double DistanceThresholdP;
        double DistanceThresholdP0;
        double NormalDistanceWeightC;
        int MaxIterationsC;
        double DistanceThresholdC;
        double RadiusLimitsMinC;
        double RadiusLimitsMaxC;
        double StddevMulThresh;
        int MeanK;

        pcl::PointCloud<pcl::PointXYZ>::Ptr
        ret_sphere(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
        {
            pcl::PointCloud<pcl::PointXYZ>::Ptr final(new pcl::PointCloud<pcl::PointXYZ>);
            std::vector<int> inliers;
            pcl::SampleConsensusModelSphere<pcl::PointXYZ>::Ptr
                model(new pcl::SampleConsensusModelSphere<pcl::PointXYZ>(cloud));
            model->setRadiusLimits(0.0, 0.25);
            pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(model);
            ransac.setDistanceThreshold(.01);
            ransac.computeModel();
            ransac.getInliers(inliers);
            pcl::copyPointCloud(*cloud, inliers, *final);
            return final;
        }

        pcl::PointCloud<pcl::PointXYZ>::Ptr
        ret_cylinder(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
        {
            pcl::PassThrough<pcl::PointXYZ> pass;
            pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
            pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg;
            pcl::ExtractIndices<pcl::PointXYZ> extract;
            pcl::ExtractIndices<pcl::Normal> extract_normals;
            pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered2(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
            pcl::PointCloud<pcl::Normal>::Ptr cloud_normals2(new pcl::PointCloud<pcl::Normal>);
            pcl::ModelCoefficients::Ptr coefficients_plane(new pcl::ModelCoefficients), coefficients_cylinder(new pcl::ModelCoefficients);
            pcl::PointIndices::Ptr inliers_plane(new pcl::PointIndices), inliers_cylinder(new pcl::PointIndices);
            ne.setSearchMethod(tree);
            ne.setInputCloud(cloud);
            ne.setKSearch(50);
            ne.compute(*cloud_normals);
            seg.setOptimizeCoefficients(true);
            seg.setModelType(pcl::SACMODEL_NORMAL_PLANE);
            seg.setNormalDistanceWeight(NormalDistanceWeightP);
            seg.setMethodType(pcl::SAC_RANSAC);
            seg.setMaxIterations(MaxIterationsP);
            seg.setDistanceThreshold(DistanceThresholdP);
            seg.setInputCloud(cloud);
            seg.setInputNormals(cloud_normals);
            // Obtain the plane inliers and coefficients
            seg.segment(*inliers_plane, *coefficients_plane);
            extract.setInputCloud(cloud);
            extract.setIndices(inliers_plane);
            extract.setNegative(false);
            // Remove the planar inliers, extract the rest
            extract.setNegative(true);
            extract.filter(*cloud_filtered2);
            extract_normals.setNegative(true);
            extract_normals.setInputCloud(cloud_normals);
            extract_normals.setIndices(inliers_plane);
            extract_normals.filter(*cloud_normals2);
            // Create the segmentation object for cylinder segmentation and set all the parameters
            seg.setOptimizeCoefficients(true);
            seg.setModelType(pcl::SACMODEL_CYLINDER);
            seg.setMethodType(pcl::SAC_RANSAC);
            seg.setNormalDistanceWeight(NormalDistanceWeightC);
            seg.setMaxIterations(MaxIterationsC);
            seg.setDistanceThreshold(DistanceThresholdC);
            seg.setRadiusLimits(RadiusLimitsMinC, RadiusLimitsMaxC);
            seg.setInputCloud(cloud_filtered2);
            seg.setInputNormals(cloud_normals2);
            seg.segment(*inliers_cylinder, *coefficients_cylinder);
            extract.setInputCloud(cloud);
            extract.setIndices(inliers_cylinder);
            extract.setNegative(false);
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cylinder(new pcl::PointCloud<pcl::PointXYZ>());
            extract.filter(*cloud_cylinder);
            return cloud_cylinder;
        }

        pcl::PointCloud<pcl::PointXYZRGB>
        shapefind(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster, int j)
        {
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_s(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_c(new pcl::PointCloud<pcl::PointXYZ>);
            size_t size_cloud = cloud_cluster->size();
            int big_plane = 0;
            int horizontal_floor = 0;
            int horizontal_ceiling = 0;
            int vertical = 0;
            int cluster_shape = 0; //0-nothing(sub threshold-th), 1-plane, 2-sphere, 3-cylinder
            printf("cluster %d, size: %zu\n", j, size_cloud);
            pcl::SACSegmentation<pcl::PointXYZ> seg;
            pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
            pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_p(new pcl::PointCloud<pcl::PointXYZ>());
            seg.setOptimizeCoefficients(true);
            seg.setModelType(pcl::SACMODEL_PLANE);
            seg.setMethodType(pcl::SAC_RANSAC);
            seg.setMaxIterations(50);
            seg.setDistanceThreshold(DistanceThresholdP0);
            seg.setInputCloud(cloud_cluster);
            seg.segment(*inliers, *coefficients);
            pcl::ExtractIndices<pcl::PointXYZ> extract;
            extract.setInputCloud(cloud_cluster);
            extract.setIndices(inliers);
            extract.setNegative(false);
            extract.filter(*cloud_p);

            cloud_s = ret_sphere(cloud_cluster);
            cloud_c = ret_cylinder(cloud_cluster);
            size_t size_cloud_s = cloud_s->size();
            printf("Size sphere point cloud: %zu\n", size_cloud_s);
            float overlap_s = (float)size_cloud_s / (float)size_cloud;
            printf("sphere overlap: %f\n", overlap_s);
            size_t size_cloud_p = cloud_p->size();
            printf("Size plane point cloud: %zu\n", size_cloud_p);
            float overlap_p = (float)size_cloud_p / (float)size_cloud;
            printf("plane overlap: %f\n", overlap_p);
            size_t size_cloud_c = cloud_c->size();
            printf("Size cylinder point cloud: %zu\n", size_cloud_c);
            float overlap_c = (float)size_cloud_c / (float)size_cloud;
            printf("cylinder overlap: %f\n", overlap_c);
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_colored_cluster(new pcl::PointCloud<pcl::PointXYZRGB>);

            if ((overlap_p > overlap_s) && (overlap_p > overlap_c) && (overlap_p > th) && (size_cloud_p > min_cloud_size))
            {
                cluster_shape = 1;
                pcl::copyPointCloud(*cloud_p, *cloud_colored_cluster);
                /*Eigen::Vector3f centroid = Eigen::Vector3f::Zero();
      for (unsigned int i = 0; i < cloud_colored_cluster->size(); i++)
      {
        centroid += cloud_colored_cluster->points[i].getVector3fMap();
      }
      centroid /= cloud_colored_cluster->size();*/
                Eigen::Vector3f normal;
                normal << coefficients->values[0], coefficients->values[1], coefficients->values[2];
                if (coefficients->values[3] < 0)
                {
                    normal *= -1;
                }
                float angle = acos(gravity.dot(normal));
                if (cloud_p->size() > big_plane_size)
                    big_plane = 1;
                if (angle > PI - hv_tolerance)
                    horizontal_floor = 1;
                if (angle < hv_tolerance)
                    horizontal_ceiling = 1;
                if (angle > PI / 2 - hv_tolerance && angle < PI / 2 + hv_tolerance)
                    vertical = 1;
            }

            if ((overlap_s > overlap_p) && (overlap_s > overlap_c) && (overlap_s > th) && (size_cloud_s > min_cloud_size))
            {
                cluster_shape = 2;
                pcl::copyPointCloud(*cloud_s, *cloud_colored_cluster);
            }

            if ((overlap_c > overlap_s) && (overlap_c > overlap_p) && (overlap_c > th) && (size_cloud_c > min_cloud_size))
            {
                cluster_shape = 3;
                pcl::copyPointCloud(*cloud_c, *cloud_colored_cluster);
            }

            std::uint8_t r = 0, g = 0, b = 0;

            switch (cluster_shape)
            {
            case 0:
            {
                r = 255;
                g = 255;
                b = 255; // white
                break;
            }
            case 1:
            {
                if (big_plane)
                {
                    r = 0;
                    g = 125;
                    b = 0; // dark green
                    if (vertical)
                    {
                        r = 255;
                        g = 255;
                        b = 0; //yellow
                    }
                    if (horizontal_floor)
                    {
                        r = 178;
                        g = 102;
                        b = 255; // purple
                    }
                    if (horizontal_ceiling)
                    {
                        r = 255;
                        g = 0;
                        b = 153; //magenta
                    }
                }
                else
                {
                    r = 0;
                    g = 255;
                    b = 0; // light green
                    if (horizontal_floor)
                    {
                        r = 178;
                        g = 102;
                        b = 255; // purple
                    }
                }
                break;
            }
            case 2:
            {
                r = 255;
                g = 0;
                b = 0; // red
                break;
            }
            case 3:
            {
                r = 0;
                g = 0;
                b = 255; // blue
                break;
            }
            }

            std::uint32_t rgb = ((std::uint32_t)r << 16 | (std::uint32_t)g << 8 | (std::uint32_t)b);
            printf("cluster+shape=%d\nrgb:%d,%d,%d\n", cluster_shape, r, g, b);
            for (int i = 0; i < cloud_colored_cluster->size(); i++)
            {
                cloud_colored_cluster->points[i].rgb = *reinterpret_cast<float *>(&rgb);
            }

            return *cloud_colored_cluster;
        }

        pcl::PointCloud<pcl::PointXYZRGB>
        planefind(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster, int j, pcl::ModelCoefficients::Ptr coefficients)
        {
            size_t size_cloud = cloud_cluster->size();
            int big_plane = 0;
            int horizontal_floor = 0;
            int horizontal_ceiling = 0;
            int vertical = 0;
            printf("cluster %d, size: %zu\n", j, size_cloud);
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_colored_cluster(new pcl::PointCloud<pcl::PointXYZRGB>);
            pcl::copyPointCloud(*cloud_cluster, *cloud_colored_cluster);
            /*Eigen::Vector3f centroid = Eigen::Vector3f::Zero();
    for (unsigned int i = 0; i < cloud_colored_cluster->size(); i++)
    {
      centroid += cloud_colored_cluster->points[i].getVector3fMap();
    }
    centroid /= cloud_colored_cluster->size();*/
            Eigen::Vector3f normal;
            normal << coefficients->values[0], coefficients->values[1], coefficients->values[2];
            if (coefficients->values[3] < 0)
            {
                normal *= -1;
            }
            float angle = acos(gravity.dot(normal));
            std::cout << "The plane angle is: " << angle << std::endl;
            if (cloud_colored_cluster->size() > big_plane_size)
                big_plane = 1;
            if (angle > PI - hv_tolerance)
                horizontal_floor = 1;
            if (angle < hv_tolerance)
                horizontal_ceiling = 1;
            if (angle > PI / 2 - hv_tolerance && angle < PI / 2 + hv_tolerance)
                vertical = 1;
            std::uint8_t r = 0, g = 0, b = 0;
            if (big_plane)
            {
                r = 0;
                g = 125;
                b = 0; // dark green
                if (vertical)
                {
                    r = 255;
                    g = 255;
                    b = 0; //yellow
                }
                if (horizontal_floor)
                {
                    r = 178;
                    g = 102;
                    b = 255; // purple
                }
                if (horizontal_ceiling)
                {
                    r = 255;
                    g = 0;
                    b = 153; //magenta
                }
            }
            else
            {
                r = 0;
                g = 255;
                b = 0; // light green
                if (horizontal_floor)
                {
                    r = 178;
                    g = 102;
                    b = 255; // purple
                }
            }
            std::uint32_t rgb = ((std::uint32_t)r << 16 | (std::uint32_t)g << 8 | (std::uint32_t)b);
            for (int i = 0; i < cloud_colored_cluster->size(); i++)
            {
                cloud_colored_cluster->points[i].rgb = *reinterpret_cast<float *>(&rgb);
            }
            return *cloud_colored_cluster;
        }

        void
        cloudCallback(const sensor_msgs::PointCloud2::ConstPtr &cloud_in)
        {
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_colored_pcl(new pcl::PointCloud<pcl::PointXYZRGB>);
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
            sensor_msgs::PointCloud2 cloud_colored_sensor;
            pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
            pcl::fromROSMsg(*cloud_in, *cloud);
            sor.setInputCloud(cloud);
            sor.setMeanK(MeanK);
            sor.setStddevMulThresh(StddevMulThresh);
            sor.setNegative(false);
            sor.filter(*cloud_filtered);
            // Create the segmentation object for the planar model and set all the parameters
            pcl::SACSegmentation<pcl::PointXYZ> seg;
            pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
            pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZ>());
            seg.setOptimizeCoefficients(true);
            seg.setModelType(pcl::SACMODEL_PLANE);
            seg.setMethodType(pcl::SAC_RANSAC);
            seg.setMaxIterations(100);
            seg.setDistanceThreshold(0.02);
            int i = 0, nr_points = (int)cloud_filtered->size(), j = 0;

            while (cloud_filtered->size() > 0.3 * nr_points)
            {
                // Segment the largest planar component from the remaining cloud
                seg.setInputCloud(cloud_filtered);
                seg.segment(*inliers, *coefficients);
                if (inliers->indices.size() == 0)
                {
                    std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
                    break;
                }
                // Extract the planar inliers from the input cloud
                pcl::ExtractIndices<pcl::PointXYZ> extract;
                extract.setInputCloud(cloud_filtered);
                extract.setIndices(inliers);
                extract.setNegative(false);
                // Get the points associated with the planar surface
                extract.filter(*cloud_plane);
                std::cout << "PointCloud representing the planar component: " << cloud_plane->size() << " data points." << std::endl;
                *cloud_colored_pcl = *cloud_colored_pcl + planefind(cloud_plane, j, coefficients);
                // Remove the planar inliers, extract the rest
                extract.setNegative(true);
                extract.filter(*cloud_f);
                *cloud_filtered = *cloud_f;
                j++;
            }
            // Creating the KdTree object for the search method of the extraction
            pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
            tree->setInputCloud(cloud_filtered);
            std::vector<pcl::PointIndices> cluster_indices;
            pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
            ec.setClusterTolerance(0.02); // 2cm
            ec.setMinClusterSize(100);
            ec.setMaxClusterSize(25000);
            ec.setSearchMethod(tree);
            ec.setInputCloud(cloud_filtered);
            ec.extract(cluster_indices);

            for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
            {
                pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
                for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
                    cloud_cluster->push_back((*cloud_filtered)[*pit]);
                cloud_cluster->width = cloud_cluster->size();
                cloud_cluster->height = 1;
                cloud_cluster->is_dense = true;
                size_t size_cloud = cloud_cluster->size();
                *cloud_colored_pcl = *cloud_colored_pcl + shapefind(cloud_cluster, j);
                j++;
            }

            pcl::toROSMsg(*cloud_colored_pcl, cloud_colored_sensor);
            cloud_colored_sensor.header.frame_id = "pico_zense_depth_frame";
            std::string fields_list = pcl::getFieldsList(cloud_colored_sensor);
            std::cout << "Colored PointCloud before filtering has: " << cloud_colored_sensor.width << " data points."
                      << " points " << fields_list << "\" in frame \"" << cloud_colored_sensor.header.frame_id << std::endl;
            cloud_colored_sensor.header.stamp = ros::Time::now();
            pub_.publish(cloud_colored_sensor);
        }

        void
        dynReconfCallback(shape_finder::shape_finder_nodeletConfig &config, uint32_t level)
        {
            th = config.overlap_threshold;
            big_plane_size = config.big_plane_size;
            min_cloud_size = config.min_cloud_size;
            hv_tolerance = config.hv_tolerance * PI / 180;
            NormalDistanceWeightP = config.NormalDistanceWeightPlane;    //0.1
            MaxIterationsP = config.MaxIterationsPlane;                  //100
            DistanceThresholdP0 = config.DistanceThresholdPlane0;        //0.01
            DistanceThresholdP = config.DistanceThresholdPlane;          //0.001
            NormalDistanceWeightC = config.NormalDistanceWeightCyilnder; //0.1
            MaxIterationsC = config.MaxIterationsCylinder;               //10000
            DistanceThresholdC = config.DistanceThresholdCylinder;       // 0.1
            RadiusLimitsMinC = config.RadiusLimitsMinCylinder;           //0.0
            RadiusLimitsMaxC = config.RadiusLimitsMaxCylinder;           //0.5
            StddevMulThresh = config.StddevMulThresh;                    //1.0
            MeanK = config.MeanK;                                        //50
        }

        void
        gCallback(const geometry_msgs::Point &msg)
        {
            gravity[0] = msg.x;
            gravity[1] = msg.y;
            gravity[2] = msg.z;
            gravity.normalize();
        }

    public:
        virtual void onInit()
        {
            pub_.advertise(nh_, "objects", 1);
            normal_marker_pub = nh_.advertise<visualization_msgs::Marker>("normal_out", 1);
            sub_ = nh_.subscribe("point_cloud_in", 1, &ShapeFinderNodelet::cloudCallback, this);
            config_server_.setCallback(boost::bind(&ShapeFinderNodelet::dynReconfCallback, this, _1, _2));
            sub_g = nh_.subscribe("gravity_in", 1, &ShapeFinderNodelet::gCallback, this);
            ros::NodeHandle private_nh("~");
            private_nh.param("frame_id", tf_frame, std::string("pico_zense_depth_frame"));
        }
    };
} // namespace shape_finder
PLUGINLIB_EXPORT_CLASS(shape_finder::ShapeFinderNodelet, nodelet::Nodelet);