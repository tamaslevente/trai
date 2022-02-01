#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/search/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl/filters/extract_indices.h>

#include <vector>



#define PI 3.14159265

void Recursive_region_growing(pcl::PointCloud<pcl::PointXYZ>::Ptr &bin_cloud,
							  pcl::PointCloud<pcl::PointXYZ>::Ptr front_cloud,
							  int K,
							  float tau,
							  pcl::PointCloud<pcl::PointXYZ>::Ptr &pcd_conect_comp,
							  int iteration_number,
							  float &tau_final)
{

	if (front_cloud->size() != 0)
	{
		for (int i = 0; i < front_cloud->size(); i++)
		{

			float tau2;

			float comp_x;
			float comp_y;
			float comp_z;

			float dist;

			if (bin_cloud->size() != 0)
			{
				float dist_max_K_points = -561262;
				for (int j = 0; j < K; j++)
				{
					float dist_min = 12345634;

					int pos_front_point = -1;
					for (int t = 0; t < bin_cloud->size(); t++)
					{
						comp_x = front_cloud->points[i].x - bin_cloud->points[t].x;
						comp_y = front_cloud->points[i].y - bin_cloud->points[t].y;
						comp_z = front_cloud->points[i].z - bin_cloud->points[t].z;

						dist = sqrt(comp_x * comp_x + comp_y * comp_y + comp_z * comp_z);

						if ((dist_min > dist) && (dist < tau))
						{
							dist_min = dist;
							pos_front_point = t;
						}
					}

					if (dist_max_K_points < dist_min)
					{
						dist_max_K_points = dist_min;
					}

					pcl::PointCloud<pcl::PointXYZ>::Ptr front_cloud_2(new pcl::PointCloud<pcl::PointXYZ>);
					if (pos_front_point != -1)
					{

						pcl::PointXYZ moving_front_point;

						moving_front_point.x = bin_cloud->points[pos_front_point].x;
						moving_front_point.y = bin_cloud->points[pos_front_point].y;
						moving_front_point.z = bin_cloud->points[pos_front_point].z;

						front_cloud_2->points.push_back(moving_front_point);

						front_cloud_2->width = front_cloud_2->points.size();
						front_cloud_2->height = 1;
						front_cloud_2->points.resize(front_cloud_2->width * front_cloud_2->height);
						front_cloud_2->is_dense = false;

						//Remove points from front_cloud_2 from bin_cloud

						pcl::PointXYZ point_remaining;

						pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_point_remaining(new pcl::PointCloud<pcl::PointXYZ>);

						for (int iter_change_bin = 0; iter_change_bin < bin_cloud->size(); iter_change_bin++)
						{
							bool ok = 1;
							for (int iter_change_region = 0; iter_change_region < front_cloud_2->size(); iter_change_region++)
							{
								if ((bin_cloud->points[iter_change_bin].x == front_cloud_2->points[iter_change_region].x) && (bin_cloud->points[iter_change_bin].y == front_cloud_2->points[iter_change_region].y) && (bin_cloud->points[iter_change_bin].z == front_cloud_2->points[iter_change_region].z))
								{
									ok = 0;
								}
							}
							if (ok)
							{
								point_remaining.x = bin_cloud->points[iter_change_bin].x;
								point_remaining.y = bin_cloud->points[iter_change_bin].y;
								point_remaining.z = bin_cloud->points[iter_change_bin].z;

								cloud_point_remaining->points.push_back(point_remaining);
							}
							cloud_point_remaining->width = cloud_point_remaining->points.size();
							cloud_point_remaining->height = 1;
							cloud_point_remaining->points.resize(cloud_point_remaining->width * cloud_point_remaining->height);
							cloud_point_remaining->is_dense = false;
						}

						copyPointCloud(*cloud_point_remaining, *bin_cloud);
					}

					if (front_cloud_2->size() != 0)
					{
						if (dist_max_K_points > 0)
						{
							float alpha = dist_max_K_points;

							if (front_cloud_2->size() == K)
							{
								//std::cout<<"cloud_front size"<<front_cloud_2->size() <<" is equal to"<<K<<std::endl;

								tau2 = (alpha + iteration_number * tau) / (iteration_number + 1);

								//std::cout<<"("<<alpha<<"+"<<iteration_number<<"*"<<tau<<") / ("<<(iteration_number+1)<<") ="<<tau2<<std::endl;
							}
							else
							{
								//std::cout<<"cloud_front size"<<front_cloud_2->size() <<" is less than"<<K<<std::endl;
								tau2 = (2 * alpha + iteration_number * tau) / (iteration_number + 1);

								//std::cout<<"(2*"<<alpha<<"+"<<iteration_number<<"*"<<tau<<") / ("<<(iteration_number+1)<<") ="<<tau2<<std::endl;
							}

							if (tau2 > tau_final)
							{
								tau_final = tau2;
								//std::cout<<"Tau_nivel_urmator:"<<tau_final<<std::endl;
							}

							//Add validated front 2 cloud to region cloud
							for (int k = 0; k < front_cloud_2->size(); k++)
							{
								pcl::PointXYZ moving_front_point;

								moving_front_point.x = front_cloud_2->points[k].x;
								moving_front_point.y = front_cloud_2->points[k].y;
								moving_front_point.z = front_cloud_2->points[k].z;

								pcd_conect_comp->points.push_back(moving_front_point);

								pcd_conect_comp->width = pcd_conect_comp->points.size();
								pcd_conect_comp->height = 1;
								pcd_conect_comp->points.resize(pcd_conect_comp->width * pcd_conect_comp->height);
								pcd_conect_comp->is_dense = false;
							}

							int new_iteration = iteration_number + 1;

							if (bin_cloud->size() != 0)
							{
								Recursive_region_growing(bin_cloud, front_cloud_2, K, tau2, pcd_conect_comp, new_iteration, tau_final);
							}
						}
					}
				}
			}
		}
	}
}

int main()
{

	std::ifstream input_parameters("../config_files/config_Reeb.txt");




	

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>), cloud_f(new pcl::PointCloud<pcl::PointXYZ>);

	std::string input_cloud_name;
	std::string input_cloud_location;

	std::cout << "input_cloud location:";
	//std::cin >> input_cloud_location;
	std::getline(input_parameters, input_cloud_location);
	std::cout<<input_cloud_location<<std::endl;


	std::cout << "input_cloud name:";
	//std::cin >> cloud_selection;
	std::getline(input_parameters, input_cloud_name);
	std::cout<<input_cloud_name<<std::endl;

	std::stringstream ss;

	//ss << "/home/alex-pop/Desktop/Doctorat/Side_projects/trai/trai/PC-NBV/Alex_pop_work/data_in/" << cloud_selection;
	ss<<input_cloud_location<<input_cloud_name;


	//if (pcl::io::loadPCDFile<pcl::PointXYZ> ("/home/alex-pop/Desktop/BAckups/Alex_pop_work/data_in/test_single.pcd", *cloud) == -1) //* load the file
	//if (pcl::io::loadPCDFile<pcl::PointXYZ>("/home/alex-pop/Desktop/Doctorat/Side_projects/trai/trai/PC-NBV/Alex_pop_work/data_in/chair_0983.pcd", *cloud) == -1) //* load the file
	if (pcl::io::loadPCDFile<pcl::PointXYZ>(ss.str(), *cloud) == -1) //* load the file
	{
		PCL_ERROR("Couldn't read file test_pcd.pcd \n");
		return (-1);
	}
	std::cout << "Loaded "
			  << cloud->width * cloud->height
			  << " data points from test_pcd.pcd with the following fields: "
			  << std::endl;

	pcl::VoxelGrid<pcl::PointXYZ> vg;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

	///////////////////////////////////////////////////////////////////////////////////////////////////////
	////////////////Selecting if user wants floor removal or not

	bool choice_floor;

	std::cout << "Remove floor?(0/1)" << std::endl;
	//std::cin >> choice_floor;

	input_parameters>>choice_floor;

	if (choice_floor == 1)
	{
		std::cout<<"Removing floor"<<std::endl;
		// Create the filtering object: downsample the dataset using a leaf size of 1cm

		vg.setInputCloud(cloud);
		vg.setLeafSize(0.01f, 0.01f, 0.01f);
		vg.filter(*cloud_filtered);
		std::cout << "PointCloud after filtering has: " << cloud_filtered->size() << " data points." << std::endl; //*

		// Create the segmentation object for the planar model and set all the parameters
		pcl::SACSegmentation<pcl::PointXYZ> seg;
		pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
		pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZ>());
		pcl::PCDWriter writer;
		seg.setOptimizeCoefficients(true);
		seg.setModelType(pcl::SACMODEL_PLANE);
		seg.setMethodType(pcl::SAC_RANSAC);
		seg.setMaxIterations(100);
		seg.setDistanceThreshold(0.02);

		int nr_points = (int)cloud_filtered->size();
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

			// Remove the planar inliers, extract the rest
			extract.setNegative(true);
			extract.filter(*cloud_f);
			*cloud_filtered = *cloud_f;
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
	}

	else
	{
		std::cout<<"No floor removal"<<std::endl;
		copyPointCloud(*cloud, *cloud_filtered);
	}

	pcl::io::savePCDFileASCII("test_single_out.pcd", *cloud_filtered);
	std::cerr << "Saved data points to test_single_out.pcd." << std::endl;

	//////////////////////////////////////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////////////////////////
	//Computing the centroid of the point cloud

	float z_centroid = 0;
	float y_centroid = 0;
	float x_centroid = 0;

	pcl::PointXYZ cloud_point;

	for (int i = 0; i < cloud_filtered->size(); i++)
	{
		cloud_point.x = cloud_filtered->points[i].x;
		x_centroid = x_centroid + cloud_point.x;

		cloud_point.y = cloud_filtered->points[i].y;
		y_centroid = y_centroid + cloud_point.y;

		cloud_point.z = cloud_filtered->points[i].z;
		z_centroid = z_centroid + cloud_point.z;
	}

	//std::cout << "x_centroid=" << x_centroid << std::endl;
	//std::cout << "y_centroid=" << y_centroid << std::endl;
	//std::cout << "z_centroid=" << z_centroid << std::endl;
	//std::cout << "cloud_size=" << cloud_filtered->size() << std::endl;

	x_centroid = x_centroid / cloud_filtered->size();
	y_centroid = y_centroid / cloud_filtered->size();
	z_centroid = z_centroid / cloud_filtered->size();

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_centroid(new pcl::PointCloud<pcl::PointXYZ>);
	cloud_point.x = x_centroid;
	cloud_point.y = y_centroid;
	cloud_point.z = z_centroid;
	cloud_centroid->points.push_back(cloud_point);

	cloud_centroid->width = cloud_centroid->points.size();
	cloud_centroid->height = 1;
	cloud_centroid->points.resize(cloud_centroid->width * cloud_centroid->height);
	cloud_centroid->is_dense = false;

	pcl::io::savePCDFileASCII("test_centroid.pcd", *cloud_centroid);
	//std::cerr << "Saved centroid to test_centroid.pcd." << std::endl;

	////////////////////////////////////////////////////////////////////////
	/////////////////////////////////////////////////
	//Computing all the distances to the centroid in the point cloud

	float Distances[1000000];

	float comp_x;
	float comp_y;
	float comp_z;

	float dist;

	float maximum_distance = -345678;
	float minimum_distance = 123546778;

	for (int i = 0; i < cloud_filtered->size(); i++)
	{

		comp_x = x_centroid - cloud_filtered->points[i].x;
		comp_y = y_centroid - cloud_filtered->points[i].y;
		comp_z = z_centroid - cloud_filtered->points[i].z;

		dist = sqrt(comp_x * comp_x + comp_y * comp_y + comp_z * comp_z);

		if (maximum_distance < dist)
		{
			maximum_distance = dist;
		}

		if (minimum_distance > dist)
		{
			minimum_distance = dist;
		}

		Distances[i] = dist;
	}

	std::cout << "minimum_distance=" << minimum_distance << std::endl;
	std::cout << "maximum_distance=" << maximum_distance << std::endl;

	////////////////////////////////////////////////////////////

	int Nr_bins;
	int Nr_neighbors;

	std::cout << "Nr_bins=";
	//std::cin >> Nr_bins;
	input_parameters>>Nr_bins;

	std::cout << Nr_bins << std::endl;

	std::cout << "Nr_neighbors=";
	//std::cin >> Nr_neighbors;
	input_parameters>>Nr_neighbors;

	std::cout << Nr_neighbors << std::endl;

	

	

	float lim_inf;
	float lim_sup;

	float Bin_intervals[2][500];

	pcl::PointCloud<pcl::PointXYZ>::Ptr Array_bin_clouds[4000];

	for (int j = 1; j <= Nr_bins; j++)
	{

		lim_inf = minimum_distance + (float((j - 1))) / (float(Nr_bins)) * (maximum_distance - minimum_distance);
		lim_sup = minimum_distance + (float(j)) / (float(Nr_bins)) * (maximum_distance - minimum_distance);

		Bin_intervals[0][j - 1] = lim_inf;
		Bin_intervals[1][j - 1] = lim_sup;
	}

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_conex_parts(new pcl::PointCloud<pcl::PointXYZ>);

	float dist_min_connect_points;
	float dist_min_connect_parts = -977235;

	std::cout << "Dist min connex points=";
	//std::cin >> dist_min_connect_points;
	input_parameters>>dist_min_connect_points;
	std::cout << dist_min_connect_points << std::endl;

	// std::cout << "Dist min connected parts=";
	// std::cin >> dist_min_connect_parts;

	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_copy(new pcl::PointCloud<pcl::PointXYZ>);
	//copyPointCloud(*cloud_filtered, *cloud_filtered_copy);

	pcl::PointXYZ point_Reeb;

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_Reeb(new pcl::PointCloud<pcl::PointXYZ>);

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_Reeb_final(new pcl::PointCloud<pcl::PointXYZ>);

	int nr_connected_parts = 0;

	int sum_check_bins = 0;

	int Array_nr_components_in_region[Nr_bins + 1];

	for (int i = 0; i < Nr_bins + 1; i++)
	{
		Array_nr_components_in_region[i] = 0;
	}

	//std::cout << "Nr points in point cloud " << cloud_filtered->size() << std::endl;

	for (int i = 0; i < Nr_bins; i++)
	{
		Array_nr_components_in_region[i] = nr_connected_parts;

		////////////////////////////////////////////////////////
		//For each bin we compute the point cloud

		//std::cout << "starting bin " << (i + 1) << std::endl;

		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_bin_actual(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointXYZ cloud_point_bin;

		for (int j = 0; j < cloud_filtered->size(); j++)
		{
			if ((Distances[j] >= Bin_intervals[0][i]) && (Distances[j] < Bin_intervals[1][i]))
			{
				cloud_point_bin.x = cloud_filtered->points[j].x;
				cloud_point_bin.y = cloud_filtered->points[j].y;
				cloud_point_bin.z = cloud_filtered->points[j].z;

				cloud_bin_actual->points.push_back(cloud_point_bin);

				//std::cout<<Distances[j]<<" >= "<<Bin_intervals[0][i]<<" and "<<Distances[j]<<" < "<<Bin_intervals[0][i]<<std::endl;
			}

			cloud_bin_actual->width = cloud_bin_actual->points.size();
			cloud_bin_actual->height = 1;
			cloud_bin_actual->points.resize(cloud_bin_actual->width * cloud_bin_actual->height);
			cloud_bin_actual->is_dense = false;
		}

		//std::cout << "Nr points in bin is " << cloud_bin_actual->size() << std::endl;

		int nr_remaining_elements = cloud_bin_actual->points.size();

		int random_point;

		while (cloud_bin_actual->size() > 0)
		{

			random_point = rand() % cloud_bin_actual->size();

			//std::cout<<"New random point="<<random_point<<std::endl;

			pcl::PointXYZ moving_random_point;

			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_random_point_bin(new pcl::PointCloud<pcl::PointXYZ>);
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_front(new pcl::PointCloud<pcl::PointXYZ>);

			// Adding the Reeb points

			point_Reeb.x = cloud_bin_actual->points[random_point].x;
			point_Reeb.y = cloud_bin_actual->points[random_point].y;
			point_Reeb.z = cloud_bin_actual->points[random_point].z;

			cloud_Reeb->points.push_back(point_Reeb);

			cloud_Reeb->width = cloud_Reeb->points.size();
			cloud_Reeb->height = 1;
			cloud_Reeb->points.resize(cloud_Reeb->width * cloud_Reeb->height);
			cloud_Reeb->is_dense = false;

			//Must change to centroid of region ////////////////TO DO

			//////////////////////////////

			cloud_front->points.push_back(point_Reeb);

			cloud_front->width = cloud_Reeb->points.size();
			cloud_front->height = 1;
			cloud_front->points.resize(cloud_Reeb->width * cloud_Reeb->height);
			cloud_front->is_dense = false;
			///////////////////////////////////////////

			//Add chosen point to region point cloud
			moving_random_point.x = cloud_bin_actual->points[random_point].x;
			moving_random_point.y = cloud_bin_actual->points[random_point].y;
			moving_random_point.z = cloud_bin_actual->points[random_point].z;

			cloud_random_point_bin->points.push_back(moving_random_point);

			cloud_random_point_bin->width = cloud_random_point_bin->points.size();
			cloud_random_point_bin->height = 1;
			cloud_random_point_bin->points.resize(cloud_random_point_bin->width * cloud_random_point_bin->height);
			cloud_random_point_bin->is_dense = false;

			bool find_points = 1;

			////////////////////////////////////////
			//Remove point from point cloud

			pcl::PointXYZ point_remaining;

			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_point_remaining(new pcl::PointCloud<pcl::PointXYZ>);

			for (int iter_change_bin = 0; iter_change_bin < cloud_bin_actual->size(); iter_change_bin++)
			{
				bool ok = 1;
				for (int iter_change_region = 0; iter_change_region < cloud_random_point_bin->size(); iter_change_region++)
				{
					if ((cloud_bin_actual->points[iter_change_bin].x == cloud_random_point_bin->points[iter_change_region].x) && (cloud_bin_actual->points[iter_change_bin].y == cloud_random_point_bin->points[iter_change_region].y) && (cloud_bin_actual->points[iter_change_bin].z == cloud_random_point_bin->points[iter_change_region].z))
					{
						ok = 0;
					}
				}
				if (ok)
				{
					point_remaining.x = cloud_bin_actual->points[iter_change_bin].x;
					point_remaining.y = cloud_bin_actual->points[iter_change_bin].y;
					point_remaining.z = cloud_bin_actual->points[iter_change_bin].z;

					cloud_point_remaining->points.push_back(point_remaining);
				}
				cloud_point_remaining->width = cloud_point_remaining->points.size();
				cloud_point_remaining->height = 1;
				cloud_point_remaining->points.resize(cloud_point_remaining->width * cloud_point_remaining->height);
				cloud_point_remaining->is_dense = false;
			}

			copyPointCloud(*cloud_point_remaining, *cloud_bin_actual);

			///////////////////////////////////////

			///Region growing using centroid
			//Must change to make the new region growing

			// for (int j=0;j<cloud_bin_actual->size();j++)
			// {

			// 		float comp_x_bin_element=cloud_bin_actual->points[random_point].x-cloud_bin_actual->points[j].x;
			// 		float comp_y_bin_element=cloud_bin_actual->points[random_point].y-cloud_bin_actual->points[j].y;
			// 		float comp_z_bin_element=cloud_bin_actual->points[random_point].z-cloud_bin_actual->points[j].z;

			// 		float dist_connected_points=sqrt(comp_x_bin_element*comp_x_bin_element + comp_y_bin_element*comp_y_bin_element + comp_z_bin_element*comp_z_bin_element);

			// 		if( dist_connected_points< dist_min_connect_points){

			// 			moving_random_point.x=cloud_bin_actual->points[j].x;
			// 			moving_random_point.y=cloud_bin_actual->points[j].y;
			// 			moving_random_point.z=cloud_bin_actual->points[j].z;

			// 			cloud_random_point_bin->points.push_back(moving_random_point);
			// 			}

			// 		cloud_random_point_bin->width = cloud_random_point_bin->points.size();
			// 		cloud_random_point_bin->height = 1;
			// 		cloud_random_point_bin->points.resize(cloud_random_point_bin->width * cloud_random_point_bin->height);
			// 		cloud_random_point_bin->is_dense = false;

			// }

			/////////////////////////////////////////

			//std::cout << "Size of bin is " << cloud_bin_actual->size() << std::endl;
			//std::cout << "Added initial point to region, size is now " << cloud_random_point_bin->size() << std::endl;

			//////////////////////////////
			//Region growing using fixed distance
			//Must change to use recursive region growing

			// while (find_points)
			// {
			// 	find_points = 0;

			// 	int nr_points = cloud_random_point_bin->size();

			// 	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_new_points_region(new pcl::PointCloud<pcl::PointXYZ>);

			// 	for (int iterate_bin = 0; iterate_bin < cloud_bin_actual->size(); iterate_bin++)
			// 	{

			// 		for (int iterate_region = 0; iterate_region < cloud_random_point_bin->size(); iterate_region++)
			// 		{
			// 			float comp_x_bin_element = cloud_random_point_bin->points[iterate_region].x - cloud_bin_actual->points[iterate_bin].x;
			// 			float comp_y_bin_element = cloud_random_point_bin->points[iterate_region].y - cloud_bin_actual->points[iterate_bin].y;
			// 			float comp_z_bin_element = cloud_random_point_bin->points[iterate_region].z - cloud_bin_actual->points[iterate_bin].z;

			// 			float dist_connected_points = sqrt(comp_x_bin_element * comp_x_bin_element + comp_y_bin_element * comp_y_bin_element + comp_z_bin_element * comp_z_bin_element);

			// 			if ((dist_connected_points < dist_min_connect_points) && (dist_connected_points != 0))
			// 			{

			// 				//std::cout<<"Found connected point"<<std::endl;

			// 				find_points = 1;
			// 			}
			// 		}
			// 		if (find_points)
			// 		{
			// 			moving_random_point.x = cloud_bin_actual->points[iterate_bin].x;
			// 			moving_random_point.y = cloud_bin_actual->points[iterate_bin].y;
			// 			moving_random_point.z = cloud_bin_actual->points[iterate_bin].z;

			// 			cloud_new_points_region->points.push_back(moving_random_point);

			// 			cloud_new_points_region->width = cloud_new_points_region->points.size();
			// 			cloud_new_points_region->height = 1;
			// 			cloud_new_points_region->points.resize(cloud_new_points_region->width * cloud_new_points_region->height);
			// 			cloud_new_points_region->is_dense = false;
			// 		}
			// 	}

			// 	*cloud_random_point_bin += *cloud_new_points_region;

			// 	cloud_random_point_bin->width = cloud_random_point_bin->points.size();
			// 	cloud_random_point_bin->height = 1;
			// 	cloud_random_point_bin->points.resize(cloud_random_point_bin->width * cloud_random_point_bin->height);
			// 	cloud_random_point_bin->is_dense = false;

			// 	//std::cout << "Dimension of new region is " << cloud_random_point_bin->size() << std::endl;

			// 	if (find_points)
			// 	{

			// 		pcl::PointXYZ point_remaining;

			// 		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_point_remaining(new pcl::PointCloud<pcl::PointXYZ>);

			// 		for (int iter_change_bin = 0; iter_change_bin < cloud_bin_actual->size(); iter_change_bin++)
			// 		{
			// 			bool ok = 1;
			// 			for (int iter_change_region = 0; iter_change_region < cloud_random_point_bin->size(); iter_change_region++)
			// 			{
			// 				if ((cloud_bin_actual->points[iter_change_bin].x == cloud_random_point_bin->points[iter_change_region].x) && (cloud_bin_actual->points[iter_change_bin].y == cloud_random_point_bin->points[iter_change_region].y) && (cloud_bin_actual->points[iter_change_bin].z == cloud_random_point_bin->points[iter_change_region].z))
			// 				{
			// 					ok = 0;
			// 				}
			// 			}
			// 			if (ok)
			// 			{
			// 				point_remaining.x = cloud_bin_actual->points[iter_change_bin].x;
			// 				point_remaining.y = cloud_bin_actual->points[iter_change_bin].y;
			// 				point_remaining.z = cloud_bin_actual->points[iter_change_bin].z;

			// 				cloud_point_remaining->points.push_back(point_remaining);
			// 			}
			// 			cloud_point_remaining->width = cloud_point_remaining->points.size();
			// 			cloud_point_remaining->height = 1;
			// 			cloud_point_remaining->points.resize(cloud_point_remaining->width * cloud_point_remaining->height);
			// 			cloud_point_remaining->is_dense = false;
			// 		}

			// 		copyPointCloud(*cloud_point_remaining, *cloud_bin_actual);

			// 		//std::cout << "Nr_points_remaining in bin:" << cloud_bin_actual->size() << std::endl;
			// 	}
			// }

			////////////////////////////////
			//////////////////////////////////
			//////////////////////////////////////////////////////////////////////////////////////////////////
			int nr_points = cloud_random_point_bin->size();

			float dist_min_region = -612327;

			Recursive_region_growing(cloud_bin_actual, cloud_front, Nr_neighbors, dist_min_connect_points, cloud_random_point_bin, 1, dist_min_region);

			//std::cout<<"Distance in final iteration="<<dist_min_region<<std::endl;

			if (dist_min_region > dist_min_connect_parts)
			{
				dist_min_connect_parts = dist_min_region;
			}

			//std::cout << "Dimension of new region is " << cloud_random_point_bin->size() << std::endl;

			///////////////////////////////////////////////////////////////////////////////////////////

			if (cloud_random_point_bin->size() != 0)
			{

				Array_bin_clouds[nr_connected_parts] = cloud_random_point_bin;
				nr_connected_parts++;
			}

			//std::cout<<"Nr_points_close_to_random:"<<cloud_random_point_bin->size()<<std::endl;

			sum_check_bins = sum_check_bins + cloud_random_point_bin->size();

			//std::cout<<"Nr_points_remaining:"<<cloud_bin_actual->size()<<std::endl;
		}
		//std::cout << "Nr_points safety check bins:" << sum_check_bins << std::endl;
		//std::cout << "Nr points in point cloud " << cloud_filtered->size() << std::endl;

		std::cout << "Nr of components in region=" << Array_nr_components_in_region[i] << std::endl;
	}
	Array_nr_components_in_region[Nr_bins + 1] = nr_connected_parts;
	std::cout << "Chosen distance between connected components:" << dist_min_connect_parts * 2 << std::endl;

	float Weight[500][500];

	for (int i = 0; i < 500; i++)
	{
		for (int j = 0; j < 500; j++)
		{
			Weight[i][j] = 0;
		}
	}

	//Minimum Distance between connected components is 2 times the minimum distance for connected component points

	//Need to change here to compare only two adject regions
	//Here it compares all connected components

	// for (int iterate_colection = 0; iterate_colection < nr_connected_parts - 1; iterate_colection++)
	// {

	// 	for (int iterate_colection_2 = iterate_colection + 1; iterate_colection_2 < nr_connected_parts; iterate_colection_2++)
	// 	{

	// 		float dist_minimum = 123425;
	// 		bool is_connected = 0;

	// 		for (int iterate_pcd = 0; iterate_pcd < Array_bin_clouds[iterate_colection]->size(); iterate_pcd++)
	// 		{
	// 			for (int iterate_pcd_2 = 0; iterate_pcd_2 < Array_bin_clouds[iterate_colection_2]->size(); iterate_pcd_2++)
	// 			{
	// 				float comp_x_bin_element = Array_bin_clouds[iterate_colection]->points[iterate_pcd].x - Array_bin_clouds[iterate_colection_2]->points[iterate_pcd].x;
	// 				float comp_y_bin_element = Array_bin_clouds[iterate_colection]->points[iterate_pcd].y - Array_bin_clouds[iterate_colection_2]->points[iterate_pcd].y;
	// 				float comp_z_bin_element = Array_bin_clouds[iterate_colection]->points[iterate_pcd].z - Array_bin_clouds[iterate_colection_2]->points[iterate_pcd].z;

	// 				float dist_colection = sqrt(comp_x_bin_element * comp_x_bin_element + comp_y_bin_element * comp_y_bin_element + comp_z_bin_element * comp_z_bin_element);

	// 				if (dist_minimum > dist_colection)
	// 				{
	// 					dist_minimum = dist_colection;
	// 				}

	// 				////////Dist min for collections is double dist min for connect points

	// 				if (dist_minimum < dist_min_connect_parts*2)
	// 				{
	// 					is_connected = 1;
	// 				}
	// 			}

	// 			//std::cout << "Dist minimum between component " << iterate_colection << " and component " << iterate_colection_2 << " is " << dist_minimum << std::endl;
	// 		}

	// 		if (is_connected)
	// 		{
	// 			Weight[iterate_colection][iterate_colection_2] = 1;
	// 			Weight[iterate_colection_2][iterate_colection] = 1;
	// 		}
	// 	}
	// }

	///////////////////////////////////////////////////////////////////////////////
	//Compares connected components in same region

	std::cout << "Collections from same regions" << std::endl;

	for (int iterate_colection = 0; iterate_colection < Nr_bins; iterate_colection++)
	{
		for (int iterate_pcd = Array_nr_components_in_region[iterate_colection]; iterate_pcd < Array_nr_components_in_region[iterate_colection + 1] - 1; iterate_pcd++)
		{
			for (int iterate_pcd_2 = iterate_pcd + 1; iterate_pcd_2 < Array_nr_components_in_region[iterate_colection + 1]; iterate_pcd_2++)
			{

				float dist_minimum = 123425;
				bool is_connected = 0;

				for (int iterate_pcd_points = 0; iterate_pcd_points < Array_bin_clouds[iterate_pcd]->size(); iterate_pcd_points++)
				{
					for (int iterate_pcd_points_2 = 0; iterate_pcd_points_2 < Array_bin_clouds[iterate_pcd_2]->size(); iterate_pcd_points_2++)
					{
						float comp_x_bin_element = Array_bin_clouds[iterate_colection]->points[iterate_pcd].x - Array_bin_clouds[iterate_colection]->points[iterate_pcd].x;
						float comp_y_bin_element = Array_bin_clouds[iterate_colection]->points[iterate_pcd].y - Array_bin_clouds[iterate_colection]->points[iterate_pcd].y;
						float comp_z_bin_element = Array_bin_clouds[iterate_colection]->points[iterate_pcd].z - Array_bin_clouds[iterate_colection]->points[iterate_pcd].z;

						float dist_colection = sqrt(comp_x_bin_element * comp_x_bin_element + comp_y_bin_element * comp_y_bin_element + comp_z_bin_element * comp_z_bin_element);

						if (dist_minimum > dist_colection)
						{
							dist_minimum = dist_colection;
						}

						////////Dist min for collections is double dist min for connect points

						if (dist_minimum < dist_min_connect_parts * 2)
						{
							is_connected = 1;
						}
					}
				}

				if (is_connected)
				{
					Weight[iterate_pcd][iterate_pcd_2] = 1;
					Weight[iterate_pcd_2][iterate_pcd] = 1;

					std::cout << iterate_pcd << " " << iterate_pcd_2 << std::endl;
				}
			}
		}
	}

	//Compares connected components in different regions

	std::cout << "Collections from consequent regions" << std::endl;

	for (int iterate_colection = 0; iterate_colection < Nr_bins - 1; iterate_colection++)
	{
		for (int iterate_pcd = Array_nr_components_in_region[iterate_colection]; iterate_pcd < Array_nr_components_in_region[iterate_colection + 1] - 1; iterate_pcd++)
		{
			for (int iterate_pcd_2 = Array_nr_components_in_region[iterate_colection + 1]; iterate_pcd_2 < Array_nr_components_in_region[iterate_colection + 2]; iterate_pcd_2++)
			{

				float dist_minimum = 123425;
				bool is_connected = 0;

				for (int iterate_pcd_points = 0; iterate_pcd_points < Array_bin_clouds[iterate_pcd]->size(); iterate_pcd_points++)
				{
					for (int iterate_pcd_points_2 = 0; iterate_pcd_points_2 < Array_bin_clouds[iterate_pcd_2]->size(); iterate_pcd_points_2++)
					{
						float comp_x_bin_element = Array_bin_clouds[iterate_colection]->points[iterate_pcd].x - Array_bin_clouds[iterate_colection]->points[iterate_pcd].x;
						float comp_y_bin_element = Array_bin_clouds[iterate_colection]->points[iterate_pcd].y - Array_bin_clouds[iterate_colection]->points[iterate_pcd].y;
						float comp_z_bin_element = Array_bin_clouds[iterate_colection]->points[iterate_pcd].z - Array_bin_clouds[iterate_colection]->points[iterate_pcd].z;

						float dist_colection = sqrt(comp_x_bin_element * comp_x_bin_element + comp_y_bin_element * comp_y_bin_element + comp_z_bin_element * comp_z_bin_element);

						if (dist_minimum > dist_colection)
						{
							dist_minimum = dist_colection;
						}

						////////Dist min for collections is double dist min for connect points

						if (dist_minimum < dist_min_connect_parts * 2)
						{
							is_connected = 1;
						}
					}
				}

				if (is_connected)
				{
					Weight[iterate_pcd][iterate_pcd_2] = 1;
					Weight[iterate_pcd_2][iterate_pcd] = 1;

					std::cout << iterate_pcd << " " << iterate_pcd_2 << std::endl;
				}
			}
		}
	}

	/////
	for (int i = 0; i < nr_connected_parts; i++)
	{
		bool element_check = 0;
		for (int j = 0; (j < nr_connected_parts) && (element_check == 0); j++)
		{
			if (Weight[i][j] != 0)
			{
				element_check = 1;
			}
		}
		if (element_check)
		{

			pcl::PointXYZ point_Reeb_final;
			point_Reeb_final.x = cloud_Reeb->points[i].x;
			point_Reeb_final.y = cloud_Reeb->points[i].y;
			point_Reeb_final.z = cloud_Reeb->points[i].z;

			cloud_Reeb_final->points.push_back(point_Reeb_final);

			cloud_Reeb_final->width = cloud_Reeb_final->points.size();
			cloud_Reeb_final->height = 1;
			cloud_Reeb_final->points.resize(cloud_Reeb_final->width * cloud_Reeb_final->height);
			cloud_Reeb_final->is_dense = false;
		}
	}

	std::cout << "cloud Reeb final has " << cloud_Reeb_final->size() << " points" << std::endl;

	/////////////////////////////////////////////
	


	
	std::string output_folder_location;
	std::string output_Reeb_pcd_name;
	std::string output_inc_matrix_name;
	

	std::cout << "Output folder location:";
	//std::cin >> input_cloud_location;
	std::getline(input_parameters, output_folder_location);
	std::cout<<output_folder_location<<std::endl;


	std::cout << "Reeb pcd name:";
	//std::cin >> cloud_selection;
	std::getline(input_parameters, output_Reeb_pcd_name);
	std::cout<<output_Reeb_pcd_name<<std::endl;

	std::cout << "Incidence matrix file name:";
	//std::cin >> cloud_selection;
	std::getline(input_parameters, output_inc_matrix_name);
	std::cout<<output_inc_matrix_name<<std::endl;

	std::stringstream ss_out_pcd;
	std::stringstream ss_out_pcd_final;

	std::stringstream ss_out_matrix;

	
	ss_out_pcd<<output_folder_location<<output_Reeb_pcd_name<<".pcd";
	ss_out_pcd_final<<output_folder_location<<output_Reeb_pcd_name<<"_final"<<".pcd";
	ss_out_matrix<<output_folder_location<<output_inc_matrix_name;
	
	std::ofstream incid_matrix(ss_out_matrix.str());

	incid_matrix<<nr_connected_parts<<"\n";

	for (int i = 0; i < nr_connected_parts; i++)
	{
		for (int j = 0; j < nr_connected_parts; j++)
		{
			std::cout << Weight[i][j] << " ";
			incid_matrix << Weight[i][j] << " ";
		}
		incid_matrix <<"\n";
		std::cout << std::endl;
	}
	incid_matrix.close();

	std::cout<<"Incidence matrix written to file"<<std::endl;

	std::cout << "Number of connected parts:" << nr_connected_parts << std::endl;

	// for(int i=0;i<nr_connected_parts;i++){
	// 	std::cout<<"Connected part "<<i<<""<<Array_bin_clouds[i]->size()<<std::endl;
	// }

	pcl::io::savePCDFileASCII(ss_out_pcd.str(), *cloud_Reeb);
	std::cerr << "Saved Reeb pcd" << std::endl;

	pcl::io::savePCDFileASCII(ss_out_pcd_final.str(), *cloud_Reeb_final);
	std::cerr << "Saved Reeb pcd final" << std::endl;

	return (0);
}
