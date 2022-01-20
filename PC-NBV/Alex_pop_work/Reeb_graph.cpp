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








int
main ()
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>),cloud_f (new pcl::PointCloud<pcl::PointXYZ>);

  //if (pcl::io::loadPCDFile<pcl::PointXYZ> ("/home/alex-pop/Desktop/BAckups/Alex_pop_work/data_in/test_single.pcd", *cloud) == -1) //* load the file
  if (pcl::io::loadPCDFile<pcl::PointXYZ> ("/home/alex-pop/Desktop/Doctorat/Side_projects/trai/trai/PC-NBV/Alex_pop_work/data_in/chair_0983.pcd", *cloud) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
    return (-1);
  }
  std::cout << "Loaded "
            << cloud->width * cloud->height
            << " data points from test_pcd.pcd with the following fields: "
            << std::endl;

	pcl::VoxelGrid<pcl::PointXYZ> vg;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
	
	
	///////////////////////////////////////////////////////////////////////////////////////////////////////
   ////////////////Selecting if user wants floor removal or not

	
	
	bool choice_floor;

	std::cout<<"Remove floor?(0/1)"<<std::endl;
	std::cin>>choice_floor;

	

	if(choice_floor==1)
	{
		// Create the filtering object: downsample the dataset using a leaf size of 1cm
   
   vg.setInputCloud (cloud);
   vg.setLeafSize (0.01f, 0.01f, 0.01f);
   vg.filter (*cloud_filtered);
  std::cout << "PointCloud after filtering has: " << cloud_filtered->size ()  << " data points." << std::endl; //*

  // Create the segmentation object for the planar model and set all the parameters
   pcl::SACSegmentation<pcl::PointXYZ> seg;
   pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
   pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ> ());
   pcl::PCDWriter writer;
   seg.setOptimizeCoefficients (true);
   seg.setModelType (pcl::SACMODEL_PLANE);
   seg.setMethodType (pcl::SAC_RANSAC);
   seg.setMaxIterations (100);
   seg.setDistanceThreshold (0.02);
 
   int nr_points = (int) cloud_filtered->size ();
   while (cloud_filtered->size () > 0.3 * nr_points)
   {
     // Segment the largest planar component from the remaining cloud
     seg.setInputCloud (cloud_filtered);
     seg.segment (*inliers, *coefficients);
     if (inliers->indices.size () == 0)
     {
       std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
       break;
     }
 
     // Extract the planar inliers from the input cloud
     pcl::ExtractIndices<pcl::PointXYZ> extract;
     extract.setInputCloud (cloud_filtered);
     extract.setIndices (inliers);
     extract.setNegative (false);
 
     // Get the points associated with the planar surface
     extract.filter (*cloud_plane);
     std::cout << "PointCloud representing the planar component: " << cloud_plane->size () << " data points." << std::endl;
 
     // Remove the planar inliers, extract the rest
     extract.setNegative (true);
     extract.filter (*cloud_f);
     *cloud_filtered = *cloud_f;
   }
 
   // Creating the KdTree object for the search method of the extraction
   pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
   tree->setInputCloud (cloud_filtered);
 
   std::vector<pcl::PointIndices> cluster_indices;
   pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
   ec.setClusterTolerance (0.02); // 2cm
   ec.setMinClusterSize (100);
   ec.setMaxClusterSize (25000);
   ec.setSearchMethod (tree);
   ec.setInputCloud (cloud_filtered);
   ec.extract (cluster_indices);

	}

	else{
		copyPointCloud(*cloud, *cloud_filtered);
	}


	pcl::io::savePCDFileASCII ("test_single_out.pcd", *cloud_filtered);
	std::cerr << "Saved data points to test_single_out.pcd." << std::endl;

	//////////////////////////////////////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////////////////////////
	//Computing the centroid of the point cloud
	
	float z_centroid=0;
	float y_centroid=0;
	float x_centroid=0;
	
	pcl::PointXYZ cloud_point; 
	
	
	for (int i = 0; i < cloud_filtered->size(); i++)
    {
      cloud_point.x = cloud_filtered->points[i].x;
      x_centroid=x_centroid+cloud_point.x;
      
      cloud_point.y = cloud_filtered->points[i].y;
      y_centroid=y_centroid+cloud_point.y;
      
      cloud_point.z = cloud_filtered->points[i].z;
      z_centroid=z_centroid+cloud_point.z;
       
    }     
    
    std::cout<<"x_centroid="<<x_centroid<<std::endl;
     std::cout<<"y_centroid="<<y_centroid<<std::endl;
      std::cout<<"z_centroid="<<z_centroid<<std::endl;
       std::cout<<"cloud_size="<<cloud_filtered->size()<<std::endl;
	
	x_centroid=x_centroid/cloud_filtered->size();
	y_centroid=y_centroid/cloud_filtered->size();
	z_centroid=z_centroid/cloud_filtered->size();
	
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_centroid(new pcl::PointCloud<pcl::PointXYZ>);
		cloud_point.x = x_centroid;
		cloud_point.y = y_centroid;
		cloud_point.z = z_centroid;
	cloud_centroid->points.push_back(cloud_point);
	
	cloud_centroid->width = cloud_centroid->points.size();
    cloud_centroid->height = 1;
    cloud_centroid->points.resize(cloud_centroid->width * cloud_centroid->height);
    cloud_centroid->is_dense = false;
	
	pcl::io::savePCDFileASCII ("test_centroid.pcd", *cloud_centroid);
	std::cerr << "Saved centroid to test_centroid.pcd." << std::endl;

	////////////////////////////////////////////////////////////////////////
	/////////////////////////////////////////////////
	//Computing all the distances to the centroid in the point cloud
	
	float Distances[1000000];
	
	float comp_x;
	float comp_y;
	float comp_z;
	
	float dist;
	
	float maximum_distance=-345678;
	float minimum_distance= 123546778;
	
	
	for (int i = 0; i < cloud_filtered->size(); i++)
    {
		
		
      comp_x=x_centroid-cloud_filtered->points[i].x;
      comp_y=y_centroid-cloud_filtered->points[i].y;
      comp_z=z_centroid-cloud_filtered->points[i].z;
      
      dist=sqrt(comp_x*comp_x + comp_y*comp_y+comp_z*comp_z);
      
      if (maximum_distance<dist){
		  maximum_distance=dist;
		  }
		  
	   if (minimum_distance>dist){
		  minimum_distance=dist;
		  }  
      
      Distances[i]=dist;
      
       
    }     
    
    std::cout<<"minimum_distance="<<minimum_distance<<std::endl;
	std::cout<<"maximum_distance="<<maximum_distance<<std::endl;

	////////////////////////////////////////////////////////////
    
	
	int Nr_bins;
	
	std::cout<<"Nr_bins="<<std::endl;
	std::cin>>Nr_bins;
	
	std::cout<<"Nr_bins_ales="<<Nr_bins<<std::endl;
	
	float lim_inf;
	float lim_sup;
	
	float Bin_intervals[2][500];

	pcl::PointCloud<pcl::PointXYZ>::Ptr Array_bin_clouds[4000];
	
	for(int j=1;j<=Nr_bins;j++)
	{
		
		
		lim_inf=minimum_distance+(float((j-1)))/ (float (Nr_bins))*(maximum_distance-minimum_distance);
		lim_sup=minimum_distance+(float (j))/(float (Nr_bins))*(maximum_distance-minimum_distance);
		
		Bin_intervals[0][j-1]=lim_inf;
		Bin_intervals[1][j-1]=lim_sup;
		
	}
		
	
	for(int i=0;i<Nr_bins;i++)
			{
				std::cout<<"lim_inf["<<(i)<<"]="<<Bin_intervals[0][i]<<std::endl;
				std::cout<<"lim_sup["<<(i)<<"]="<<Bin_intervals[1][i]<<std::endl;
				std::cout<<std::endl;
			}
			
	
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_conex_parts(new pcl::PointCloud<pcl::PointXYZ>);
	
	float dist_min_connect_points;
	
	std::cout<<"Dist min connex points="<<std::endl;
	std::cin>>dist_min_connect_points;
	
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_copy(new pcl::PointCloud<pcl::PointXYZ>);
	//copyPointCloud(*cloud_filtered, *cloud_filtered_copy);
	
	pcl::PointXYZ point_Reeb; 
	
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_Reeb(new pcl::PointCloud<pcl::PointXYZ>);


	

	int nr_connected_parts=0;
	
	int sum_check_bins=0;

	std::cout<<"Nr points in point cloud "<<cloud_filtered->size()<<std::endl;
	
	for(int i=0;i<Nr_bins;i++)
	{
		////////////////////////////////////////////////////////
		//For each bin we compute the point cloud

		std::cout<<"starting bin "<<(i+1)<<std::endl;
		
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_bin_actual(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointXYZ cloud_point_bin; 
		
		
		
		for (int j = 0; j < cloud_filtered->size(); j++)
			{
				if ( (Distances[j]>=Bin_intervals[0][i])&& (Distances[j]< Bin_intervals[1][i]))
				{
					cloud_point_bin.x=cloud_filtered->points[j].x;
					cloud_point_bin.y=cloud_filtered->points[j].y;
					cloud_point_bin.z=cloud_filtered->points[j].z;
					
					cloud_bin_actual->points.push_back(cloud_point_bin);
					
					//std::cout<<Distances[j]<<" >= "<<Bin_intervals[0][i]<<" and "<<Distances[j]<<" < "<<Bin_intervals[0][i]<<std::endl;
					}
					
				cloud_bin_actual->width = cloud_bin_actual->points.size();
				cloud_bin_actual->height = 1;
				cloud_bin_actual->points.resize(cloud_bin_actual->width * cloud_bin_actual->height);
				cloud_bin_actual->is_dense = false;
			}
			
		std::cout<<"Nr points in bin is "<<cloud_bin_actual->size()<<std::endl;
		
		
		int nr_remaining_elements=cloud_bin_actual->points.size();
		
		
		int random_point;


		
		
		while(cloud_bin_actual->size()>0){
			
			
		
		random_point=rand() % cloud_bin_actual->size();
		
		//std::cout<<"New random point="<<random_point<<std::endl;
		
		
		pcl::PointXYZ moving_random_point;
		
		
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_random_point_bin(new pcl::PointCloud<pcl::PointXYZ>);
		
		
		// Adding the Reeb points
		
		point_Reeb.x=cloud_bin_actual->points[random_point].x;
	    point_Reeb.y=cloud_bin_actual->points[random_point].y;
		point_Reeb.z=cloud_bin_actual->points[random_point].z;
					
					
		cloud_Reeb->points.push_back(point_Reeb);
		
		cloud_Reeb->width = cloud_Reeb->points.size();
		cloud_Reeb->height = 1;
		cloud_Reeb->points.resize(cloud_Reeb->width * cloud_Reeb->height);
		cloud_Reeb->is_dense = false;
		
		
		///////////////////////////////////////////

		//Add chosen point to region point cloud
		moving_random_point.x=cloud_bin_actual->points[random_point].x;
		moving_random_point.y=cloud_bin_actual->points[random_point].y;
		moving_random_point.z=cloud_bin_actual->points[random_point].z;


		cloud_random_point_bin->points.push_back(moving_random_point);

		cloud_random_point_bin->width = cloud_random_point_bin->points.size();
		cloud_random_point_bin->height = 1;
		cloud_random_point_bin->points.resize(cloud_random_point_bin->width * cloud_random_point_bin->height);
		cloud_random_point_bin->is_dense = false;

		bool find_points=1;

		

		////////////////////////////////////////
		//Remove point from point cloud

		pcl::PointXYZ point_remaining;
		
		
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_point_remaining(new pcl::PointCloud<pcl::PointXYZ>);
		
		
		for (int iter_change_bin = 0; iter_change_bin < cloud_bin_actual->size(); iter_change_bin++)
			{
				bool ok=1;
				for (int iter_change_region=0;iter_change_region<cloud_random_point_bin->size();iter_change_region++){
					if ((cloud_bin_actual->points[iter_change_bin].x== cloud_random_point_bin->points[iter_change_region].x)&& (cloud_bin_actual->points[iter_change_bin].y== cloud_random_point_bin->points[iter_change_region].y) && (cloud_bin_actual->points[iter_change_bin].z== cloud_random_point_bin->points[iter_change_region].z))
					{
						ok=0;
						}
					}
				if(ok)
				{
					point_remaining.x=cloud_bin_actual->points[iter_change_bin].x;
					point_remaining.y=cloud_bin_actual->points[iter_change_bin].y;
					point_remaining.z=cloud_bin_actual->points[iter_change_bin].z;
					
					
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

		std::cout<<"Size of bin is "<<cloud_bin_actual->size()<<std::endl;
		std::cout<<"Added initial point to region, size is now "<<cloud_random_point_bin->size()<<std::endl;

		while(find_points)
		{
			find_points=0;

		int nr_points = cloud_random_point_bin->size();


		
		
		
		
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_new_points_region(new pcl::PointCloud<pcl::PointXYZ>);

		for (int iterate_bin=0;iterate_bin<cloud_bin_actual->size();iterate_bin++)
		{

			for( int iterate_region=0;iterate_region<cloud_random_point_bin->size();iterate_region++)
			{
				float comp_x_bin_element=cloud_random_point_bin->points[iterate_region].x-cloud_bin_actual->points[iterate_bin].x;
				float comp_y_bin_element=cloud_random_point_bin->points[iterate_region].y-cloud_bin_actual->points[iterate_bin].y;
				float comp_z_bin_element=cloud_random_point_bin->points[iterate_region].z-cloud_bin_actual->points[iterate_bin].z;
				
				float dist_connected_points=sqrt(comp_x_bin_element*comp_x_bin_element + comp_y_bin_element*comp_y_bin_element + comp_z_bin_element*comp_z_bin_element);
				
				
				if( (dist_connected_points< dist_min_connect_points)&&(dist_connected_points!=0)){

					//std::cout<<"Found connected point"<<std::endl;

					find_points=1;

					
					}	
				
		
					
			}
			if(find_points)
			{
				moving_random_point.x=cloud_bin_actual->points[iterate_bin].x;
				moving_random_point.y=cloud_bin_actual->points[iterate_bin].y;
				moving_random_point.z=cloud_bin_actual->points[iterate_bin].z;
					
				cloud_new_points_region->points.push_back(moving_random_point);

				cloud_new_points_region->width = cloud_new_points_region->points.size();
				cloud_new_points_region->height = 1;
				cloud_new_points_region->points.resize(cloud_new_points_region->width * cloud_new_points_region->height);
				cloud_new_points_region->is_dense = false;

			}


			
			
		}

		*cloud_random_point_bin += *cloud_new_points_region;

		cloud_random_point_bin->width = cloud_random_point_bin->points.size();
		cloud_random_point_bin->height = 1;
		cloud_random_point_bin->points.resize(cloud_random_point_bin->width * cloud_random_point_bin->height);
		cloud_random_point_bin->is_dense = false;


		std::cout<<"Dimension of new region is "<<cloud_random_point_bin->size()<<std::endl;

		if(find_points)
		{


				pcl::PointXYZ point_remaining;
		
		
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_point_remaining(new pcl::PointCloud<pcl::PointXYZ>);
		
		
		for (int iter_change_bin = 0; iter_change_bin < cloud_bin_actual->size(); iter_change_bin++)
			{
				bool ok=1;
				for (int iter_change_region=0;iter_change_region<cloud_random_point_bin->size();iter_change_region++){
					if ((cloud_bin_actual->points[iter_change_bin].x== cloud_random_point_bin->points[iter_change_region].x)&& (cloud_bin_actual->points[iter_change_bin].y== cloud_random_point_bin->points[iter_change_region].y) && (cloud_bin_actual->points[iter_change_bin].z== cloud_random_point_bin->points[iter_change_region].z))
					{
						ok=0;
						}
					}
				if(ok)
				{
					point_remaining.x=cloud_bin_actual->points[iter_change_bin].x;
					point_remaining.y=cloud_bin_actual->points[iter_change_bin].y;
					point_remaining.z=cloud_bin_actual->points[iter_change_bin].z;
					
					
					cloud_point_remaining->points.push_back(point_remaining);
					}
					cloud_point_remaining->width = cloud_point_remaining->points.size();
					cloud_point_remaining->height = 1;
					cloud_point_remaining->points.resize(cloud_point_remaining->width * cloud_point_remaining->height);
					cloud_point_remaining->is_dense = false;
			}
		
		
		copyPointCloud(*cloud_point_remaining, *cloud_bin_actual);


		std::cout<<"Nr_points_remaining in bin:"<<cloud_bin_actual->size()<<std::endl;
		}

		


		}
		


		
        //////////////
		
		if(cloud_random_point_bin->size()!=0)
		{

			Array_bin_clouds[nr_connected_parts]=cloud_random_point_bin;
			nr_connected_parts++;
		}
		

		
		//std::cout<<"Nr_points_close_to_random:"<<cloud_random_point_bin->size()<<std::endl;
		
		sum_check_bins=sum_check_bins+cloud_random_point_bin->size();
		
		
			
		//std::cout<<"Nr_points_remaining:"<<cloud_bin_actual->size()<<std::endl;
		
		
		
		
		
	}
	std::cout<<"Nr_points safety check bins:"<<sum_check_bins<<std::endl;
	std::cout<<"Nr points in point cloud "<<cloud_filtered->size()<<std::endl;
		
		
		
		
	}

	std::cout<<"Number of connected parts:"<<nr_connected_parts<<std::endl;

	// for(int i=0;i<nr_connected_parts;i++){
	// 	std::cout<<"Connected part "<<i<<""<<Array_bin_clouds[i]->size()<<std::endl;
	// }
	
	pcl::io::savePCDFileASCII ("Reeb_pcd.pcd", *cloud_Reeb);
	std::cerr << "Saved Reeb pcd" << std::endl;
	
	
  return (0);
}
