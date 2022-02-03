#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

int
main ()
{
	
	std::ifstream input_parameters("../config_files/config_Reeb_VIZ.txt");


	std::string input_folder_path;
	std::string input_cloud_name;
	std::string input_inc_matrix_name;

	std::cout << "input folder path:";
	//std::cin >> input_cloud_location;
	std::getline(input_parameters, input_folder_path);
	std::cout << input_folder_path << std::endl;

	std::cout << "input_cloud name:";
	//std::cin >> cloud_selection;
	std::getline(input_parameters, input_cloud_name);
	std::cout << input_cloud_name << std::endl;

	std::stringstream ss;

	ss << input_folder_path << input_cloud_name;
	
	
	
	std::stringstream ss_in_matrix;
	
	std::cout << "Incidence matrix file name:";
	//std::cin >> cloud_selection;
	std::getline(input_parameters, input_inc_matrix_name);
	std::cout << input_inc_matrix_name << std::endl;
	
	ss_in_matrix << input_folder_path << input_inc_matrix_name;
	
	//////////////////////////////////
	
	
	std::string output_folder_location;
	std::string output_Viz_pcd_name;
	

	std::cout << "Output folder location:";
	//std::cin >> input_cloud_location;
	std::getline(input_parameters, output_folder_location);
	std::cout << output_folder_location << std::endl;

	std::cout << "Vizualization cloud name:";
	//std::cin >> cloud_selection;
	std::getline(input_parameters, output_Viz_pcd_name);
	std::cout << output_Viz_pcd_name << std::endl;

	
	std::stringstream ss_out_pcd;
	

	ss_out_pcd << output_folder_location << output_Viz_pcd_name;
	
	

/////////////////////////////
	int nr_points_line;
	std::cout << "nr_points_line=";
	input_parameters >> nr_points_line;
	std::cout << nr_points_line<<std::endl;


//////////
	int n;


	std::ifstream incid_matrix_file(ss_in_matrix.str());
	incid_matrix_file>>n;
	
	int Incid_Matrix[n][n];
	
	for (int i = 0; i < n; i++)
	{
		for (int j = 0; j < n; j++)
		{
			incid_matrix_file >> Incid_Matrix[i][j];
			}
		}
		
	for (int i = 0; i < n; i++)
	{
		for (int j = 0; j < n; j++)
		{
			std::cout<< Incid_Matrix[i][j];
			}
			std::cout<<std::endl;
		}	
	
	
	
	////////////////////////
	
	
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_lines_Reeb(new pcl::PointCloud<pcl::PointXYZ>);

  if (pcl::io::loadPCDFile<pcl::PointXYZ> (ss.str(), *cloud) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
    return (-1);
  }
  std::cout << "Loaded "<< cloud->width * cloud->height<<std::endl;

  if(cloud->size()==n)
  {
	  std::cout<<"Point cloud size matches incidence matrix size "<<n<<std::endl;
  }
  else{
	  std::cout<<"Point cloud size does not match incidence matrix size"<<n<<"!="<<cloud->size()<<std::endl;
  }

  pcl::PointXYZ point_iterator;

  float xn;
  float yn;
  float zn;

	for (int i = 0; i < n-1; i++)
	{
		for (int j = i+1; j < n; j++)
		{
			if(Incid_Matrix[i][j]==1)
			{
				std::cout<<"X components:"<<cloud->points[i].x<<" "<<cloud->points[j].x<<std::endl;
				std::cout<<"Y components:"<<cloud->points[i].y<<" "<<cloud->points[j].y<<std::endl;
				std::cout<<"Z components:"<<cloud->points[i].z<<" "<<cloud->points[j].z<<std::endl;

				for(int k=1;k<nr_points_line;k++)
				{

				

					



					xn=cloud->points[i].x + (cloud->points[j].x-cloud->points[i].x) * (float)k* (1/(float)(nr_points_line+1));
					yn=cloud->points[i].y + (cloud->points[j].y-cloud->points[i].y) * (float)k* (1/(float)(nr_points_line+1));
					zn=cloud->points[i].z + (cloud->points[j].z-cloud->points[i].z) * (float)k* (1/(float)(nr_points_line+1));

					

					//std::cout<<xn<<" "<<yn<<" "<<zn<<" "<<std::endl;

					point_iterator.x = xn;
					point_iterator.y = yn;
					point_iterator.z = zn;

					cloud_lines_Reeb->points.push_back(point_iterator);
				}
			}
		}
		
	}	
	cloud_lines_Reeb->width = cloud_lines_Reeb->points.size();
	cloud_lines_Reeb->height = 1;
	cloud_lines_Reeb->points.resize(cloud_lines_Reeb->width * cloud_lines_Reeb->height);
	cloud_lines_Reeb->is_dense = false;


	pcl::io::savePCDFileASCII(ss_out_pcd.str(), *cloud_lines_Reeb);
	std::cerr << "Saved Reeb lines" << std::endl;


  return (0);
}
