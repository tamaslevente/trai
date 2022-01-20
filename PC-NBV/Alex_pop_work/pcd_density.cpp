#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <iostream>
#include <fstream>
#include <string>
using namespace std;

int
main ()
{
	
	string myText;
	
  string path="/home/alex-pop/Desktop/Doctorat/Side_projects/orthogonal-planes/data/pcd_conversions/";
  
  std::cout<<path<<"\n";

  
 
  ifstream MyReadFile(path+"config_file/"+"input_file_check_density.txt");
  
  getline (MyReadFile, myText);
  
	
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  
  
  if (pcl::io::loadPCDFile<pcl::PointXYZ> (path+myText+".pcd", *cloud) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
    return (-1);
  }
  std::cout << "Loaded "
            << cloud->width * cloud->height
            << " data points from test_pcd.pcd with the following fields: "
            << std::endl;
 /* for (const auto& point: *cloud)
    std::cout << "    " << point.x
              << " "    << point.y
              << " "    << point.z << std::endl;
              * 
              
              * */

	

	float densitate_totala=0;
	size_t num_points = cloud->size();  
	
	float d_min=0.1;
	float distanta_totala=0;
	float distanta_minima_globala=120002.0;
	
	
	for (int i = 0; i < cloud->points.size ()-1; i++)
		{
			int densitate_punct=1;
			float  distanta_minima=1230000.0;
		for (int j = 0; j < cloud->points.size (); j++)
			{
				if(j!=i){
					float coord_x_i,coord_y_i,coord_z_i,coord_x_j,coord_y_j,coord_z_j;
				
				coord_x_i=cloud->points[i].x;
				coord_y_i=cloud->points[i].y;
				coord_z_i=cloud->points[i].z;
				
				coord_x_j=cloud->points[j].x;
				coord_y_j=cloud->points[j].y;
				coord_z_j=cloud->points[j].z;
				
				float componenta_x_patrat=(coord_x_i-coord_x_j)*(coord_x_i-coord_x_j);
				float componenta_y_patrat=(coord_y_i-coord_y_j)*(coord_y_i-coord_y_j);
				float componenta_z_patrat=(coord_z_i-coord_z_j)*(coord_z_i-coord_z_j);
				
				float distanta = sqrt(componenta_x_patrat+componenta_y_patrat+componenta_z_patrat);
				
				
				if(distanta<distanta_minima){
					distanta_minima=distanta;
					//std::cout<<cloud->points[i].x<<" "<<cloud->points[i].y<<" "<<cloud->points[i].z <<cloud->points[j].x<<" "<<cloud->points[j].y<<" "<<cloud->points[j].z<<std::endl;
					}
					
					}
				
			}
		//std::cout<<"Distanta minima langa: "<<i<<" este:"<<distanta_minima<<std::endl;
		if(distanta_minima_globala>distanta_minima){
			distanta_minima_globala=distanta_minima;
			}
		distanta_totala=distanta_totala+distanta_minima;
		}
		
		float distanta_finala_medie=distanta_totala/(cloud->points.size ()-1);
	std::cout<<"Distanta medie este:"<<distanta_finala_medie<<std::endl;
	std::cout<<"Distanta minima este:"<<distanta_minima_globala<<std::endl;


  return (0);
}
