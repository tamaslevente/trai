#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <iostream>
#include <fstream>
using namespace std;

int
main ()
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  
  string myText;

  
  string path="/home/alex-pop/Desktop/Doctorat/Side_projects/orthogonal-planes/data/pcd_conversions/";
  
  string d_min_text;
  ifstream MyReadFile(path+"config_file/"+"input_file_downsample.txt");
  
  getline (MyReadFile, d_min_text);
  
  float d_min=std::stof(d_min_text);
  
  
  
  
  getline (MyReadFile, myText);
  
  
  
  std::cout << myText;
  
  MyReadFile.close();
  
  

  if (pcl::io::loadPCDFile<pcl::PointXYZ> (myText+".pcd", *cloud) == -1) //* load the file
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
	
	
	float distanta_totala=0;
	float distanta_minima_globala=120002.0;
	
	bool point_check[cloud->size()];
	
	for(int t=0;t<cloud->size();t++){
		point_check[t]=1;
		}
	
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_final(new pcl::PointCloud<pcl::PointXYZ>);
	
	cloud_final->width=1;
	cloud_final->height=0;
	
	cloud_final->is_dense=false;
	
	int index_final=0;
	
	for (int i = 0; i < cloud->points.size() ; i++)
		{
			if(point_check[i]!=0){
				//std::cout<<"Punct bun"<<std::endl;
			int densitate_punct=1;
			float  distanta_minima=1230000.0;
			
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_new (new pcl::PointCloud<pcl::PointXYZ>);
	
			cloud_new->width=1;
			cloud_new->height=0;
			
			int index=0;
			cloud_new->height=cloud_new->height+1;
			
			cloud_new->is_dense = false;
           cloud_new->resize(cloud_new->width * cloud_new->height);
			
			
			cloud_new->points[index].x=cloud->points[i].x;
			cloud_new->points[index].y=cloud->points[i].y;
			cloud_new->points[index].z=cloud->points[i].z;
			
			index=index+1;
			
			point_check[i]=0;
	
		for (int j = 0; j < cloud->points.size (); j++)
			{
				if((i!=j) && (point_check[j]!=0))
				{
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
				
				
				if(distanta<d_min){
					
					cloud_new->height=cloud_new->height+1;
					cloud_new->resize(cloud_new->width * cloud_new->height);
					cloud_new->points[index].x=cloud->points[j].x;
					cloud_new->points[index].y=cloud->points[j].y;
					cloud_new->points[index].z=cloud->points[j].z;
					index=index+1;
					
					point_check[j]=0;
					
					//std::cout<<cloud->points[i].x<<" "<<cloud->points[i].y<<" "<<cloud->points[i].z <<cloud->points[j].x<<" "<<cloud->points[j].y<<" "<<cloud->points[j].z<<std::endl;
					}
				}
				
			}
			
		float pozitie_medie_x=0,pozitie_medie_y=0,pozitie_medie_z=0;
		
		for (int k=0;k<index;k++){
			pozitie_medie_x=pozitie_medie_x+cloud_new->points[k].x;
			pozitie_medie_y=pozitie_medie_y+cloud_new->points[k].y;
			pozitie_medie_z=pozitie_medie_z+cloud_new->points[k].z;
			}
			
		pozitie_medie_x=pozitie_medie_x/index;
		pozitie_medie_y=pozitie_medie_y/index;
		pozitie_medie_z=pozitie_medie_z/index;
		
			
		std::cout<<"Nr puncte colectie punctul: "<<i<<" este:"<<index<<std::endl;
		
		if(index!= 1){
			cloud_final->height=cloud_final->height+1;
		cloud_final->resize(cloud_final->width * cloud_final->height);
		cloud_final->points[index_final].x=pozitie_medie_x;
		cloud_final->points[index_final].y=pozitie_medie_y;
		cloud_final->points[index_final].z=pozitie_medie_z;
		index_final=index_final+1;
			}
		
		
				}
			
		
	}
	
	if(cloud_final->size()!=0){
		std::string save_string=path+myText;
		
		std::cout<<index_final;
		pcl::io::savePCDFileASCII (save_string+"_downsampled.pcd", *cloud_final);
		std::cerr << "Saved data points to downsampled_"<<myText<<".pcd" << std::endl;
	}
	else{
		std::cout<<"No points found" << std::endl;
		}
		
  return (0);
}
