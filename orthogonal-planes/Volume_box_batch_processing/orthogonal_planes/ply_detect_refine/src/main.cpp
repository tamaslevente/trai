/**
BSD 3-Clause License

This file is part of the code accompanying the paper
From Planes to Corners: Multi-Purpose Primitive Detection in Unorganized 3D Point Clouds
by C. Sommer, Y. Sun, L. Guibas, D. Cremers and T. Birdal,
accepted for Publication in IEEE Robotics and Automation Letters (RA-L) 2020.

Copyright (c) 2019, Christiane Sommer.
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

// includes
#include <iostream>
#include <fstream>
#include <string> 
#include "definitions.h"
// libraries
#include <Eigen/Dense>
#include <opencv2/core/core.hpp>
#include <CLI/CLI.hpp>
// classes
#include "Timer.h"
#include "Plane.h"
#include "graph/PlaneGraph.h"
#include "graph/ParallelPlaneGraph.h"
#include "PPF/PairDetector.h"
// function includes
#include "io/load_ply_cloud.h"
#include "visualize/pcshow.h"

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>



#include <experimental/filesystem>

namespace fs = std::experimental::filesystem;

/*
 * main
 */
int main(int argc, char** argv) {

  int nr_volume=0;
  int nr_pointclouds=0;

  std::string string_pointcloud;
  std::ifstream MyReadFile2("../data/main_programs/config_files/input_file_plane_dataset.txt");
  std::getline (MyReadFile2, string_pointcloud);

 


  std::ofstream myfile("/home/alex-pop/Desktop/Doctorat/Side_projects/Volume_box_batch_processing/orthogonal-planes/data/output_volumes.txt");
  

  myfile<<"Hello"<<'\n';



  
	
	//fs::path path("/home/alex-pop/Desktop/Doctorat/Side_projects/Volume_box_batch_processing/orthogonal-planes/data/Pointcloud_recordings");
  fs::path path(string_pointcloud);
	for (auto& p : fs::directory_iterator(path))
	{
		
	std::stringstream ss;
	
    ss << p ;
    
   std::string str = ss.str();
   
   str.erase(std::remove(str.begin(), str.end(), '"'), str.end());
   
   std::cout<<str<<'\n';
    
   pcl::PointCloud<pcl::PointNormal>::Ptr cloud(new pcl::PointCloud<pcl::PointNormal>);

    if (pcl::io::loadPCDFile<pcl::PointNormal> (str, *cloud) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
    return (-1);
  }
  std::cout << "Loaded "
            << cloud->width * cloud->height
            << " data points from "<<str<<".pcd with the following fields: "
            << std::endl;
            
  std::vector<Eigen::Vector3f> points;
  std::vector<Eigen::Vector3f> normals;

  nr_pointclouds++;



  for (int i = 0; i < cloud->size(); i++)
    {
       
        
        Eigen::Vector3f position_aux= Eigen::Vector3f( (float) cloud->points[i].x,(float) cloud->points[i].y,(float) cloud->points[i].z);
        points.push_back(position_aux);

        Eigen::Vector3f normal_aux= Eigen::Vector3f( (float) cloud->points[i].normal_x,(float) cloud->points[i].normal_y,(float) cloud->points[i].normal_z);
        normals.push_back(normal_aux);
    }

     std::cout << points.size() << " points loaded." << std::endl;
     std::cout << normals.size() << " normals loaded." << std::endl;
    std::cout<<"\n";

    Timer T;
    
    // parse setttings from command line
    std::string ply_file;
    int min_votes = 5;
    double d_min = .1, d_max = 1.;
    int sampling = 10;


    std::ifstream MyReadFile("../data/main_programs/config_files/input_file_plane.txt");

    std::ifstream MyReadFile2("../data/main_programs/config_files/input_file_plane_dataset.txt");

    std::string min_votes_text;
    std::string d_min_text;
    std::string d_max_text;
    std::string sampling_text;
    std::string threshold_text;

    std::getline (MyReadFile, min_votes_text);
    std::getline (MyReadFile, d_min_text);
    std::getline (MyReadFile, d_max_text);
    std::getline (MyReadFile, sampling_text);
    std::getline (MyReadFile, threshold_text);


    min_votes=std::stoi(min_votes_text);
    std::cout<<"min_votes="<<min_votes<< std::endl;
    d_min=std::stod(d_min_text);
     std::cout<<"d_min="<<d_min<< std::endl;
     d_max=std::stod(d_max_text);
     std::cout<<"d_max="<<d_max<< std::endl;
     sampling=std::stoi(sampling_text);
     std::cout<<"sampling_text="<<sampling<< std::endl;
     double threshold=std::stod(threshold_text);
     std::cout<<"threshold lines="<<threshold<< std::endl;

  
     for(int nr_exec=0;nr_exec<3;nr_exec++)
     {

       
       
    
    // pairing
    ppf::PairDetector pairDet(d_max, d_min, min_votes);
    PlaneGraph planeMap(pairDet.para_thresh(), pairDet.distance_bin());
    T.tic();
    pairDet.detect_ortho_pairs(points, normals, planeMap);
    T.toc("time pairing PPF ");
    //planeMap.print_info();
    
    std::cout << std::endl << "Thresholds:\tAngle:\t" << pairDet.para_thresh() << "\tDistance:\t" << pairDet.distance_bin() << std::endl << std::endl;

    // clustering & filtering
    T.tic();
    planeMap.cluster_graph_vertices();
    T.toc("clustering planes");
    //planeMap.print_info();
    //planeMap.print_parameters();
    //planeMap.print_edges();
    
    // find triangles
    std::vector<Graph<Plane>::Triangle> triangles;
    T.tic();
    planeMap.find_triangles(triangles);
    T.toc("Finding triangles");
    //planeMap.print_triangles(triangles);
    
    // reduce graph to ParallelPlaneGraph
    T.tic();
    ParallelPlaneGraph redMap = planeMap.reduce_graph();
    T.toc("graph reduction");
    //redMap.print_info();
    //redMap.print_edges();
    T.tic();
    redMap.triangle_reduce();
    T.toc("triangle reduction");
    //redMap.print_info();
    //redMap.print_parameters();
    //redMap.print_edges();
    
    T.tic();
    redMap.filter_outliers(points, normals, pairDet.distance_bin(), .5*sampling);
    T.toc("outlier filtering");
    //redMap.print_info();
    //redMap.print_parameters();
    //redMap.print_edges();
 
    // CERES problem setup and solving
    double lambda = 0.01 * points.size();
    T.tic();
    redMap.refine_coarse_fine(points, normals, lambda, pairDet.distance_bin(), sampling);
    T.toc("Multi-plane parameter optimization");
    //redMap.print_info();
    //redMap.print_parameters();
    //redMap.print_edges();
    
    T.tic();
    redMap.filter_outliers(points, normals, pairDet.distance_bin(), .5*sampling);
    T.toc("outlier filtering");
    //redMap.print_info();
    //redMap.print_parameters();
    //redMap.print_edges();
    
    // From here on visualization / debugging

//    // get points located on planes and not-planes
    PlaneGraph finalMap = PlaneGraph::from_ppg(redMap, 2*pairDet.para_thresh()*pairDet.para_thresh()-1, pairDet.distance_bin()); // double angle
    std::vector<Eigen::Matrix<float, 6, 1>> labeled;
    T.tic();
    finalMap.go_thr_pcl(points, normals, labeled);
    T.toc("labeling planes");

    // visualize
   // pcshow(labeled, "Result");
    
   std::vector<Vec6> lines;
   // line extraction and visualization
   T.tic();
   lines = finalMap.extract_lines(points, normals);
   T.toc("get lines");
   //std::cout << lines.size() << " lines found." << std::endl;

   std::vector<Vec6> new_lines;

   std::vector<Vec6> final_lines;
   std::vector<double> distances;

   std::vector<double> Volume_edge;

   std::vector<int> positions_wrong;
   std::vector<double> Volumes;

   

    std::cout<<'\n';

    
  if (lines.size()>=3){
    for(int i = 0; i != lines.size()-2; i++) {
    //std::cout<< lines[i][0]<<" "<<lines[i][1]<<" "<<lines[i][2]<<" "<<lines[i][3]<<" "<<lines[i][4]<<" "<<lines[i][5]<<" "<<'\n';
      for(int j = i+1; j != lines.size()-1; j++) {
          for(int k = j+1; k != lines.size(); k++) {

                bool ok_ij=1;
                bool ok_jk=1;
                bool ok_ki=1;

                double average_0,average_1,average_2;

                
                if( (lines[i][0]==lines[j][0]) && (lines[i][1]==lines[j][1]) && (lines[i][2]==lines[j][2])) {ok_ij=0;}
                if( (lines[j][0]==lines[k][0]) && (lines[j][1]==lines[k][1]) && (lines[j][2]==lines[k][2])) {ok_jk=0;}
                if( (lines[k][0]==lines[i][0]) && (lines[k][1]==lines[i][1]) && (lines[k][2]==lines[i][2])) {ok_ki=0;}
                
                if (ok_ij && ok_jk && ok_ki){
                  
              average_0=(lines[i][0]+lines[j][0]+lines[k][0]) / 3;
              average_1=(lines[i][1]+lines[j][1]+lines[k][1]) /3 ;
              average_2=(lines[i][2]+lines[j][2]+lines[k][2]) /3;


              double dist_i_average= sqrt (  (lines[i][0]-average_0)*(lines[i][0]-average_0)  +  (lines[i][1]-average_1)*(lines[i][1]-average_1) +(lines[i][2]-average_2)*(lines[i][2]-average_2)   );
              double dist_j_average= sqrt (  (lines[j][0]-average_0)*(lines[j][0]-average_0)  +  (lines[j][1]-average_1)*(lines[j][1]-average_1) +(lines[j][2]-average_2)*(lines[j][2]-average_2)   );
              double dist_k_average= sqrt (  (lines[k][0]-average_0)*(lines[k][0]-average_0)  +  (lines[k][1]-average_1)*(lines[k][1]-average_1) +(lines[k][2]-average_2)*(lines[k][2]-average_2)   );


              if(  (dist_i_average<threshold) && (dist_j_average<threshold) &&(dist_k_average<threshold) )
              {
                  /*
                  std::cout<< lines[i][0]<<" "<<lines[i][1]<<" "<<lines[i][2]<<" "<<'\n';      
                  std::cout<< lines[j][0]<<" "<<lines[j][1]<<" "<<lines[j][2]<<" "<<'\n';
                  std::cout<< lines[k][0]<<" "<<lines[k][1]<<" "<<lines[k][2]<<" "<<'\n';
                  */

                  distances.push_back(dist_i_average);
                  distances.push_back(dist_j_average);
                  distances.push_back(dist_k_average);
                  

                  /* 
                  std::cout<<"Average:"<<average_0<<" "<<average_1<<" "<<average_2<<"\n";
                  std::cout<<"\n";
                  */
                  /*
                std::cout<< lines[i][0]<<" "<<lines[i][1]<<" "<<lines[i][2]<<" "<<lines[i][3]<<" "<<lines[i][4]<<" "<<lines[i][5]<<" "<<'\n';
                std::cout<< lines[j][0]<<" "<<lines[j][1]<<" "<<lines[j][2]<<" "<<lines[j][3]<<" "<<lines[j][4]<<" "<<lines[j][5]<<" "<<'\n';
                std::cout<< lines[k][0]<<" "<<lines[k][1]<<" "<<lines[k][2]<<" "<<lines[k][3]<<" "<<lines[k][4]<<" "<<lines[k][5]<<" "<<'\n';
                */
                double latura_i = sqrt (  (lines[i][0]-lines[i][3])*(lines[i][0]-lines[i][3])  +  (lines[i][1]-lines[i][4])*(lines[i][1]-lines[i][4]) +(lines[i][2]-lines[i][5])*(lines[i][2]-lines[i][5])   );
                double latura_j = sqrt (  (lines[j][0]-lines[j][3])*(lines[j][0]-lines[j][3])  +  (lines[j][1]-lines[j][4])*(lines[j][1]-lines[j][4]) +(lines[j][2]-lines[j][5])*(lines[j][2]-lines[j][5])   );
                double latura_k = sqrt (  (lines[k][0]-lines[k][3])*(lines[k][0]-lines[k][3])  +  (lines[k][1]-lines[k][4])*(lines[k][1]-lines[k][4]) +(lines[k][2]-lines[k][5])*(lines[k][2]-lines[k][5])   );

                double Volum = latura_i*latura_j*latura_k;
                 /*
                std::cout<<"Latura 1: "<<latura_1<<'\n';
                std::cout<<"Latura 2: "<<latura_3<<'\n';
                std::cout<<"Latura 3: "<<latura_3<<'\n';
                */

                //std::cout<<"Volum="<<Volum<<'\n';

                new_lines.push_back(lines[i]);
                new_lines.push_back(lines[j]);
                new_lines.push_back(lines[k]);

                Volume_edge.push_back(latura_i);
                Volume_edge.push_back(latura_j);
                Volume_edge.push_back(latura_k);

                Volumes.push_back(Volum);
                Volumes.push_back(Volum);
                Volumes.push_back(Volum);
              }

              }
            }     
          }

       }


   if (new_lines.size()>1){
    for(int i = 0; i != new_lines.size(); i++) {
    //std::cout<< new_lines[i][0]<<" "<<new_lines[i][1]<<" "<<new_lines[i][2]<<" "<<new_lines[i][3]<<" "<<new_lines[i][4]<<" "<<new_lines[i][5]<<" "<<'\n';
    }   
    

    for(int i = 0; i != new_lines.size()-1; i++) {
    
      for(int j = i+1; j != new_lines.size(); j++) {
          if( ( (new_lines[i][0]==new_lines[j][0])  && (new_lines[i][1]==new_lines[j][1]) && (new_lines[i][2]==new_lines[j][2]) && (new_lines[i][3]==new_lines[j][3]) && (new_lines[i][4]==new_lines[j][4]) && (new_lines[i][5]==new_lines[j][5]) ) ||
              ( (new_lines[i][0]==new_lines[j][3])  && (new_lines[i][1]==new_lines[j][4])&&  (new_lines[i][2]==new_lines[j][5]) && (new_lines[i][3]==new_lines[j][0]) && (new_lines[i][4]==new_lines[j][1]) && (new_lines[i][5]==new_lines[j][2]) ) )  
               {
            //std::cout<<"Dublura gasita la"<<i<<" "<<j<<'\n';

            double total_distance_i;
            double total_distance_j;
            
            int offset_i= (i % 3);
            int offset_j= (j % 3);

            

            total_distance_i=distances[i - offset_i]+distances[i+1 - offset_i]+distances[i+2 - offset_i];
            total_distance_j=distances[j - offset_j]+distances[j+1 - offset_j]+distances[j+2 - offset_j];

            //std::cout<<"Distance i: "<<total_distance_i<<"\n";
           // std::cout<<"Distance j: "<<total_distance_j<<"\n";

            if(total_distance_i>total_distance_j){
              positions_wrong.push_back(i - offset_i);
            }
            else
            {
               positions_wrong.push_back(j - offset_j);
            }

            
          }
          
      }
    }
   }
  
  /*
  std::cout<<"Positions wrong:"<<'\n';


   for(int i = 0; i != positions_wrong.size(); i++) {
    std::cout<<positions_wrong[i]<<'\n';
    }   

    */
    if (new_lines.size()>=3){
      nr_volume++;

       //myfile<<str<<" "<<'\n';

       //myfile<<nr_exec<<" iteration"<<'\n';

    for(int i = 0; i < new_lines.size(); i=i+3) {
      bool ok_position=1;
      for(int j = 0; j != positions_wrong.size(); j++) {
        if(i==positions_wrong[j]){
          ok_position=0;
        }
      }
      if(ok_position){
        final_lines.push_back(new_lines[i]);
        final_lines.push_back(new_lines[i+1]);
        final_lines.push_back(new_lines[i+2]);
        std::cout<<"Volum "<<(i/3)<<": "<<Volumes[i]<<" cm^3"<<'\n';

        myfile<<Volumes[i]<<" "<<nr_pointclouds<<'\n';

        /*
        std::cout<< "("<<new_lines[i][0]<<" , "<<new_lines[i][1]<<" , "<<new_lines[i][2]<<" , "<<new_lines[i][3]<<" , "<<new_lines[i][4]<<" , "<<new_lines[i][5]<<")"<<" =>distance1= "<<Volume_edge[i] <<'\n';
        std::cout<< "("<<new_lines[i+1][0]<<" , "<<new_lines[i+1][1]<<" , "<<new_lines[i+1][2]<<" , "<<new_lines[i+1][3]<<" , "<<new_lines[i+1][4]<<" , "<<new_lines[i+1][5]<<")"<<" =>distance2= "<<Volume_edge[i+1] <<'\n';
        std::cout<< "("<<new_lines[i+2][0]<<" , "<<new_lines[i+2][1]<<" , "<<new_lines[i+2][2]<<" , "<<new_lines[i+2][3]<<" , "<<new_lines[i+2][4]<<" , "<<new_lines[i+2][5]<<")"<<" =>distance3= "<<Volume_edge[i+2] <<'\n';
        */

        std::cout<< "distance1= "<<Volume_edge[i] <<" cm"<<'\n';
        std::cout<< "distance2= "<<Volume_edge[i+1]<<" cm" <<'\n';
        std::cout<< "distance3= "<<Volume_edge[i+2]<<" cm" <<'\n';
        
        std::cout<<'\n';

       

      }
    }
   }

   
     
  
   
   //pcshow_lines(points, lines, "Orth Plane Intersections");

   //pcshow_lines(points, new_lines, "Volume Computation Edges");

   //pcshow_lines(points, final_lines, "Volume Final Edges");
   
  }
     }
  }
   std::cout<<nr_pointclouds<<" pointclouds found"<<'\n';
    std::cout<<nr_volume<<" pointclouds with volumes"<<'\n';


   
    
}
