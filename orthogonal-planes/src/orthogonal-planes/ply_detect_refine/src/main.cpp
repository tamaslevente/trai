/* /**
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
//#include "io/load_ply_cloud.h"
//#include "visualize/pcshow.h" 

// ROS deps
#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>

#include <ppfplane/line_detect_nodeConfig.h>

#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>

// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <visualization_msgs/Marker.h>

#include <dynamic_reconfigure/server.h>
#include "std_msgs/String.h"
#include <vector>






class LineDetectNode
{
public:
  typedef pcl::PointXYZRGB Point;
  typedef pcl::PointCloud<Point> PointCloud;

  LineDetectNode()
  {
    vis_pub = nh_.advertise<visualization_msgs::Marker>("/Volum_final", 0);
    pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/lines_all",1);
    pub2_ = nh_.advertise<sensor_msgs::PointCloud2>("/lines_final",1);
    sub_ = nh_.subscribe ("/norm_out", 1,  &LineDetectNode::cloudCallback, this);
    config_server_.setCallback(boost::bind(&LineDetectNode::dynReconfCallback, this, _1, _2));
  }

  ~LineDetectNode() {}

  void
  set_marker(visualization_msgs::Marker &marker)
  {
    marker.header.stamp = ros::Time::now();
    marker.pose.position.x = 1;
    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker.action = visualization_msgs::Marker::ADD;

    marker.pose.position.y = 1;
    marker.pose.position.z = 1;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color.a = 1.0; // Don't forget to set the alpha! Otherwise it is invisible
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.lifetime = ros::Duration();
  }

  void
  dynReconfCallback(ppfplane::line_detect_nodeConfig &config, uint32_t level)
  {
     min_votes = config.min_votes;
     d_min = config.d_min;
    d_max = config.d_max;
    sampling = config.sampling;
    threshold=config.threshold;
  }

  void
  cloudCallback(const sensor_msgs::PointCloud2ConstPtr &cloud_msg)
  {

    std::ofstream log("/home/alex-pop/Desktop/Doctorat/Side_projects/Volume_Box_2/catkin_ws/Volumes.txt", std::ios_base::app | std::ios_base::out);

    std::stringstream ss;
    pcl::PointCloud<pcl::PointNormal> cloud_Test;
    pcl::fromROSMsg(*cloud_msg, cloud_Test);
    
    pcl::PointCloud<pcl::PointNormal>::Ptr cloudPTR(new pcl::PointCloud<pcl::PointNormal>);
    *cloudPTR = cloud_Test;

     std::cout<<"Min votes: "<<min_votes<<'\n';
    std::cout<<"d_min: "<<d_min<<'\n';
    std::cout<<"d_max: "<<d_max<<'\n';
    std::cout<<"Sampling: "<<sampling<<'\n';
     std::cout<<"threshold: "<<threshold<<'\n';
    std::cout<<"Pointcloud size: "<<cloudPTR->size()<<"\n";

    std::vector<Eigen::Vector3f> points;
    std::vector<Eigen::Vector3f> normals;

    if(cloudPTR->size()>0)
    {

    

    for (int i = 0; i < cloudPTR->size(); i++)
    {
       
        
        Eigen::Vector3f position_aux= Eigen::Vector3f( (float) cloudPTR->points[i].x,(float) cloudPTR->points[i].y,(float) cloudPTR->points[i].z);
        points.push_back(position_aux);

        Eigen::Vector3f normal_aux= Eigen::Vector3f( (float) cloudPTR->points[i].normal_x,(float) cloudPTR->points[i].normal_y,(float) cloudPTR->points[i].normal_z);
        normals.push_back(normal_aux);
    }

     std::cout << points.size() << " points loaded." << std::endl;
     std::cout << normals.size() << " normals loaded." << std::endl;
    std::cout<<"\n";

    

     //pairing
    ppf::PairDetector pairDet(d_max, d_min, min_votes);
    PlaneGraph planeMap(pairDet.para_thresh(), pairDet.distance_bin());
    pairDet.detect_ortho_pairs(points, normals, planeMap);
    //planeMap.print_info();
    
    std::cout << std::endl << "Thresholds:\tAngle:\t" << pairDet.para_thresh() << "\tDistance:\t" << pairDet.distance_bin() << std::endl << std::endl;

    // // clustering & filtering
    
    planeMap.cluster_graph_vertices();
    // //planeMap.print_info();
    // //planeMap.print_parameters();
    // //planeMap.print_edges();
    
    // // find triangles
    std::vector<Graph<Plane>::Triangle> triangles;
    planeMap.find_triangles(triangles);
        
    // // reduce graph to ParallelPlaneGraph

    if (triangles.size()==0){
       std::cout<<"Nu sunt triunghiuri"<<'\n'; 
    }

    
    
    ParallelPlaneGraph redMap = planeMap.reduce_graph();
    
    
   
    redMap.triangle_reduce();
    redMap.filter_outliers(points, normals, pairDet.distance_bin(), .5*sampling);
  
 
//     // // CERES problem setup and solving
     double lambda = 0.01 * points.size();
    
    redMap.refine_coarse_fine(points, normals, lambda, pairDet.distance_bin(), sampling);
    
    //redMap.print_info();
   // redMap.print_parameters();
    //redMap.print_edges();
    
    
    redMap.filter_outliers(points, normals, pairDet.distance_bin(), .5*sampling);
    
   // redMap.print_info();
   // redMap.print_parameters();
   // redMap.print_edges();

    //
     // MUST ADD CHECK HERE

    ////

     PlaneGraph finalMap = PlaneGraph::from_ppg(redMap, 2*pairDet.para_thresh()*pairDet.para_thresh()-1, pairDet.distance_bin()); // double angle
      std::vector<Eigen::Matrix<float, 6, 1>> labeled;   

      //std::cout<<"I AM ALIVE"<<"\n";
      //std::cout<<labeled.size()<<"\n";

      finalMap.go_thr_pcl(points, normals, labeled);

       std::vector<Vec6> lines;
// //    // line extraction and visualization

     // std::cout<<"I am ALive n"<<"\n";
 
       lines = finalMap.extract_lines(points, normals);
   
      std::cout << lines.size() /2<< " lines found." << std::endl;

     std::vector<Vec6> new_lines;

     std::vector<Vec6> final_lines;
     std::vector<double> distances;

     std::vector<double> Volume_edge;

     std::vector<int> positions_wrong;
     std::vector<double> Volumes;

   

     std::cout<<'\n';

 if (lines.size()>=3)  
 {
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
                double latura_i = sqrt (  (lines[i][0]-lines[i][3])*(lines[i][0]-lines[i][3])  +  (lines[i][1]-lines[i][4])*(lines[i][1]-lines[i][4]) +(lines[i][2]-lines[i][5])*(lines[i][2]-lines[i][5])   ) +0.02;
                double latura_j = sqrt (  (lines[j][0]-lines[j][3])*(lines[j][0]-lines[j][3])  +  (lines[j][1]-lines[j][4])*(lines[j][1]-lines[j][4]) +(lines[j][2]-lines[j][5])*(lines[j][2]-lines[j][5])   ) +0.02;
                double latura_k = sqrt (  (lines[k][0]-lines[k][3])*(lines[k][0]-lines[k][3])  +  (lines[k][1]-lines[k][4])*(lines[k][1]-lines[k][4]) +(lines[k][2]-lines[k][5])*(lines[k][2]-lines[k][5])   ) +0.02;

                double Volum = latura_i*latura_j*latura_k;
                 
                // std::cout<<"Latura 1: "<<latura_i<<'\n';
                // std::cout<<"Latura 2: "<<latura_j<<'\n';
                // std::cout<<"Latura 3: "<<latura_k<<'\n';
                

                // std::cout<<"Volum="<<Volum<<'\n';

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

     
    if (new_lines.size()>=3){

      log<<"New Volume"<<'\n';
       for(int i = 0; i < new_lines.size(); i=i+3) {
      bool ok_position=1;

      
      if(positions_wrong.size()>0){
      for(int j = 0; j != positions_wrong.size(); j++) {
        if(i==positions_wrong[j]){
          ok_position=0;
        }
      
      
      }
      }
      if(ok_position){
        final_lines.push_back(new_lines[i]);
        final_lines.push_back(new_lines[i+1]);
        final_lines.push_back(new_lines[i+2]);
        std::cout<<"Volum "<<(i/3)<<": "<<Volumes[i]<<" m^3"<<'\n';
        ss<<"Volum "<<(i/3)<<": "<<Volumes[i]<<" m^3"<<'\n';

        log<<"Volum "<<(i/3)<<": "<<Volumes[i]<<" m^3 "<< ros::Time::now()<<'\n';

        log<<'\n';
       

        std::cout<< "distance1= "<<Volume_edge[i] <<" m"<<'\n';
        std::cout<< "distance2= "<<Volume_edge[i+1]<<" m" <<'\n';
        std::cout<< "distance3= "<<Volume_edge[i+2]<<" m" <<'\n';
        
        std::cout<<'\n';

      }
       }
      }



  
  

  
    
  
 }

 pcl::PointCloud<pcl::PointXYZ>::Ptr points_all_lines(new pcl::PointCloud<pcl::PointXYZ>);

  pcl::PointXYZ p;
  if(lines.size()>0){
    for(int i = 0; i < lines.size(); i=i+1) {
      p.x = lines[i][0];
      p.y = lines[i][1];
      p.z = lines[i][2];

      points_all_lines->points.push_back(p);

       p.x = lines[i][3];
       p.y = lines[i][4];
       p.z = lines[i][5];

      points_all_lines->points.push_back(p);
  }

   points_all_lines->width = points_all_lines->points.size();
   points_all_lines->height = 1;
   points_all_lines->points.resize(points_all_lines->width * points_all_lines->height);
   points_all_lines->is_dense = false;

  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr points_final_lines(new pcl::PointCloud<pcl::PointXYZ>);

  if(final_lines.size()>0){
    for(int i = 0; i < final_lines.size(); i=i+1) {
      p.x = final_lines[i][0];
      p.y = final_lines[i][1];
      p.z = final_lines[i][2];

      points_final_lines->points.push_back(p);

       p.x = final_lines[i][3];
       p.y = final_lines[i][4];
       p.z = final_lines[i][5];

      points_final_lines->points.push_back(p);
  }

   points_final_lines->width = points_final_lines->points.size();
   points_final_lines->height = 1;
   points_final_lines->points.resize(points_final_lines->width * points_final_lines->height);
   points_final_lines->is_dense = false;

   std::cout<<"Nr lines"<<points_final_lines->size()<<'\n';

  }



   std::stringstream header_camera;
    //header_camera << "camera_depth_optical_frame";
    header_camera << "pico_zense_depth_frame";
    //header_camera << "base_link";

    sensor_msgs::PointCloud2 tempROSMsg;
    pcl::toROSMsg(*points_all_lines, tempROSMsg);

    
    tempROSMsg.header.frame_id = header_camera.str();
    pub_.publish(tempROSMsg);

    sensor_msgs::PointCloud2 tempROSMsg2;
    pcl::toROSMsg(*points_final_lines, tempROSMsg2);

    
    tempROSMsg2.header.frame_id = header_camera.str();
    pub2_.publish(tempROSMsg2);



    

    

    visualization_msgs::Marker marker;
    marker.header.frame_id = header_camera.str();
    marker.text = ss.str();
    set_marker(marker);

    vis_pub.publish(marker);


    
       

 } 

   


   
    

    


}

private:
  ros::NodeHandle nh_;
  ros::Subscriber sub_;
  ros::Publisher pub_;
  ros::Publisher pub2_;
  ros::Publisher vis_pub;
  dynamic_reconfigure::Server<ppfplane::line_detect_nodeConfig> config_server_;

  int min_votes ;
  double d_min ;
  double d_max;
  int sampling ;
  double threshold;



};

/*
 * main
 */
int main(int argc, char** argv) {


   ros::init (argc, argv, "line_detect_node");
  

   

   
   

   LineDetectNode ldn;

   ros::spin();


    
}
 