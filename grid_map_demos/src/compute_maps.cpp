#include <ros/ros.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_cv/grid_map_cv.hpp>
#include <vector>
#include <string>
#include <cmath>
#include <limits>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>


using namespace grid_map;

int compute_cost_flatness(float*,float*,int,int);


ros::Publisher publisher;
void compute_depth_score(grid_map::GridMap& map)
{
  grid_map::Matrix copy = map["elevation"];
  grid_map::Matrix data = Eigen::MatrixXf::Zero(copy.rows(), copy.cols());
  //////////////////eigen-cv

cv::Mat_<float> a = cv::Mat_<float>::ones(copy.rows(),copy.cols());
cv::Mat_<float> one = cv::Mat_<float>::zeros(copy.rows(),copy.cols());
cv::Mat_<float> zero = cv::Mat_<float>::zeros(copy.rows(),copy.cols());

eigen2cv(copy,a);

cv::Mat img(copy.rows(),copy.cols(), CV_8UC1, cv::Scalar(2));

double min,max;
double diff =cv::sum(a)[0];
std::cout<<diff<<"\n";

grid_map::Matrix data_cv = Eigen::MatrixXf::Zero(copy.rows(), copy.cols());

cv2eigen(img,data_cv);

map.add("depth_score1", data_cv);
  

 
  
}

void compute_normals_score(grid_map::GridMap& map)
{
  grid_map::Matrix& data = map["normal_vectors_z"];

  float min_angle = 15;
  //find theta
  data = Eigen::abs(data.array()).acos();
  data = data * (180/3.14);
  data = Eigen::exp(-1 * Eigen::pow(data.array(),2)/(1.5*std::pow(min_angle,2)));
  
  
  map.add("normals_score", data);
  
  
}


void compute_flatness_score(grid_map::GridMap& map)
{ 
  grid_map::Matrix copy = map["elevation"];
  grid_map::Matrix data = Eigen::MatrixXf::Ones(copy.rows(), copy.cols());
  float *cell = (float*)malloc(sizeof(float)*copy.rows()*copy.cols());
  float *node = (float*)malloc(sizeof(float)*copy.rows()*copy.cols());
  


  memcpy(cell, copy.data(),sizeof(float)*copy.rows()*copy.cols()) ;
  memcpy(node,data.data(),sizeof(float)*copy.rows()*copy.cols()) ;
  free(cell); free(node);

  for (GridMapIterator it(map); !it.isPastEnd(); ++it) {
      
      Position position;
      map.getPosition(*it, position);
      Index index(*it);
      Position center(position.x(), position.y());
      float center_val = copy(index(0),index(1));
      float point_val=0.0;
      double radius = 0.4;
      float sum=0.0;
      if(!std::isnan(center_val))
      {// data(index(0), index(1)) =1;
        
          for (int x=index(0)-25 ; x< index(0)+25 ; x++)
         {
            for(int y=index(1)-25; y< index(1)+25 ; y++)
           { 
            if (x>0 && y>0 &&x < copy.rows() && y<copy.cols() && !std::isnan(copy(x,y)))
            sum= sum + pow(center_val - copy(x,y),2);
            
          
            }
            
          if(index(0)>25 && index(1)>25 && index(0)<copy.rows()-25 && index(1)<copy.cols()-25)
          {float norm =sqrt(sum);
          
           data(index(0), index(1)) = norm>10?10:norm; //max tolerable limit of diff norm =10
           }
            else
              data(index(0), index(1)) = 10;  
          
          }

      }
      else
        data(index(0), index(1)) =10; 
        

     data(index(0),index(1))= (10 - data(index(0),index(1)))/5; //flatness score now in range 0 to 2



    }
  
  std::cout<<"######  1  ######"<<std::endl;




  map.add("flatness_score", data);
  
  
}



void compute_distance_transform_score(grid_map::GridMap& map)
{ 
  grid_map::Matrix copy = map["total_score_no_normals"];
  grid_map::Matrix data = copy;
  

 //////////////////eigen-cv

cv::Mat_<float> a = cv::Mat_<float>::ones(copy.rows(),copy.cols());
cv::Mat_<float> flat = cv::Mat_<float>::ones(copy.rows(),copy.cols());


eigen2cv(data,a);

cv::Mat img(copy.rows(),copy.cols(), CV_8UC1, cv::Scalar(0));
cv::threshold(a, img, 0.4, 1, cv::THRESH_BINARY);
cv::normalize(img,img, 1, 0, cv::NORM_MINMAX, CV_8UC1);

cv::Mat dist2;

cv::distanceTransform(img, dist2, cv::DIST_L2, 3);
//cv::normalize(dist2,dist2,2,0, cv::NORM_MINMAX, CV_8UC1);

if(cv::sum(dist2)[0]<0.001)
  dist2 = flat*5.0;

grid_map::Matrix data_cv = Eigen::MatrixXf::Zero(copy.rows(), copy.cols());

cv2eigen(dist2,data_cv);

 //////////////////

  map.add("distance_transform_score", data_cv);
  
  
}


void compute_total_score(grid_map::GridMap& map)
{ 
  grid_map::Matrix depth = map["depth_score1"];
  grid_map::Matrix normals = map["normals_score"];
  grid_map::Matrix flatness = map["flatness_score"];
  grid_map::Matrix total_score = Eigen::MatrixXf::Zero(depth.rows(), depth.cols());
  

  for (GridMapIterator it(map); !it.isPastEnd(); ++it) {
      
      Index index(*it);
      float score_val = depth(index(0),index(1)) + normals(index(0),index(1)) + flatness(index(0),index(1));
      
      if(!std::isnan(score_val))
      { float c1 = 0.2;
        float c2 = 0.3;
        float c3 = 0.5;

        total_score(index(0),index(1)) = (c1*depth(index(0),index(1)) + c2*normals(index(0),index(1)) + c3*flatness(index(0),index(1)))/3.0;    
      }

      else
        total_score(index(0),index(1)) = 0.0;


    }
  
 

  map.add("total_score", total_score);
  
  
}


void compute_total_score_no_normals(grid_map::GridMap& map)
{ 
  grid_map::Matrix depth = map["depth_score1"];
  grid_map::Matrix flatness = map["flatness_score"];
  grid_map::Matrix total_score = Eigen::MatrixXf::Zero(depth.rows(), depth.cols());
  

  for (GridMapIterator it(map); !it.isPastEnd(); ++it) {
      
      Index index(*it);
      float score_val = depth(index(0),index(1)) + flatness(index(0),index(1));
      
      if(!std::isnan(score_val))
      { float c1 = 0.3;
        
        float c3 = 0.7;

        total_score(index(0),index(1)) = (c1*depth(index(0),index(1)) +  c3*flatness(index(0),index(1)))/3.0;    
      }

      else
        total_score(index(0),index(1)) = 0.0;


    }
  
 

  map.add("total_score_no_normals", total_score);
  
  
}


void callback(const grid_map_msgs::GridMap& message)
{
  // Convert message to map.
  GridMap inputMap;
  GridMapRosConverter::fromMessage(message, inputMap);

  // Apply filter chain.
  grid_map::GridMap outputMap,map;
  outputMap=inputMap;
 
 compute_depth_score(outputMap);
 compute_flatness_score(outputMap);
  
 compute_total_score_no_normals(outputMap);
 compute_distance_transform_score(outputMap);


 grid_map_msgs::GridMap message1;
 grid_map::GridMapRosConverter::toMessage(outputMap, message1);
 publisher.publish(message1);

 

  

}



int main(int argc, char** argv)
{
  // Initialize node and publisher.
  ros::init(argc, argv, "grid_map_test_demo");
  ros::NodeHandle nh("~");
  publisher = nh.advertise<grid_map_msgs::GridMap>("my_grid_map", 1, true);
  //ros::Subscriber sub = nh.subscribe("/grid_map_filter_demo/filtered_map", 1, callback);
  ros::Subscriber sub = nh.subscribe("/elevation_mapping/elevation_map", 1, callback);
  ros::spin();
 
  return 0;
}
