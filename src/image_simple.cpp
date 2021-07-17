#include <ros/ros.h>
#include <cstdlib>
#include <math.h>
#include <string>
#include <ctime>
#include <sstream>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <ros/console.h>

int probability;
int trans_rand; 
int surface_num;
int image_width;
int image_height;

struct parameter{
  int color[3];
  float constant;
  float power;
  int transparency;
};
struct parameter seg_to_constant;
// static uint8_t color[5][3] = {{6, 206, 29},{70, 76, 194},{26, 27, 45},{152, 67, 200},{1, 60, 44}};
// static float constant[5][2] = {{0.001106, 0.05335}, {0.001507, 0.0843}, {0.03485, 0.1851}, {0.001082, 0.01455}, {0.001552, 0.00654}};
// static float power[5][2] = {{1.966, 0.2968}, {1.526, 0.3572}, {1.497, 0.2523}, {1.245, 0.4688}, {0.997, 0.4292}};
// static int trans_prob;

float max(float a, float b){
  if(a > b) {
    return a;
  }
  else{
    return b;
  }
}

float min(float a, float b){
  if(a > b) {
    return b;
  }
  else{
    return a;
  }
}

float gaussianRandom(float average, float stdev);
class ImageConverter
{
  ros::NodeHandle nh_;
  

  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Subscriber image_sub_seg_;
  image_transport::Publisher image_pub_;
  cv_bridge::CvImagePtr Segmentation_ptr;
  cv::Mat img_original_ptr;
public:

  ImageConverter()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/airsim_node/drone_1/front_center_custom/DepthPlanar", 1, &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/image_example", 1);
  }
  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    sensor_msgs::ImagePtr message;
    float deviation = 0; 
    float average ;
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
    img_original_ptr = cv_ptr->image.clone();

    for(int i=0; i<image_height; i++) {
      for(int j=0; j<image_width; j++){
        probability = rand() % 10000;
        trans_rand = rand() % 10000;

        //fill-out error
        if(cv_ptr->image.at<float>(i,j) > 13. || probability < 10000*0.01) {
          cv_ptr->image.at<float>(i,j) = 0;
        }
        else{
            // 노이즈 적용
            average = cv_ptr->image.at<float>(i,j);
            //printf("%f\n", average);
            if(average < 3.){
              deviation = 0.0014 * exp(1.1 * average);
              cv_ptr->image.at<float>(i,j) =  gaussianRandom(average, deviation);
              // cv_ptr->image.at<float>(i,j) =  gaussianRandom(average, seg_to_constant[k].constant[0] * exp(seg_to_constant[k].power[0]));
              // deviation = seg_to_constant[k].constant[0] * exp(seg_to_constant[k].power[0] * average);
            }
            else {
              deviation = seg_to_constant.constant * exp(seg_to_constant.power * average);
              cv_ptr->image.at<float>(i,j) = gaussianRandom(average, deviation);
              // cv_ptr->image.at<float>(i,j) =  gaussianRandom(average, seg_to_constant[k].constant[1] * exp(seg_to_constant[k].power[1]));
              // deviation = seg_to_constant[k].constant[1] * exp(seg_to_constant[k].power[1] * average);
            }
            break;
          
          }
        }
      }
    
    printf("ori_dist : %f\n", cv_ptr->image.at<float>(200,200));
    printf("deviation : %f\n", deviation);
    // Output modified video stream
    //printf("%f\n", cv_ptr->image.at<float>(100,600));
    // message = cv_bridge::CvImage(std_msgs::Header(), "32FC1", int_image).toImageMsg();
    image_pub_.publish(cv_ptr->toImageMsg());
  }
  
};

float gaussianRandom(float average, float stdev) {
  float v1, v2, s, temp;

  do {
    v1 =  2 * ((float) rand() / RAND_MAX) - 1;      // -1.0 ~ 1.0 까지의 값
    v2 =  2 * ((float) rand() / RAND_MAX) - 1;      // -1.0 ~ 1.0 까지의 값
    s = v1 * v1 + v2 * v2;
  } while (s >= 1 || s == 0);

  s = sqrt( (-2 * log(s)) / s );

  temp = v1 * s;
  temp = (stdev * temp) + average;

  return temp;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ros::NodeHandle nh;
  nh.getParam("/image_pub_cpp/surface_number",surface_num); //surface 개수
  nh.getParam("/image_pub_cpp/image/height",image_height); //image 크기
  nh.getParam("/image_pub_cpp/image/width",image_width); 
  nh.getParam("/image_pub_cpp/surface/power",seg_to_constant.power); // 지수승
  seg_to_constant.constant = 0.03796 / exp(seg_to_constant.power * 3); // power 값이 들어오면 constant 값 계산

  srand((unsigned int)time(NULL));
  // for(int i=0; i<5; i++) {
  //   for(int j=0; j<3; j++) {
  //     seg_to_constant[i].color[j] = color[i][j];
  //     if(j<3) {
  //       seg_to_constant[i].constant[j] = constant[i][j];
  //       seg_to_constant[i].power[j] = power[i][j];
  //     }
  //   }
  // }
  ImageConverter ic;
  ros::spin();
  return 0;
}
