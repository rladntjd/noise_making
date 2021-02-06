#include <ros/ros.h>
#include <cstdlib>
#include <math.h>
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
double average ;
double standardDev; 
double result ;

float gaussianRandom(float average, float stdev);
class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;

public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/airsim_node/drone_1/front_left_custom/DepthPlanner", 1, &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/image_example", 1);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    sensor_msgs::ImagePtr message;

    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
    //cv::Mat int_image(480,640,CV_16UC1);

    for(int i=0; i<480; i++) {
      for(int j=0; j<640; j++){
        //  int_image.at<uint16_t>(i,j) = (uint16_t) (1000 * cv_ptr->image.at<_Float32>(i,j));
        probability = rand() % 10000;
        if(cv_ptr->image.at<float>(i,j) > 15.) {
          cv_ptr->image.at<float>(i,j) = 15.;
        }
        if(probability < 10000*0.01) {
          cv_ptr->image.at<float>(i,j) = 0;
        }
        else {
          average = cv_ptr->image.at<float>(i,j);
          cv_ptr->image.at<float>(i,j) = (float) gaussianRandom(average, 0.005 * pow(average, 2.24));
        }
        //cv_ptr->image.at<_Float32>(i,j) = (_Float32) 1./1000. * int_image.at<uint16_t>(i,j);
      }
    }
    // Output modified video stream
    printf("%f\n", cv_ptr->image.at<float>(100,600));
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
  srand((unsigned int)time(NULL));
  ImageConverter ic;
  ros::spin();
  return 0;
}
