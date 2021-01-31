#include <ros/ros.h>
#include <cstdlib>
#include <math.h>
#include <ctime>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <ros/console.h>

double gaussianRandom(double average, double stdev);
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
    image_sub_ = it_.subscribe("/camera/depth/image_rect_raw", 1, &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/image_example", 1);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);

    for(int i=0; i<cv_ptr->image.rows; i++) {
      for(int j=0; j<cv_ptr->image.cols; j++){
        int probability = rand() % 10000;
        if(probability < 10000*0.01) {
          cv_ptr->image.at<uint16_t>(i,j) = 0;
        }
        else {
          int average = cv_ptr->image.at<uint16_t>(i,j);
          double standardDev = 7 * pow(10, -7) * pow(average, 2.24);
          double result = gaussianRandom(average, standardDev);
          cv_ptr->image.at<uint16_t>(i,j) = (uint16_t) result;
        }
      }
    }
    // Output modified video stream
    printf("%d\n", cv_ptr->image.at<uint16_t>(340,250));
    image_pub_.publish(cv_ptr->toImageMsg());
  }
  
};

double gaussianRandom(double average, double stdev) {
  double v1, v2, s, temp;

  do {
    v1 =  2 * ((double) rand() / RAND_MAX) - 1;      // -1.0 ~ 1.0 까지의 값
    v2 =  2 * ((double) rand() / RAND_MAX) - 1;      // -1.0 ~ 1.0 까지의 값
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