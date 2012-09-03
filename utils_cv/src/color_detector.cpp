#include <ros/ros.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <sstream>

using namespace cv;
bool write_frame;
std::string window_name;


struct binder {
  boost::shared_ptr<Mat> image;
  std::string window_name;
};
binder callback_data;


void create_display_windows(std::string window_name="window") {
  namedWindow(window_name);
  waitKey(10);
}


void mouseHandler(int event, int x, int y, int flags, void* data) {
  std::stringstream ss;
  if (event == CV_EVENT_LBUTTONDOWN) {
    binder image_data = *((binder*) data);
    boost::shared_ptr<Mat> image_in(image_data.image);

    std::string window = image_data.window_name;

    
    Mat image, imageHSV;
    image_in->copyTo(image);
 
    cvtColor(image, imageHSV,CV_RGB2HSV);

    /* read pixel : RGB and HSV*/
    IplImage img = image;
    uchar* ptr_rgb = cvPtr2D(&img, y, x, NULL);
    int blue = (int) ptr_rgb[0];
    int green = (int) ptr_rgb[1];
    int red   = (int) ptr_rgb[2];

    img = imageHSV;
    uchar* ptr_hsv = cvPtr2D(&img, y, x, NULL);
    int hue = (int) ptr_hsv[0];
    int sat = (int) ptr_hsv[1];
    int value   = (int) ptr_hsv[2];

    char pix[25];
    std::sprintf(pix, "HSV : (%d, %d, %d)", hue, sat, value);
    std::string pixel_data = pix;

    /* display the HSV value */
    rectangle(image, Point(0,0), Point(50,50), CV_RGB(red,green,blue),CV_FILLED);    
    putText(image, pixel_data, Point(x+2,y+2), CV_FONT_HERSHEY_PLAIN,
	    1, Scalar(0,0,0));

    imshow(window, image);
  }
}

void imageCB(const sensor_msgs::Image::ConstPtr img) {
  if (write_frame) {
    cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(img);
    } catch (cv_bridge::Exception& ex1) {
      ROS_ERROR("cv_bridge exception: %s", ex1.what());
      return;
    }

    callback_data.image.reset();
    callback_data.image = boost::shared_ptr<Mat>(new Mat(cv_ptr->image.clone()));
    imshow(window_name, *(callback_data.image));
  }
}


int main(int argc, char** argv) {

  ros::init(argc, argv, "color_reader");
  ros::NodeHandle nh("color_reader");

  write_frame = true;
  window_name = "color reader";

  callback_data.image = boost::shared_ptr<Mat>(new Mat(480,640,CV_8UC3));
  callback_data.window_name = window_name;

  create_display_windows(window_name);
  cvSetMouseCallback(window_name.c_str(), mouseHandler, (void*)&callback_data);

  std::string topic;
  
  nh.param<std::string>("topic", topic, "/wide_stereo/left/image_rect_color");
  ros::Subscriber sub = nh.subscribe(topic, 1, imageCB);

  while(ros::ok()) {
    char k = waitKey(10);
    if (k=='t')
      write_frame = !write_frame;
    ros::spinOnce();	  
  }

  return 0;
}
