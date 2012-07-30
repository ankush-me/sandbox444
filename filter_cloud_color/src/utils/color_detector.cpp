#include <cv.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <iostream>
#include <sstream>

using namespace cv;

VideoCapture init_video(std::string path) {
  // Try to open string, this will attempt to open it as a video file.
  VideoCapture capture(path);

  // If this fails, try to open as a video camera,
  // through the use of an integer param.
  if (!capture.isOpened())
    capture.open(atoi(path.c_str()));

  if (!capture.isOpened()) {
    std::cout<<"Failed to open a video instance with the given input. Trying index 0."<<std::endl;
    capture.open(0);
    if (!capture.isOpened()) {
      std::cerr << "Error: Failed to open a video device or video file!" << std::endl;
      return -1;
    }
  }

  //tell the capture to convert frames to RGB format
  capture.set(CV_CAP_PROP_CONVERT_RGB,1);
  //std::cout<<"Frame Rate: "<<capture.get(CV_CAP_PROP_FPS)<<std::endl;
  return capture;
}


void create_display_windows(std::string window_name="window") {
  namedWindow(window_name); //resizable
  //cvMoveWindow(window_name.c_str(), 675, 0);             // Position window
  //cvResizeWindow(window_name.c_str(), 675, 700);         // Resize window
}


struct binder {
  Mat *image;
  std::string window_name;
};


void mouseHandler(int event, int x, int y, int flags, void* data) {

  std::stringstream ss;
  if (event == CV_EVENT_LBUTTONDOWN) {
    binder image_data = *((binder*) data);
    Mat * image_in = image_data.image;
    std::string window = image_data.window_name;
    
    Mat image, imageHSV;
    image_in->copyTo(image);
    cvtColor(image, imageHSV,CV_BGR2HSV);

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

    char pix[20];
    std::sprintf(pix, "HSV : (%d, %d, %d)", hue, sat, value);
    std::string pixel_data = pix;


    /* display the BGR value */
    rectangle(image, Point(0,0), Point(50,50), CV_RGB(red,green,blue),CV_FILLED);    
    putText(image, pixel_data, Point(x+2,y+2), CV_FONT_HERSHEY_PLAIN,
	    1, Scalar(0,0,0));

    imshow(window, image);
  }
}


void process(VideoCapture &src, int delay=0) {
  Mat tmp;

  std::string window_name = "Color Sampler";
  create_display_windows(window_name);

  binder callback_data;
  callback_data.image = &tmp;
  callback_data.window_name = window_name;
  cvSetMouseCallback(window_name.c_str(), mouseHandler, (void*)&callback_data);

  while(1) {
    src.read(tmp);
    //tmp.copyTo(frame_p3c);
    //cvtColor(tmp,dst,CV_RGB2GRAY);
    imshow(window_name, tmp);
    waitKey(delay);
  }
}


int main(int argc, char** argv) {
  Mat src1, src2, dst;
 
  /// Ask the user enter path to the video file.
  string input;
  std::cout<<" Color value Sampler "<<std::endl;
  std::cout<<"---------------------"<<std::endl;
  std::cout<<"* Enter the full path to the videofile or the index of the camera: ";
  std::cin>>input;
  VideoCapture src = init_video(input);

  process(src);
  return 0;
}
