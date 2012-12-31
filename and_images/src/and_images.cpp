#include <ros/ros.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <sstream>
#include <vector>

#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/common/common_headers.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <utils_cv/ImageAND.hpp>
#include <utils_cv/CannyBlur.hpp>
#include <utils_cv/ImageProcessor.hpp>
#include <utils_cv/HueFilter.hpp>
#include <utils_cv/SaturationFilter.hpp>
#include <utils_pcl/CloudImageComm.hpp>


using namespace cv;


/** Dilates the SRC matrix into DST. */
void dilation(cv::Mat &src, cv::Mat &dst, int s=5 ) {
  int dilation_type = cv::MORPH_ELLIPSE;
  int dilation_size = s;
  cv::Mat element = cv::getStructuringElement(dilation_type,
					      cv::Size( 2*dilation_size + 1, 2*dilation_size+1),
					      cv::Point( dilation_size, dilation_size));
  cv::dilate(src, dst, element);
}

/** Erodes the SRC matrix into DST.
    See Morphological functions in opencv.*/
void erosion(cv::Mat &src, cv::Mat &dst, int s=5 ) {
  int erosion_type = cv::MORPH_ELLIPSE;
  int erosion_size = s;
  cv::Mat element = cv::getStructuringElement(erosion_type,
					      cv::Size( 2*erosion_size + 1, 2*erosion_size+1),
					      cv::Point( erosion_size, erosion_size));
  cv::erode(src, dst, element);
}


/** Returns a 2D pointcloud corresponding to the pixels in the
    IMG which are non-zero.
    The image is expected to be a single channel image. If not
    only the first channel is taken into consideration. */
pcl::PointCloud<pcl::PointXYZ>::Ptr
cloud3D_from_image(cv::Mat img) {
  std::vector<cv::Mat> channels;
  cv::split(img, channels);
  cv::Mat ch0 =  channels[0];

  pcl::PointCloud<pcl::PointXYZ>::Ptr
    out_cloud(new pcl::PointCloud<pcl::PointXYZ>);

  int non_zero_pixels = 0;

  for (int r = 0; r < img.rows; r+=1) {
    for (int c = 0; c < img.cols; c+=1) {
      if (ch0.at<uint8_t>(r,c) != 0) {
	pcl::PointXYZ pt;
	pt.x = c; pt.y = r; pt.z = 0;
	out_cloud->points.push_back(pt);
      }
    }
  }
  return out_cloud;
}


/** Returns an image corresponding to point in the cloud.*/
cv::Mat image_from_cloud3D(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
			   unsigned int rows, unsigned int cols) {
  cv::Mat img =  cv::Mat::zeros(cv::Size(cols, rows), CV_8UC1);
  for (int i = 0; i < cloud->points.size();i+=1)
    img.at<uint8_t>(cvRound(cloud->points.at(i).y),
		    cvRound(cloud->points.at(i).x)) = 255;
  return img;
}


/** Fits a circle to the points in the given 2D  pointcloud
    and saves them in the CENTER and RADIUS. */
void get_circle2D_ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr points,
			  cv::Vec3f &center,
			  float &radius, float ransac_thresh=0.002) {

  boost::shared_ptr<pcl::SampleConsensusModelCircle2D<pcl::PointXYZ> >
    model(new pcl::SampleConsensusModelCircle2D<pcl::PointXYZ>(points));

  pcl::RandomSampleConsensus<pcl::PointXYZ> sac(model, ransac_thresh);
  bool result = sac.computeModel();
  Eigen::VectorXf xyr;
  sac.getModelCoefficients (xyr);
  center[0] = xyr(0);
  center[1] = xyr(1);
  radius    = xyr(2);
}


/** Structure to hold the parametes of 2D circles. */
struct circle_info {
  cv::Point center;
  float radius;

  circle_info(circle_info  &o) {
    center = cv::Point(o.center.x, o.center.y);
    radius  = o.radius;
  }

  circle_info(Point pt, float r) {
    center = pt;
    radius  = r;
  }

  circle_info(int x=0, int y=0, float r=0) {
    center = cv::Point(x,y);
    radius  = r;
  }

  void operator=(const circle_info& o) {
    center = cv::Point(o.center.x, o.center.y);
    radius  = o.radius;
  }

  circle_info operator+(const circle_info&  other) {
    circle_info sum;
    sum.center = cv::Point(center.x + other.center.x,
			   center.y + other.center.y);
    sum.radius = radius + other.radius;
    return sum;
  }

  circle_info operator/(const float f) {
    circle_info div;
    div.center = cv::Point((int) (center.x/f),
			   (int) (center.y/f));
    div.radius = radius/f;
    return div;
  }
};


/** Class for calculating moving averages.
    Can also be used for accumulating last N
    samples of a moving quantity. */
template<typename T>
class averager {
private:
  std::list<boost::shared_ptr<T> > elements;
  unsigned int N;
  bool ready;

public:
  averager(unsigned int n) : elements(), N(n),
			     ready(false) {}
  
  void append_element(T &element) {
    if (ready)
      elements.pop_front();
    else
      ready = (elements.size() >= N);
    elements.push_back(boost::shared_ptr<T>(new T(element)));
  }

  bool is_ready() {return ready;}
  void clear() {elements.clear(); ready = false;}

  T average() {
    if (ready) {
      T avg;
      typename std::list<boost::shared_ptr<T> >::iterator it = elements.begin();
      for (; it != elements.end(); it++)
	avg = avg + (**it);
      avg = avg/N;
      return avg;
    } else {
      throw("Average requested prematurely.");
    }
  }
};



/** Class for calculating moving averages.
    Can also be used for accumulating last N
    samples of a moving quantity. */
template<typename T>
class cloud_accumulator {
private:
  std::list<typename pcl::PointCloud<T>::Ptr> _clouds;
  unsigned int N;
  bool ready;

public:
  cloud_accumulator(unsigned int n) : _clouds(), N(n),
				      ready(false) {}
  
  void append_element(typename pcl::PointCloud<T>::Ptr cloud) {
    if (ready)
      _clouds.pop_front();
    else
      ready = (_clouds.size() >= N);
    _clouds.push_back(cloud);
  }

  bool is_ready() {return ready;}
  void clear() {_clouds.clear(); ready = false;}
  unsigned int size() {return _clouds.size();}

  typename pcl::PointCloud<T>::Ptr get() {
    if (ready) {
      typename pcl::PointCloud<T>::Ptr sum(new pcl::PointCloud<T>);
      typename std::list<typename pcl::PointCloud<T>::Ptr >::iterator it = _clouds.begin();
      for (; it != _clouds.end(); it++)
	*sum = *sum + (**it);
      return sum;
    } else {
      throw("Error: Cloud sum requested prematurely.");
    }
  }
};


/** Structure to store HSV values. */
struct HSV {
  uint8_t h,s,v;
  typedef boost::shared_ptr<HSV> Ptr;

  HSV (uint8_t _h, uint8_t _s, uint8_t _v) {
    h=_h; s=_s; v=_v;
  }

  HSV (const HSV &o) {
    h=o.h; s=o.s; v=o.v;
  }
};


/** Structure to store RGB values. */
struct RGB {
  uint8_t r,g,b;
  typedef boost::shared_ptr<RGB> Ptr;

  RGB (uint8_t _r, uint8_t _g, uint8_t _b) {
    r=_r; g=_g; b=_b;
  }

  RGB (const RGB &o) {
    r=o.r; g=o.g; b=o.b;
  }
};


/** Converts opencv RGB values to HSV. */
HSV RGB_to_HSV(uint8_t r, uint8_t g, uint8_t b) {
  uint8_t pixel_data[] = {r,g,b};
  cv::Mat pixel(1,1,CV_8UC3, (void*) pixel_data);
  cv::Mat pixelHSV;
  cv::cvtColor(pixel,pixelHSV,CV_RGB2HSV);
  return HSV(pixelHSV.at<uint8_t>(0),
	     pixelHSV.at<uint8_t>(1),
	     pixelHSV.at<uint8_t>(2));
}

/** Converts opencv HSV values to RGB. */
RGB HSV_to_RGB(uint8_t h, uint8_t s, uint8_t v) {
  uint8_t pixel_data[] = {h,s,v};
  cv::Mat pixel(1,1,CV_8UC3, (void*) pixel_data);
  cv::Mat pixelRGB;
  cv::cvtColor(pixel,pixelRGB,CV_HSV2RGB);
  return RGB(pixelRGB.at<uint8_t>(0),
	     pixelRGB.at<uint8_t>(1),
	     pixelRGB.at<uint8_t>(2));
}



cv::Vec2i histograms(cv::Mat src,bool debug=false, int range_l = 40, 
		     int range_u = 90, int num_bins=20 ) {
  Mat hsv;
  cvtColor(src, hsv, CV_BGR2HSV);
  vector<Mat> channels;
  split(hsv, channels);

  Mat hue_channel = channels[0];

  /// Establish the number of bins
  int histSize = num_bins;

  /// Set the ranges ( for B,G,R) )
  float range[] = {range_l, range_u} ;
  const float* histRange = { range };

  bool uniform = true; bool accumulate = false;
  Mat hue_hist;

  // Compute the histograms:
  calcHist(&hue_channel, 1, 0, Mat(), hue_hist,
	   1, &histSize, &histRange, uniform, accumulate );

  ROS_INFO("HIST SIZE: %d", histSize);

  // Draw the histograms for B, G and R
  int hist_w = 512; int hist_h = 400;
  int bin_w = cvRound( (double) hist_w/histSize );

  Mat histImage( hist_h, hist_w, CV_8UC3, Scalar( 0,0,0) );
  /// Normalize the result to [ 0, histImage.rows ]
  normalize(hue_hist, hue_hist, 0, histImage.rows, NORM_MINMAX, -1, Mat());

  /// Draw for each channel
  for( int i = 1; i < histSize; i++ )  {
    RGB rgb =  HSV_to_RGB(range[0] + (range[1] - range[0])*(i-1)/histSize,255,100);

    line(histImage, Point( bin_w*(i-1), hist_h - cvRound(hue_hist.at<float>(i-1)) ) ,
	 Point(bin_w*(i), hist_h - cvRound(hue_hist.at<float>(i)) ),
	 CV_RGB(rgb.r,rgb.g,rgb.b), 2, 8, 0);
  }


  double max_val = -1;
  int max_loc;
  int lb=0;
  int ub=histSize-1;

  for(int i = 0; i < histSize; i++ )  {
    if (max_val < hue_hist.at<float>(i)) {
      max_val = hue_hist.at<float>(i);
      max_loc = i;
    }
  }

  int lower_bound = range[0];
  for(int i = max_loc ; i >=0; i--) {
    if (hue_hist.at<float>(i) < max_val/10) {
      std::cout<<"\tlb: "<<i<<std::endl;
      lower_bound = (int) (range[0] + (range[1] - range[0])*i/float(histSize));
      lb = i;
      break;
    }
  }


  int upper_bound = range[1]-1;
  for(int i = max_loc ; i < histSize; i++) {
    if (hue_hist.at<float>(i) < max_val/10) {
      std::cout<<"\tub: "<<i<<std::endl;
      upper_bound = (int) (range[0] + (range[1] - range[0])*i/float(histSize));
      ub = i;
      break;
    }
  }

  line(histImage, Point(lb*bin_w,0), Point(lb*bin_w, hist_h), CV_RGB(255,0,0), 2, 8, 0);
  line(histImage, Point(ub*bin_w,0), Point(ub*bin_w, hist_h), CV_RGB(255,0,0), 2, 8, 0);
  line(histImage, Point(max_loc*bin_w,0), Point(max_loc*bin_w, hist_h), CV_RGB(0,255,0), 2, 8, 0);

  if (debug) {
    ROS_INFO("Peak: %f | Lower: %d | Upper: %d",
	     max_val, lower_bound, upper_bound);

    namedWindow("calcHist Demo", CV_WINDOW_AUTOSIZE );
    imshow("calcHist Demo", histImage );
    waitKey(10);
  }

  return Vec2i(lower_bound, upper_bound);
}




/** Class for finding a needle (macro sized, circular)
    in an organized point cloud. */
class NeedleFinder : CloudImageComm {

  /** used for canny edge dectiong subsequent gaussian blurring. */
  ImageProcessor::Ptr cannyblur;

  /** ANDS several images together and presents the cummulative image. */
  ImageAND ander;

  /** Filters pixels in an image based on their hue value. */
  HueFilter hFilter;

  /** Filters pixels in an image based on their saturation values. */
  SaturationFilter sFilter;

  /** Used to visualize the point cloud of the needle found. */
  pcl::visualization::PCLVisualizer _viewer;

  /** _DEBUG if true prints out debugging statements and
      shows the needle point-cloud.
      _SHOULD_PROCESS is an internal flag which when true,
      indicates that the needle needs to be found. */
  bool _debug, _should_process;

  /** Averages the circle information. */
  averager<circle_info> circle_averager;
  
  /** Accumulates the needle pointcloud over the
      past five clouds found by 2d processing. */
  cloud_accumulator<pcl::PointXYZRGB> _needle_accumulator;

  /** Returns a pointcloud corresponding to the pixels in the
      IMG which are non-zero. Pixels which have NaN distance information
      are filtered out.
      The image is expected to be a single channel image. If not
      only the first channel is taken into consideration. */
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr
  cloud_from_image(cv::Mat img) {

    std::vector<cv::Mat> channels;
    cv::split(img, channels);
    cv::Mat ch0 =  channels[0];

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr
      out_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    int non_zero_pixels = 0;

    for (int r = 0; r < img.rows; r+=1) {
      for (int c = 0; c < img.cols; c+=1) {
	if (ch0.at<uint8_t>(r,c) != 0) {
	  non_zero_pixels += 1;
	  pcl::PointXYZRGB pt = _cloud_ptr->at(c,r);
	  if (pcl::isFinite(pt))
	    out_cloud->points.push_back(pt);
	}
      }
    }

    if (_debug) {
      ROS_INFO("Found %d non-zero pixels. Found %d finite points.",
	       non_zero_pixels, out_cloud->points.size());     
      if (!_viewer.updatePointCloud(out_cloud, "cloud")) {
	_viewer.addCoordinateSystem(1.0);
	_viewer.addPointCloud(out_cloud,"cloud");
      }
    }
    return out_cloud;
  }


  /** This is called whenever a new point-cloud is recieved. */
  void process() {
    cv::Rect roi(100,130,350,230);
    cv::Mat img(_img_cv, roi);
    if (_should_process) {
    

      Mat img2 = img.clone();
      Mat temp;
      sFilter.filter(img2,temp,false);
      imshow("sat", temp);
      waitKey(5);

      Vec2i bounds = histograms(temp,true);
      hFilter.set_min(bounds[0]);
      hFilter.set_max(bounds[1]);
      ROS_INFO("BOUNDS : %d,%d", bounds[0], bounds[1]);
      Mat hue_filtered;
      hFilter.filter(temp, hue_filtered);
      
      imshow("Hue", hue_filtered);
      waitKey(5);

      Mat dilated;
      cvtColor(hue_filtered, dilated, CV_BGR2GRAY);
      threshold(dilated, dilated, 5, 255, THRESH_BINARY);
      dilation(dilated, dilated);
      imshow("Dilation", dilated);
      waitKey(5);

    }
    imshow("Original", img);
    waitKey(5);
  }


public:
  
  void spin_viewer() {
    _viewer.spinOnce(50);
  }

  void get_needle_transform() {
    _should_process = true;
    _needle_accumulator.clear();
  }

  NeedleFinder(ros::NodeHandle * nh_ptr,
	       std::string cloud_topic="/camera/depth_registered/points",
	       bool debug=false,
	       uint8_t h_min=75, uint8_t h_max=90,
	       uint8_t s_min=50, uint8_t s_max=255) 
    : CloudImageComm(nh_ptr, cloud_topic),
      _viewer(), _debug(debug),
      _should_process(false),
      _needle_accumulator(2),
      cannyblur(new CannyBlur),
      ander(cannyblur, 5),
      circle_averager(10),
      hFilter(h_min, h_max),
      sFilter(s_min, s_max){ }
};


int main(int argc, char** argv) {
  ros::init(argc, argv, "and_image_node");
  ros::NodeHandle nh("~");

  bool DEBUG;
  nh.param<bool>("debug", DEBUG, false);
  std::cout<<"DEBUG: "<<DEBUG<<std::endl;

  std::string topic; 
  nh.param<std::string>("cloud_topic", topic, 
			"/drop/points");

  NeedleFinder n_finder(&nh, topic, DEBUG);
  n_finder.get_needle_transform();

  ros::Rate rate(120);
  while(ros::ok()) {
    if (DEBUG)
      n_finder.spin_viewer();     
  ros::spinOnce();
  rate.sleep();
  }

  return 0;
}

/*////////////////// EXPERIMENTAL /////////////////////
Mat temp,inliers;
      
dilation(color_mask, temp,2);
GaussianBlur(temp, temp, cv::Size(3,3), 2, 2);
threshold(temp, temp, 5, 255, THRESH_BINARY);
erosion(temp, temp, 3);

imshow("Color Mask", color_mask);
waitKey(5);
      
vector<vector<Point> > contours;
vector<Vec4i> hierarchy;
findContours(temp, contours, hierarchy,
	     CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0,0));

Mat drawing = Mat::zeros(temp.size(), CV_8UC1);
for( int i = 0; i< contours.size(); i++ ) {
  double contArea = contourArea(contours[i]);
  if (contArea > 10)
    drawContours(drawing, contours, i, 255, 2, 8,
		 hierarchy, 0, Point());
 }

imshow("blob", drawing);
waitKey(5);*/
