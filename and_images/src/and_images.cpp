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


/** Class for finding a needle (macro sized, circular)
    in an organized point cloud. */
class NeedleFinder : CloudImageComm {

  ImageProcessor::Ptr cannyblur;
  ImageAND ander;
  HueFilter hFilter;
  SaturationFilter sFilter;
  pcl::visualization::PCLVisualizer _viewer;
  bool _debug;
  averager<circle_info> circle_averager;

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
    cv::Rect roi(100,50,420,350);
    cv::Mat img(_img_cv, roi);

    ander.update(img);

    if (ander.is_ready()) {
      cv::Mat and_img = ander.get();

      cv::Mat hue_mask, sat_mask, color_mask;
      hFilter.filter(img, hue_mask, true);
      sFilter.filter(img, sat_mask, true);
      cv::bitwise_and(hue_mask, sat_mask, color_mask);
      
      Mat circular_mask = Mat::zeros(img.size(), CV_8UC1);
      pcl::PointCloud<pcl::PointXYZ>::Ptr image_cloud =  cloud3D_from_image(color_mask);
      if (image_cloud->points.size() != 0) {
	Vec3f center; float radius;
	get_circle2D_ransac(image_cloud, center, radius);
	Point pcenter(cvRound(center[0]), cvRound(center[1]));

	if (radius < 150 && radius > 50) {
	  circle_info info(pcenter, radius);
	  circle_averager.append_element(info);
	}

	if (circle_averager.is_ready()) {
	  circle_info avg_circle;
	  avg_circle = circle_averager.average();
	
	  circle(circular_mask, avg_circle.center, avg_circle.radius, 255, 10, 8, 0);
	  imshow("Circle mask", circular_mask);
	  waitKey(5);

	  bitwise_and(color_mask, circular_mask, circular_mask);
	  cloud_from_image(circular_mask);
	}
      }
    }
  }

public:
  
  void spin_viewer() {
    _viewer.spinOnce(50);
  }

  NeedleFinder(ros::NodeHandle * nh_ptr,
	       std::string cloud_topic="/camera/depth_registered/points",
	       bool debug=false,
	       uint8_t h_min=75, uint8_t h_max=90,
	       uint8_t s_min=50, uint8_t s_max=255) 
    : CloudImageComm(nh_ptr, cloud_topic),
      _viewer(), _debug(debug),
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
			"/camera/depth_registered/points");

  NeedleFinder n_finder(&nh, topic, DEBUG);

  ros::Rate rate(60);
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
