#include <ros/ros.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <iostream>

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

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <utils_cv/ImageAND.hpp>
#include <utils_cv/ImageProcessor.hpp>
#include <utils_cv/HueFilter.hpp>
#include <utils_cv/SaturationFilter.hpp>
#include <utils_pcl/CloudImageComm.hpp>
#include <utils_pcl/cloud_utils.hpp>


using namespace std;
using namespace cv;

const cv::Scalar RED = Scalar(0,0,255);
const cv::Scalar PINK = Scalar(230,130,255);
const cv::Scalar BLUE = Scalar(255,0,0);
const cv::Scalar LIGHTBLUE = Scalar(255,255,160);
const cv::Scalar GREEN = Scalar(0,255,0);

const int BGD_KEY = CV_EVENT_FLAG_CTRLKEY;
const int FGD_KEY = CV_EVENT_FLAG_SHIFTKEY;

void getBinMask( const Mat& comMask, Mat& binMask ) {
    if( comMask.empty() || comMask.type()!=CV_8UC1 )
        CV_Error( CV_StsBadArg, "comMask is empty or has incorrect type (not CV_8UC1)" );
    if( binMask.empty() || binMask.rows!=comMask.rows || binMask.cols!=comMask.cols )
        binMask.create( comMask.size(), CV_8UC1 );
    binMask = comMask & 1;
}

/** Dilates the SRC matrix into DST. */
void dilation(cv::Mat &src, cv::Mat &dst, int s=5 ) {
  int dilation_type = cv::MORPH_ELLIPSE;
  int dilation_size = s;
  cv::Mat element = cv::getStructuringElement(dilation_type,
					      cv::Size( 2*dilation_size + 1, 2*dilation_size+1),
					      cv::Point( dilation_size, dilation_size));
  cv::dilate(src, dst, element);
}

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
      lower_bound = (int) (range[0] + (range[1] - range[0])*i/float(histSize));
      lb = i;
      break;
    }
  }


  int upper_bound = range[1]-1;
  for(int i = max_loc ; i < histSize; i++) {
    if (hue_hist.at<float>(i) < max_val/10) {
      upper_bound = (int) (range[0] + (range[1] - range[0])*i/float(histSize));
      ub = i;
      break;
    }
  }


  if (debug) {
    line(histImage, Point(lb*bin_w,0), Point(lb*bin_w, hist_h), CV_RGB(255,0,0), 2, 8, 0);
    line(histImage, Point(ub*bin_w,0), Point(ub*bin_w, hist_h), CV_RGB(255,0,0), 2, 8, 0);
    line(histImage, Point(max_loc*bin_w,0), Point(max_loc*bin_w, hist_h), CV_RGB(0,255,0), 2, 8, 0);

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
class GrabcutNeedle : CloudImageComm {

  /** Filters pixels in an image based on their hue value. */
  HueFilter hFilter;

  /** Filters pixels in an image based on their saturation values. */
  SaturationFilter sFilter;

  /** ANDS several images together and presents the cummulative image. */
  ImageAND ander;

  /** Used to visualize the point cloud of the needle found. */
  pcl::visualization::PCLVisualizer _viewer;

  /** Returns a pointcloud corresponding to the pixels in the
      IMG which are non-zero. Pixels which have NaN distance information
      are filtered out.
      The image is expected to be a single channel image. If not
      only the first channel is taken into consideration. */
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr
  cloud_from_image(cv::Mat img, bool _debug=true) {

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
    cv::Rect roi(120,130,350,230);
    cv::Mat img(_img_cv, roi);
    Mat img2 = img.clone();

    Mat temp;
    sFilter.filter(img2,temp,false);

    Vec2i bounds = histograms(temp);
    hFilter.set_min(bounds[0]);
    hFilter.set_max(bounds[1]);
    Mat hue_filtered;
    hFilter.filter(temp, hue_filtered);

    Mat dilated;
    cvtColor(hue_filtered, dilated, CV_BGR2GRAY);
    threshold(dilated, dilated, 10, 255, THRESH_BINARY);
    dilation(dilated, dilated,10);
    dilation(dilated, dilated,2);
    threshold(dilated, dilated, 0, GC_PR_FGD, THRESH_BINARY);
    imshow("Dilation-GC", 50*dilated);
    waitKey(5);

    Mat bgModel, fgModel;
    grabCut(img2, dilated, cv::Rect(), bgModel,
	    fgModel, 5, GC_INIT_WITH_MASK);
    Mat binMask, gc_out;
    getBinMask(dilated, binMask);

    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
    findContours(binMask, contours, hierarchy,
		 CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0,0));
    Mat contoured = Mat::zeros(binMask.size(), CV_8UC1);
    for( int i = 0; i< contours.size(); i++ ) {
      double contArea = contourArea(contours[i]);
      if (contArea > 10)
	drawContours(contoured, contours, i, 255, 2, 8,
		     hierarchy, 2, Point());
    }

    ander.update (contoured);

    if (ander.is_ready()) {
      Mat needle_pix = ander.get();
      img2.copyTo(gc_out, needle_pix);
      imshow("GrabCut", gc_out);
      waitKey(5);

      pcl::PointCloud<pcl::PointXYZRGB>::Ptr needle_cloud = cloud_from_image(gc_out,false);
      circle3d c3d(needle_cloud);
      c3d.compute_circle3d(false);
      Eigen::Vector3f pt(needle_cloud->points[0].x,
			 needle_cloud->points[0].y,
			 needle_cloud->points[0].z);

      Eigen::MatrixXf frame1, frame2;
      c3d.get_end_frames(frame1, frame2);
      c3d.add_frame(frame1, "frame1");
      c3d.add_frame(frame2, "frame2");  

      while (ros::ok()) {
	c3d.spin_viewer();
	boost::this_thread::sleep (boost::posix_time::microseconds (100000));
      }
    }

    imshow("Original", img);
    waitKey(5);
  }


public:
  
  void spin_viewer() {
    _viewer.spinOnce(50);
    
  }

  
  GrabcutNeedle(ros::NodeHandle * nh_ptr,
	       std::string cloud_topic="/camera/depth_registered/points",
	       uint8_t h_min=75, uint8_t h_max=90,
	       uint8_t s_min=150, uint8_t s_max=255) 
    : CloudImageComm(nh_ptr, cloud_topic),
      ander(5), _viewer(),
      hFilter(h_min, h_max),
      sFilter(s_min, s_max){ }
};


int main(int argc, char** argv) {
  ros::init(argc, argv, "grabcut_needle");
  ros::NodeHandle nh("~");

  bool DEBUG;
  nh.param<bool>("debug", DEBUG, false);
  std::cout<<"DEBUG: "<<DEBUG<<std::endl;

  std::string topic; 
  nh.param<std::string>("cloud_topic", topic, 
			"/drop/points");

  GrabcutNeedle gNeedle(&nh, topic);

  ros::Rate rate(120);
  while(ros::ok()) {
    if (DEBUG)
      gNeedle.spin_viewer();     
  ros::spinOnce();
  rate.sleep();
  }

  return 0;
}
