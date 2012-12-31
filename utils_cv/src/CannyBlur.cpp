/** Author: Ankush Gupta
    Date  : 4th Sept, 2012. */

#include <utils_cv/CannyBlur.hpp>

/** Parameterless constructor. */
CannyBlur::CannyBlur(bool debug) : ImageProcessor(),
			 _roi(), _use_roi(false), _is_debug(debug) {}

/** ROI sets the region of interest.
    if DEBUG==true : shows the images. */ 
CannyBlur::  CannyBlur(cv::Rect roi, bool debug) : ImageProcessor(),
						   _roi(roi), _is_debug(debug),
						   _use_roi(true) {}

void CannyBlur::process(cv::Mat &src, cv::Mat &dst) {

  cv::Mat in_img0  = src.clone();
  cv::Mat in_img;
  // Set the image region of interest.
  if (_use_roi)
    in_img = cv::Mat(in_img0, _roi);
  else
    in_img = in_img0;

  // Do canny edge detection and blurring.
  int thresh = 100;
  cv::Canny(in_img, dst, thresh, thresh*2, 3);

  if (_is_debug) {
    cv::imshow( "Canny", dst);
    cv::waitKey(5);
  }



  if (_is_debug) {
    cv::Mat debug_mat;
    cv::GaussianBlur(dst, debug_mat, cv::Size(9,9), 2, 2);

    std::vector<cv::Vec3f> circles;
    cv::HoughCircles(debug_mat, circles, CV_HOUGH_GRADIENT,
		     2, debug_mat.rows/3, 200, 100,50,75);

    for( size_t i = 0; i < circles.size(); i++ ) {
      cv::Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
      int radius = cvRound(circles[i][2]);
      // Draw the circle center
      cv::circle(in_img0, center, 3, cv::Scalar(0,255,0), -1, 8, 0);
      // Draw the circle outline
      cv::circle(in_img0, center, radius, cv::Scalar(0,0,255), 7, 8, 0);
    }
  
    cv::imshow("Hough Circles", in_img0);
    cv::waitKey(5);
  }
}

