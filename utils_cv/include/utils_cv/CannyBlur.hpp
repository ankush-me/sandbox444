/** Author: Ankush Gupta
    Date  : 4th Sept, 2012. */

#include <utils_cv/ImageProcessor.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <iostream>
#include <vector>


/** Does canny edge detection followed by gaussian blurring
    followed by thresholding all non-zero values to 255. */
class CannyBlur : ImageProcessor {
private:

  /** TRUE if ROI is given in the constructor. */
  bool _use_roi;

  /** The ROI bounds. */
  cv::Rect _roi;

  /** If true shows the image at every process call. */
  bool _is_debug;

public:

  /** Parameterless constructor. */
  CannyBlur();

  /** ROI sets the region of interest.
      if DEBUG==true : shows the images. */ 
  CannyBlur(cv::Rect roi, bool debug=false);

  void process(cv::Mat &src, cv::Mat &dst);
};

