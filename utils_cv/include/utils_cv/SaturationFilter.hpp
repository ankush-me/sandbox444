/** Author: Ankush Gupta
    Date  : 13th September, 2012. */

#ifndef _SATURATION_FILTER_CV_UTILS_H_
#define _SATURATION_FILTER_CV_UTILS_H_

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

/** Simple class to filter an openCV mat in HUE SPACE. */
class SaturationFilter {
  uint8_t _s_min, _s_max;
 
public:
  SaturationFilter(uint8_t s_min=0, uint8_t s_max=255);

  void set_max(uint8_t max);
  void set_min(uint8_t min);

  uint8_t get_max();
  uint8_t get_min();

  /** Filters the input RGB image, SRC
      based on hue and stores the result in DEST.

      if MASK is set to true, it saves a binary
      [0==0, 1==255] image in DEST, else saves
      the inlier pixels into DEST [RGB values].

      An inlier is:
      _s_min <= pixel.saturation <= _s_max. */
  void filter(cv::Mat &src, cv::Mat &dest, bool mask=false);
};

#endif
