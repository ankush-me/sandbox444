/** Author: Ankush Gupta
    Date  : 31st August, 2012. */

#ifndef _HUE_FILTER_CV_UTILS_H_
#define _HUE_FILTER_CV_UTILS_H_

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

/** Simple class to filter an openCV mat in HUE SPACE. */
class HueFilter {
  uint8_t _h_min, _h_max;
 
public:
  HueFilter(uint8_t h_min=0, uint8_t h_max=255);

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
      _h_min <= pixel.hue <= _h_max    
      
      [for circular wrapping around hue=180,
        set _h_min > h_max ]                 */
  void filter(cv::Mat &src, cv::Mat &dest, bool mask=false);
};

#endif
