/** Author: Ankush Gupta
    Date  : 31st August, 2012. */

#include <cv.hpp>

/** Simple class to filter an openCV mat in HUE SPACE. */
class hueFilter {
  uint8_t _h_min, _h_max;
 
public:
  hueFilter(uint8_t h_min=0, uint8_t h_max=255);

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
