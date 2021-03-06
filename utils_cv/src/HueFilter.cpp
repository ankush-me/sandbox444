#include "utils_cv/HueFilter.hpp"
#include <vector>

/** Simple class to filter an openCV mat in HUE SPACE. */
HueFilter::HueFilter(uint8_t h_min, uint8_t h_max) {
  _h_min = h_min;
  _h_max = h_max;
}

void HueFilter::set_max(uint8_t max) {_h_max = max;}
void HueFilter::set_min(uint8_t min) {_h_min = min;}

uint8_t HueFilter::get_max() { return _h_max;}
uint8_t HueFilter::get_min() { return _h_min;}

/** Filters the input RGB image, SRC
    based on hue and stores the result in DEST.

    if ISMASK is set to true, it saves a binary
    [0==0, 1==255] image in DEST, else saves
    the inlier pixels into DEST [RGB values].

    An inlier is:
    _h_min <= pixel.hue <= _h_max    
      
    [for circular wrapping around hue=180,
    set _h_min > h_max ]                 */
void HueFilter::filter(cv::Mat &src, cv::Mat &dest, bool isMask) {
  cv::Mat imgHSV;
  if (src.channels() == 3)
    cv::cvtColor(src,imgHSV,CV_BGR2HSV);
  else
    src.copyTo(imgHSV);

  std::vector<cv::Mat> channels;
  cv::split(imgHSV, channels);
  cv::Mat hue_channel =  channels[0];
 
  cv::Mat mask(hue_channel.size(), hue_channel.type());
  cv::inRange(hue_channel, _h_min, _h_max, mask);

  if (isMask)
    mask.copyTo(dest);
  else
    src.copyTo(dest, mask);

  return;
}
