/** Author: Ankush Gupta
    Date  : 4th Sept, 2012. */

#include <utils_cv/ImageAND.hpp>


/** The image processor you want to apply to every incoming image. */
ImageAND::ImageAND(ImageProcessor::Ptr processor, int n) : _img_proc(processor),
							   _N(n), _and_img(),
							   _is_publishing(false),
							   _and_images()  {}

/** call this to update the ANDed image. */
void ImageAND::update(cv::Mat &img) {
  cv::Mat dst;
  _img_proc->process(img, dst);

  if (_and_images.size() < _N)
    _and_images.push_back(dst);
  else {
    _is_publishing = true;

    for(int i = 0; i < _and_images.size()-1; i += 1) {
      _and_images[i] = _and_images[i+1];
    }
    _and_images[_N-1] = dst;

    for(int i =0; i < _N-1; i += 1) {
      cv::bitwise_and(_and_images[i], _and_images[i+1], _and_img);
    }
  }
}

/** Returns true iff, this.get_and() has a result. */
bool ImageAND::is_ready() {return _is_publishing;}

/** Returns the result of AND. */
cv::Mat ImageAND::get() {
  if (_is_publishing) {
    cv::Mat ret_img = _and_img.clone();
    return ret_img;
  } else {
    throw("ImageAND : Not enough images!");
  }
}
