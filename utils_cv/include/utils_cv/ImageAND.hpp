/** Author: Ankush Gupta
    Date  : 4th Sept, 2012. */

#include <utils_cv/ImageProcessor.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <vector>


/** Class maintains an ANDed image for all the images
    it recieves in the call to UPDATE. */

class ImageAND {

private:

  /** The number of frames for moving AND. */
  unsigned int _N;

  /** The image processing function that should be applied
      to every incoming image before being ANDed. */
  ImageProcessor::Ptr _img_proc;

  /** This is TRUE is this is ready to give an output image. */
  bool _is_publishing;

  /** The internal images. */
  std::vector<cv::Mat> _and_images;

  /** The ANDed image. */
  cv::Mat _and_img;

public:

  /** The image processor you want to apply to every incoming image. */
  ImageAND(ImageProcessor::Ptr processor, int n);

  /** call this to update the ANDed image. */
  void update(cv::Mat &img);

  /** Returns true iff, this.get_and() has a result. */
  bool is_ready();

  /** Returns the result of AND. */
  cv::Mat get();

};
