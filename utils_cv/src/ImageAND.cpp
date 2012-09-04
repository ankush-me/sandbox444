/** Author: Ankush Gupta
    Date  : 4th Sept, 2012. */


#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>


/** Class maintains an ANDed image for all the images
    it recieves in the call to UPDATE. */

class ImageAND {
  
private:

  /** Keeps a count of the images recieved. */
  unsigned int num_images;

  /** The image processing function that should be applied
      to every incoming image before being ANDed. */
  ImageProcessor::Ptr _img_proc;

  /** This is TRUE is this is ready to give an output image. */
  bool _is_publishing;

  /** The internal ANDed image. */
  cv::Mat _and_img;

public:

  /** The image processor you want to apply to every incoming image. */
  ImageAND(ImageProcessor::Ptr processor) : _img_proc(processor),
					    _num_images(0),
					    _is_publishing(false),
					    _and_image()  {}


  /** call this to update the ANDed image. */
  void update(cv::Mat &img) {

    if (_num_images == 0)
      _and_img = cv::Mat::ones(img.size(), img.type());

    cv::Mat dst;
    cv::Mat src = img.clone();
    _img_proc->process(src, dst);

    AND THE IMAGES TOGETHER HERE.

  }
    
  bool is_ready() {return _is_publishing;}

    
}
