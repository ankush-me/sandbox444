/** Author: Ankush Gupta
    Date  : 14th August, 2012 */

#ifndef _SURGICAL_GUI_
#define _SURGICAL_GUI_

#include "ui_surgical_gui.h"

#include "Hole.hpp"
#include "Cut.hpp"
#include "ListInteractor.hpp"
#include "ImageCommunicator.hpp"
#include "ROSCommunicator.hpp"


#include <iostream>
#include <pcl/point_types.h>


class SurgicalGUI : public QMainWindow {
  Q_OBJECT

 public:
  /** Constructor
      IMG_COMM : The image communicator, which handles the communication
                 with ros for displaying the point-cloud as a flat image
		 and the mouse-event callbacks.
      PARENT   : The Qt parent (usually NULL).*/
  SurgicalGUI(ImageCommunicator * img_comm,
	      ROSCommunicator * ros_comm,
	      QWidget *parent = 0);

  /**  Called by the image-communicator to handle user-clicks. */
  void interact(pcl::PointXYZRGB* pt, int row_idx, int col_idx);

  /** Returns the image-communicator it is bound to. */
  ImageCommunicator* get_image_communicator();

 private slots:
  /** These are call-backs for the "clicked" signal
      of the pushbuttons in the gui. */
  void on_selectFrame_clicked();
  void on_sendButton_clicked();

  void on_addHole_clicked();
  void on_removeHole_clicked();

  void on_addCut_stateChanged(int state);
  void on_removeCut_clicked();

 private:

  void _all_false();
  void repaint();

  /**  A class which communicates with ros: publishes the HOLES and CUTS,
       i.e. spits out the output. */
  ROSCommunicator* _ros_comm;

  /**  A class which handles the ros<-->pcl stuff. */
  ImageCommunicator* _image_comm;

  /** The ui specified by QtDesigner. */
  Ui::surgical_gui ui;

  /** True if the user is specifying a cut on the screen. */
  bool create_new_cut, cutting, removing_cut, adding_hole, removing_hole;

  /** The specified holes.*/
  ListInteractor<Hole::Ptr> holes;

  /** The specified cuts. */
  ListInteractor<Cut::Ptr> cuts;
};

#endif
