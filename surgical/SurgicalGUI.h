/** Author: Ankush Gupta
    Date  : 14th August, 2012 */

#ifndef _SURGICAL_GUI_
#define _SURGICAL_GUI_

#include "ui_surgical_gui.h"

#include "Hole.hpp"
#include "Cut.hpp"
#include "ListInteractor.hpp"

#include <iostream>
#include <pcl/point_types.h>
#include "ImageCommunicator.hpp"

class SurgicalGUI : public QMainWindow {
  Q_OBJECT

 public:
  SurgicalGUI(QWidget *parent = 0);
  void interact(pcl::PointXYZRGB* pt, int row_idx, int col_idx);

 private slots:
  /** these are call-backs for the "clicked" signal
      of the pushbuttons in the gui. */
  void on_selectFrame_clicked();
  void on_sendButton_clicked();

  void on_addHole_clicked();
  void on_removeHole_clicked();

  void on_addCut_stateChanged(int state);
  void on_removeCut_clicked();

 private:

  int hole_selection;
  int cut_selection;

  void _all_false();
  void repaint();

  /**  A class which communicates ros/ handles other ros stuff. */
  //ros_communicator _ros_comm;

  /**  A class which handles the pcl stuff. */
  ImageCommunicator _image_comm;

  /** The ui specified by QtDesigner. */
  Ui::surgical_gui ui;

  /** True if the user is specifying a cut on the screen. */
  bool create_new_cut, cutting, removing_cut, adding_hole, removing_hole;

  /** Number of holes specified by the user. */
  int num_holes;

  /** Number of cuts specified by the user. */
  int num_cuts;

  /** The specified holes.*/
  ListInteractor<Hole::Ptr> holes;

  /** The specified cuts. */
  ListInteractor<Cut::Ptr> cuts;
};

#endif
