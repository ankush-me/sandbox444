/** Author: Ankush Gupta
    Date  : 14th August 2012 */

#include "SurgicalGUI.h"

/** Default constructor. */
SurgicalGUI::SurgicalGUI(QWidget *parent) : QMainWindow(parent),
					    _ros_comm(this),
					    _image_comm(this),
					    create_new_cut(true),
					    cutting(false), removing_cut(false),
					    adding_hole(false), removing_hole(false),
					    num_holes(0), num_cuts(0),
					    holes(), cuts(),
					    hole_selection(-1),
					    cut_selection(-1)
{
  ui.setupUi( this );
  ui.holesList->setModel(&holes);
  ui.cutsList->setModel(&cuts);
}


/** Falsifies all the booles. */
void SurgicalGUI::_all_false() {
  adding_hole   = false;
  removing_hole = false;
  cutting       = false;
  removing_cut  = false;
}

/** Ask the image communicator to fix the frame, so that user can
    interact with it. */
void SurgicalGUI::on_selectFrame_clicked() {
  std::cout<<"frame"<<std::cout;
  _image_comm.fix_frame();
}

/** Ask the ros_communicator to publish the info on the topic. */
void SurgicalGUI::on_sendButton_clicked() {
  std::cout<<"send"<<std::endl;
  //_ros_comm.publish_info();
}

/** Update internal state to add holes. */
void SurgicalGUI::on_addHole_clicked() {
  _all_false();
  adding_hole = true;
}

/** Update internal state to remove holes. */
void SurgicalGUI::on_removeHole_clicked() {
  std::cout<<"remove hole"<<std::endl;
  _all_false();
  removing_hole = true;
}


/** Update internal state to add cuts. */
void SurgicalGUI::on_addCut_stateChanged(int state) {
  std::cout<<"check: "<<state<<std::endl;
  create_new_cut = (state == Qt::Unchecked);
  bool prev_cut = cutting;
  _all_false(); 
  cutting = !prev_cut;
}


/** Update internal state to remove cuts. */
void SurgicalGUI::on_removeCut_clicked() {
  std::cout<<"remove cut"<<std::endl;
  _all_false();
  removing_cut = true;
}

void repaint() {
  _image_comm.repaint(&holes, &cuts);
}


/** PointXYZRGB PT     : is the pcl point corresponding to the pixel
                         the user clicked on.
    (ROW_IDX, COL_IDX) : the location of the pixel. */
void SurgicalGUI::interact(pcl::PointXYZRGB* pt,
			   int row_idx, int col_idx) {
  if (adding_hole) {
    Hole::Ptr hole_ptr(new Hole(pt, row_idx, col_idx));
    holes.push_back(hole_ptr);
    repaint();
  }

  if (removing_hole) {
    if (hole_selection >= 0 && hole_selection <= holes.length()) {
      holes.removeAt(hole_selection);
      repaint();
    }
  }

  if (cutting) {
    Cut::Ptr cut;
    if(create_new_cut) {
      Cut::Ptr cut_ptr(new cut);
      cuts.push_back(cut_ptr);
      create_new_cut != create_new_cut;
    } 
    cut = cuts.at(cuts.length() - 1);
    cut->add_point(pt, row_idx, col_idx);
    repaint();
  }

  if (removing_cut) {
    if (cut_selection >= 0 && cut_selection <= cuts.length()) {
      cuts.removeAt(cut_selection);
      repaint();
    }
  }
}
