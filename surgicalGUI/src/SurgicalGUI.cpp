/** Author: Ankush Gupta
    Date  : 14th August 2012 */

#include "SurgicalGUI.h"

/** Default constructor. */
SurgicalGUI::SurgicalGUI(ImageCommunicator * img_comm,
			 ROSCommunicator * ros_comm,
			 QWidget *parent) : QMainWindow(parent),
					    _ros_comm(ros_comm),
					    _image_comm(img_comm),
					    create_new_cut(true),
					    cutting(false),
					    removing_cut(false),
					    adding_hole(false),
					    removing_hole(false),
					    holes("Hole"),
					    cuts("Cut")
{
  _image_comm->set_gui(this);
  ui.setupUi( this );
  holes.set_widget(ui.holesList);
  cuts.set_widget(ui.cutsList);
}

/** Return the image communicator. */
ImageCommunicator* SurgicalGUI::get_image_communicator() {
  return _image_comm;
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
  _image_comm->fix_frame();
}

/** Ask the ros_communicator to publish the info on the topic. */
void SurgicalGUI::on_sendButton_clicked() {
  std::list<Hole::Ptr> hole_lst = holes.get_std_list();
  std::list<Cut::Ptr>  cut_lst  = cuts.get_std_list();
 
  _ros_comm->publish(hole_lst, cut_lst, _image_comm->get_cloud_ptr_ros());
}

/** Update internal state to add holes. */
void SurgicalGUI::on_addHole_clicked() {
  _all_false();
  adding_hole = true;
}


/** Update internal state to remove holes. */
void SurgicalGUI::on_removeHole_clicked() {
  int hole_selection = holes.get_widget_ptr()->currentRow();
  if (hole_selection >= 0
      && hole_selection <= holes.length()) {
    holes.remove_item(hole_selection);
    _all_false();
    repaint();
  }
}

/** Update internal state to add cuts. */
void SurgicalGUI::on_addCut_stateChanged(int state) {
  create_new_cut = (state == Qt::Checked);
  _all_false(); 
  cutting = (state == Qt::Checked);
}


/** Update internal state to remove cuts. */
void SurgicalGUI::on_removeCut_clicked() {
  int cut_selection = cuts.get_widget_ptr()->currentRow();
  if (cut_selection >= 0 && cut_selection <= cuts.length()) {
    cuts.remove_item(cut_selection);
    _all_false();
    repaint();
  }
}

void SurgicalGUI::repaint() {
  std::list<Hole::Ptr> hole_lst = holes.get_std_list();
  std::list<Cut::Ptr>  cut_lst  = cuts.get_std_list();

  _image_comm->repaint(hole_lst, cut_lst);
}


/** PointXYZRGB PT     : is the pcl point corresponding to the pixel
                         the user clicked on.
    (ROW_IDX, COL_IDX) : the location of the pixel. 

    This is called from the IMAGE_COMMUNICATOR to register a user click.*/
void SurgicalGUI::interact(pcl::PointXYZRGB* pt,
			   int row_idx, int col_idx) {
  if (adding_hole) {
    Hole::Ptr hole_ptr(new Hole(pt, row_idx, col_idx));
    holes.add_item(hole_ptr);
    _all_false();
    repaint();
  }

  if (cutting) {
    Cut::Ptr cut;
    if(create_new_cut) {
      Cut::Ptr cut_ptr(new Cut);
      cuts.add_item(cut_ptr);
      create_new_cut = !create_new_cut;
    }
    cut = cuts.last();
    cut->add_point(pt, row_idx, col_idx);
    repaint();
  }
}
