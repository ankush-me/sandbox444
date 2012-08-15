/** Author: Ankush Gupta
    Date  : 14th August 2012 */

#include <QtGui>
#include "SurgicalGUI.h"

SurgicalGUI::SurgicalGUI(QWidget *parent) : QMainWindow(parent) {
  ui.setupUi( this );
}

void SurgicalGUI::on_selectFrame_clicked() {
  std::cout<<"select frame clicked!"<<std::endl;
}
