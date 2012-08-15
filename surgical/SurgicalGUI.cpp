/** Author: Ankush Gupta
    Date  : 14th August 2012 */

#include "SurgicalGUI.h"

SurgicalGUI::SurgicalGUI(QWidget *parent) : QWidget(parent) {
  ui.setupUi(new QtMainWindow());
}

void SurgicalGUI::on_selectFrame_clicked() {
  std::cout<<"select frame clicked!"<<std::endl;
}
