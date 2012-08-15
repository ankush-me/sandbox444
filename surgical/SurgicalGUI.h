/** Author: Ankush Gupta
    Date  : 4th August, 2012 */

#ifndef _SURGICAL_GUI_
#define _SURGICAL_GUI_

#include "ui_surgical_gui.h"
#include <iostream>

class SurgicalGUI : public QMainWindow {
  Q_OBJECT

 public:
  SurgicalGUI(QWidget *parent = 0);
  
  private slots:
  void on_selectFrame_clicked();

 private:
  Ui::surgical_gui ui;
};

#endif
