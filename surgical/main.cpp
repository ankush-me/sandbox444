/** Author: Ankush Gupta
    Date  : 14th August, 2012 */

#include "SurgicalGUI.h"
#include <QDesktopWidget>
#include <QApplication>


int main(int argc, char *argv[]) {
  QApplication app(argc, argv);  
  SurgicalGUI gui;

  gui.show();
  return app.exec();
}
