#include "model.h"
#include <iostream>

Model::Model(int rows, int columns, QObject *parent)
  : AbsModel(rows,columns,parent) { }


int Model::rowCount(const QModelIndex &parent) const {
  int ret = (parent.isValid() && parent.column() != 0) ? 0 : rc;
  std::cout<<"in row count : "<<ret<<std::endl;
  return ret;
}
