
#ifndef _A_MODEL_H
#define _A_MODEL_H

#include "abs.h"

class Model : public AbsModel  {
 public:
  Model(int rows, int columns, QObject *parent = 0);

  int rowCount(const QModelIndex &parent) const;
};

#endif
