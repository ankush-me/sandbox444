/** Author: Ankush Gupta
    Date  : 15th August 2012. */

#include "NameListModel.hpp"

/* Returns an item of data for each model index supplied by a view. */
QVariant NameListModel::data(const QModelIndex &index, int role) const {
  if (!index.isValid())
    return QVariant();

  if (index.row() >= rowCount())
    return QVariant();

  if (role == Qt::DisplayRole || role == Qt::EditRole) {
    QString num_str = QString(" %1").arg(index.row());
    QString prefix(_prefix.c_str());
    prefix.append(num_str);
    return prefix;
  } else
    return QVariant();
}


/** Vertical and horizontal headers are supplied by the
    headerData() function. In this model, the value returned
    for these items is the row or column number, depending on the header */
QVariant NameListModel::headerData(int section,
				   Qt::Orientation orientation,
				   int role) const {
  if (role != Qt::DisplayRole)
    return QVariant();

  if (orientation == Qt::Horizontal)
    return QString("Column %1").arg(section);
  else
    return QString("Row %1").arg(section);
}
