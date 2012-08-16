/** Author: Ankush Gupta
    Date  : 15th August 2012. */

#ifndef _NAME_LIST_MODEL_
#define _NAME_LIST_MODEL_

#include <QAbstractListModel>
#include <QObject>
#include <QList>

/** For displaying a list of type T with a prefix string: _PREFIX.
    i.e. if the length of the list is N, then it would display
            the list as:
	    _prefix 1, _prefix 2, ....., _prefix N.      

     ADAPTED from: http://qt-project.org/doc/qt-4.8/qt4-interview.html   */


class NameListModel : public QAbstractListModel {
  Q_OBJECT

  private:
  /** The prefix string for displaying items. */
  std::string _prefix;

  public:
  NameListModel(std::string prefix_name="item", QObject *parent = 0)
    : QAbstractListModel(parent),
      _prefix(prefix_name) {}

  /** Returns the number of items in the list. */
  virtual int rowCount(const QModelIndex &parent = QModelIndex()) const {
    return -1;
  }

  /* Returns an item of data for each model index supplied by a view. */
  QVariant data(const QModelIndex &index, int role) const;

  /** Vertical and horizontal headers are supplied by the
    headerData() function. In this model, the value returned
    for these items is the row or column number, depending on the header */  
  QVariant headerData(int section, Qt::Orientation orientation,
		      int role = Qt::DisplayRole) const;
  
};

#endif
