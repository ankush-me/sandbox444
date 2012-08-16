/** Author: Ankush Gupta
    Date  : 15th August 2012. */

#ifndef _HOLE_LIST_MODEL_
#define _HOLE_LIST_MODEL_

include "NameListModel.hpp"
#include "Hole.hpp"

/** For displaying a list of type T with a prefix string: _PREFIX.
    i.e. if the length of the list is N, then it would display
            the list as:
	    _prefix 1, _prefix 2, ....., _prefix N.      

     ADAPTED from: http://qt-project.org/doc/qt-4.8/qt4-interview.html   */


class HoleListModel : virtual public NameListModel {
  private:
  /** The list to store items. */
  QList<Hole::Ptr> _list;

  public:
  NameListModel(std::string prefix_name="item", QObject *parent = 0)
    : NameListModel(prefix_name, parent),
      _list() {}

  /** Returns a pointer to the list. */
  QList<Hole::Ptr>* list_ptr() { return & _list; }

  /** Returns the number of items in the list. */
  int rowCount(const QModelIndex &parent = QModelIndex()) const {
    return _list.length();
  }
 
};

#endif
