/** Author: Ankush Gupta
    Date  : 15th August 2012. */

#ifndef _LIST_INTERACTOR_
#define _LIST_INTERACTOR_

#include <QListWidget>
#include <QListWidgetItem>
#include <QList>
 
/** Stores PREFIX 1, PREFIX 2, ... PREFIX N
    in a QListWidget. 
    And maintains an internal list. */
template <class T>
class ListInteractor {
public:
  ListInteractor(std::string prefix="item") : _prefix(prefix),
					      _list(),
					      _list_widget() { }

  /** Add an item to the internal list as well as display list. */
  void add_item(T item) {
    _list.push_back(item);
    QString item_name(_prefix.c_str());
    item_name.append(QString::number(_list.size()));
    _list_widget->addItem(item_name);
  }

  /** Remove the item at row IDX. */
  void remove_item(int idx) {
    if (0 <= idx && idx < _list.length()) {
      _list.removeAt(idx);
      QListWidgetItem * item = _list_widget->takeItem(idx);
      delete item;
    }
  }

  T last() {return _list.last();}

  void set_widget(QListWidget * widget) { _list_widget = widget; }
  QListWidget*  get_widget_ptr() { return _list_widget; }

  std::list<T> get_std_list() { return _list.toStdList(); }
  int length() {return _list.length(); }
private:
  std::string _prefix;
  QList<T> _list;
  QListWidget * _list_widget;
};

#endif
