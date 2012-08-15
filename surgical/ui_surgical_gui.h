/********************************************************************************
** Form generated from reading UI file 'surgical_gui.ui'
**
** Created: Tue Aug 14 21:01:31 2012
**      by: Qt User Interface Compiler version 4.8.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_SURGICAL_GUI_H
#define UI_SURGICAL_GUI_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QFrame>
#include <QtGui/QHeaderView>
#include <QtGui/QListView>
#include <QtGui/QMainWindow>
#include <QtGui/QPushButton>
#include <QtGui/QWidget>

QT_BEGIN_NAMESPACE

class Ui_surgical_gui
{
public:
    QWidget *centralwidget;
    QPushButton *selectFrame;
    QPushButton *sendButton;
    QFrame *frame;
    QPushButton *removeHole;
    QPushButton *addHole;
    QListView *holesList;
    QFrame *frame_2;
    QPushButton *removeCut;
    QListView *holesList_2;
    QPushButton *addCut;

    void setupUi(QMainWindow *surgical_gui)
    {
        if (surgical_gui->objectName().isEmpty())
            surgical_gui->setObjectName(QString::fromUtf8("surgical_gui"));
        surgical_gui->resize(336, 318);
        QSizePolicy sizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(surgical_gui->sizePolicy().hasHeightForWidth());
        surgical_gui->setSizePolicy(sizePolicy);
        surgical_gui->setMinimumSize(QSize(336, 318));
        surgical_gui->setMaximumSize(QSize(336, 318));
        surgical_gui->setMouseTracking(false);
        centralwidget = new QWidget(surgical_gui);
        centralwidget->setObjectName(QString::fromUtf8("centralwidget"));
        selectFrame = new QPushButton(centralwidget);
        selectFrame->setObjectName(QString::fromUtf8("selectFrame"));
        selectFrame->setGeometry(QRect(120, 20, 98, 27));
        sendButton = new QPushButton(centralwidget);
        sendButton->setObjectName(QString::fromUtf8("sendButton"));
        sendButton->setGeometry(QRect(120, 260, 98, 27));
        sendButton->setAutoFillBackground(false);
        frame = new QFrame(centralwidget);
        frame->setObjectName(QString::fromUtf8("frame"));
        frame->setGeometry(QRect(30, 60, 120, 181));
        frame->setFrameShape(QFrame::StyledPanel);
        frame->setFrameShadow(QFrame::Raised);
        removeHole = new QPushButton(frame);
        removeHole->setObjectName(QString::fromUtf8("removeHole"));
        removeHole->setGeometry(QRect(10, 140, 101, 27));
        addHole = new QPushButton(frame);
        addHole->setObjectName(QString::fromUtf8("addHole"));
        addHole->setGeometry(QRect(10, 10, 98, 27));
        holesList = new QListView(frame);
        holesList->setObjectName(QString::fromUtf8("holesList"));
        holesList->setGeometry(QRect(10, 40, 101, 91));
        frame_2 = new QFrame(centralwidget);
        frame_2->setObjectName(QString::fromUtf8("frame_2"));
        frame_2->setGeometry(QRect(180, 60, 120, 181));
        frame_2->setFrameShape(QFrame::StyledPanel);
        frame_2->setFrameShadow(QFrame::Raised);
        removeCut = new QPushButton(frame_2);
        removeCut->setObjectName(QString::fromUtf8("removeCut"));
        removeCut->setGeometry(QRect(10, 140, 101, 27));
        holesList_2 = new QListView(frame_2);
        holesList_2->setObjectName(QString::fromUtf8("holesList_2"));
        holesList_2->setGeometry(QRect(10, 40, 101, 91));
        addCut = new QPushButton(frame_2);
        addCut->setObjectName(QString::fromUtf8("addCut"));
        addCut->setGeometry(QRect(10, 10, 98, 27));
        surgical_gui->setCentralWidget(centralwidget);

        retranslateUi(surgical_gui);

        QMetaObject::connectSlotsByName(surgical_gui);
    } // setupUi

    void retranslateUi(QMainWindow *surgical_gui)
    {
        surgical_gui->setWindowTitle(QApplication::translate("surgical_gui", "SurgiC@l", 0, QApplication::UnicodeUTF8));
#ifndef QT_NO_TOOLTIP
        surgical_gui->setToolTip(QString());
#endif // QT_NO_TOOLTIP
#ifndef QT_NO_TOOLTIP
        selectFrame->setToolTip(QApplication::translate("surgical_gui", "Select the frame to work with", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_TOOLTIP
        selectFrame->setText(QApplication::translate("surgical_gui", "Select Frame", 0, QApplication::UnicodeUTF8));
#ifndef QT_NO_TOOLTIP
        sendButton->setToolTip(QApplication::translate("surgical_gui", "Publish the hole/ cut coordinates on the topic", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_TOOLTIP
        sendButton->setText(QApplication::translate("surgical_gui", "Send Info", 0, QApplication::UnicodeUTF8));
#ifndef QT_NO_TOOLTIP
        removeHole->setToolTip(QApplication::translate("surgical_gui", "Remove the selected hole", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_TOOLTIP
        removeHole->setText(QApplication::translate("surgical_gui", "Remove", 0, QApplication::UnicodeUTF8));
#ifndef QT_NO_TOOLTIP
        addHole->setToolTip(QApplication::translate("surgical_gui", "Add needle hole location", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_TOOLTIP
        addHole->setText(QApplication::translate("surgical_gui", "Add Hole", 0, QApplication::UnicodeUTF8));
#ifndef QT_NO_TOOLTIP
        holesList->setToolTip(QApplication::translate("surgical_gui", "Info about holes", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_TOOLTIP
#ifndef QT_NO_TOOLTIP
        removeCut->setToolTip(QApplication::translate("surgical_gui", "Remove the selected cut", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_TOOLTIP
        removeCut->setText(QApplication::translate("surgical_gui", "Remove", 0, QApplication::UnicodeUTF8));
#ifndef QT_NO_TOOLTIP
        holesList_2->setToolTip(QApplication::translate("surgical_gui", "Info about holes", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_TOOLTIP
#ifndef QT_NO_TOOLTIP
        addCut->setToolTip(QApplication::translate("surgical_gui", "Specify points for the cut", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_TOOLTIP
        addCut->setText(QApplication::translate("surgical_gui", "Add Cut", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class surgical_gui: public Ui_surgical_gui {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_SURGICAL_GUI_H
