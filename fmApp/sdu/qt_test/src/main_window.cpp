/**
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the qt gui.
 *
 * @date February 2011
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include "../include/qt_test/main_window.hpp"
#include <std_msgs/UInt32.h>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace qt_test {

using namespace Qt;

/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
	: QMainWindow(parent)
	, qnode(argc,argv)
{
    grip_pos = 0;
    con1_start = false;
    con2_start = false;
    con1_dir = 1;
    con2_dir = 1;
	ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.
    QObject::connect(ui.actionAbout_Qt, SIGNAL(triggered(bool)), qApp, SLOT(aboutQt())); // qApp is a global variable for the application

	setWindowIcon(QIcon(":/images/icon.png"));
	ui.tab_manager->setCurrentIndex(0); // ensure the first tab is showing - qt-designer should have this already hardwired, but often loses it (settings?).
    QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));

	/*********************
	** Logging
	**********************/
	ui.view_logging->setModel(qnode.loggingModel());
    QObject::connect(&qnode, SIGNAL(loggingUpdated()), this, SLOT(updateLoggingView()));
    QObject::connect(ui.btn_con1_start, SIGNAL(clicked()), this, SLOT(btn_con1_start_clicked()));
    QObject::connect(ui.btn_con2_start, SIGNAL(clicked()), this, SLOT(btn_con2_start_clicked()));
    QObject::connect(ui.btn_con1_change, SIGNAL(clicked()), this, SLOT(btn_con1_direction_clicked()));
    QObject::connect(ui.btn_con2_change, SIGNAL(clicked()), this, SLOT(btn_con2_direction_clicked()));
    QObject::connect(ui.btn_grip_grasp, SIGNAL(pressed()), this, SLOT(btn_grip_grasp_pressed()));
    QObject::connect(ui.btn_grip_release, SIGNAL(pressed()), this, SLOT(btn_grip_release_pressed()));
    QObject::connect(ui.btn_test_robot, SIGNAL(pressed()), this, SLOT(btn_test_robot_pressed()));

    // create a timer
    timer = new QTimer(this);

    // setup signal and slot
    connect(timer, SIGNAL(timeout()),
          this, SLOT(MyTimerSlot()));

    // msec
    timer->start(500);

    /*********************
    ** Auto Start
    **********************/
    qnode.init();
}

MainWindow::~MainWindow() {}

/*****************************************************************************
** Implemenation [Slots][manually connected]
*****************************************************************************/

/**
 * This function is signalled by the underlying model. When the model changes,
 * this will drop the cursor down to the last line in the QListview to ensure
 * the user can always see the latest log message.
 */
void MainWindow::updateLoggingView() {
        ui.view_logging->scrollToBottom();
}


/*****************************************************************************
** Implementation [Configuration]
*****************************************************************************/

void MainWindow::closeEvent(QCloseEvent *event)
{
    //WriteSettings();
	QMainWindow::closeEvent(event);
}

}  // namespace qt_test


void qt_test::MainWindow::btn_con1_start_clicked()
{
    if(MainWindow::con1_start == false)
    {
        MainWindow::con1_start = true;
        ui.btn_con1_start->setText("Stop");
        ui.btn_con1_start->setStyleSheet("background-color: red");
        qnode.send_command("1On");
        msg.data = 0x302;
        qnode.dummyCallback(msg);
    }
    else
    {
        MainWindow::con1_start = false;
        ui.btn_con1_start->setText("Start");
        ui.btn_con1_start->setStyleSheet("background-color: green");
        qnode.send_command("1Off");
        msg.data = 0x304;
        qnode.dummyCallback(msg);
    }
}

void qt_test::MainWindow::btn_con2_start_clicked()
{
    if(MainWindow::con2_start == false)
    {
        MainWindow::con2_start = true;
        ui.btn_con2_start->setText("Stop");
        ui.btn_con2_start->setStyleSheet("background-color: red");
        qnode.send_command("2On");
        msg.data = 0x303;
        qnode.dummyCallback(msg);
    }
    else
    {
        MainWindow::con2_start = false;
        ui.btn_con2_start->setText("Start");
        ui.btn_con2_start->setStyleSheet("background-color: green");
        qnode.send_command("2Off");
        msg.data = 0x305;
        qnode.dummyCallback(msg);
    }
}

void qt_test::MainWindow::btn_con1_direction_clicked()
{
    if(MainWindow::con1_dir == 1)
    {
        MainWindow::con1_dir = 0;
        ui.lbl_con1_dir->setText("Direction: Reverse");
        qnode.send_command("1Reverse");
        msg.data = 0x308;
        qnode.dummyCallback(msg);
    }
    else
    {
        MainWindow::con1_dir = 1;
        ui.lbl_con1_dir->setText("Direction: Forward");
        qnode.send_command("1Forward");
        msg.data = 0x306;
        qnode.dummyCallback(msg);
    }
}

void qt_test::MainWindow::btn_con2_direction_clicked()
{
    if(MainWindow::con2_dir == 1)
    {
        MainWindow::con2_dir = 0;
        ui.lbl_con2_dir->setText("Direction: Forward");
        qnode.send_command("2Reverse");
        msg.data = 0x307;
        qnode.dummyCallback(msg);
    }
    else
    {
        MainWindow::con2_dir = 1;
        ui.lbl_con2_dir->setText("Direction: Reverse");
        qnode.send_command("2Forward");
        msg.data = 0x309;
        qnode.dummyCallback(msg);
    }
}

void qt_test::MainWindow::MyTimerSlot()
{
    ui.value_force->setText(QString::number(qnode.force));
    ui.value_pos->setText(QString::number(qnode.position));
}

void qt_test::MainWindow::btn_grip_release_pressed()
{
  qnode.release();
}

void qt_test::MainWindow::btn_grip_grasp_pressed()
{
  qnode.grasp();
}

void qt_test::MainWindow::btn_test_robot_pressed()
{
    qnode.test_robot();
}
