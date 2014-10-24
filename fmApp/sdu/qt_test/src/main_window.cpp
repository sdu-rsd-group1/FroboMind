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
    }
    else
    {
        MainWindow::con1_start = false;
        ui.btn_con1_start->setText("Start");
        ui.btn_con1_start->setStyleSheet("background-color: green");
        qnode.send_command("1Off");
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
    }
    else
    {
        MainWindow::con2_start = false;
        ui.btn_con2_start->setText("Start");
        ui.btn_con2_start->setStyleSheet("background-color: green");
        qnode.send_command("2Off");
    }
}

void qt_test::MainWindow::btn_con1_direction_clicked()
{
    if(MainWindow::con1_dir == 1)
    {
        MainWindow::con1_dir = 0;
        ui.lbl_con1_dir->setText("Direction: Reverse");
        qnode.send_command("1Reverse");
    }
    else
    {
        MainWindow::con1_dir = 1;
        ui.lbl_con1_dir->setText("Direction: Forward");
        qnode.send_command("1Forward");
    }
}

void qt_test::MainWindow::btn_con2_direction_clicked()
{
    if(MainWindow::con2_dir == 1)
    {
        MainWindow::con2_dir = 0;
        ui.lbl_con2_dir->setText("Direction: Forward");
        qnode.send_command("2Reverse");
    }
    else
    {
        MainWindow::con2_dir = 1;
        ui.lbl_con2_dir->setText("Direction: Reverse");
        qnode.send_command("2Forward");
    }
}
