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
#include "../include/HMI/main_window.hpp"
#include <std_msgs/UInt32.h>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace HMI {

using namespace Qt;

/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
	: QMainWindow(parent)
	, qnode(argc,argv)
{
       cout << "mainwindow start" << endl;
//    grip_pos = 0;
//    con1_start = false;
//    con2_start = false;
//    con1_dir = 1;
//    con2_dir = 1;
    ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.
    cout << "mainwindow start1" << endl;
    QObject::connect(ui.actionAbout_Qt, SIGNAL(triggered(bool)), qApp, SLOT(aboutQt())); // qApp is a global variable for the application
cout << "mainwindow start2" << endl;
	setWindowIcon(QIcon(":/images/icon.png"));
    cout << "mainwindow start3" << endl;
    ui.tab_manager->setCurrentIndex(0); // ensure the first tab is showing - qt-designer should have this already hardwired, but often loses it (settings?).
    cout << "mainwindow start4" << endl;
    QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));

	/*********************
	** Logging
	**********************/
//	ui.view_logging->setModel(qnode.loggingModel());
//    QObject::connect(&qnode, SIGNAL(loggingUpdated()), this, SLOT(updateLoggingView()));
//    QObject::connect(ui.btn_con1_start, SIGNAL(clicked()), this, SLOT(btn_con1_start_clicked()));
//    QObject::connect(ui.btn_con2_start, SIGNAL(clicked()), this, SLOT(btn_con2_start_clicked()));
//    QObject::connect(ui.btn_con1_change, SIGNAL(clicked()), this, SLOT(btn_con1_direction_clicked()));
//    QObject::connect(ui.btn_con2_change, SIGNAL(clicked()), this, SLOT(btn_con2_direction_clicked()));
//    QObject::connect(ui.btn_grip_grasp, SIGNAL(pressed()), this, SLOT(btn_grip_grasp_pressed()));
//    QObject::connect(ui.btn_grip_release, SIGNAL(pressed()), this, SLOT(btn_grip_release_pressed()));
//    QObject::connect(ui.btn_test_config, SIGNAL(pressed()), this, SLOT(btn_test_config_pressed()));
//    QObject::connect(ui.btn_reset_config, SIGNAL(pressed()), this, SLOT(btn_reset_config_pressed()));
//    QObject::connect(ui.btn_set_position, SIGNAL(clicked()), this, SLOT(btn_set_position_clicked()));
//    QObject::connect(&qnode, SIGNAL(new_config(double *)), this, SLOT(newest_configuration(double *)));
//    QObject::connect(&qnode, SIGNAL(new_pose(double *)), this, SLOT(newest_pose(double *)));

//    QObject::connect(ui.x_axis_positive, SIGNAL(pressed()), this, SLOT(btn_x_axis_positive()));
//    QObject::connect(ui.y_axis_positive, SIGNAL(pressed()), this, SLOT(btn_y_axis_positive()));
//    QObject::connect(ui.z_axis_positive, SIGNAL(pressed()), this, SLOT(btn_z_axis_positive()));
//    QObject::connect(ui.rotate_positive, SIGNAL(pressed()), this, SLOT(btn_rotate_positive()));
//    QObject::connect(ui.x_axis_negative, SIGNAL(pressed()), this, SLOT(btn_x_axis_negative()));
//    QObject::connect(ui.y_axis_negative, SIGNAL(pressed()), this, SLOT(btn_y_axis_negative()));
//    QObject::connect(ui.z_axis_negative, SIGNAL(pressed()), this, SLOT(btn_z_axis_negative()));
//    QObject::connect(ui.rotate_negative, SIGNAL(pressed()), this, SLOT(btn_rotate_negative()));

//    // create a timer
//    timer = new QTimer(this);

//    // setup signal and slot
//    connect(timer, SIGNAL(timeout()),
//          this, SLOT(MyTimerSlot()));

//    // msec
//    timer->start(500);

    cout << "mainwindow constructor end" << endl;

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
//void MainWindow::updateLoggingView() {
//        ui.view_logging->scrollToBottom();
//}


/*****************************************************************************
** Implementation [Configuration]
*****************************************************************************/

void MainWindow::closeEvent(QCloseEvent *event)
{
    //WriteSettings();
	QMainWindow::closeEvent(event);
}

}  // namespace HMI


//void HMI::MainWindow::btn_con1_start_clicked()
//{
//    if(MainWindow::con1_start == false)
//    {
//        MainWindow::con1_start = true;
//        ui.btn_con1_start->setText("Stop");
//        ui.btn_con1_start->setStyleSheet("background-color: red");
//        qnode.send_command("1On");
//        msg.data = 0x302;
//        qnode.dummyCallback(msg);
//    }
//    else
//    {
//        MainWindow::con1_start = false;
//        ui.btn_con1_start->setText("Start");
//        ui.btn_con1_start->setStyleSheet("background-color: green");
//        qnode.send_command("1Off");
//        msg.data = 0x304;
//        qnode.dummyCallback(msg);
//    }
//}

//void HMI::MainWindow::btn_con2_start_clicked()
//{
//    if(MainWindow::con2_start == false)
//    {
//        MainWindow::con2_start = true;
//        ui.btn_con2_start->setText("Stop");
//        ui.btn_con2_start->setStyleSheet("background-color: red");
//        qnode.send_command("2On");
//        msg.data = 0x303;
//        qnode.dummyCallback(msg);
//    }
//    else
//    {
//        MainWindow::con2_start = false;
//        ui.btn_con2_start->setText("Start");
//        ui.btn_con2_start->setStyleSheet("background-color: green");
//        qnode.send_command("2Off");
//        msg.data = 0x305;
//        qnode.dummyCallback(msg);
//    }
//}

//void HMI::MainWindow::btn_con1_direction_clicked()
//{
//    if(MainWindow::con1_dir == 1)
//    {
//        MainWindow::con1_dir = 0;
//        ui.lbl_con1_dir->setText("Direction: Reverse");
//        qnode.send_command("1Reverse");
//        msg.data = 0x308;
//        qnode.dummyCallback(msg);
//    }
//    else
//    {
//        MainWindow::con1_dir = 1;
//        ui.lbl_con1_dir->setText("Direction: Forward");
//        qnode.send_command("1Forward");
//        msg.data = 0x306;
//        qnode.dummyCallback(msg);
//    }
//}

//void HMI::MainWindow::btn_con2_direction_clicked()
//{
//    if(MainWindow::con2_dir == 1)
//    {
//        MainWindow::con2_dir = 0;
//        ui.lbl_con2_dir->setText("Direction: Forward");
//        qnode.send_command("2Reverse");
//        msg.data = 0x307;
//        qnode.dummyCallback(msg);
//    }
//    else
//    {
//        MainWindow::con2_dir = 1;
//        ui.lbl_con2_dir->setText("Direction: Reverse");
//        qnode.send_command("2Forward");
//        msg.data = 0x309;
//        qnode.dummyCallback(msg);
//    }
//}

//void HMI::MainWindow::MyTimerSlot()
//{
////    ui.value_force->setText(QString::number(qnode.force));
////    ui.value_pos->setText(QString::number(qnode.position));

//    ui.current_q_1->setText(QString::number(qnode.current_config[0]));
//    ui.current_q_2->setText(QString::number(qnode.current_config[1]));
//    ui.current_q_3->setText(QString::number(qnode.current_config[2]));
//    ui.current_q_4->setText(QString::number(qnode.current_config[3]));
//    ui.current_q_5->setText(QString::number(qnode.current_config[4]));
//    ui.current_q_6->setText(QString::number(qnode.current_config[5]));

//    ui.x_pos->setText(QString::number(qnode.current_pose[0]));
//    ui.y_pos->setText(QString::number(qnode.current_pose[1]));
//    ui.z_pos->setText(QString::number(qnode.current_pose[2]));
//    ui.R_rot->setText(QString::number(qnode.current_pose[3]));
//    ui.P_rot->setText(QString::number(qnode.current_pose[4]));
//    ui.Y_rot->setText(QString::number(qnode.current_pose[5]));
//}

//void HMI::MainWindow::btn_grip_release_pressed()
//{
//  //qnode.release();
//}

//void HMI::MainWindow::btn_grip_grasp_pressed()
//{
//  //qnode.grasp();
//}

//void HMI::MainWindow::btn_reset_config_pressed()
//{
////    float config[] = {130,0.0,0.0,0.0,0.0,0.0};
////    qnode.set_robot_config(config);
////    float config1[] = {0.0,0.0,0.0,0.0,0.0,0.0};
////    qnode.set_robot_config(config1);
//}

//void HMI::MainWindow::btn_test_config_pressed()
//{
////    float config[] = {130,0,0,0,0,40};
////    qnode.set_robot_config(config);
////    float config1[] = {130,-30,-60,0,-90,40};
////    qnode.set_robot_config(config1);
////    float config2[] = {130,-38,-80,0,-60,40};
////    qnode.set_robot_config(config2);
//}

//void HMI::MainWindow::btn_set_position_clicked()
//{
//    double x = ui.set_x_pos->text().toDouble();
//    double y = ui.set_y_pos->text().toDouble();
//    double rot = ui.set_rot->text().toDouble();

//   // qnode.graspBrick(x,y,rot);
//}

////void HMI::MainWindow::newest_configuration(double *configuration)
////{
//// //   cout << configuration[0] << ", " << configuration[1] << ", " << configuration[2] << ", " << configuration[3] << ", " << configuration[4] << ", " << configuration[5] << endl;
////    ui.current_q_1->setText(QString::number(configuration[0]));
////    ui.current_q_2->setText(QString::number(configuration[1]));
////    ui.current_q_3->setText(QString::number(configuration[2]));
////    ui.current_q_4->setText(QString::number(configuration[3]));
////    ui.current_q_5->setText(QString::number(configuration[4]));
////    ui.current_q_6->setText(QString::number(configuration[5]));
////}

////void HMI::MainWindow::newest_pose(double *pose)
////{
//// //   cout << configuration[0] << ", " << configuration[1] << ", " << configuration[2] << ", " << configuration[3] << ", " << configuration[4] << ", " << configuration[5] << endl;
////    ui.x_pos->setText(QString::number(pose[0]));
////    ui.y_pos->setText(QString::number(pose[1]));
////    ui.z_pos->setText(QString::number(pose[2]));
////    ui.R_rot->setText(QString::number(pose[3]));
////    ui.P_rot->setText(QString::number(pose[4]));
////    ui.Y_rot->setText(QString::number(pose[5]));
////}
//#define pos_size 1.0/100.0
//#define angle_size 0.01745*2
////void HMI::MainWindow::btn_x_axis_positive()
////{
////    pose gripper_pose = qnode.getGripperPose();
////    gripper_pose.x_pos += pos_size; //0.5cm
////    qnode.setGripperPose(gripper_pose);
////}

////void HMI::MainWindow::btn_y_axis_positive()
////{
////    pose gripper_pose = qnode.getGripperPose();
////    gripper_pose.y_pos += pos_size; //0.5cm
////    qnode.setGripperPose(gripper_pose);
////}

////void HMI::MainWindow::btn_z_axis_positive()
////{
////    pose gripper_pose = qnode.getGripperPose();
////    gripper_pose.z_pos += pos_size; //0.5cm
////    qnode.setGripperPose(gripper_pose);
////}

////void HMI::MainWindow::btn_rotate_positive()
////{
////    pose gripper_pose = qnode.getGripperPose();
////    gripper_pose.orientation += angle_size; //20 degrees / second
////    qnode.setGripperPose(gripper_pose);
////}

////void HMI::MainWindow::btn_x_axis_negative()
////{
////    pose gripper_pose = qnode.getGripperPose();
////    gripper_pose.x_pos -= pos_size; //0.5cm
////    qnode.setGripperPose(gripper_pose);
////}

////void HMI::MainWindow::btn_y_axis_negative()
////{
////    pose gripper_pose = qnode.getGripperPose();
////    gripper_pose.y_pos -= pos_size; //0.5cm
////    qnode.setGripperPose(gripper_pose);
////}

////void HMI::MainWindow::btn_z_axis_negative()
////{
////    pose gripper_pose = qnode.getGripperPose();
////    gripper_pose.z_pos -= pos_size; //0.5cm
////    qnode.setGripperPose(gripper_pose);
////}

////void HMI::MainWindow::btn_rotate_negative()
////{
////    pose gripper_pose = qnode.getGripperPose();
////    gripper_pose.orientation -= angle_size; //20 degrees / second
////    qnode.setGripperPose(gripper_pose);
////}
