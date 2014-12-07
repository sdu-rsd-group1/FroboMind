/**
 * @file /include/HMI/main_window.hpp
 *
 * @brief Qt based gui for HMI.
 *
 * @date November 2010
 **/
#ifndef HMI_MAIN_WINDOW_H
#define HMI_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui/QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"
#include <std_msgs/UInt32.h>
#include <QTimer>

/*****************************************************************************
** Namespace
*****************************************************************************/

namespace HMI {

/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.
 */
class MainWindow : public QMainWindow {
Q_OBJECT

public:
	MainWindow(int argc, char** argv, QWidget *parent = 0);
	~MainWindow();

    QTimer *timer;

	void closeEvent(QCloseEvent *event); // Overloaded function


public Q_SLOTS:

//    void btn_con1_start_clicked();
//    void btn_con2_start_clicked();
//    void btn_con1_direction_clicked();
//    void btn_con2_direction_clicked();
//    void btn_grip_grasp_pressed();
//    void btn_grip_release_pressed();
//    void btn_test_config_pressed();
//    void btn_reset_config_pressed();

//    void btn_set_position_clicked();

//    void btn_x_axis_positive(){}
//    void btn_y_axis_positive(){}
//    void btn_z_axis_positive(){}
//    void btn_rotate_positive(){}

//    void btn_x_axis_negative(){}
//    void btn_y_axis_negative(){}
//    void btn_z_axis_negative(){}
//    void btn_rotate_negative(){}
    /******************************************
    ** Manual connections
    *******************************************/
//    void updateLoggingView(); // no idea why this can't connect automatically

//        void MyTimerSlot();

//    void newest_configuration(double *configuration);
//    void newest_pose(double *pose);





private:
	Ui::MainWindowDesign ui;
	QNode qnode;
    bool con1_start;
    bool con1_dir;

    bool con2_start;
    bool con2_dir;

    std_msgs::UInt32 msg;
    int grip_pos;

};

}  // namespace HMI

#endif // HMI_MAIN_WINDOW_H
