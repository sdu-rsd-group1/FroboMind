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
#include <QTimer>
#include "../../../shared.hpp"
#include "std_msgs/UInt32.h"
#include "../../../../RX60Controller/src/rx60_controller/RX60Driver.hpp"

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

public Q_SLOTS:
	/******************************************
	** Auto-connections (connectSlotsByName())
	*******************************************/

    void btn_master_clicked();
    void StateMachine();


    void hmi_debug_checked(bool setting);
    void rob_debug_checked(bool setting);
    void vis_debug_checked(bool setting);
    void mes_debug_checked(bool setting);
    void con_debug_checked(bool setting);
    void conf_vision_checked(bool setting);

    void mes_status(int status);

    void update_OEE();

    void abort_security();
    /******************************************
    ** Manual connections
    *******************************************/
    void updateLoggingView(); // no idea why this can't connect automatically


private:
	Ui::MainWindowDesign ui;
	QNode qnode;

    states state;
//    RX60Driver * RobotStatus;
    void initialize();

    void updatePositions();

    /******************************************
    ** States
    *******************************************/
    void stateStop();
    void stateStart();
    void stateReady();
    void stateExecute();
    void stateSuspended();
    void stateUpperBrick();
    void stateLowerBrick();
    void stateGraspBrick();
    void stateBrickToMiddle();
    void stateMiddleToBox();
    void stateBoxToMiddle();
    void stateCompleted();
    void stateReleaseBrick(states next_state);
    void stateGraspBrick(states next_state);
    void switch_state_color();



};
}  // namespace HMI

#endif // HMI_MAIN_WINDOW_H
