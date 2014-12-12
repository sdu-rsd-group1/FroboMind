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
#include <cmath>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace HMI {

using namespace std;
using namespace Qt;

/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
	: QMainWindow(parent)
	, qnode(argc,argv)
{


	ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.

	setWindowIcon(QIcon(":/images/icon.png"));
	ui.tab_manager->setCurrentIndex(0); // ensure the first tab is showing - qt-designer should have this already hardwired, but often loses it (settings?).
    QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));
    QObject::connect(ui.btn_master, SIGNAL(clicked()), this, SLOT(btn_master_clicked()));

	/*********************
	** Logging
	**********************/
    ui.view_log_HMI->setModel(qnode.HmiLogModel());
    ui.view_log_Rob->setModel(qnode.RobLogModel());
    ui.view_log_Vis->setModel(qnode.VisLogModel());
    ui.view_log_MES->setModel(qnode.MesLogModel());
    ui.view_log_Con->setModel(qnode.ConLogModel());
    ui.view_log_Complete->setModel(qnode.CompleteLogModel());
    QObject::connect(&qnode, SIGNAL(loggingUpdated()), this, SLOT(updateLoggingView()));


    // setup signal and slot
    connect(&qnode, SIGNAL(runStateMachine()),this, SLOT(StateMachine()));

    initialize();

    qnode.init();
}

void MainWindow::initialize(){
    state = STOP;
    ui.btn_master->setText("Start");
    ui.btn_master->setStyleSheet("background-color: green");

    ui.lbl_state->setStyleSheet("background-color: red");
    ui.lbl_state->setText("Stopped");
}

MainWindow::~MainWindow() {}

/*****************************************************************************
** Implementation [Slots]
*****************************************************************************/

void MainWindow::btn_master_clicked(){
    switch(state)
    {
        case STOP:
        {
            cout << "State: Start" << endl;
            state = START;
            qnode.publish_state(state);
            break;
        }
        default:
        {
            state = STOP;
            qnode.publish_state(state);
            cout << "State: Stop" << endl;
            break;
        }
    }
}

/*****************************************************************************
** Implemenation [Slots][manually connected]
*****************************************************************************/

/**
 * This function is signalled by the underlying model. When the model changes,
 * this will drop the cursor down to the last line in the QListview to ensure
 * the user can always see the latest log message.
 */
void MainWindow::updateLoggingView() {
        ui.view_log_HMI->scrollToBottom();
        ui.view_log_Rob->scrollToBottom();
        ui.view_log_Vis->scrollToBottom();
        ui.view_log_MES->scrollToBottom();
        ui.view_log_Con->scrollToBottom();
        ui.view_log_Complete->scrollToBottom();
}

/*****************************************************************************
** Implementation [Configuration]
*****************************************************************************/

void MainWindow::updatePositions(){
        ui.lbl_current_q_1->setText(QString::number(qnode.current_config[0]));
        ui.lbl_current_q_2->setText(QString::number(qnode.current_config[1]));
        ui.lbl_current_q_3->setText(QString::number(qnode.current_config[2]));
        ui.lbl_current_q_4->setText(QString::number(qnode.current_config[3]));
        ui.lbl_current_q_5->setText(QString::number(qnode.current_config[4]));
        ui.lbl_current_q_6->setText(QString::number(qnode.current_config[5]));

        ui.lbl_current_pos_1->setText(QString::number(qnode.current_pose[0]));
        ui.lbl_current_pos_2->setText(QString::number(qnode.current_pose[1]));
        ui.lbl_current_pos_3->setText(QString::number(qnode.current_pose[2]));
        ui.lbl_current_pos_4->setText(QString::number(qnode.current_pose[3]));
        ui.lbl_current_pos_5->setText(QString::number(qnode.current_pose[4]));
        ui.lbl_current_pos_6->setText(QString::number(qnode.current_pose[5]));
}

void MainWindow::StateMachine(){

    updatePositions();

    switch(state)
    {
        case STOP:
        {
            stateStop();
            break;
        }
        case START:
        {
            stateStart();
            break;
        }
        case READY:
        {
            stateReady();
            break;
        }
        case EXECUTE:
        {
            stateExecute();
            break;
        }
        case SUSPENDED:
        {
            stateSuspended();
            break;
        }
        case GO_TO_UPPER_BRICK:
        {
            stateUpperBrick();
            break;
        }
        case OPEN_GRIP:
        {
            stateReleaseBrick(GO_TO_LOWER_BRICK);
            break;
        }
        case GO_TO_LOWER_BRICK:
        {
            stateLowerBrick();
            break;
        }
        case GRASP_BRICK:
        {
            stateGraspBrick();
            break;
        }
        case BRICK_TO_MIDDLE:
        {
            stateBrickToMiddle();
            break;
        }
        case MIDDLE_TO_BOX:
        {
            stateMiddleToBox();
            break;
        }
        case RELEASE_BRICK:
        {
            stateReleaseBrick(BOX_TO_MIDDLE);
            break;
        }
        case BOX_TO_MIDDLE:
        {
            stateBoxToMiddle();
            break;
        }
        case COMPLETED:
        {
            stateCompleted();
            break;
        }
    }
}

void MainWindow::stateStop(){
    ui.lbl_state->setStyleSheet("background-color: red");
    ui.lbl_state->setText("Stopped");

    ui.btn_master->setText("Start");
    ui.btn_master->setStyleSheet("background-color: green");
}

void MainWindow::stateStart(){

    ui.btn_master->setText("Stop");
    ui.btn_master->setStyleSheet("background-color: red");

    ui.lbl_state->setStyleSheet("background-color: yellow");
    ui.lbl_state->setText("Starting");

    double math = (qnode.current_pose[0]-PICKUP_BOX_CENTERX)*(qnode.current_pose[0]-PICKUP_BOX_CENTERX) + (qnode.current_pose[1]-PICKUP_BOX_CENTERY)*(qnode.current_pose[1]-PICKUP_BOX_CENTERY);

    cout << "Start " << sqrt(math) <<   endl;

    if(sqrt(math) < 0.02)
    {
        state = READY;
        cout << "State: Ready" << endl;
        qnode.publish_state(state);
    }
}

void MainWindow::stateReady(){

    ui.lbl_state->setStyleSheet("background-color: green");
    ui.lbl_state->setText("Ready");

    state = EXECUTE;
    cout << "State: Execute" << endl;
    qnode.publish_state(state);
}

void MainWindow::stateExecute(){

    ui.lbl_state->setStyleSheet("background-color: blue");
    ui.lbl_state->setText("Executing");

    state = SUSPENDED;
    cout << "State: Suspended" << endl;
    qnode.publish_state(state);
}

void MainWindow::stateSuspended(){

    ui.lbl_state->setText("Waiting for brick");

    state = GO_TO_UPPER_BRICK;
    cout << "State: Going to brick" << endl;
    qnode.publish_state(state);
}

void MainWindow::stateUpperBrick(){

    ui.lbl_state->setText("Finding new brick");
double math = 0;
    for(int i = 0; i < 2; i++)
    {
        math += (qnode.current_pose[i]-qnode.next_brick_pos[i])*(qnode.current_pose[i]-qnode.next_brick_pos[i]);
    }

    if(sqrt(math) < 0.02)
    {
        state = OPEN_GRIP;
        qnode.publish_state(state);
        cout << "State: open grip" << endl;
    }
}

void MainWindow::stateReleaseBrick(states next_state){

    ui.lbl_state->setText("Opening grip");

    if(qnode.wsg_width > RELEASE_WIDTH_THRESHOLD)
    {
        state = next_state;
        qnode.publish_state(state);
        cout << "State: next state" << endl;
    }
}

void MainWindow::stateGraspBrick(){

    ui.lbl_state->setText("Grasping brick");

    if(qnode.wsg_width < GRASP_WIDTH_THRESHOLD)
    {
        state = START;
        qnode.publish_state(state);
        cout << "State: Start"  <<qnode.wsg_width << endl;
    }
    else if(qnode.wsg_width < RELEASE_WIDTH_THRESHOLD && qnode.wsg_width > GRASP_WIDTH_THRESHOLD)
    {
        state = BRICK_TO_MIDDLE;
        qnode.publish_state(state);
        cout << "State: Brick To middle" << endl;
    }
}

void MainWindow::stateLowerBrick(){

    ui.lbl_state->setText("Lowering tool");

    if(qnode.current_pose[2] < PICKUP_BOX_ZNEG_UP-0.02)
    {
        state = GRASP_BRICK;
        cout << "State: Grasp Brick" << endl;
        qnode.publish_state(state);
    }
}

void MainWindow::stateBrickToMiddle(){

    ui.lbl_state->setText("Going to middle");

    double math = (qnode.current_pose[0]-PICKUP_BOX_CENTERX)*(qnode.current_pose[0]-PICKUP_BOX_CENTERX) + (qnode.current_pose[1]-PICKUP_BOX_CENTERY)*(qnode.current_pose[1]-PICKUP_BOX_CENTERY);

    cout <<"going to middle " << sqrt(math) <<   endl;

    if(sqrt(math) < 0.2)//0.02)//qnode.wsg_width > 60 && )
    {
        state = MIDDLE_TO_BOX;
        qnode.publish_state(state);
        cout << "State: Middle to box" << endl;
    }
}

void MainWindow::stateMiddleToBox(){
        ui.lbl_state->setText("Going to box");

        double math = (qnode.current_pose[0]-DELIVER_BOX_X)*(qnode.current_pose[0]-DELIVER_BOX_X) + (qnode.current_pose[1]-DELIVER_BOX_Y)*(qnode.current_pose[1]-DELIVER_BOX_Y);

        cout <<"going to box " << sqrt(math) <<   endl;

    if(sqrt(math) < 0.02)//qnode.wsg_width > 60 && )
    {
        state = RELEASE_BRICK;
        qnode.publish_state(state);
        cout << "State: Release brick" << endl;
    }
}

void MainWindow::stateBoxToMiddle(){
    ui.lbl_state->setText("Going to middle");

    double math = (qnode.current_pose[0]-PICKUP_BOX_CENTERX)*(qnode.current_pose[0]-PICKUP_BOX_CENTERX) + (qnode.current_pose[1]-PICKUP_BOX_CENTERY)*(qnode.current_pose[1]-PICKUP_BOX_CENTERY);

    cout <<"going to middle " << sqrt(math) <<   endl;

    if(sqrt(math) < 0.02)//qnode.wsg_width > 60 && )
    {
        state = SUSPENDED;
        qnode.publish_state(state);
        cout << "State: SUSPENDED" << endl;
    }
}

void MainWindow::stateCompleted(){
    cout << "State: Completed" << endl;
    ui.lbl_state->setStyleSheet("background-color: yellow");
    ui.lbl_state->setText("Order Completed");
}

}  // namespace HMI

