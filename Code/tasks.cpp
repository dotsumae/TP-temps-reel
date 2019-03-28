/*
 * Copyright (C) 2018 dimercur
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "tasks.h"
#include <stdexcept>

// Déclaration des priorités des taches
#define PRIORITY_TSERVER 30
#define PRIORITY_TOPENCOMROBOT 20
#define PRIORITY_TMOVE 30
#define PRIORITY_TSENDTOMON 24
#define PRIORITY_TRECEIVEFROMMON 29
#define PRIORITY_TSTARTROBOT 20
#define PRIORITY_TCHECKBAT 5 
#define PRIORITY_TRELOADWD 23
#define PRIORITY_TLOSTCOMRS 28
#define PRIORITY_TVISION 18
#define PRIORITY_TLOSTCOMSM 22
/*
 * Some remarks:
 * 1- This program is mostly a template. It shows you how to create tasks, semaphore
 *   message queues, mutex ... and how to use them
 * 
 * 2- semDumber is, as name say, useless. Its goal is only to show you how to use semaphore
 * 
 * 3- Data flow is probably not optimal
 * 
 * 4- Take into account that ComRobot::Write will block your task when serial buffer is full,
 *   time for internal buffer to flush
 * 
 * 5- Same behavior existe for ComMonitor::Write !
 * 
 * 6- When you want to write something in terminal, use cout and terminate with endl and flush
 * 
 * 7- Good luck !
 */

/**
 * @brief Initialisation des structures de l'application (tâches, mutex, 
 * semaphore, etc.)
 */
void Tasks::Init() {
    int status;
    int err;

    /**************************************************************************************/
    /* 	Mutex creation                                                                    */
    /**************************************************************************************/
    if (err = rt_mutex_create(&mutex_monitor, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_robot, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_robotStarted, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_move, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_withWd, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_findPosition, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_openCamera, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_arenaOk, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_findArena, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    cout << "Mutexes created successfully" << endl << flush;

    /**************************************************************************************/
    /* 	Semaphors creation       							  */
    /**************************************************************************************/
    if (err = rt_sem_create(&sem_barrier, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_openComRobot, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_serverOk, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_startRobot, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_startReloadWd, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_vision, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_arenaOk, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_lostComSM, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    cout << "Semaphores created successfully" << endl << flush;

    /**************************************************************************************/
    /* Tasks creation                                                                     */
    /**************************************************************************************/
    if (err = rt_task_create(&th_server, "th_server", 0, PRIORITY_TSERVER, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_sendToMon, "th_sendToMon", 0, PRIORITY_TSENDTOMON, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_receiveFromMon, "th_receiveFromMon", 0, PRIORITY_TRECEIVEFROMMON, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_openComRobot, "th_openComRobot", 0, PRIORITY_TOPENCOMROBOT, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_startRobot, "th_startRobot", 0, PRIORITY_TSTARTROBOT, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_move, "th_move", 0, PRIORITY_TMOVE, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_checkBattery, "th_checkBattery", 0, PRIORITY_TCHECKBAT, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_reloadWd, "th_reloadWd", 0, PRIORITY_TRELOADWD, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_lostComRS, "th_lostComRS", 0, PRIORITY_TLOSTCOMRS, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }

    
    if (err = rt_task_create(&th_lostComSM, "th_lostComSM", 0, PRIORITY_TLOSTCOMSM, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }

    cout << "Tasks created successfully" << endl << flush;

    /**************************************************************************************/
    /* Message queues creation                                                            */
    /**************************************************************************************/
    if ((err = rt_queue_create(&q_messageToMon, "q_messageToMon", sizeof (Message*)*100, Q_UNLIMITED, Q_FIFO)) < 0) {
        cerr << "Error msg queue create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if ((err = rt_queue_create(&q_messageToRobot, "q_messageToRobot", sizeof (Message*)*100, Q_UNLIMITED, Q_FIFO)) < 0) {
        cerr << "Error msg queue create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    cout << "Queues created successfully" << endl << flush;

}

/**
 * @brief Démarrage des tâches
 */
void Tasks::Run() {
    rt_task_set_priority(NULL, T_LOPRIO);
    int err;

    if (err = rt_task_start(&th_server, (void(*)(void*)) & Tasks::ServerTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_sendToMon, (void(*)(void*)) & Tasks::SendToMonTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_receiveFromMon, (void(*)(void*)) & Tasks::ReceiveFromMonTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_openComRobot, (void(*)(void*)) & Tasks::OpenComRobot, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_startRobot, (void(*)(void*)) & Tasks::StartRobotTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_move, (void(*)(void*)) & Tasks::MoveTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_checkBattery, (void(*)(void*)) & Tasks::CheckBatteryTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_reloadWd, (void(*)(void*)) & Tasks::ReloadWdTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_lostComRS, (void(*)(void*)) & Tasks::LostComRSTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    
    if (err = rt_task_start(&th_lostComSM, (void(*)(void*)) & Tasks::LostComSMTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    

    cout << "Tasks launched" << endl << flush;
}

/**
 * @brief Arrêt des tâches
 */
void Tasks::Stop() {
    monitor.Close();
    robot.Close();
}

/**
 */
void Tasks::Join() {
    cout << "Tasks synchronized" << endl << flush;
    rt_sem_broadcast(&sem_barrier);
    pause();
}

/**
 * @brief Thread handling server communication with the monitor.
 */
void Tasks::ServerTask(void *arg) {
    int status;

    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are started)
    rt_sem_p(&sem_barrier, TM_INFINITE);

    /**************************************************************************************/
    /* The task server starts here                                                        */
    /**************************************************************************************/
    rt_mutex_acquire(&mutex_monitor, TM_INFINITE);
    status = monitor.Open(SERVER_PORT);
    rt_mutex_release(&mutex_monitor);

    cout << "Open server on port " << (SERVER_PORT) << " (" << status << ")" << endl;

    if (status < 0) throw std::runtime_error {
        "Unable to start server on port " + std::to_string(SERVER_PORT)
    };
    monitor.AcceptClient(); // Wait the monitor client
    cout << "Rock'n'Roll baby, client accepted!" << endl << flush;
    rt_sem_broadcast(&sem_serverOk);
}

/**
 * @brief Thread sending data to monitor.
 */
void Tasks::SendToMonTask(void* arg) {
    Message *msg;

    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);

    /**************************************************************************************/
    /* The task sendToMon starts here                                                     */
    /**************************************************************************************/
    rt_sem_p(&sem_serverOk, TM_INFINITE);

    while (1) {
        cout << "wait msg to send" << endl << flush;
        msg = ReadInQueue(&q_messageToMon);
        cout << "Send msg to mon: " << msg->ToString() << endl << flush;
        rt_mutex_acquire(&mutex_monitor, TM_INFINITE);
        monitor.Write(msg); // The message is deleted with the Write
        rt_mutex_release(&mutex_monitor);
        
    }
}

/**
 * @brief Thread receiving data from monitor.
 */
void Tasks::ReceiveFromMonTask(void *arg) {
    Message *msgRcv;

    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);

    /**************************************************************************************/
    /* The task receiveFromMon starts here                                                */
    /**************************************************************************************/
    rt_sem_p(&sem_serverOk, TM_INFINITE);
    cout << "Received message from monitor activated" << endl << flush;

    while (1) {
        msgRcv = monitor.Read();
        cout << "Rcv <= " << msgRcv->ToString() << endl << flush;

        if (msgRcv->CompareID(MESSAGE_MONITOR_LOST)) {
            rt_sem_v(&sem_lostComSM);
            delete(msgRcv);
            rt_task_delete(&th_receiveFromMon);
        }

            ////////////////th_openComRobot////////////////////

        else if (msgRcv->CompareID(MESSAGE_ROBOT_COM_OPEN)) {
            rt_sem_v(&sem_openComRobot);
        }
            ////////////////th_startRobot////////////////////
        else if (msgRcv->CompareID(MESSAGE_ROBOT_START_WITH_WD)) { //start without WD
            rt_mutex_acquire(&mutex_withWd, TM_INFINITE);
            withWd = 1;
            rt_mutex_release(&mutex_withWd);
            rt_sem_v(&sem_startRobot);
        } else if (msgRcv->CompareID(MESSAGE_ROBOT_START_WITHOUT_WD)) { //start with WD
            rt_mutex_acquire(&mutex_withWd, TM_INFINITE);
            withWd = 0;
            rt_mutex_release(&mutex_withWd);
            rt_sem_v(&sem_startRobot);
        }

            ////////////////th_move////////////////////
        else if (msgRcv->CompareID(MESSAGE_ROBOT_GO_FORWARD) ||
                msgRcv->CompareID(MESSAGE_ROBOT_GO_BACKWARD) ||
                msgRcv->CompareID(MESSAGE_ROBOT_GO_LEFT) ||
                msgRcv->CompareID(MESSAGE_ROBOT_GO_RIGHT) ||
                msgRcv->CompareID(MESSAGE_ROBOT_STOP)) {

            rt_mutex_acquire(&mutex_move, TM_INFINITE);
            move = msgRcv->GetID();
            rt_mutex_release(&mutex_move);
        }

            ////////////////th_vision////////////////////


        else if (msgRcv->CompareID(MESSAGE_CAM_OPEN) || //openCamera, closeCamera, findArena
                msgRcv->CompareID(MESSAGE_CAM_CLOSE) ||
                msgRcv->CompareID(MESSAGE_CAM_ASK_ARENA) ||
                msgRcv->CompareID(MESSAGE_CAM_POSITION_COMPUTE_START) ||
                msgRcv->CompareID(MESSAGE_CAM_POSITION_COMPUTE_STOP))
 {




            if (msgRcv->CompareID(MESSAGE_CAM_OPEN)) {
                rt_mutex_acquire(&mutex_openCam, TM_INFINITE);
                openCamera = 1;
                rt_mutex_release(&mutex_openCam);
                if (int err = rt_task_create(&th_vision, "th_vision", 0, PRIORITY_TVISION, 0)) {
                    cerr << "Error task create: " << strerror(-err) << endl << flush;
                    exit(EXIT_FAILURE);
                }
                if (int err = rt_task_start(&th_vision, (void(*)(void*)) & Tasks::VisionTask, this)) {
                    cerr << "Error task start: " << strerror(-err) << endl << flush;
                    exit(EXIT_FAILURE);
                }
            } else if (msgRcv->CompareID(MESSAGE_CAM_CLOSE)) {
                rt_mutex_acquire(&mutex_openCam, TM_INFINITE);
                openCamera = 0;
                rt_mutex_release(&mutex_openCam);
            } else if (msgRcv->CompareID(MESSAGE_CAM_POSITION_COMPUTE_START)) { //findPosition
                rt_mutex_acquire(&mutex_findPosition, TM_INFINITE);
                findPosition = 1;
                rt_mutex_release(&mutex_findPosition);

            } else if (msgRcv->CompareID(MESSAGE_CAM_POSITION_COMPUTE_STOP)) { //stopfindPosition
                rt_mutex_acquire(&mutex_findPosition, TM_INFINITE);
                findPosition = 0;
                rt_mutex_release(&mutex_findPosition);
            } else if (msgRcv->CompareID(MESSAGE_CAM_ASK_ARENA)) {
                rt_mutex_acquire(&mutex_findArena, TM_INFINITE);
                findArena = 1;
                rt_mutex_release(&mutex_findArena);
            }
            rt_sem_v(&sem_vision);
        }
        else if (msgRcv->CompareID(MESSAGE_CAM_ARENA_CONFIRM) ||
                msgRcv->CompareID(MESSAGE_CAM_ARENA_INFIRM)) {


            
            if (msgRcv->CompareID(MESSAGE_CAM_ARENA_CONFIRM)) {
                rt_mutex_acquire(&mutex_arenaOk, TM_INFINITE);
                arenaOk = 1;
                rt_mutex_release(&mutex_arenaOk);

            } else if (msgRcv->CompareID(MESSAGE_CAM_ARENA_INFIRM)) {
                rt_mutex_acquire(&mutex_arenaOk, TM_INFINITE);
                arenaOk = 0;
                rt_mutex_release(&mutex_arenaOk);
            }
            rt_sem_v(&sem_arenaOk);
            rt_sem_v(&sem_vision);
        }
        delete(msgRcv); // mus be deleted manually, no consumer
    }
}

/**
 * @brief Thread opening communication with the robot.
 */
void Tasks::OpenComRobot(void *arg) {
    int status;
    int err;

    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);

    /**************************************************************************************/
    /* The task openComRobot starts here                                                  */
    /**************************************************************************************/
    while (1) {
        rt_sem_p(&sem_openComRobot, TM_INFINITE);
        cout << "Open serial com (";
        rt_mutex_acquire(&mutex_robot, TM_INFINITE);
        status = robot.Open();
        rt_mutex_release(&mutex_robot);
        cout << status;
        cout << ")" << endl << flush;

        Message * msgSend;
        if (status < 0) {
            msgSend = new Message(MESSAGE_ANSWER_NACK);
        } else {
            msgSend = new Message(MESSAGE_ANSWER_ACK);
        }
        WriteInQueue(&q_messageToMon, msgSend); // msgSend will be deleted by sendToMon
    }
}

/**
 * @brief Thread starting the communication with the robot.
 */
void Tasks::StartRobotTask(void *arg) {
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    int withWdLocal;
    /**************************************************************************************/
    /* The task startRobot starts here                                                    */
    /**************************************************************************************/
    while (1) {

        Message * msgSend;
        rt_sem_p(&sem_startRobot, TM_INFINITE);

        rt_mutex_acquire(&mutex_withWd, TM_INFINITE);
        withWdLocal = withWd;
        rt_mutex_release(&mutex_withWd);
        if (withWdLocal == 0) {
            cout << "Start robot without watchdog (";
            rt_mutex_acquire(&mutex_robot, TM_INFINITE);
            msgSend = robot.Write(robot.StartWithoutWD());
            rt_mutex_release(&mutex_robot);
            cout << msgSend->GetID();
            cout << ")" << endl;
        } else if (withWdLocal == 1) {
            cout << "Start robot with watchdog (";
            rt_mutex_acquire(&mutex_robot, TM_INFINITE);
            msgSend = robot.Write(robot.StartWithWD());
            rt_mutex_release(&mutex_robot);
            cout << msgSend->GetID();
            cout << ")" << endl;
            rt_sem_v(&sem_startReloadWd);
        }
        WriteInQueue(&q_messageToRobot, msgSend); //to verify if we lost connectivity
        cout << "Movement answer: " << msgSend->ToString() << endl << flush;
        WriteInQueue(&q_messageToMon, msgSend); // msgSend will be deleted by sendToMon

        if (msgSend->GetID() == MESSAGE_ANSWER_ACK) {
            rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
            robotStarted = 1;
            rt_mutex_release(&mutex_robotStarted);
        }
    }
}

/**
 * @brief Thread handling control of the robot.
 */
void Tasks::MoveTask(void *arg) {
    int rs;
    int cpMove;
    Message *msgSend;

    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);

    /**************************************************************************************/
    /* The task starts here                                                               */
    /**************************************************************************************/
    rt_task_set_periodic(NULL, TM_NOW, 100000000);

    while (1) {
        rt_task_wait_period(NULL);
        //cout << "Periodic movement update"<<endl;
        rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
        rs = robotStarted;
        rt_mutex_release(&mutex_robotStarted);
        if (rs == 1) {
            rt_mutex_acquire(&mutex_move, TM_INFINITE);
            cpMove = move;
            rt_mutex_release(&mutex_move);

            cout << " move: " << cpMove << endl;

            rt_mutex_acquire(&mutex_robot, TM_INFINITE);
            msgSend = robot.Write(new Message((MessageID) cpMove));
            rt_mutex_release(&mutex_robot);
            WriteInQueue(&q_messageToRobot, msgSend); //to verify if we lost connectivity
        }
        //cout << endl << flush;
    }
}

/**
 * Write a message in a given queue
 * @param queue Queue identifier
 * @param msg Message to be stored
 */
void Tasks::WriteInQueue(RT_QUEUE *queue, Message *msg) {
    int err;
    if ((err = rt_queue_write(queue, (const void *) &msg, sizeof ((const void *) &msg), Q_NORMAL)) < 0) {
        cerr << "Write in queue failed: " << strerror(-err) << endl << flush;
        throw std::runtime_error{"Error in write in queue"};
    }
}

/**
 * Read a message from a given queue, block if empty
 * @param queue Queue identifier
 * @return Message read
 */
Message *Tasks::ReadInQueue(RT_QUEUE *queue) {
    int err;
    Message *msg;

    if ((err = rt_queue_read(queue, &msg, sizeof ((void*) &msg), TM_INFINITE)) < 0) {
        cout << "Read in queue failed: " << strerror(-err) << endl << flush;
        throw std::runtime_error{"Error in read in queue"};
    }/** else {
        cout << "@msg :" << msg << endl << flush;
    } /**/

    return msg;
}

void Tasks::CheckBatteryTask(void *arg) {
    MessageBattery * lvlbat;
    int rs;

    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);

    rt_task_set_periodic(NULL, TM_NOW, 500000000);

    while (1) {
        rt_task_wait_period(NULL);

        rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
        rs = robotStarted;
        rt_mutex_release(&mutex_robotStarted);
        if (rs == 1) {
            cout << "Periodic battery update" << endl;
            rt_mutex_acquire(&mutex_robot, TM_INFINITE);
            lvlbat = (MessageBattery*) robot.Write(new Message(MESSAGE_ROBOT_BATTERY_GET));
            rt_mutex_release(&mutex_robot);

            WriteInQueue(&q_messageToRobot, lvlbat); //to verify if we lost connectivity

            cout << "Send msg to mon: " << lvlbat->ToString() << endl << flush;
            WriteInQueue(&q_messageToMon,lvlbat);
        }
    }
}

void Tasks::ReloadWdTask(void *args) {
    int rs = 0;
    Message *msgSend;
    int withWdLocal=0;
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);

    rt_sem_p(&sem_startReloadWd, TM_INFINITE);

    rt_task_set_periodic(NULL, TM_NOW, 1000000000); //T=1s

    while (1) {
        rt_mutex_acquire(&mutex_withWd, TM_INFINITE);
        withWdLocal = withWd;
        rt_mutex_release(&mutex_withWd);
        rt_task_wait_period(NULL);
        if (withWdLocal == 1) {
            rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
            rs = robotStarted;
            rt_mutex_release(&mutex_robotStarted);
            if (rs == 1) {
                rt_mutex_acquire(&mutex_robot, TM_INFINITE);
                msgSend = robot.Write(new Message(MESSAGE_ROBOT_RELOAD_WD));
                rt_mutex_release(&mutex_robot);
                WriteInQueue(&q_messageToRobot, msgSend); //to verify if we lost connectivity
            }
        }
    }
}

void Tasks::LostComRSTask(void *args) {

    int rs = 0;
    Message *msg;
    int compteur = 0;

    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);

    while (1) {
        rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
        rs = robotStarted;
        rt_mutex_release(&mutex_robotStarted);
        if (rs == 1) {
            cout << "wait msg" << endl << flush;
            msg = ReadInQueue(&q_messageToRobot);
            cout << "message ID : "<< msg->GetID() << endl;
            if (msg->GetID() == MESSAGE_ANSWER_ACK) {
                compteur = 0;
            } else {
                compteur++;
            }
            cout << "compteur perte rs : " << compteur << endl;
            if (compteur == 5) {

                //////////////////////l'envoi du message ne fonctionne pas


                rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
                robotStarted = 0;
                rs = robotStarted;
                rt_mutex_release(&mutex_robotStarted);
                //usleep(1200);
                rt_mutex_acquire(&mutex_robot, TM_INFINITE);
                robot.Close();
                cout << "com robot close " << endl;
                rt_mutex_release(&mutex_robot);

                rt_sem_v(&sem_openComRobot);
            }
        }
    }
}

void Tasks::VisionTask(void *args) {
    

    /*cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    int x = rt_sem_p(&sem_barrier, TM_INFINITE);
    cout << "retour : " << x << endl;
*/
    Message *msgSend;
    ImageMat temp;
    Camera *cam;
    Arena *arena=new Arena();
    Position *position = new Position();
    list<Position> listPosition;
    int findPositionLocal;
    int openCameraLocal;
    int findArenaLocal;
    int arenaOkLocal;
    cam = new Camera(xs, 10);
    Img image(temp);

    /**************************************************************************************/
    /* The task Vision starts here                                                    */
    /**************************************************************************************/
    while (1) {


        //////////////////////////////OPEN CAMERA////////// //////////////////
       
        rt_sem_p(&sem_vision, TM_INFINITE);

        rt_mutex_acquire(&mutex_openCamera, TM_INFINITE);
        openCameraLocal = openCamera;
        rt_mutex_release(&mutex_openCamera);
        if (openCameraLocal == 1) {
            cam->Open();
            if (cam->IsOpen() == 0) { //open cam non ok
                cout << "Problem in opening camera" << endl << flush;
                WriteInQueue(&q_messageToMon,new Message(MESSAGE_ANSWER_NACK));
            } else { //open cam ok
                cout << "Camera open" << endl << flush;
                WriteInQueue(&q_messageToMon,new Message(MESSAGE_ANSWER_ACK));
                rt_task_set_periodic(NULL, TM_NOW, 300000000);
                while (1) {
                    
                    /////////////////////////////ARENA////////////////////////////////////////////////////

                    rt_mutex_acquire(&mutex_findArena, TM_INFINITE);
                    findArenaLocal = findArena;
                    rt_mutex_release(&mutex_findArena);
                    if (findArenaLocal == 1) {
                        rt_mutex_acquire(&mutex_findArena, TM_INFINITE);
                        findArena=0;
                        rt_mutex_release(&mutex_findArena);
                        image = cam->Grab();
                        delete arena;
                        arena = new Arena(image.SearchArena());
                        if (arena->IsEmpty()==false) {

                            cout << "Arena found" << endl << flush;
                            image.DrawArena(*arena);
                            WriteInQueue(&q_messageToMon,new MessageImg(MESSAGE_CAM_IMAGE,&image));
                            cout << "Waiting arena ACK" << endl;
                            rt_sem_p(&sem_arenaOk, TM_INFINITE);
                            
                            rt_mutex_acquire(&mutex_arenaOk, TM_INFINITE);
                            arenaOkLocal=arenaOk;
                            rt_mutex_release(&mutex_arenaOk);
                            if (arenaOkLocal == 0) {
                                delete arena;
                                arena = new Arena();
                            }
                        } else {

                            cout << "No arena found" << endl << flush;
                            WriteInQueue(&q_messageToMon,new Message(MESSAGE_ANSWER_NACK));
                        }
                    }
                    rt_mutex_acquire(&mutex_openCamera, TM_INFINITE);
                    openCameraLocal = openCamera;
                    rt_mutex_release(&mutex_openCamera);

                    if (openCameraLocal == 0) {
                        cam->Close();
                        delete cam;
                        cout << "Camera closing, unrestartable : restart the program" << endl << flush;
                        rt_task_delete(&th_vision);
                    } else {
                        
                        rt_task_wait_period(NULL);
                        cout << "Periodic image update" << endl << flush;
                        image =cam->Grab();
                            /////////////////////////////////////////////FIND POSITION////////////////////////////////////
                        
                        rt_mutex_acquire(&mutex_findPosition, TM_INFINITE);
                        findPositionLocal = findPosition;
                        rt_mutex_release(&mutex_findPosition);

                        if (findPositionLocal == 0) {
                            cout << "Stop find position" << endl << flush;
                        }

                        else { //find position=1
                            cout << "Start find position" << endl << flush;
                            listPosition = image.SearchRobot(*arena);
                            
                            delete position;
                            
                            if (listPosition.empty()==false) { //robot in arena 
                                image.DrawRobot(listPosition.front());
                                position = new Position(listPosition.front());
                            } else { //robot not in arena
                                position = new Position();
                                position->center = {-1.0, -1.0};
                            }
                            MessagePosition * msg = new MessagePosition(MESSAGE_CAM_POSITION, *position);
                            WriteInQueue(&q_messageToMon,msg);

                        }
                        ////////////////////////////////////////SEND IMG/////////////////////////////////////////

                        cout << "Sending image" << endl << flush;
                        MessageImg *msgImg = new MessageImg(MESSAGE_CAM_IMAGE, &image);
                        WriteInQueue(&q_messageToMon,msgImg);                        
                    }
                    
                }
                

            }

            //delete(msgSend);
            //delete(image);
            delete(cam);
            delete(arena);
        }
    }
}

void Tasks::LostComSMTask(void *args) {
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    
    
    //Connexion établie ?
    rt_sem_p(&sem_serverOk, TM_INFINITE);    
    rt_sem_p(&sem_lostComSM, TM_INFINITE);

    rt_mutex_acquire(&mutex_monitor, TM_INFINITE);
    monitor.Close(); 
    rt_mutex_release(&mutex_monitor);
    cout << "com monitor close " << endl;
    rt_mutex_acquire(&mutex_robot, TM_INFINITE);
    robot.Write(new Message(MESSAGE_ROBOT_STOP));
    rt_mutex_release(&mutex_robot);
    rt_mutex_acquire(&mutex_robot, TM_INFINITE);
    robot.Close();
    cout << "com robot close " << endl;
    rt_mutex_release(&mutex_robot);
    
    rt_mutex_acquire(&mutex_openCamera, TM_INFINITE);
        openCamera=0;
    rt_mutex_release(&mutex_openCamera);
    cout<<"MOOOOORTTT"<<endl; 
    usleep(1000);
    exit(1);
}//Reset non fait