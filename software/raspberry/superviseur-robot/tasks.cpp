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
#include <vector>

// Task priority: higher has the best priority 
#define PRIORITY_TSERVER 30
#define PRIORITY_TOPENCOMROBOT 20
#define PRIORITY_TMOVE 20
#define PRIORITY_TSENDTOMON 22
#define PRIORITY_TRECEIVEFROMMON 25
#define PRIORITY_TSTARTROBOT 20

// Added Priorities
// Camera is the most important as it has the most demanding timing request (10ms)
#define PRIORITY_TCAMERA 21
// Battery has a timing request of 500ms which is not important
#define PRIORITY_TBATTERY 19
// These priorities are for "Semaphored" tasks, not periodic.
// Their priority is lower.
#define PRIORITY_TSETCAMERA 18
#define PRIOTITY_TARENA 17
#define PRIOTITY_TPOSITION 16

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
 *   time for interponal buffer to flush
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
    
    // Added Mutexes
    if (err = rt_mutex_create(&mutex_batteryGet, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_cameraStatus, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_camera, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_arena, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_arenaStatus, NULL)) {
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
    
    // Added Semaphores
    if (err = rt_sem_create(&sem_instantBattery, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_manageCamera, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_arenaChoice, NULL, 0, S_FIFO)) {
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
    // Added tasks
    if (err = rt_task_create(&th_batteryPeriodic, "th_batteryPeriodic", 0, PRIORITY_TBATTERY, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_battery, "th_battery", 0, PRIORITY_TBATTERY, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_cameraManage, "th_cameraManage", 0, PRIORITY_TSETCAMERA, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_cameraImage, "th_cameraImage", 0, PRIORITY_TCAMERA, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_arenaChoice, "th_arenaChoice", 0, PRIOTITY_TARENA, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    
    cout << "Tasks created successfully" << endl << flush;

    /**************************************************************************************/
    /* Message queues creation                                                            */
    /**************************************************************************************/
    if ((err = rt_queue_create(&q_messageToMon, "q_messageToMon", sizeof (Message*)*50, Q_UNLIMITED, Q_FIFO)) < 0) {
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
    
    // Added Tasks
    if (err = rt_task_start(&th_batteryPeriodic, (void(*)(void*)) & Tasks::BatteryPeriodicTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_battery, (void(*)(void*)) & Tasks::BatteryTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_cameraManage, (void(*)(void*)) & Tasks::ManageCameraTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_cameraImage, (void(*)(void*)) & Tasks::ImageCameraTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_arenaChoice, (void(*)(void*)) & Tasks::ArenaChoiceTask, this)) {
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
            delete(msgRcv);
            exit(-1);
        } else if (msgRcv->CompareID(MESSAGE_ROBOT_COM_OPEN)) {
            rt_sem_v(&sem_openComRobot);
        } else if (msgRcv->CompareID(MESSAGE_ROBOT_START_WITHOUT_WD)) {
            rt_sem_v(&sem_startRobot);
        } else if (msgRcv->CompareID(MESSAGE_ROBOT_GO_FORWARD) ||
                msgRcv->CompareID(MESSAGE_ROBOT_GO_BACKWARD) ||
                msgRcv->CompareID(MESSAGE_ROBOT_GO_LEFT) ||
                msgRcv->CompareID(MESSAGE_ROBOT_GO_RIGHT) ||
                msgRcv->CompareID(MESSAGE_ROBOT_STOP)) {

            rt_mutex_acquire(&mutex_move, TM_INFINITE);
            move = msgRcv->GetID();
            rt_mutex_release(&mutex_move);
        }

        // Added messages
        // Battery
        else if (msgRcv->CompareID(MESSAGE_ROBOT_BATTERY_GET)) {
            rt_mutex_acquire(&mutex_batteryGet, TM_INFINITE);
            batteryEnabled = true;
            rt_mutex_release(&mutex_batteryGet);
            
            // Calling BatteryTask, unblocking the thread.
            rt_sem_v(&sem_instantBattery);
        }
        // Camera
        else if (msgRcv->CompareID(MESSAGE_CAM_OPEN)) {
            rt_mutex_acquire(&mutex_cameraStatus, TM_INFINITE);
            if(CameraStatusEnum::CLOSED == cameraStatus)
                cameraStatus = CameraStatusEnum::OPENING;
            rt_mutex_release(&mutex_cameraStatus);
            
            // Calling manageCamera, unblocking the thread.
            rt_sem_v(&sem_manageCamera);
        }
        else if (msgRcv->CompareID(MESSAGE_CAM_CLOSE)) {
            rt_mutex_acquire(&mutex_cameraStatus, TM_INFINITE);
            if(CameraStatusEnum::OPENED == cameraStatus)
                cameraStatus = CameraStatusEnum::CLOSING;
            rt_mutex_release(&mutex_cameraStatus);
            
            // Calling manageCamera, unblocking the thread.
            rt_sem_v(&sem_manageCamera);
        }
        // Arena
        else if (msgRcv->CompareID(MESSAGE_CAM_ASK_ARENA)) {
            rt_mutex_acquire(&mutex_arenaStatus, TM_INFINITE);
            if(ArenaStatusEnum::NONE == arenaStatus)
                arenaStatus = ArenaStatusEnum::SEARCHING;
            rt_mutex_release(&mutex_arenaStatus);
            
            // Calling arenaChoice, unblocking the thread.
            rt_sem_v(&sem_arenaChoice);
        }
        else if (msgRcv->CompareID(MESSAGE_CAM_ARENA_CONFIRM)) {
            rt_mutex_acquire(&mutex_arenaStatus, TM_INFINITE);
            if(ArenaStatusEnum::SEARCHED == arenaStatus)
                arenaStatus = ArenaStatusEnum::CONFIRM;
            rt_mutex_release(&mutex_arenaStatus);
            
            // Calling arenaChoice, unblocking the thread.
            rt_sem_v(&sem_arenaChoice);
        }
        else if (msgRcv->CompareID(MESSAGE_CAM_ARENA_INFIRM)) {
            rt_mutex_acquire(&mutex_arenaStatus, TM_INFINITE);
            if(ArenaStatusEnum::SEARCHED == arenaStatus)
                arenaStatus = ArenaStatusEnum::INFIRM;
            rt_mutex_release(&mutex_arenaStatus);
            
            // Calling arenaChoice, unblocking the thread.
            rt_sem_v(&sem_arenaChoice);
        }
        else if (msgRcv->CompareID(MESSAGE_CAM_POSITION_COMPUTE_START)){   
            rt_mutex_acquire(&mutex_positionEnabled, TM_INFINITE);
            positionEnabled = true;
            rt_mutex_release(&mutex_positionEnabled);
        }
        else if (msgRcv->CompareID(MESSAGE_CAM_POSITION_COMPUTE_STOP)){   
            rt_mutex_acquire(&mutex_positionEnabled, TM_INFINITE);
            positionEnabled = false;
            rt_mutex_release(&mutex_positionEnabled);
        }
        else if (msgRcv->CompareID(MESSAGE_CAM_IMAGE)) {
            //?
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

    /**************************************************************************************/
    /* The task startRobot starts here                                                    */
    /**************************************************************************************/
    while (1) {

        Message * msgSend;
        rt_sem_p(&sem_startRobot, TM_INFINITE);
        cout << "Start robot without watchdog (";
        rt_mutex_acquire(&mutex_robot, TM_INFINITE);
        msgSend = robot.Write(robot.StartWithoutWD());
        rt_mutex_release(&mutex_robot);
        cout << msgSend->GetID();
        cout << ")" << endl;

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

    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);

    /**************************************************************************************/
    /* The task starts here                                                               */
    /**************************************************************************************/
    rt_task_set_periodic(NULL, TM_NOW, 100000000);

    while (1) {
        rt_task_wait_period(NULL);
        //cout << "Periodic movement update";
        rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
        rs = robotStarted;
        rt_mutex_release(&mutex_robotStarted);
        if (rs == 1) {
            rt_mutex_acquire(&mutex_move, TM_INFINITE);
            cpMove = move;
            rt_mutex_release(&mutex_move);

            cout << " move: " << cpMove;

            rt_mutex_acquire(&mutex_robot, TM_INFINITE);
            robot.Write(new Message((MessageID) cpMove));
            rt_mutex_release(&mutex_robot);
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

/************************************************************************/
/* Added Tasks                                                          */
/************************************************************************/

void Tasks::BatteryPeriodicTask(void * arg) {
    // Variables
    int rs(0);
    bool be(false);

    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);


    // Periodic task set to 500ms
    rt_task_set_periodic(NULL, TM_NOW, 500000000);

    while (1) {
        rt_task_wait_period(NULL);
        this->SendBattery(rs,be);
    }
}

void Tasks::BatteryTask(void * arg) {
    // Variables
    int rs(0);
    bool be(false);
    
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    
    while(1)
    {
        // Called when semaphore arenChoice incremented
        rt_sem_p(&sem_instantBattery, TM_INFINITE);
        this->SendBattery(rs,be);
    }
}

void Tasks::ManageCameraTask(void * arg)
{
    // Variables
    int rs(0);
    CameraStatusEnum cs(CameraStatusEnum::CLOSED);

    // Checking if all tasks started
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    rt_sem_p(&sem_barrier, TM_INFINITE);


    while(1)
    {
        // Called when semaphore manageCamera incremented
        rt_sem_p(&sem_manageCamera, TM_INFINITE);
        
        // Check the status of the camera and store it in a local variable
        rt_mutex_acquire(&mutex_cameraStatus, TM_INFINITE);
        cs = cameraStatus;
        rt_mutex_release(&mutex_cameraStatus);

        // Opening the camera
        if(CameraStatusEnum::OPENING == cs)
        {
            const MessageID tempMessage = this->OpenCamera();
            WriteInQueue(&q_messageToMon, new Message(tempMessage));
        }
        // Closing the camera
        if(CameraStatusEnum::CLOSING == cs)
        {
            this->CloseCamera();
            WriteInQueue(&q_messageToMon, new Message(MESSAGE_ANSWER_ACK));
        }
    }
}

void Tasks::ImageCameraTask(void * arg)
{
    // Variables
    CameraStatusEnum cs = CameraStatusEnum::CLOSED;
    Arena a = Arena();
    std::vector<MessagePosition*> msgPos = {};
    MessageImg *msgImg = nullptr;
    bool pe = false;
    std::list<Position> listPos = {};
    
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    
    // Periodic task set to 10ms
    rt_task_set_periodic(NULL, TM_NOW, 100000000);

    while(1)
    {
        rt_task_wait_period(NULL);
        
        // Check the status of the camera
        rt_mutex_acquire(&mutex_cameraStatus, TM_INFINITE);
        const CameraStatusEnum cs = cameraStatus;
        rt_mutex_release(&mutex_cameraStatus);
        
        // Gathering arena
        rt_mutex_acquire(&mutex_arena, TM_INFINITE);
        a = arena;
        rt_mutex_release(&mutex_arena);
        
        rt_mutex_acquire(&mutex_positionEnabled, TM_INFINITE);
        pe = positionEnabled;
        rt_mutex_release(&mutex_positionEnabled);
        
        // Only gather image if opened
        if(CameraStatusEnum::OPENED == cs)
        {
            // Gathering an image from the camera
            rt_mutex_acquire(&mutex_camera, TM_INFINITE);
            Img * img = new Img(cam->Grab());
            rt_mutex_release(&mutex_camera);
            
            // If arena has been found, draw overlay
            if(!a.IsEmpty())
                img->DrawArena(a);
            
            // Gathering position
            if(pe)
            {
                // Gather every robot available
                listPos = img->SearchRobot(a);
                // For every position, draw the robot and send the message
                for(auto l : listPos)
                {
                    img->DrawRobot(l);
                    msgPos.emplace_back(new MessagePosition(MESSAGE_CAM_POSITION, l));
                }
            }
            
            msgImg= new MessageImg(MESSAGE_CAM_IMAGE, img);
            rt_mutex_acquire(&mutex_monitor, TM_INFINITE);
            // Writing every message position
            for(auto mp : msgPos)
                monitor.Write(mp);
            // Writing every message image
            monitor.Write(msgImg);
            rt_mutex_release(&mutex_monitor);
            
            // clearing the vector because all its values are unassigned
            msgPos.clear();
        }
    }
}

void Tasks::ArenaChoiceTask(void * arg)
{   
    // Variables
    Img* img = nullptr;
    Arena a;
    ArenaStatusEnum as(ArenaStatusEnum::NONE);
    
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    rt_sem_p(&sem_barrier, TM_INFINITE);
    
    while(1)
    {
        // Called when semaphore arenChoice incremented
        rt_sem_p(&sem_arenaChoice, TM_INFINITE);

        // Check the status of the arena
        rt_mutex_acquire(&mutex_arenaStatus, TM_INFINITE);
        as = arenaStatus;
        rt_mutex_release(&mutex_arenaStatus);
        
        // ASK_ARENA
        if(as == ArenaStatusEnum::SEARCHING)
        {   
            // Change status for next command
            rt_mutex_acquire(&mutex_arenaStatus, TM_INFINITE);
            arenaStatus = ArenaStatusEnum::SEARCHED;
            rt_mutex_release(&mutex_arenaStatus);
            
            rt_mutex_acquire(&mutex_cameraStatus, TM_INFINITE);
            const CameraStatusEnum cs = cameraStatus;
            rt_mutex_release(&mutex_cameraStatus);
            
            // Only search if camera is already opened
            if(CameraStatusEnum::OPENED == cs)
            {
                // Prepare to close the camera
                rt_mutex_acquire(&mutex_cameraStatus, TM_INFINITE);
                cameraStatus = CameraStatusEnum::CLOSING;
                rt_mutex_release(&mutex_cameraStatus);
                
                // Gathering last image
                rt_mutex_acquire(&mutex_camera, TM_INFINITE);
                img = new Img(cam->Grab());
                rt_mutex_release(&mutex_camera);
                // Closing camera when prompted
                this->CloseCamera();
                // Putting the arena overlay if found
                a = img->SearchArena();
            }
            
            if(a.IsEmpty())
                WriteInQueue(&q_messageToMon, new Message(MESSAGE_ANSWER_NACK));
            // If arena is not empty show the arena
            else {
                // Adding overlay on the image
                img->DrawArena(a);
                // Sending it to the writing queue
                MessageImg *msgImg = new MessageImg(MESSAGE_CAM_IMAGE, img);
                rt_mutex_acquire(&mutex_monitor, TM_INFINITE);
                monitor.Write(msgImg);
                rt_mutex_release(&mutex_monitor);
                WriteInQueue(&q_messageToMon, new Message(MESSAGE_ANSWER_ACK));
            }
        }
        
        // ARENA_CONFIRM / INFIRM
        if((ArenaStatusEnum::CONFIRM == as) || (ArenaStatusEnum::INFIRM == as))
        {
            rt_mutex_acquire(&mutex_arena, TM_INFINITE);
            arena = a;
            rt_mutex_release(&mutex_arena);
            
            // empty the temporary arena
            a = Arena();
            // Re-open the camera (showing overlay if arena is stored)
            rt_mutex_acquire(&mutex_cameraStatus, TM_INFINITE);
            cameraStatus = CameraStatusEnum::OPENING;
            rt_mutex_release(&mutex_cameraStatus);
            this->OpenCamera();

            // Change the status of the arena to default value
            rt_mutex_acquire(&mutex_arenaStatus, TM_INFINITE);
            arenaStatus = ArenaStatusEnum::NONE;
            rt_mutex_release(&mutex_arenaStatus);
        }
    }
}

/************************************************************************/
/* Added functions                                                      */
/************************************************************************/

void Tasks::SendBattery(int& rs, bool& be)
{
    // Verify that the robot has started
    rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
    rs = robotStarted;
    rt_mutex_release(&mutex_robotStarted);

    // Checking if battery should be gathered
    rt_mutex_acquire(&mutex_batteryGet, TM_INFINITE);
    be = batteryEnabled;
    rt_mutex_release(&mutex_batteryGet);

    if ((rs != 0) && be) {
        // Acquire the battery level and send it to the message queue
        rt_mutex_acquire(&mutex_robot, TM_INFINITE);
        Message* msg = robot.Write(new Message(MESSAGE_ROBOT_BATTERY_GET));
        rt_mutex_release(&mutex_robot);
        WriteInQueue(&q_messageToMon, msg);
    }
}

MessageID Tasks::OpenCamera()
{
    cout << "Called " << __PRETTY_FUNCTION__ << endl << flush;
    // Opening camera
    rt_mutex_acquire(&mutex_camera, TM_INFINITE);
    const bool isOpened = cam->Open();
    rt_mutex_release(&mutex_camera);

    // Changing status if succeeded
    if(isOpened)
    {
        rt_mutex_acquire(&mutex_cameraStatus, TM_INFINITE);
        cameraStatus = CameraStatusEnum::OPENED;
        rt_mutex_release(&mutex_cameraStatus);
    }
    return (isOpened ? MESSAGE_ANSWER_ACK : MESSAGE_ANSWER_NACK);
}

void Tasks::CloseCamera()
{
    cout << "\n\nCalled " << __PRETTY_FUNCTION__ << "\n\n" << flush;
    // Closing Camera
    rt_mutex_acquire(&mutex_camera, TM_INFINITE);
    cam->Close();
    rt_mutex_release(&mutex_camera);
    
    // Changing Status
    rt_mutex_acquire(&mutex_cameraStatus, TM_INFINITE);
    cameraStatus = CameraStatusEnum::CLOSED;
    rt_mutex_release(&mutex_cameraStatus);
}