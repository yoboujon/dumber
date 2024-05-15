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

#ifndef __TASKS_H__
#define __TASKS_H__

#include <unistd.h>
#include <iostream>

#include <sys/mman.h>
#include <alchemy/task.h>
#include <alchemy/timer.h>
#include <alchemy/mutex.h>
#include <alchemy/sem.h>
#include <alchemy/queue.h>

#include "messages.h"
#include "commonitor.h"
#include "comrobot.h"
#include "camera.h"
#include "img.h"

using namespace std;

/**
 * @brief The monitor can change the shared data 'cameraStatus' to
 *  either 'OPENING' or 'CLOSING'. ManageCamera() will be awake afterwards.
 * Then the variable can switch state to open or close the camera.
 * Default value should be 'CLOSED'.
 */
enum class CameraStatusEnum {
    CLOSED,
    OPENING,
    OPENED,
    CLOSING
};

/**
 * @ref CameraStatusEnum "Same principle"
 * @brief The arena can change the shared data 'arenaStatus' to
 * either 'SEARCHING', 'CONFIRM' or 'INFIRM'. The other status will be
 * changed by the task ArenaChoice().
 */
enum class ArenaStatusEnum {
    NONE,
    SEARCHING,
    SEARCHED,
    CONFIRM,
    INFIRM
};

class Tasks {
public:
    /**
     * @brief Initializes main structures (semaphores, tasks, mutex, etc.)
     */
    void Init();

    /**
     * @brief Starts tasks
     */
    void Run();

    /**
     * @brief Stops tasks
     */
    void Stop();

    /**
     * @brief Suspends main thread
     */
    void Join();

private:
    /**********************************************************************/
    /* Shared data                                                        */
    /**********************************************************************/
    ComMonitor monitor;
    ComRobot robot;
    int robotStarted = 0;
    int move = MESSAGE_ROBOT_STOP;

    /************************************************************************/
    /* Added variables                                                      */
    /************************************************************************/
    
    // when gathering battery, set to true
    bool robotBatteryGet = false;
    // current status of the camera
    CameraStatusEnum cameraStatus = CameraStatusEnum::CLOSED;
    // camera object
    Camera* cam = new Camera(sm, 10);
    // current status of the arena
    ArenaStatusEnum arenaStatus = ArenaStatusEnum::NONE;
    // arena object (overlay of the arena on the img)
    Arena arena;
    // when gathering position, set to true
    bool positionEnabled = false;

    /**********************************************************************/
    /* Tasks                                                              */
    /**********************************************************************/
    RT_TASK th_server;
    RT_TASK th_sendToMon;
    RT_TASK th_receiveFromMon;
    RT_TASK th_openComRobot;
    RT_TASK th_startRobot;
    RT_TASK th_move;
    RT_TASK th_battery;
    RT_TASK th_cameraManage;
    RT_TASK th_cameraImage;
    RT_TASK th_arenaChoice;

    /**********************************************************************/
    /* Mutex                                                              */
    /**********************************************************************/
    RT_MUTEX mutex_monitor;
    RT_MUTEX mutex_robot;
    RT_MUTEX mutex_robotStarted;
    RT_MUTEX mutex_move;

    // Added mutexes

    // Locking the variable 'robotBatteryGet'
    RT_MUTEX mutex_batteryGet;
    // Locking the variable 'cameraStatus'
    RT_MUTEX mutex_cameraStatus;
    // Locking the variable 'cam'
    RT_MUTEX mutex_camera;
    // Locking the variable 'arena'
    RT_MUTEX mutex_arena;
    // Locking the variable 'arenaStatus'
    RT_MUTEX mutex_arenaStatus;
    // Locking the variable 'positionEnabled'
    RT_MUTEX mutex_positionEnabled;

    /**********************************************************************/
    /* Semaphores                                                         */
    /**********************************************************************/
    RT_SEM sem_barrier;
    RT_SEM sem_openComRobot;
    RT_SEM sem_serverOk;
    RT_SEM sem_startRobot;

    // Added semaphores

    // Used to call ManageCameraTask (non-periodic)
    RT_SEM sem_manageCamera;
    // Used to call ArenaChoiceTask (non-periodic)
    RT_SEM sem_arenaChoice;
    // Used to call FindPositionTask (non-periodic)
    RT_SEM sem_findPosition;

    /**********************************************************************/
    /* Message queues                                                     */
    /**********************************************************************/
    int MSG_QUEUE_SIZE;
    RT_QUEUE q_messageToMon;

    /**********************************************************************/
    /* Tasks' functions                                                   */
    /**********************************************************************/
    /**
     * @brief Thread handling server communication with the monitor.
     */
    void ServerTask(void *arg);

    /**
     * @brief Thread sending data to monitor.
     */
    void SendToMonTask(void *arg);

    /**
     * @brief Thread receiving data from monitor.
     */
    void ReceiveFromMonTask(void *arg);

    /**
     * @brief Thread opening communication with the robot.
     */
    void OpenComRobot(void *arg);

    /**
     * @brief Thread starting the communication with the robot.
     */
    void StartRobotTask(void *arg);

    /**
     * @brief Thread handling control of the robot.
     */
    void MoveTask(void *arg);


    /************************************************************************/
    /* Added Tasks                                                          */
    /************************************************************************/

    /**
     * @brief [periodic: 500ms]
     * Task managing the battery. When prompted will get the battery
     * from the robot and resend it to the monitor.
     */
    void BatteryStatusTask(void * arg);

    /**
     * @brief Task managing the camera opening and closing. Will check what
     * state the camera has and change depending on what the monitor asks.
     */
    void ManageCameraTask(void * arg);

    /**
     * @brief [periodic: 10ms]
     * Task Managing the image received by the camera. Will send each frame
     * if camera's state is opened.
     */
    void ImageCameraTask(void * arg);

    /**
     * @brief Task managing the Arena.
     * Will search for a camera and overlay if found.
     * Will store the arena and then show it afterwards.
     */
    void ArenaChoiceTask(void * arg);
    
    
    void FindPositionTask(void * arg);
    
    /************************************************************************/
    /* Added functions                                                      */
    /************************************************************************/
    
    /**
     * @brief Thread safe. Will open the camera and change the status if succeeded.
     * 
     * @return MessageID returns MESSAGE_ANSWER_ACK if successful, NACK if not.
     */
    MessageID OpenCamera();

    /**
     * @brief Thread safe. Will close the camera and change the status.
     */
    void CloseCamera();


    /**********************************************************************/
    /* Queue services                                                     */
    /**********************************************************************/
    /**
     * Write a message in a given queue
     * @param queue Queue identifier
     * @param msg Message to be stored
     */
    void WriteInQueue(RT_QUEUE *queue, Message *msg);

    /**
     * Read a message from a given queue, block if empty
     * @param queue Queue identifier
     * @return Message read
     */
    Message *ReadInQueue(RT_QUEUE *queue);



};


#endif // __TASKS_H__ 

