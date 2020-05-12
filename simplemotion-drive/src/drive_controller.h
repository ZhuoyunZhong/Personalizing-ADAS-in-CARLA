/**
 * Project: Simple MotionV2 based drive controller for Carla
 *
 * @file </src/drive_controller.h>
 *
 * @author Prajankya Sonar - <prajankya@gmail.com>
 *
 * MIT License
 * Copyright (c) 2020 Prajankya Sonar
 */

#ifndef _DRIVE_CONTROLLER_H_
#define _DRIVE_CONTROLLER_H_

#include <math.h>

#include <functional>
#include <iostream>
#include <mutex>
#include <queue>
#include <thread>
#include <type_traits>

#define LOGURU_WITH_STREAMS 1
#include <simplemotion.h>

#include <loguru.hpp>

#include "utility/CTimer.h"
#include "utility/Events.h"
#include "utility/StringUtils.h"

/**
 * @brief Driver Controller based on SimpleMotionV2
 *
 */
class DriveController {
   public:
    typedef long long int longint;

    /**
     * @brief Construct a new Drive Controller object
     *
     * @param str_portname COM Port name
     * @param un_TargetAddress Drive BUS address
     * @param b_UseHighBaudRate To use High baudrate
     */
    DriveController(
        const std::string str_portname,
        unsigned short un_TargetAddress,
        bool b_UseHighBaudRate);

    ~DriveController();

    void setEnabled(bool is);

    /**
     * @brief To connect and start running the controller
     *
     */

    void connect();

    /**
     * @brief
     *
     */
    void disconnect();

    /**
     * @brief set current position as Zero
     *
     */
    void setZero();

    /**
     * @brief
     *
     * Note: Change is not of type long long int as change should be small.
     * Smaller than "TrackingErrorLimit" to not let drive turn suddenly
     *
     * @param n_change
     * @param max_torque
     * @param Kp
     * @param Ki
     * @param Kd
     */
    void setIncrementSetpoint(
        int n_change, int n_max_torque, float f_Kp, float f_Ki, float f_Kd);

    /**
     * @brief Set the Absolute Setpoint
     *
     * @param n_pos
     * @param max_torque
     * @param Kp
     * @param Ki
     * @param Kd
     */
    void setAbsoluteSetpoint(
        longint n_pos, int n_max_torque, float f_Kp, float f_Ki, float f_Kd);

    /**
     * @brief
     * TODO: Resistance is not yet implemented
     * @param n_resistive_torque
     * @param un_Kp
     */
    void disableSetpointTracking(int n_resistive_torque = 0, float un_Kp = 0);

    /**
     * @brief
     *
     */
    void enableSetpointTracking();

    /**
     * @brief
     *
     */
    void clearTrackingError();

    /**
     * @brief
     *
     */
    void clearDriveErrors();

    /**
     * @brief Set the Tracking Error Limit
     *
     * @param n_tracking_error_limit
     */
    void setTrackingErrorLimit(longint n_tracking_error_limit);

    /**
     * @brief Set the Added Torque Constant
     *
     * @param n_added_constant_torque
     */
    void setAddedConstantTorque(unsigned int n_added_constant_torque);

    /* Event Callback variables for python binding */

    std::function<void(const LogEvent&)> LogCallback = nullptr;
    std::function<void(const ReadingEvent&)> ReadingCallback = nullptr;
    std::function<void(const ErrorDetectedEvent&)> ErrorCallback = nullptr;
    std::function<void(const ConnectedStateChangedEvent&)> ConnectedCallback =
        nullptr;

   protected:
    void Run();

   private:
    /**
     * @brief clip a number with max and min
     *
     */
    template <typename T>
    T ClipNumber(const T, const T, const T);

    /* Event Dispatcher */
    template <typename T>
    void dispatchEvent(const T&);

    /* Event Callbacks */
    void CallbackDispatcher(const LogEvent&);
    void CallbackDispatcher(const ReadingEvent&);
    void CallbackDispatcher(const ErrorDetectedEvent&);
    void CallbackDispatcher(const ConnectedStateChangedEvent&);

    /*  */
    void DoSetZero();
    void DoStopAndDisconnect();
    void DoConnectAndStart();

    /**
     * @brief this is the core function that does the actual control and uses
     * smFastUpdateCycle to transmit setpoint & motor feedback and drive
     * status/control bits
     *
     */
    void DoUpdateCycle();

    /**
     * @brief
     *
     * @param fast
     * @return true
     * @return false
     */
    bool checkAndReportSMBusErrors(bool fast = false);

    /**
     * @brief
     *
     * @param smStat
     * @param smDeviceErrors
     * @return std::string
     */
    std::string stringifySMBusErrors(SM_STATUS smStat, smint32 smDeviceErrors);

    /**
     * @brief Enum Class Task
     *
     */
    enum class ETask {
        None,
        ConnectAndStart,
        StopAndDisconnect,
        SetParams,
        ClearTrackingError,
        ClearDriveErrors,
        IncrementSetpoint,
        SetZero,
        Quit
    };

    // ------------------------------------------------------------

    // ============== PID loop variables =============
    /**
     * @brief Kp for the PID loop
     *
     */
    float m_fKp;

    /**
     * @brief Ki for the PID loop
     *
     */
    float m_fKi;

    /**
     * @brief Kd for the PID loop
     *
     */
    float m_fKd;

    /**
     * @brief Kd for the resistive torque
     *
     */
    float m_fResistiveKd;

    /**
     * @brief Previous loop iteration error, used to calculate delta-error
     *
     */
    double m_fPrevError;

    /**
     * @brief Cummulative position tracking error, used for integral control
     *
     */
    double m_fSumError;

    /**
     * @brief Timer used to calculate dt for PID loop
     *
     */
    CTimer m_cUpdateCycleTimer;

    /**
     * @brief Torque Limits used for control loop
     *
     */
    int m_unUserTorqueLimit;

    /**
     * @brief Torque Limits used for control loop, outside user's function
     *
     */
    int m_MaxTorqueLimit;

    /**
     * @brief Torque added constant to every torque output
     *
     */
    unsigned int m_AddedConstantTorque;

    /**
     * @brief Minimum Position diff below which diff is counted zero
     *
     */
    unsigned int m_minPosThreshold;

    /**
     * @brief is Setpoint-Tracking mode is enabled
     *
     */
    bool m_bIsSetpointTrackingEnabled;

    /**
     * @brief Resistive torque when not in setpoint tracking mode
     *
     */
    int m_nResistiveTorque;

    //  ================ Other variables ================

    /* Mutex to protect access to all setpoint access */
    std::mutex m_mutexSetpoint;

    /* Mutex to protect access to m_listTasks */
    std::mutex m_mutexTasks;

    /**
     * @brief List of tasks to do
     *
     */
    std::queue<ETask> m_tasksQueue;

    /**
     * @brief string: COM Port name
     *
     */
    std::string m_strPortName;

    /**
     * @brief bool: to Use High BaudRate Specific logic
     *
     */
    bool m_bUseHighBaudRate;

    /**
     * @brief bool: does drive have any tracking Error fault
     *
     */
    bool m_bTrackingErrorFault;

    /**
     * @brief bool: Is drive in running state
     *
     */
    bool m_bIsRunning;

    /**
     * @brief
     *
     */
    bool m_bClearDriveErrorsFlag;

    /**
     * @brief
     *
     */
    bool m_bPrevDriveFaultStopState;

    /**
     * @brief
     *
     */
    bool m_bPrevServoReadyState;

    /**
     * @brief unsigned short: BUS address of the drive(motor)
     *
     */
    unsigned short m_unTargetAddress;

    /**
     * @brief Maximum duration for maintaining Update frequency
     *
     */
    std::chrono::milliseconds m_cUpdateFrequencyMaxDuration;

    /**
     * @brief
     *
     */
    longint m_nTrackingErrorLimit;

    /**
     * @brief int: Setpoint for positional control
     *
     */
    longint m_nAbsolutePosSetpoint;

    /**
     * @brief
     *
     */
    longint m_nPositionFeedback;

    /**
     * @brief double: resolution of the encoder
     *
     */
    double m_fFeedbackDeviceResolution;

    /**
     * @brief smbus(long): Handle used for connecting to SimpleMotion Bus
     *
     */
    smbus m_cBusHandle;

    std::thread m_cControllerThread;
};

#endif