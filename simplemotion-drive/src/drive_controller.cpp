/**
 * Project: Simple MotionV2 based drive controller for Carla
 *
 * @file </src/drive_controller.cpp>
 *
 * @author Prajankya Sonar - <prajankya@gmail.com>
 *
 * MIT License
 * Copyright (c) 2020 Prajankya Sonar
 */

#include "drive_controller.h"

/************************************/
/************************************/

DriveController::DriveController(
    const std::string str_portname,
    unsigned short un_TargetAddress,
    bool b_UseHighBaudRate)
    : /* Initialize variables */
      m_bTrackingErrorFault(false),
      m_bIsRunning(false),
      m_bClearDriveErrorsFlag(false),
      m_bPrevDriveFaultStopState(false),
      m_bPrevServoReadyState(false),
      m_bIsSetpointTrackingEnabled(true),
      m_nTrackingErrorLimit(
          12000),  // Hardcoded limit, as it can become a safety hazard
      m_MaxTorqueLimit(
          5000),  // Hardcoded limit, as it can become a safety hazard
      m_unUserTorqueLimit(m_MaxTorqueLimit),
      m_minPosThreshold(10),  // +- error in position is counted as 0
      m_AddedConstantTorque(50),
      m_nAbsolutePosSetpoint(0),
      m_nResistiveTorque(0),
      m_fPrevError(0),
      m_fSumError(0),
      m_cBusHandle(-1),
      /* arguments */
      m_strPortName(str_portname),
      m_unTargetAddress(un_TargetAddress),
      m_bUseHighBaudRate(b_UseHighBaudRate) {
    /* Initializa LOGuru */
    loguru::g_preamble_date = false;
    loguru::g_preamble_time = false;

    /* Update frequency of 50 Hz */
    m_cUpdateFrequencyMaxDuration = std::chrono::milliseconds(1000 / 50);

    /* Start a thread with run function */
    m_cControllerThread = std::thread([&]() { this->Run(); });
}

/************************************/
/************************************/

DriveController::~DriveController() {
    /* To mitigate deadlock in join() below */
    {
        /* Lock Tasks List */
        std::lock_guard<std::mutex> guard(m_mutexTasks);

        /* Delete all pending actions */
        std::queue<ETask>().swap(m_tasksQueue);

        /* Just disconnect and quit thread */
        m_tasksQueue.push(ETask::Quit);
    }

    m_cControllerThread.join();
}

/************************************/
/************************************/

template <typename T>
void DriveController::dispatchEvent(const T& event) {
    CallbackDispatcher(event);
}

/************************************/
/************************************/

void DriveController::CallbackDispatcher(const LogEvent& event) {
    if (LogCallback != nullptr) {
        LogCallback(event);
    }
}

/************************************/
/************************************/

void DriveController::CallbackDispatcher(const ReadingEvent& event) {
    if (ReadingCallback != nullptr) {
        ReadingCallback(event);
    }
}

/************************************/
/************************************/

void DriveController::CallbackDispatcher(const ErrorDetectedEvent& event) {
    if (ErrorCallback != nullptr) {
        ErrorCallback(event);
    }
}

/************************************/
/************************************/

void DriveController::CallbackDispatcher(
    const ConnectedStateChangedEvent& event) {
    if (ConnectedCallback != nullptr) {
        ConnectedCallback(event);
    }
}

/************************************/
/************************************/

template <typename T>
T DriveController::ClipNumber(const T nNumber, const T nMin, const T nMax) {
    if (nMax < nMin) {
        /* if Max and min are swapped */
        return std::max(nMax, std::min(nNumber, nMin));
    } else {
        return std::max(nMin, std::min(nNumber, nMax));
    }
}

/************************************/
/************************************/

void DriveController::connect() {
    /* Lock Tasks List */
    std::lock_guard<std::mutex> guard(m_mutexTasks);

    /* Connect to tasks */
    m_tasksQueue.push(ETask::ConnectAndStart);
}

/************************************/
/************************************/

void DriveController::disconnect() {
    /* Lock Tasks List */
    std::lock_guard<std::mutex> guard(m_mutexTasks);

    m_tasksQueue.push(ETask::StopAndDisconnect);
}

/************************************/
/************************************/

void DriveController::setTrackingErrorLimit(longint n_tracking_error_limit) {
    /* Lock Tasks List */
    std::lock_guard<std::mutex> guard(m_mutexTasks);

    this->m_nTrackingErrorLimit = n_tracking_error_limit;
    m_tasksQueue.push(ETask::SetParams);
}

/************************************/
/************************************/

void DriveController::setAddedConstantTorque(
    unsigned int n_added_constant_torque) {
    /* Lock Tasks List */
    std::lock_guard<std::mutex> guard(m_mutexTasks);

    this->m_AddedConstantTorque = n_added_constant_torque;
    m_tasksQueue.push(ETask::SetParams);
}

/************************************/
/************************************/

void DriveController::enableSetpointTracking() {
    /* Lock Tasks List */
    std::lock_guard<std::mutex> guard(m_mutexTasks);

    this->m_bIsSetpointTrackingEnabled = true;

    /* Reset feedback values */
    m_nAbsolutePosSetpoint = m_nPositionFeedback;
    m_fPrevError = 0;
    m_fSumError = 0;

    m_tasksQueue.push(ETask::SetParams);
}

/************************************/
/************************************/

void DriveController::disableSetpointTracking(
    int n_resistive_torque, float un_Kp) {
    /* Lock Tasks List */
    std::lock_guard<std::mutex> guard(m_mutexTasks);

    this->m_nResistiveTorque = n_resistive_torque;
    this->m_bIsSetpointTrackingEnabled = false;
    this->m_fResistiveKd = un_Kp;
    m_tasksQueue.push(ETask::SetParams);
}

/************************************/
/************************************/

void DriveController::clearTrackingError() {
    /* Lock Tasks List */
    std::lock_guard<std::mutex> guard(m_mutexTasks);

    m_tasksQueue.push(ETask::ClearTrackingError);
}

/************************************/
/************************************/

void DriveController::clearDriveErrors() {
    /* Lock Tasks List */
    std::lock_guard<std::mutex> guard(m_mutexTasks);

    /* Connect to tasks */
    m_tasksQueue.push(ETask::ClearDriveErrors);
}

/************************************/
/************************************/

void DriveController::setZero() {
    /* Lock Tasks List */
    std::lock_guard<std::mutex> guard(m_mutexTasks);

    /* Connect to tasks */
    m_tasksQueue.push(ETask::SetZero);
}

/************************************/
/************************************/

void DriveController::setIncrementSetpoint(
    int n_change, int n_max_torque, float f_Kp, float f_Ki, float f_Kd) {
    /* Lock Tasks List */
    std::lock_guard<std::mutex> guard(m_mutexSetpoint);

    m_nAbsolutePosSetpoint += n_change;

    m_unUserTorqueLimit = n_max_torque;

    /* We cannot control the output of negative gain  */
    m_fKp = abs(f_Kp);

    m_fKi = abs(f_Ki);

    m_fKd = abs(f_Kd);
}

/************************************/
/************************************/

void DriveController::setAbsoluteSetpoint(
    longint n_pos, int n_max_torque, float f_Kp, float f_Ki, float f_Kd) {
    /* Lock Tasks List */
    std::lock_guard<std::mutex> guard(m_mutexSetpoint);

    m_nAbsolutePosSetpoint = n_pos;
    m_unUserTorqueLimit = n_max_torque;

    /* We cannot control the output of negative gain  */
    m_fKp = abs(f_Kp);

    m_fKi = abs(f_Ki);

    m_fKd = abs(f_Kd);
}

/************************************/
/************************************/

void DriveController::Run() {
    ETask eCurrentTask;
    CTimer cUpdateTimer;

    loguru::set_thread_name("Controller thread");

    LOG_S(INFO) << "Started Drive Controller thread";

    while (true) {
        {  // Mutex scope
            /* Lock Tasks List */
            std::lock_guard<std::mutex> guard(m_mutexTasks);

            if (m_tasksQueue.empty()) {  // Do nothing
                eCurrentTask = ETask::None;
            } else {
                eCurrentTask = m_tasksQueue.front();
                m_tasksQueue.pop();
            }
        }
        /* execute task */
        switch (eCurrentTask) {
            case ETask::ConnectAndStart:
                DoConnectAndStart();
                break;
            case ETask::StopAndDisconnect:
                DoStopAndDisconnect();
                break;
            case ETask::ClearDriveErrors:
                m_bClearDriveErrorsFlag = true;
                break;
            case ETask::ClearTrackingError:
                m_bTrackingErrorFault = false;
                m_nAbsolutePosSetpoint = m_nPositionFeedback;

                dispatchEvent(LogEvent(
                    LOG_TYPE::INFO,
                    "Tracking error cleared, position target have "
                    "been set as current feedback position and "
                    "torque setpoint has been released"));
                break;
            case ETask::Quit:
                DoStopAndDisconnect();
                LOG_S(WARNING) << "Exiting Controller Thread safely";
                return; /* break the while loop and Do not go to sleep below
                         */
                break;
            case ETask::SetZero:
                DoSetZero();
                break;
            case ETask::IncrementSetpoint:
            case ETask::SetParams:
            case ETask::None:
            default:
                break;
        }

        /* do servo control */
        if (m_bIsRunning) {
            DoUpdateCycle();
        }

        /* stop the timer to get total time spent */
        cUpdateTimer.Stop();

        /* If the elapsed time is lower than the tick length, wait */
        if (cUpdateTimer.Elapsed() < m_cUpdateFrequencyMaxDuration) {
            /* Sleep for the difference duration */
            std::this_thread::sleep_for(
                m_cUpdateFrequencyMaxDuration - cUpdateTimer.Elapsed());
        } else {
            LOG_S(WARNING) << "Update tick took " << cUpdateTimer
                           << " milli-secs, more than the expected "
                           << m_cUpdateFrequencyMaxDuration.count()
                           << " milli-secs. ";
            // break;  // Stop the timer
        }

        /* Restart Timer */
        cUpdateTimer.Start();
    }
}

/************************************/
/************************************/

void DriveController::DoUpdateCycle() {
    FastUpdateCycleReadData sReadData;
    FastUpdateCycleWriteData sWriteData;
    int nTorqueSetpoint;

    /* this app uses fast update cycle format 1 (ALT1):
     *  description: this type has 28 bits absolute setpoint and 30 bits
     * absolute feedback value + 4 output bits for control + 2 input bits for
     * status
     */

    /* calculate new torque setpoint by using proportional error amplifier
     * from position tracking error */

    int posTrackingError = m_nAbsolutePosSetpoint - m_nPositionFeedback;

    /* Reset tracking error per revolution */
    // posTrackingError = int(posTrackingError) %
    // int(m_fFeedbackDeviceResolution);

    /* If less then threshold, consider error to be zero */
    if (abs(posTrackingError) < m_minPosThreshold) {
        posTrackingError = 0;
    }

    double delta_error = posTrackingError - m_fPrevError;

    /* Limit huge position diff */
    if (abs(posTrackingError) > m_nTrackingErrorLimit &&
        m_bIsSetpointTrackingEnabled) {
        if (m_bTrackingErrorFault == false) {
            LOG_S(ERROR)
                << "Application entered in position tracking error state "
                << "(position tracking error was " << posTrackingError
                << "). This means that torque setpoint has been forced to 0. "
                << "Try clearing faults to resume.";
            dispatchEvent(LogEvent(
                LOG_TYPE::ERROR,
                "Application entered in position tracking error state "
                "(position tracking error was " +
                    std::to_string(posTrackingError) +
                    "). This means that torque setpoint has been forced to "
                    "0. "
                    "Try clearing faults to resume."));

            dispatchEvent(ErrorDetectedEvent(true, false));
        }

        m_bTrackingErrorFault = true;
    }

    /* Calc Torque */
    if (m_bTrackingErrorFault) {
        nTorqueSetpoint = 0;
    } else {
        m_cUpdateCycleTimer.Stop();

        // millisecond to second
        float dt = (0.001 * m_cUpdateCycleTimer.Elapsed().count());

        if (dt > 1) {
            /* If Dt is bigger than 1 second, something is wrong */
            /* maybe its first iteration of loop, where dt >> 1 */
            dt = 1;
        }

        if (dt < 0) {
            /* something is crazy wrong !!! */
            /* dt can never be negative, except if user has changed system
             * clock backwards */
            dt = 0.001;  // 1 millisecond
        }

        /* Calculating Torque based on PID equation */

        /* Proportional component */
        nTorqueSetpoint = round(m_fKp * posTrackingError);

        /* Integral component */
        nTorqueSetpoint += round(m_fKi * m_fSumError * dt);

        /*  Differential component */
        if (dt != 0) {
            /* to skip "divided-by-zero" */
            nTorqueSetpoint += round((m_fKd * delta_error / dt));
        }

        //
        /* Update variables for next loop iteration */
        m_fPrevError = posTrackingError;
        m_fSumError += posTrackingError;
        m_cUpdateCycleTimer.Start();
    }

    /* If Torque is not zero, add static constant */
    if (nTorqueSetpoint != 0) {
        nTorqueSetpoint = ((posTrackingError > 0) ? m_AddedConstantTorque
                                                  : -m_AddedConstantTorque) +
                          nTorqueSetpoint;
    }

    /* Limit the max torque given by user function */
    nTorqueSetpoint =
        ClipNumber(nTorqueSetpoint, -m_unUserTorqueLimit, m_unUserTorqueLimit);

    /* Enable / disable Setpoint tracking mode */
    if (!m_bIsSetpointTrackingEnabled) {
        /* If disabled,apply resistance in the direction of user turns */

        /* If delta error is big enough */
        if (abs(delta_error) > m_minPosThreshold) {
            /* Constant resistive torque gain */
            nTorqueSetpoint = m_nResistiveTorque * m_fResistiveKd * delta_error;
        } else {
            nTorqueSetpoint = 0;
        }
    }

    /* Limit the max torque with hardcoded limits */
    nTorqueSetpoint =
        ClipNumber(nTorqueSetpoint, -m_MaxTorqueLimit, m_MaxTorqueLimit);

    LOG_S(INFO) << "Torque:" << nTorqueSetpoint << ":" << m_nPositionFeedback
                << ":" << m_nAbsolutePosSetpoint << ":" << delta_error;

    // ===================== Building write data ======================

    /* write setpoint */
    sWriteData.ALT1_Write.Setpoint = nTorqueSetpoint;

    if (m_bClearDriveErrorsFlag) {
        // write clearfaults flag with fast command
        sWriteData.ALT1_Write.CB1_ClearFaults = 1;
        m_bClearDriveErrorsFlag = false;
        LOG_S(INFO) << "Drive errors cleared";

        dispatchEvent(LogEvent(LOG_TYPE::INFO, "Drive errors cleared"));
    } else {
        sWriteData.ALT1_Write.CB1_ClearFaults = 0;
    }

    /* write enable with fast command. without this, drive gets disabled */
    sWriteData.ALT1_Write.CB1_Enable = 1;

    /* write bypass trajectory planner with fast command */
    sWriteData.ALT1_Write.CB1_BypassTrajPlanner = 1;

    /* do not activate quick stop */
    sWriteData.ALT1_Write.CB1_QuickStopSet = 0;

    // ========= send the fast update cycle to drive & check errors ===========
    if (smFastUpdateCycleWithStructs(
            m_cBusHandle, m_unTargetAddress, sWriteData, &sReadData) != SM_OK) {
        // for clearer debug:
        smCloseBus(m_cBusHandle);
        m_bIsRunning = false;

        // report & handle error
        checkAndReportSMBusErrors();
        disconnect();
        LOG_S(ERROR) << "Aborted due to connection error.";

        dispatchEvent(
            LogEvent(LOG_TYPE::ERROR, "Aborted due to connection error."));
    }

    /* extract data from fastUpdateCycle command */
    m_nPositionFeedback = sReadData.ALT1_ALT2_Read.PositionFeedback;

    /* get fault stop state of drive */
    bool faultstop = sReadData.ALT1_ALT2_Read.Stat_FaultStop;

    /* get servo ready state of drive */
    bool servoready = sReadData.ALT1_ALT2_Read.Stat_ServoReady;

    /* drive just faulted, report it once */
    if (faultstop == true && m_bPrevDriveFaultStopState == false) {
        LOG_S(ERROR) << "Drive entered in fault state."
                     << " Try clearing faults to resume.";
        dispatchEvent(LogEvent(
            LOG_TYPE::ERROR,
            "Drive entered in fault state."
            " Try clearing faults to resume."));
        dispatchEvent(ErrorDetectedEvent(false, faultstop));
    }
    m_bPrevDriveFaultStopState = faultstop;

    /*  warn about servo ready==false but better handling of it is not
     * implemented here */
    if (servoready == false && m_bPrevServoReadyState == true) {
        LOG_S(ERROR) << "Drive 'Ready for use' flag is became false (drive not "
                        "enabled?). Control will not work until it's true.";
        dispatchEvent(LogEvent(
            LOG_TYPE::ERROR,
            "Drive 'Ready for use' flag is became false (drive not "
            "enabled?). Control will not work until it's true."));
    }

    m_bPrevServoReadyState = servoready;

    dispatchEvent(ReadingEvent(
        m_nAbsolutePosSetpoint, nTorqueSetpoint, m_nPositionFeedback));
}

/************************************/
/************************************/

/* if fast=true then do only checks that do not need communication via SM bus
(local checks only, such as errors in received packets, but not reporting
errors in invalid parameter values) */
bool DriveController::checkAndReportSMBusErrors(bool fast) {
    std::string strErrors;

    /**
     * @brief SM bus & SM devices have three categories of status & error bits:

     1) Bus status & error bits. These are returned on each SM library call (the
     SM_STAUTS type) and accumulated into a internal variable that may be read
     by getCumulativeStatus function. This value reports errors that happen on
     with communication layer (phyiscal device problems such as not available of
     bus device or checksum error).

     2) Device side SM status & error bits. reading this requires working
     connection to a target device in order to read SMP_CUMULATIVE_STATUS
     parameter. This value contains errors that successfully were transferred to
     target but were not accepted by some reason (i.e. if invalid paramter
     address or value was used).

     3) Device specific state & errors, such as servo drive stauts and fault
     bits on SMP_STATUS and SMP_FAULTS parameters. These states are not checked
     in this function.
     *
     */

    // read SMP_CUMULATIVE_STATUS
    smint32 SMDeviceSideCommStatus;
    if (fast == false) {
        smRead1Parameter(
            m_cBusHandle,
            m_unTargetAddress,
            SMP_CUMULATIVE_STATUS,
            &SMDeviceSideCommStatus);
        // if we have some error bits on, reset them, so we can spot new errors
        // later
        if (SMDeviceSideCommStatus != 0) {
            smSetParameter(
                m_cBusHandle, m_unTargetAddress, SMP_CUMULATIVE_STATUS, 0);
        }
    } else {
        SMDeviceSideCommStatus = 0;  // a cludge to avoid false errors being
                                     // reported in stringifySMBusErrors
    }

    // read cumulative bus status errors and all convert (1) and (2) bits to
    // human readable form:
    strErrors = stringifySMBusErrors(
        getCumulativeStatus(m_cBusHandle), SMDeviceSideCommStatus);

    // reset local errors bits
    resetCumulativeStatus(m_cBusHandle);

    // if there were errors, log them
    if (!strErrors.empty()) {
        LOG_S(ERROR) << strErrors;
        dispatchEvent(LogEvent(LOG_TYPE::ERROR, strErrors));
        return true;
    }

    return false;
}

/************************************/
/************************************/

std::string DriveController::stringifySMBusErrors(
    SM_STATUS smStat, smint32 smDeviceErrors) {
    std::string errorString;

    if (((smStat != SM_OK && smStat != SM_NONE) ||
         smDeviceErrors != SMP_CMD_STATUS_ACK)) {
        std::string errorFlags, smErrorFlags;

        // these faults are from SM bus host side
        if (smStat & SM_ERR_NODEVICE) {
            errorFlags += "* NoDevice (bus is not opened) \n";
        }

        if (smStat & SM_ERR_PARAMETER) {
            errorFlags +=
                "* InvalidParameter (invalid access to a target device "
                "parameter, i.e. read/write unsupported parameter address, or "
                "writing value that is not allowed to a parameter)\n";
        }

        if (smStat & SM_ERR_COMMUNICATION) {
            errorFlags += "* Communication (received data cheksum mismatch)\n";
        }

        if (smStat & SM_ERR_LENGTH) {
            errorFlags +=
                "* DataLegth (not enough return data received from device)\n";
        }

        if (smStat & SM_ERR_BUS) {
            errorFlags += "* BusError (communication port device error)\n";
        }

        /* ignore device side faults if nodevice is active because it would make
         * no sense */
        if (!(smStat & SM_ERR_NODEVICE)) {
            /* device errors are read from the device (so connection must be
            working). these are error flags of device side of SM bus */
            if (smDeviceErrors & SMP_CMD_STATUS_NACK) {
                smErrorFlags += "* Command fail (NACK)\n";
            }

            if (smDeviceErrors & SMP_CMD_STATUS_INVALID_ADDR) {
                smErrorFlags += "* Invalid param address\n";
            }

            if (smDeviceErrors & SMP_CMD_STATUS_INVALID_VALUE) {
                smErrorFlags += "* Invalid param value\n";
            }

            if (smDeviceErrors & SMP_CMD_STATUS_VALUE_TOO_HIGH) {
                smErrorFlags += "* Value too high\n";
            }

            if (smDeviceErrors & SMP_CMD_STATUS_VALUE_TOO_LOW) {
                smErrorFlags += "* Value too low\n";
            }
        }
        errorString = "";
        if (errorFlags.size()) {
            errorString += "Bus error flags: \n" + errorFlags + "\n";
        } else {
            errorString = "Communication error.";
        }

        if (smErrorFlags.size()) {
            errorString += "\nDevice errors: \n" + smErrorFlags;
        }
    }

    return errorString;
}

/************************************/
/************************************/

void DriveController::DoSetZero() {
    smSetParameter(
        m_cBusHandle,
        m_unTargetAddress,
        SMP_SYSTEM_CONTROL_RESET_FB_AND_SETPOINT,
        1);
    LOG_S(INFO) << "Set Zero";

    dispatchEvent(LogEvent(LOG_TYPE::INFO, "Set Zero done"));
}

/************************************/
/************************************/

void DriveController::DoStopAndDisconnect() {
    if (m_bIsRunning) {
        // stop motion
        smSetParameter(
            m_cBusHandle, m_unTargetAddress, SMP_ABSOLUTE_SETPOINT, 0);
        smCloseBus(m_cBusHandle);
        m_bIsRunning = false;
        LOG_S(INFO) << "Stopped";

        dispatchEvent(LogEvent(LOG_TYPE::INFO, "Stopped"));
        dispatchEvent(ConnectedStateChangedEvent(false));
    }
}

/************************************/
/************************************/

void DriveController::DoConnectAndStart() {
    if (m_bIsRunning == false) {
        // enable low amount of debug output to report SM bus errors to stderr
        smSetDebugOutput(SMDebugMid, stderr);

        /* set SM default baudrate which is needed to reconnect after increased
         * baudrate (so needed for consequent connect if "use high baudrate"
         * option was set) */
        smSetTimeout(1000);
        smSetBaudrate(460800);

        m_cBusHandle = smOpenBus(utf2Latin1(m_strPortName).c_str());
        if (m_cBusHandle < 0) {
            LOG_S(ERROR) << "Opening SM bus failed";
            dispatchEvent(LogEvent(LOG_TYPE::ERROR, "Opening SM bus failed"));
            return;
        }

        /* reset possible SM errors */
        resetCumulativeStatus(m_cBusHandle);

        // read some variables from device
        smint32 controlMode, deviceType, actualPosition, driveStatus,
            capabilities1, FWversion, smProtocolVersion, encoderResolutionPPR,
            maxBPS;
        smRead3Parameters(
            m_cBusHandle,
            m_unTargetAddress,
            SMP_SM_VERSION,
            &smProtocolVersion,
            SMP_DEVICE_TYPE,
            &deviceType,
            SMP_BUS_SPEED | SMP_MAX_VALUE_MASK,
            &maxBPS);

        // check if above reads failed in any way
        if (checkAndReportSMBusErrors()) {
            LOG_S(ERROR) << "Start aborted.";
            dispatchEvent(LogEvent(LOG_TYPE::ERROR, "Start aborted"));
            smCloseBus(m_cBusHandle);
            return;
        }

        // detect ARGON drive
        if ((deviceType / 1000) == 4) {
            dispatchEvent(LogEvent(
                LOG_TYPE::ERROR,
                "ARGON drive detected, it might not be supported in "
                "this application at the moment. However, if you get "
                "'Drive compatibility checked & passed' message in the "
                "log, then it's going to work."));
        }

        // required SM protocol version is 28 or greater to guarantee next
        // commands to run sucessfully (especially SMP_DEVICE_CAPABILITIES1)
        if (smProtocolVersion < 28) {
            dispatchEvent(LogEvent(
                LOG_TYPE::ERROR,
                "Target drive connected, but it has too old "
                "SimpleMotion protocol support (version " +
                    std::to_string(smProtocolVersion) +
                    "). Try upgrading drive firmware."));
            dispatchEvent(LogEvent(LOG_TYPE::ERROR, "Start aborted."));
            smCloseBus(m_cBusHandle);
            return;
        }

        smRead3Parameters(
            m_cBusHandle,
            m_unTargetAddress,
            SMP_CONTROL_MODE,
            &controlMode,
            SMP_ACTUAL_POSITION_FB_NEVER_RESETTING,
            &actualPosition,
            SMP_ENCODER_PPR,
            &encoderResolutionPPR);
        smRead3Parameters(
            m_cBusHandle,
            m_unTargetAddress,
            SMP_STATUS,
            &driveStatus,
            SMP_DEVICE_CAPABILITIES1,
            &capabilities1,
            SMP_FIRMWARE_VERSION,
            &FWversion);

        // check if above reads failed in any way
        if (checkAndReportSMBusErrors()) {
            dispatchEvent(LogEvent(LOG_TYPE::ERROR, "Start aborted."));
            smCloseBus(m_cBusHandle);
            return;
        }

        m_nPositionFeedback = m_nAbsolutePosSetpoint = actualPosition;

        m_fFeedbackDeviceResolution = encoderResolutionPPR * 4;

        // stringify control mode
        std::string CM = "Unknown";
        if (controlMode == CM_NONE)
            CM = "None";
        else if (controlMode == CM_POSITION)
            CM = "Position";
        else if (controlMode == CM_VELOCITY)
            CM = "Velocity";
        else if (controlMode == CM_TORQUE)
            CM = "Torque";

        LOG_S(INFO) << "Connected to device with type ID " << deviceType
                    << " with firmware version " << FWversion
                    << ", which is in " << CM
                    << " control mode and has initial position "
                    << "feedback at " << actualPosition
                    << " encoder counts, and encoder resolution is "
                    << m_fFeedbackDeviceResolution
                    << ". Drive's max supported bus baud rate is " << maxBPS
                    << " BPS.";
        dispatchEvent(LogEvent(
            LOG_TYPE::INFO,
            "Connected to device with type ID " + std::to_string(deviceType) +
                " with firmware version " + std::to_string(FWversion) +
                ", which is in " + CM +
                " control mode and has initial position " + "feedback at " +
                std::to_string(actualPosition) +
                " encoder counts, and encoder resolution is " +
                std::to_string(m_fFeedbackDeviceResolution) +
                ". Drive's max supported bus baud rate is " +
                std::to_string(maxBPS) + " BPS."));

        // check drive state & configuration:
        bool abort = false;

        // check if drive is in right mode
        if (controlMode != CM_TORQUE) {
            LOG_S(ERROR)
                << "This application works only if drive is in Torque control"
                   "mode, which it's not. To use this app, configure drive to"
                   "be in Torque control mode with Granity.";
            dispatchEvent(LogEvent(
                LOG_TYPE::ERROR,
                "This demo app works only if drive is in Torque control "
                "mode, which it's not. To use this app, configure drive to "
                "be in Torque control mode with Granity."));

            abort = true;
        }

        if (!(driveStatus & STAT_SERVO_READY)) {
            LOG_S(ERROR) << "Drive does not seem to be ready for control "
                            "(status register bit 'Ready for use' is False)."
                            " Configure & enable drive with Granity before"
                            " using this app.";

            dispatchEvent(LogEvent(
                LOG_TYPE::ERROR,
                "Drive does not seem to be ready for control "
                "(status register bit 'Ready for use' is False)."
                " Configure & enable drive with Granity before"
                " using this app."));
            abort = true;
        }

        if (!(driveStatus & STAT_SERVO_READY)) {
            LOG_S(ERROR) << "Drive does not seem to be ready for control "
                            "(status register "
                            "bit 'Ready for use' is False). Configure & enable "
                            "drive with "
                            "Granity before using this app.";
            dispatchEvent(LogEvent(
                LOG_TYPE::ERROR,
                "Drive does not seem to be ready for control "
                "(status register "
                "bit 'Ready for use' is False). Configure & enable "
                "drive with "
                "Granity before using this app."));
            abort = true;
        }

        if (!(capabilities1 &
              DEVICE_CAPABILITY1_SELECTABLE_FAST_UPDATE_CYCLE_FORMAT)) {
            LOG_S(ERROR)
                << "Connected drive firmware does not support required "
                   "DEVICE_CAPABILITY1_SELECTABLE_FAST_UPDATE_CYCLE_FORMAT. "
                   "Try drive upgrading firmware to latest version.";
            dispatchEvent(LogEvent(
                LOG_TYPE::ERROR,
                "Connected drive firmware does not support required "
                "DEVICE_CAPABILITY1_SELECTABLE_FAST_UPDATE_CYCLE_FORMAT. "
                "Try drive upgrading firmware to latest version."));
            abort = true;
        }

        if (!(capabilities1 & DEVICE_CAPABILITY1_CONTROL_BITS1_VERSION2)) {
            LOG_S(ERROR)
                << "Connected drive firmware does not support required "
                   "DEVICE_CAPABILITY1_CONTROL_BITS1_VERSION2. Try drive "
                   "upgrading firmware to latest version.";
            dispatchEvent(LogEvent(
                LOG_TYPE::ERROR,
                "Connected drive firmware does not support required "
                "DEVICE_CAPABILITY1_CONTROL_BITS1_VERSION2. Try drive "
                "upgrading firmware to latest version."));
            abort = true;
        }

        // if some error above occurred
        if (abort) {
            LOG_S(ERROR) << "Start aborted.";

            dispatchEvent(LogEvent(LOG_TYPE::ERROR, "Start aborted."));

            smCloseBus(m_cBusHandle);
            return;
        }

        // change bitrate
        if (m_bUseHighBaudRate) {
            int setBPS;

            if (maxBPS > 3000000) {
                /* limit BPS to 3M because the FTDI USB UART chip has that as
                 * maximum supported bitrate. */
                setBPS = 3000000;
            } else
                setBPS = maxBPS;

            LOG_S(INFO) << "Setting baudrate to " << setBPS << " BPS.";

            dispatchEvent(LogEvent(
                LOG_TYPE::ERROR,
                "Setting baudrate to " + std::to_string(setBPS) + " BPS."));

            /*max deviceTimeoutMs valid value 10230 ms. however, this should be
             *set _less_ than timeout period of SM host the value that we set
             *earlier here with smSetTimeout) so if device host timeouts, it
             *will cause certain timeout on device and reset baudrate to default
             *for successfull reinitialization*/
            const int deviceTimeoutMs = 500;

            // first set device timeout (watchdog/fault behavior), so if
            // connection is lost, they reset to default baud rate after a
            // certain time period note: we change these settings of all bus
            // devices simultaneously because errors will happen if not all
            // devices have same BPS (address 0=broadcast to all)
            smSetParameter(
                m_cBusHandle,
                0,
                SMP_FAULT_BEHAVIOR,
                (deviceTimeoutMs / 10) << 8);  // set timeout
            smSetParameter(m_cBusHandle, 0, SMP_BUS_SPEED, setBPS);  // set
                                                                     // baudrate

            // if all went ok, now device is in new baud rate, switch host PBS
            // too
            smCloseBus(m_cBusHandle);
            smSetBaudrate(setBPS);
            m_cBusHandle = smOpenBus(utf2Latin1(m_strPortName).c_str());
            if (m_cBusHandle < 0) {
                LOG_S(ERROR)
                    << "Opening SM bus failed with high baud rate ( " << maxBPS
                    << " BPS), perhaps bus device doesn't support high BPS";
                LOG_S(ERROR) << "Start aborted.";

                dispatchEvent(LogEvent(
                    LOG_TYPE::ERROR,
                    "Opening SM bus failed with high baud rate ( " +
                        std::to_string(maxBPS) +
                        " BPS), perhaps bus device doesn't support "
                        "high BPS"));

                dispatchEvent(LogEvent(LOG_TYPE::ERROR, "Start aborted."));
                return;
            }

            // test that new speed works
            resetCumulativeStatus(m_cBusHandle);  // reset possible SM errors
            smint32 devType2;
            smRead1Parameter(
                m_cBusHandle,
                m_unTargetAddress,
                SMP_DEVICE_TYPE,
                &devType2);  // just reading some parameter to test

            // check if above SM commands failed in any way
            if (checkAndReportSMBusErrors() ||
                devType2 !=
                    deviceType)  // we also compare that value is same than
                                 // before but it's quite unnecessary as
                                 // automatic CRC check will catch any data
                                 // error that might happen
            {
                LOG_S(ERROR)
                    << "Device didn't respond corrently at new baudrate, "
                       "perhaps RS485 bus termination is missing and causing "
                       "errors at high speed?";

                dispatchEvent(LogEvent(
                    LOG_TYPE::ERROR,
                    "Device didn't respond corrently at new baudrate, "
                    "perhaps RS485 bus termination is missing and causing "
                    "errors at high speed?"));

                smCloseBus(m_cBusHandle);
                LOG_S(ERROR) << "Start aborted.";

                dispatchEvent(LogEvent(LOG_TYPE::ERROR, "Start aborted."));
                return;
            }
        }

        // change smFastUpdateCycle data format
        smSetParameter(
            m_cBusHandle,
            m_unTargetAddress,
            SMP_FAST_UPDATE_CYCLE_FORMAT,
            FAST_UPDATE_CYCLE_FORMAT_ALT1);

        // check if above SM commands failed in any way
        if (checkAndReportSMBusErrors()) {
            LOG_S(ERROR) << "Start aborted.";
            dispatchEvent(LogEvent(LOG_TYPE::ERROR, "Start aborted."));
            smCloseBus(m_cBusHandle);
            return;
        }

        dispatchEvent(ConnectedStateChangedEvent(true));

        m_bIsRunning = true;
        m_bTrackingErrorFault = false;
        LOG_S(WARNING)
            << "Drive compatibility checked & passed. Control started.";
    }
}