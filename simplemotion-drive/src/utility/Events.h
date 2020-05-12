/**
 * Project: Simple MotionV2 based drive controller for Carla
 *
 * @file </src/utility/Events.h>
 *
 * @author Prajankya Sonar - <prajankya@gmail.com>
 *
 * MIT License
 * Copyright (c) 2020 Prajankya Sonar
 */

#ifndef __DRIVE_CONTROLLER_EVENTS__H__
#define __DRIVE_CONTROLLER_EVENTS__H__

#include <string>

/* Event types */
enum LOG_TYPE { INFO, WARNING, ERROR };

/* Event struct */
class Event {};

class LogEvent : public Event {
   public:
    LogEvent(LOG_TYPE logType, std::string message)
        : logType(logType), message(message){};
    LOG_TYPE logType;
    std::string message;
};

class ReadingEvent : public Event {
   public:
    ReadingEvent(
        long long int posSetpoint,
        long long int torqueSetpoint,
        long long int posFeedback)
        : posSetpoint(posSetpoint),
          torqueSetpoint(torqueSetpoint),
          posFeedback(posFeedback){};
    long long int posSetpoint;
    long long int torqueSetpoint;
    long long int posFeedback;
};

class ErrorDetectedEvent : public Event {
   public:
    ErrorDetectedEvent(bool trackingError, bool driveFault)
        : trackingError(trackingError), driveFault(driveFault){};
    bool trackingError;
    bool driveFault;
};

class ConnectedStateChangedEvent : public Event {
   public:
    ConnectedStateChangedEvent(bool isConnected) : isConnected(isConnected){};
    bool isConnected;
};

#endif