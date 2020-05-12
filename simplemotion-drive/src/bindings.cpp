/**
 * Project: Simple MotionV2 based drive controller for Carla
 *
 * @file </src/bindings.cpp>
 *
 * @author Prajankya Sonar - <prajankya@gmail.com>
 *
 * MIT License
 * Copyright (c) 2020 Prajankya Sonar
 */

#include <pybind11/functional.h>
#include <pybind11/pybind11.h>

#define LOGURU_WITH_STREAMS 1
#include <loguru.cpp>  // yes, its cpp file

#include "drive_controller.h"
#include "utility/Events.h"

namespace py = pybind11;

PYBIND11_MODULE(smv2_drive, m) {
    m.doc() = R"pbdoc(
        A Simple Motion v2 drive binding for python for steering torque control
        -----------------------
        .. currentmodule:: smv2_drive
        .. autosummary::
           :toctree: _generate
           add
           subtract
    )pbdoc";

    py::class_<DriveController> controller(m, "DriveController");

    /* Constructor */
    controller.def(py::init<const std::string &, unsigned short, bool>());

    /* Drive connection functions */
    controller.def(
        "connect",
        &DriveController::connect,
        "To connect to drive and start running the controller");

    controller.def(
        "disconnect",
        &DriveController::disconnect,
        "To disconnect from drive, but keep the controller running");

    /* Control functions */

    controller.def(
        "setIncrementSetpoint",
        &DriveController::setIncrementSetpoint,
        "Increment Current Setpoint of the drive");

    controller.def(
        "setAbsoluteSetpoint",
        &DriveController::setAbsoluteSetpoint,
        "Move drive to an absolute setpoint");

    controller.def(
        "enableSetpointTracking",
        &DriveController::enableSetpointTracking,
        "Enable PID loop");

    controller.def(
        "disableSetpointTracking",
        &DriveController::disableSetpointTracking,
        "Disable PID Loop and apply some resistance to any motion by the user");

    /* Error clear functions */

    controller.def(
        "clearTrackingError",
        &DriveController::clearTrackingError,
        "Clear tracking Error");

    controller.def(
        "clearDriveErrors",
        &DriveController::clearDriveErrors,
        "Clear Drive Errors");

    controller.def(
        "setZero", &DriveController::setZero, "set current position as Zero");

    /* Parameters functions */

    controller.def(
        "setTrackingErrorLimit",
        &DriveController::setTrackingErrorLimit,
        "Update Tracking Error limit");

    controller.def(
        "setAddedConstantTorque",
        &DriveController::setAddedConstantTorque,
        "Added Constant torque to end variable");

    /* Events logic */
    /* ****************************************************** */

    controller.def_readwrite("logCallback", &DriveController::LogCallback);
    controller.def_readwrite("errorCallback", &DriveController::ErrorCallback);
    controller.def_readwrite(
        "readingCallback", &DriveController::ReadingCallback);
    controller.def_readwrite(
        "connectedCallback", &DriveController::ConnectedCallback);

    /* ****************************************************** */
    py::enum_<LOG_TYPE>(m, "LOG_TYPE")
        .value("INFO", LOG_TYPE::INFO)
        .value("ERROR", LOG_TYPE::ERROR)
        .value("WARNING", LOG_TYPE::WARNING)
        .export_values();

    py::class_<LogEvent>(m, "LogEvent")
        .def_readonly("logType", &LogEvent::logType)
        .def_readonly("message", &LogEvent::message);

    /* ****************************************************** */
    py::class_<ErrorDetectedEvent>(m, "ErrorDetectedEvent")
        .def_readonly("driveFault", &ErrorDetectedEvent::driveFault)
        .def_readonly("trackingError", &ErrorDetectedEvent::trackingError);

    /* ****************************************************** */
    py::class_<ConnectedStateChangedEvent>(m, "ConnectedStateChangedEvent")
        .def_readonly("isConnected", &ConnectedStateChangedEvent::isConnected);

    /* ****************************************************** */
    py::class_<ReadingEvent>(m, "ReadingEvent")
        .def_readonly("posFeedback", &ReadingEvent::posFeedback)
        .def_readonly("torqueSetpoint", &ReadingEvent::torqueSetpoint)
        .def_readonly("posSetpoint", &ReadingEvent::posSetpoint);

#ifdef VERSION_INFO
    m.attr("__version__") = VERSION_INFO;
#else
    m.attr("__version__") = "dev";
#endif
}
