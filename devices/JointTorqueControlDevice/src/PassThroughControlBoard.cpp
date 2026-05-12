/**
 * @file PassThroughControlBoard.cpp
 * @authors Ines Sorrentino
 * @copyright 2024 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <BipedalLocomotion/PassThroughControlBoard.h>
#include <iostream>
#include <yarp/os/Property.h>

using namespace yarp::dev;
using namespace BipedalLocomotion;

// CONSTRUCTOR
PassThroughControlBoard::PassThroughControlBoard()
    : proxyIEncodersTimed(0)
    , proxyIPositionControl(0)
    , proxyIVelocityControl(0)
    , proxyIPositionDirect(0)
    , proxyIControlMode(0)
    , proxyITorqueControl(0)
    , proxyIOpenLoopControl(0)
    , proxyIInteractionMode(0)
    , proxyIPidControl(0)
    , proxyICurrentControl(0)
    , proxyIMotorEncoders(0)
    , proxyIAxisInfo(0)
    , proxyIAmplifierControl(0)
    , proxyIControlCalibration(0)
    , proxyIControlLimits(0)
    , proxyIMotor(0)
    , proxyIJointFault(0)
{
}

PassThroughControlBoard::~PassThroughControlBoard()
{
    this->close();
}

void PassThroughControlBoard::resetPointers()
{
    proxyIEncodersTimed = nullptr;
    proxyIPositionControl = nullptr;
    proxyIControlMode = nullptr;
    proxyITorqueControl = nullptr;
    proxyIOpenLoopControl = nullptr;
    proxyIVelocityControl = nullptr;
    proxyIPositionDirect = nullptr;
    proxyIInteractionMode = nullptr;
    proxyIPidControl = nullptr;
    proxyICurrentControl = nullptr;
    proxyIMotorEncoders = nullptr;
    proxyIAxisInfo = nullptr;
    proxyIAmplifierControl = nullptr;
    proxyIControlCalibration = nullptr;
    proxyIControlLimits = nullptr;
    proxyIMotor = nullptr;
    proxyIJointFault = nullptr;
}

// DEVICE DRIVER
bool PassThroughControlBoard::open(yarp::os::Searchable& config)
{
    return true;
}

bool PassThroughControlBoard::close()
{
    resetPointers();
    return true;
}

bool PassThroughControlBoard::attachAll(const PolyDriverList& p)
{
    if (p.size() != 1)
    {
        BipedalLocomotion::log()->error("PassThroughControlBoard error: {} devices passed to "
                                        "attachAll, but only 1 is supported.If you want to connect "
                                        "several device to a single PassThroughControlBoard, "
                                        "please use a controlboardremapper device in the middle.",
                                        p.size());
        return false;
    }

    PolyDriver* proxyDevice = p[0]->poly;
    proxyDevice->view(proxyIEncodersTimed);
    proxyDevice->view(proxyIPositionControl);
    proxyDevice->view(proxyIControlMode);
    proxyDevice->view(proxyITorqueControl);
    proxyDevice->view(proxyIOpenLoopControl);
    proxyDevice->view(proxyIVelocityControl);
    proxyDevice->view(proxyIPositionDirect);
    proxyDevice->view(proxyIInteractionMode);
    proxyDevice->view(proxyIPidControl);
    proxyDevice->view(proxyICurrentControl);
    proxyDevice->view(proxyIMotorEncoders);
    proxyDevice->view(proxyIAxisInfo);
    proxyDevice->view(proxyIAmplifierControl);
    proxyDevice->view(proxyIControlCalibration);
    proxyDevice->view(proxyIControlLimits);
    proxyDevice->view(proxyIMotor);
    proxyDevice->view(proxyIJointFault);

    return true;
}

bool PassThroughControlBoard::detachAll()
{
    resetPointers();
    return true;
}

ReturnValue PassThroughControlBoard::getLastJointFault(int j, int& fault, std::string& message)
{
    if (!proxyIJointFault)
    {
        return ReturnValue::return_code::return_value_error_method_failed;
    }
    return proxyIJointFault->getLastJointFault(j, fault, message);
}

// ENCODERS
ReturnValue PassThroughControlBoard::getAxes(int* ax)
{
    if (!proxyIEncodersTimed)
    {
        return ReturnValue::return_code::return_value_error_method_failed;
    }
    return proxyIEncodersTimed->getAxes(ax);
}

ReturnValue PassThroughControlBoard::resetEncoder(int j)
{
    if (!proxyIEncodersTimed)
    {
        return ReturnValue::return_code::return_value_error_method_failed;
    }
    return proxyIEncodersTimed->resetEncoder(j);
}

ReturnValue PassThroughControlBoard::resetEncoders()
{
    if (!proxyIEncodersTimed)
    {
        return ReturnValue::return_code::return_value_error_method_failed;
    }
    return proxyIEncodersTimed->resetEncoders();
}

ReturnValue PassThroughControlBoard::setEncoder(int j, double val)
{
    if (!proxyIEncodersTimed)
    {
        return ReturnValue::return_code::return_value_error_method_failed;
    }
    return proxyIEncodersTimed->setEncoder(j, val);
}

ReturnValue PassThroughControlBoard::setEncoders(const double* vals)
{
    if (!proxyIEncodersTimed)
    {
        return ReturnValue::return_code::return_value_error_method_failed;
    }
    return proxyIEncodersTimed->setEncoders(vals);
}

ReturnValue PassThroughControlBoard::getEncoder(int j, double* v)
{
    if (!proxyIEncodersTimed)
    {
        return ReturnValue::return_code::return_value_error_method_failed;
    }
    return proxyIEncodersTimed->getEncoder(j, v);
}

ReturnValue PassThroughControlBoard::getEncoders(double* encs)
{
    if (!proxyIEncodersTimed)
    {
        return ReturnValue::return_code::return_value_error_method_failed;
    }
    return proxyIEncodersTimed->getEncoders(encs);
}

ReturnValue PassThroughControlBoard::getEncoderSpeed(int j, double* sp)
{
    if (!proxyIEncodersTimed)
    {
        return ReturnValue::return_code::return_value_error_method_failed;
    }
    return proxyIEncodersTimed->getEncoderSpeed(j, sp);
}

ReturnValue PassThroughControlBoard::getEncoderSpeeds(double* spds)
{
    if (!proxyIEncodersTimed)
    {
        return ReturnValue::return_code::return_value_error_method_failed;
    }
    return proxyIEncodersTimed->getEncoderSpeeds(spds);
}

ReturnValue PassThroughControlBoard::getEncoderAcceleration(int j, double* spds)
{
    if (!proxyIEncodersTimed)
    {
        return ReturnValue::return_code::return_value_error_method_failed;
    }
    return proxyIEncodersTimed->getEncoderAcceleration(j, spds);
}

ReturnValue PassThroughControlBoard::getEncoderAccelerations(double* accs)
{
    if (!proxyIEncodersTimed)
    {
        return ReturnValue::return_code::return_value_error_method_failed;
    }
    return proxyIEncodersTimed->getEncoderAccelerations(accs);
}

// ENCODERS TIMED
ReturnValue PassThroughControlBoard::getEncodersTimed(double* encs, double* time)
{
    if (!proxyIEncodersTimed)
    {
        return ReturnValue::return_code::return_value_error_method_failed;
    }
    return proxyIEncodersTimed->getEncodersTimed(encs, time);
}

ReturnValue PassThroughControlBoard::getEncoderTimed(int j, double* encs, double* time)
{
    if (!proxyIEncodersTimed)
    {
        return ReturnValue::return_code::return_value_error_method_failed;
    }
    return proxyIEncodersTimed->getEncoderTimed(j, encs, time);
}

// MOTOR ENCODERS
ReturnValue PassThroughControlBoard::getNumberOfMotorEncoders(int* num)
{
    if (!proxyIMotorEncoders)
    {
        return ReturnValue::return_code::return_value_error_method_failed;
    }
    return proxyIMotorEncoders->getNumberOfMotorEncoders(num);
}

ReturnValue PassThroughControlBoard::resetMotorEncoder(int m)
{
    if (!proxyIMotorEncoders)
    {
        return ReturnValue::return_code::return_value_error_method_failed;
    }
    return proxyIMotorEncoders->resetMotorEncoder(m);
}

ReturnValue PassThroughControlBoard::resetMotorEncoders()
{
    if (!proxyIMotorEncoders)
    {
        return ReturnValue::return_code::return_value_error_method_failed;
    }
    return proxyIMotorEncoders->resetMotorEncoders();
}

ReturnValue PassThroughControlBoard::setMotorEncoderCountsPerRevolution(int m, const double cpr)
{
    if (!proxyIMotorEncoders)
    {
        return ReturnValue::return_code::return_value_error_method_failed;
    }
    return proxyIMotorEncoders->setMotorEncoderCountsPerRevolution(m, cpr);
}

ReturnValue PassThroughControlBoard::getMotorEncoderCountsPerRevolution(int m, double* cpr)
{
    if (!proxyIMotorEncoders)
    {
        return ReturnValue::return_code::return_value_error_method_failed;
    }
    return proxyIMotorEncoders->getMotorEncoderCountsPerRevolution(m, cpr);
}

ReturnValue PassThroughControlBoard::setMotorEncoder(int m, const double val)
{
    if (!proxyIMotorEncoders)
    {
        return ReturnValue::return_code::return_value_error_method_failed;
    }
    return proxyIMotorEncoders->setMotorEncoder(m, val);
}

ReturnValue PassThroughControlBoard::setMotorEncoders(const double* vals)
{
    if (!proxyIMotorEncoders)
    {
        return ReturnValue::return_code::return_value_error_method_failed;
    }
    return proxyIMotorEncoders->setMotorEncoders(vals);
}

ReturnValue PassThroughControlBoard::getMotorEncoder(int m, double* v)
{
    if (!proxyIMotorEncoders)
    {
        return ReturnValue::return_code::return_value_error_method_failed;
    }
    return proxyIMotorEncoders->getMotorEncoder(m, v);
}

ReturnValue PassThroughControlBoard::getMotorEncoders(double* encs)
{
    if (!proxyIMotorEncoders)
    {
        return ReturnValue::return_code::return_value_error_method_failed;
    }
    return proxyIMotorEncoders->getMotorEncoders(encs);
}

ReturnValue PassThroughControlBoard::getMotorEncodersTimed(double* encs, double* time)
{
    if (!proxyIMotorEncoders)
    {
        return ReturnValue::return_code::return_value_error_method_failed;
    }
    return proxyIMotorEncoders->getMotorEncodersTimed(encs, time);
}

ReturnValue PassThroughControlBoard::getMotorEncoderTimed(int m, double* encs, double* time)
{
    if (!proxyIMotorEncoders)
    {
        return ReturnValue::return_code::return_value_error_method_failed;
    }
    return proxyIMotorEncoders->getMotorEncoderTimed(m, encs, time);
}

ReturnValue PassThroughControlBoard::getMotorEncoderSpeed(int m, double* sp)
{
    if (!proxyIMotorEncoders)
    {
        return ReturnValue::return_code::return_value_error_method_failed;
    }
    return proxyIMotorEncoders->getMotorEncoderSpeed(m, sp);
}

ReturnValue PassThroughControlBoard::getMotorEncoderSpeeds(double* spds)
{
    if (!proxyIMotorEncoders)
    {
        return ReturnValue::return_code::return_value_error_method_failed;
    }
    return proxyIMotorEncoders->getMotorEncoderSpeeds(spds);
}

ReturnValue PassThroughControlBoard::getTemperatures(double *vals)
{
    if (!proxyIMotor)
    {
        return ReturnValue::return_code::return_value_error_method_failed;
    }
    return proxyIMotor->getTemperatures(vals);
}

ReturnValue PassThroughControlBoard::getTemperature(int m, double* val)
{
    if (!proxyIMotor)
    {
        return ReturnValue::return_code::return_value_error_method_failed;
    }
    return proxyIMotor->getTemperature(m, val);
}

ReturnValue PassThroughControlBoard::getTemperatureLimit(int m, double* temp)
{
    if (!proxyIMotor)
    {
        return ReturnValue::return_code::return_value_error_method_failed;
    }
    return proxyIMotor->getTemperatureLimit(m, temp);
}

ReturnValue PassThroughControlBoard::setTemperatureLimit(int m, const double temp)
{
    if (!proxyIMotor)
    {
        return ReturnValue::return_code::return_value_error_method_failed;
    }
    return proxyIMotor->setTemperatureLimit(m, temp);
}
ReturnValue PassThroughControlBoard::getMotorEncoderAcceleration(int m, double* acc)
{
    if (!proxyIMotorEncoders)
    {
        return ReturnValue::return_code::return_value_error_method_failed;
    }
    return proxyIMotorEncoders->getMotorEncoderAcceleration(m, acc);
}

ReturnValue PassThroughControlBoard::getMotorEncoderAccelerations(double* accs)
{
    if (!proxyIMotorEncoders)
    {
        return ReturnValue::return_code::return_value_error_method_failed;
    }
    return proxyIMotorEncoders->getMotorEncoderAccelerations(accs);
}

// POSITION CONTROL
ReturnValue PassThroughControlBoard::positionMove(int j, double ref)
{
    if (!proxyIPositionControl)
    {
        return ReturnValue::return_code::return_value_error_method_failed;
    }
    return proxyIPositionControl->positionMove(j, ref);
}

ReturnValue PassThroughControlBoard::positionMove(const double* refs)
{
    if (!proxyIPositionControl)
    {
        return ReturnValue::return_code::return_value_error_method_failed;
    }
    return proxyIPositionControl->positionMove(refs);
}

ReturnValue PassThroughControlBoard::relativeMove(int j, double delta)
{
    if (!proxyIPositionControl)
    {
        return ReturnValue::return_code::return_value_error_method_failed;
    }
    return proxyIPositionControl->relativeMove(j, delta);
}

ReturnValue PassThroughControlBoard::relativeMove(const double* deltas)
{
    if (!proxyIPositionControl)
    {
        return ReturnValue::return_code::return_value_error_method_failed;
    }
    return proxyIPositionControl->relativeMove(deltas);
}

ReturnValue PassThroughControlBoard::checkMotionDone(int j, bool& flag)
{
    if (!proxyIPositionControl)
    {
        return ReturnValue::return_code::return_value_error_method_failed;
    }
    return proxyIPositionControl->checkMotionDone(j, flag);
}

ReturnValue PassThroughControlBoard::checkMotionDone(bool& flag)
{
    if (!proxyIPositionControl)
    {
        return ReturnValue::return_code::return_value_error_method_failed;
    }
    return proxyIPositionControl->checkMotionDone(flag);
}

ReturnValue PassThroughControlBoard::setTrajSpeed(int j, double sp)
{
    if (!proxyIPositionControl)
    {
        return ReturnValue::return_code::return_value_error_method_failed;
    }
    return proxyIPositionControl->setTrajSpeed(j, sp);
}

ReturnValue PassThroughControlBoard::setTrajSpeeds(const double* spds)
{
    if (!proxyIPositionControl)
    {
        return ReturnValue::return_code::return_value_error_method_failed;
    }
    return proxyIPositionControl->setTrajSpeeds(spds);
}

ReturnValue PassThroughControlBoard::setTrajAcceleration(int j, double acc)
{
    if (!proxyIPositionControl)
    {
        return ReturnValue::return_code::return_value_error_method_failed;
    }
    return proxyIPositionControl->setTrajAcceleration(j, acc);
}

ReturnValue PassThroughControlBoard::setTrajAccelerations(const double* accs)
{
    if (!proxyIPositionControl)
    {
        return ReturnValue::return_code::return_value_error_method_failed;
    }
    return proxyIPositionControl->setTrajAccelerations(accs);
}

ReturnValue PassThroughControlBoard::getTrajSpeed(int j, double* ref)
{
    if (!proxyIPositionControl)
    {
        return ReturnValue::return_code::return_value_error_method_failed;
    }
    return proxyIPositionControl->getTrajSpeed(j, ref);
}

ReturnValue PassThroughControlBoard::getTrajSpeeds(double* spds)
{
    if (!proxyIPositionControl)
    {
        return ReturnValue::return_code::return_value_error_method_failed;
    }
    return proxyIPositionControl->getTrajSpeeds(spds);
}

ReturnValue PassThroughControlBoard::getTrajAcceleration(int j, double* acc)
{
    if (!proxyIPositionControl)
    {
        return ReturnValue::return_code::return_value_error_method_failed;
    }
    return proxyIPositionControl->getTrajAcceleration(j, acc);
}

ReturnValue PassThroughControlBoard::getTrajAccelerations(double* accs)
{
    if (!proxyIPositionControl)
    {
        return ReturnValue::return_code::return_value_error_method_failed;
    }
    return proxyIPositionControl->getTrajAccelerations(accs);
}

ReturnValue PassThroughControlBoard::stop(int j)
{
    if (!proxyIPositionControl)
    {
        return ReturnValue::return_code::return_value_error_method_failed;
    }
    return proxyIPositionControl->stop(j);
}

ReturnValue PassThroughControlBoard::stop()
{
    if (!proxyIPositionControl)
    {
        return ReturnValue::return_code::return_value_error_method_failed;
    }
    return proxyIPositionControl->stop();
}

// POSITION CONTROL
ReturnValue PassThroughControlBoard::positionMove(const int n_joint, const int* joints, const double* refs)
{
    if (!proxyIPositionControl)
    {
        return ReturnValue::return_code::return_value_error_method_failed;
    }
    return proxyIPositionControl->positionMove(n_joint, joints, refs);
}

ReturnValue PassThroughControlBoard::relativeMove(const int n_joint,
                                           const int* joints,
                                           const double* deltas)
{
    if (!proxyIPositionControl)
    {
        return ReturnValue::return_code::return_value_error_method_failed;
    }
    return proxyIPositionControl->relativeMove(n_joint, joints, deltas);
}

ReturnValue PassThroughControlBoard::checkMotionDone(const std::vector<int>& joints, bool& flag)
{
    if (!proxyIPositionControl)
    {
        return ReturnValue::return_code::return_value_error_method_failed;
    }
    return proxyIPositionControl->checkMotionDone(joints, flag);
}

ReturnValue PassThroughControlBoard::setTrajSpeeds(const int n_joint, const int* joints, const double* spds)
{
    if (!proxyIPositionControl)
    {
        return ReturnValue::return_code::return_value_error_method_failed;
    }
    return proxyIPositionControl->setTrajSpeeds(n_joint, joints, spds);
}

ReturnValue PassThroughControlBoard::setTrajAccelerations(const int n_joint,
                                                  const int* joints,
                                                  const double* accs)
{
    if (!proxyIPositionControl)
    {
        return ReturnValue::return_code::return_value_error_method_failed;
    }
    return proxyIPositionControl->setTrajAccelerations(n_joint, joints, accs);
}

ReturnValue PassThroughControlBoard::getTrajSpeeds(const int n_joint, const int* joints, double* spds)
{
    if (!proxyIPositionControl)
    {
        return ReturnValue::return_code::return_value_error_method_failed;
    }
    return proxyIPositionControl->getTrajSpeeds(n_joint, joints, spds);
}

ReturnValue PassThroughControlBoard::getTrajAccelerations(const int n_joint,
                                                  const int* joints,
                                                  double* accs)
{
    if (!proxyIPositionControl)
    {
        return ReturnValue::return_code::return_value_error_method_failed;
    }
    return proxyIPositionControl->getTrajAccelerations(n_joint, joints, accs);
}

ReturnValue PassThroughControlBoard::stop(const int n_joint, const int* joints)
{
    if (!proxyIPositionControl)
    {
        return ReturnValue::return_code::return_value_error_method_failed;
    }
    return proxyIPositionControl->stop(n_joint, joints);
}

ReturnValue PassThroughControlBoard::getTargetPosition(const int joint, double* ref)
{
    if (!proxyIPositionControl)
    {
        return ReturnValue::return_code::return_value_error_method_failed;
    }
    return proxyIPositionControl->getTargetPosition(joint, ref);
}

ReturnValue PassThroughControlBoard::getTargetPositions(double* refs)
{
    if (!proxyIPositionControl)
    {
        return ReturnValue::return_code::return_value_error_method_failed;
    }
    return proxyIPositionControl->getTargetPositions(refs);
}

ReturnValue PassThroughControlBoard::getTargetPositions(const int n_joint, const int* joints, double* refs)
{
    if (!proxyIPositionControl)
    {
        return ReturnValue::return_code::return_value_error_method_failed;
    }
    return proxyIPositionControl->getTargetPositions(n_joint, joints, refs);
}

// VELOCITY CONTROL
ReturnValue PassThroughControlBoard::velocityMove(int j, double sp)
{
    if (!proxyIVelocityControl)
    {
        return ReturnValue::return_code::return_value_error_method_failed;
    }
    return proxyIVelocityControl->velocityMove(j, sp);
}

ReturnValue PassThroughControlBoard::velocityMove(const double* sp)
{
    if (!proxyIVelocityControl)
    {
        return ReturnValue::return_code::return_value_error_method_failed;
    }
    return proxyIVelocityControl->velocityMove(sp);
}

// VELOCITY CONTROL
ReturnValue PassThroughControlBoard::velocityMove(const int n_joint, const int* joints, const double* spds)
{
    if (!proxyIVelocityControl)
    {
        return ReturnValue::return_code::return_value_error_method_failed;
    }
    return proxyIVelocityControl->velocityMove(n_joint, joints, spds);
}

ReturnValue PassThroughControlBoard::getTargetVelocity(const int joint, double* vel)
{
    if (!proxyIVelocityControl)
    {
        return ReturnValue::return_code::return_value_error_method_failed;
    }
    return proxyIVelocityControl->getTargetVelocity(joint, vel);
}

ReturnValue PassThroughControlBoard::getTargetVelocities(double* vels)
{
    if (!proxyIVelocityControl)
    {
        return ReturnValue::return_code::return_value_error_method_failed;
    }
    return proxyIVelocityControl->getTargetVelocities(vels);
}

ReturnValue PassThroughControlBoard::getTargetVelocities(const int n_joint, const int* joints, double* vels)
{
    if (!proxyIVelocityControl)
    {
        return ReturnValue::return_code::return_value_error_method_failed;
    }
    return proxyIVelocityControl->getTargetVelocities(n_joint, joints, vels);
}

// POSITION DIRECT CONTROL
ReturnValue PassThroughControlBoard::setPosition(int j, double ref)
{
    if (!proxyIPositionDirect)
    {
        return ReturnValue::return_code::return_value_error_method_failed;
    }
    return proxyIPositionDirect->setPosition(j, ref);
}

ReturnValue PassThroughControlBoard::setPositions(const int n_joint, const int* joints, const double* refs)
{
    if (!proxyIPositionDirect)
    {
        return ReturnValue::return_code::return_value_error_method_failed;
    }
    return proxyIPositionDirect->setPositions(n_joint, joints, refs);
}

ReturnValue PassThroughControlBoard::setPositions(const double* refs)
{
    if (!proxyIPositionDirect)
    {
        return ReturnValue::return_code::return_value_error_method_failed;
    }
    return proxyIPositionDirect->setPositions(refs);
}

ReturnValue PassThroughControlBoard::getRefPosition(const int joint, double* ref)
{
    if (!proxyIPositionDirect)
    {
        return ReturnValue::return_code::return_value_error_method_failed;
    }
    return proxyIPositionDirect->getRefPosition(joint, ref);
}

ReturnValue PassThroughControlBoard::getRefPositions(double* refs)
{
    if (!proxyIPositionDirect)
    {
        return ReturnValue::return_code::return_value_error_method_failed;
    }
    return proxyIPositionDirect->getRefPositions(refs);
}

ReturnValue PassThroughControlBoard::getRefPositions(const int n_joint, const int* joints, double* refs)
{
    if (!proxyIPositionDirect)
    {
        return ReturnValue::return_code::return_value_error_method_failed;
    }
    return proxyIPositionDirect->getRefPositions(n_joint, joints, refs);
}

// CONTROL MODE
ReturnValue PassThroughControlBoard::getAvailableControlModes(
    int j,
    std::vector<SelectableControlModeEnum>& avail)
{
    if (!proxyIControlMode)
    {
        return ReturnValue::return_code::return_value_error_method_failed;
    }
    return proxyIControlMode->getAvailableControlModes(j, avail);
}

ReturnValue PassThroughControlBoard::getControlMode(int j, ControlModeEnum& mode)
{
    if (!proxyIControlMode)
    {
        return ReturnValue::return_code::return_value_error_method_failed;
    }
    return proxyIControlMode->getControlMode(j, mode);
}

ReturnValue PassThroughControlBoard::getControlModes(std::vector<ControlModeEnum>& mode)
{
    if (!proxyIControlMode)
    {
        return ReturnValue::return_code::return_value_error_method_failed;
    }
    return proxyIControlMode->getControlModes(mode);
}

ReturnValue PassThroughControlBoard::getControlModes(const std::vector<int>& joints,
                                                     std::vector<ControlModeEnum>& mode)
{
    if (!proxyIControlMode)
    {
        return ReturnValue::return_code::return_value_error_method_failed;
    }
    return proxyIControlMode->getControlModes(joints, mode);
}

ReturnValue PassThroughControlBoard::setControlMode(int j, SelectableControlModeEnum mode)
{
    if (!proxyIControlMode)
    {
        return ReturnValue::return_code::return_value_error_method_failed;
    }
    return proxyIControlMode->setControlMode(j, mode);
}

ReturnValue PassThroughControlBoard::setControlModes(const std::vector<int>& joints,
                                                     const std::vector<SelectableControlModeEnum>& mode)
{
    if (!proxyIControlMode)
    {
        return ReturnValue::return_code::return_value_error_method_failed;
    }
    return proxyIControlMode->setControlModes(joints, mode);
}

ReturnValue PassThroughControlBoard::setControlModes(const std::vector<SelectableControlModeEnum>& mode)
{
    if (!proxyIControlMode)
    {
        return ReturnValue::return_code::return_value_error_method_failed;
    }
    return proxyIControlMode->setControlModes(mode);
}

// TORQUE CONTROL
ReturnValue PassThroughControlBoard::getRefTorques(double* t)
{
    if (!proxyITorqueControl)
    {
        return ReturnValue::return_code::return_value_error_method_failed;
    }
    return proxyITorqueControl->getRefTorques(t);
}

ReturnValue PassThroughControlBoard::getRefTorque(int j, double* t)
{
    if (!proxyITorqueControl)
    {
        return ReturnValue::return_code::return_value_error_method_failed;
    }
    return proxyITorqueControl->getRefTorque(j, t);
}

ReturnValue PassThroughControlBoard::setRefTorques(const double* t)
{
    if (!proxyITorqueControl)
    {
        return ReturnValue::return_code::return_value_error_method_failed;
    }
    return proxyITorqueControl->setRefTorques(t);
}

ReturnValue PassThroughControlBoard::setRefTorques(const int n_joint,
                                                   const int* joints,
                                                   const double* t)
{
    if (!proxyITorqueControl)
    {
        return ReturnValue::return_code::return_value_error_method_failed;
    }
    return proxyITorqueControl->setRefTorques(n_joint, joints, t);
}

ReturnValue PassThroughControlBoard::setRefTorque(int j, double t)
{
    if (!proxyITorqueControl)
    {
        return ReturnValue::return_code::return_value_error_method_failed;
    }
    return proxyITorqueControl->setRefTorque(j, t);
}

ReturnValue PassThroughControlBoard::getMotorTorqueParams(int j, MotorTorqueParameters* params)
{
    if (!proxyITorqueControl)
    {
        return ReturnValue::return_code::return_value_error_method_failed;
    }
    return proxyITorqueControl->getMotorTorqueParams(j, params);
}

ReturnValue PassThroughControlBoard::setMotorTorqueParams(int j,
                                                          const MotorTorqueParameters params)
{
    if (!proxyITorqueControl)
    {
        return ReturnValue::return_code::return_value_error_method_failed;
    }
    return proxyITorqueControl->setMotorTorqueParams(j, params);
}

ReturnValue PassThroughControlBoard::getTorque(int j, double* t)
{
    if (!proxyITorqueControl)
    {
        return ReturnValue::return_code::return_value_error_method_failed;
    }
    return proxyITorqueControl->getTorque(j, t);
}

ReturnValue PassThroughControlBoard::getTorques(double* t)
{
    if (!proxyITorqueControl)
    {
        return ReturnValue::return_code::return_value_error_method_failed;
    }
    return proxyITorqueControl->getTorques(t);
}

ReturnValue PassThroughControlBoard::getTorqueRange(int j, double* min, double* max)
{
    if (!proxyITorqueControl)
    {
        return ReturnValue::return_code::return_value_error_method_failed;
    }
    return proxyITorqueControl->getTorqueRange(j, min, max);
}

ReturnValue PassThroughControlBoard::getTorqueRanges(double* min, double* max)
{
    if (!proxyITorqueControl)
    {
        return ReturnValue::return_code::return_value_error_method_failed;
    }
    return proxyITorqueControl->getTorqueRanges(min, max);
}

// OPEN LOOP
ReturnValue PassThroughControlBoard::getNumberOfMotors(int* n)
{
    if (!proxyIOpenLoopControl)
    {
        return ReturnValue::return_code::return_value_error_method_failed;
    }
    return proxyIOpenLoopControl->getNumberOfMotors(n);
}

ReturnValue PassThroughControlBoard::setRefDutyCycle(int m, double ref)
{
    if (!proxyIOpenLoopControl)
    {
        return ReturnValue::return_code::return_value_error_method_failed;
    }
    return proxyIOpenLoopControl->setRefDutyCycle(m, ref);
}

ReturnValue PassThroughControlBoard::setRefDutyCycles(const double* refs)
{
    if (!proxyIOpenLoopControl)
    {
        return ReturnValue::return_code::return_value_error_method_failed;
    }
    return proxyIOpenLoopControl->setRefDutyCycles(refs);
}

ReturnValue PassThroughControlBoard::getRefDutyCycle(int m, double* ref)
{
    if (!proxyIOpenLoopControl)
    {
        return ReturnValue::return_code::return_value_error_method_failed;
    }
    return proxyIOpenLoopControl->getRefDutyCycle(m, ref);
}

ReturnValue PassThroughControlBoard::getRefDutyCycles(double* refs)
{
    if (!proxyIOpenLoopControl)
    {
        return ReturnValue::return_code::return_value_error_method_failed;
    }
    return proxyIOpenLoopControl->getRefDutyCycles(refs);
}

ReturnValue PassThroughControlBoard::getDutyCycle(int m, double* val)
{
    if (!proxyIOpenLoopControl)
    {
        return ReturnValue::return_code::return_value_error_method_failed;
    }
    return proxyIOpenLoopControl->getDutyCycle(m, val);
}

ReturnValue PassThroughControlBoard::getDutyCycles(double* vals)
{
    if (!proxyIOpenLoopControl)
    {
        return ReturnValue::return_code::return_value_error_method_failed;
    }
    return proxyIOpenLoopControl->getDutyCycles(vals);
}

// CURRENT CONTROL
ReturnValue PassThroughControlBoard::getCurrent(int m, double* curr)
{
    if (!proxyICurrentControl)
    {
        return ReturnValue::return_code::return_value_error_method_failed;
    }
    return proxyICurrentControl->getCurrent(m, curr);
}

ReturnValue PassThroughControlBoard::getCurrents(double* currs)
{
    if (!proxyICurrentControl)
    {
        return ReturnValue::return_code::return_value_error_method_failed;
    }
    return proxyICurrentControl->getCurrents(currs);
}

ReturnValue PassThroughControlBoard::getCurrentRange(int m, double* min, double* max)
{
    if (!proxyICurrentControl)
    {
        return ReturnValue::return_code::return_value_error_method_failed;
    }
    return proxyICurrentControl->getCurrentRange(m, min, max);
}

ReturnValue PassThroughControlBoard::getCurrentRanges(double* min, double* max)
{
    if (!proxyICurrentControl)
    {
        return ReturnValue::return_code::return_value_error_method_failed;
    }
    return proxyICurrentControl->getCurrentRanges(min, max);
}

ReturnValue PassThroughControlBoard::getGearboxRatio(int m, double *val)
{
    if (!proxyIMotor)
    {
        return ReturnValue::return_code::return_value_error_method_failed;
    }
    return proxyIMotor->getGearboxRatio(m, val);
}

ReturnValue PassThroughControlBoard::setGearboxRatio(int m, const double val)
{
    if (!proxyIMotor)
    {
        return ReturnValue::return_code::return_value_error_method_failed;
    }
    return proxyIMotor->setGearboxRatio(m, val);
}

ReturnValue PassThroughControlBoard::getRefCurrents(double* currs)
{
    if (!proxyICurrentControl)
    {
        return ReturnValue::return_code::return_value_error_method_failed;
    }
    return proxyICurrentControl->getRefCurrents(currs);
}

ReturnValue PassThroughControlBoard::setRefCurrent(int m, double curr)
{
    if (!proxyICurrentControl)
    {
        return ReturnValue::return_code::return_value_error_method_failed;
    }
    return proxyICurrentControl->setRefCurrent(m, curr);
}

ReturnValue PassThroughControlBoard::setRefCurrents(const int n_motor,
                                             const int* motors,
                                             const double* currs)
{
    if (!proxyICurrentControl)
    {
        return ReturnValue::return_code::return_value_error_method_failed;
    }
    return proxyICurrentControl->setRefCurrents(n_motor, motors, currs);
}

ReturnValue PassThroughControlBoard::setRefCurrents(const double* currs)
{
    if (!proxyICurrentControl)
    {
        return ReturnValue::return_code::return_value_error_method_failed;
    }
    return proxyICurrentControl->setRefCurrents(currs);
}

ReturnValue PassThroughControlBoard::getRefCurrent(int m, double* curr)
{
    if (!proxyICurrentControl)
    {
        return ReturnValue::return_code::return_value_error_method_failed;
    }
    return proxyICurrentControl->getRefCurrent(m, curr);
}

// INTERACTION MODE
ReturnValue PassThroughControlBoard::getInteractionMode(int axis, yarp::dev::InteractionModeEnum& mode)
{
    if (!proxyIInteractionMode)
    {
        return ReturnValue::return_code::return_value_error_method_failed;
    }
    return proxyIInteractionMode->getInteractionMode(axis, mode);
}

ReturnValue PassThroughControlBoard::getInteractionModes(const std::vector<int>& joints,
                                                  std::vector<yarp::dev::InteractionModeEnum>& modes)
{
    if (!proxyIInteractionMode)
    {
        return ReturnValue::return_code::return_value_error_method_failed;
    }
    return proxyIInteractionMode->getInteractionModes(joints, modes);
}

ReturnValue PassThroughControlBoard::getInteractionModes(std::vector<yarp::dev::InteractionModeEnum>& modes)
{
    if (!proxyIInteractionMode)
    {
        return ReturnValue::return_code::return_value_error_method_failed;
    }
    return proxyIInteractionMode->getInteractionModes(modes);
}

ReturnValue PassThroughControlBoard::setInteractionMode(int axis, yarp::dev::InteractionModeEnum mode)
{
    if (!proxyIInteractionMode)
    {
        return ReturnValue::return_code::return_value_error_method_failed;
    }
    return proxyIInteractionMode->setInteractionMode(axis, mode);
}

ReturnValue PassThroughControlBoard::setInteractionModes(const std::vector<int>& joints,
                                                  const std::vector<yarp::dev::InteractionModeEnum>& modes)
{
    if (!proxyIInteractionMode)
    {
        return ReturnValue::return_code::return_value_error_method_failed;
    }
    return proxyIInteractionMode->setInteractionModes(joints, modes);
}

ReturnValue PassThroughControlBoard::setInteractionModes(const std::vector<yarp::dev::InteractionModeEnum>& modes)
{
    if (!proxyIInteractionMode)
    {
        return ReturnValue::return_code::return_value_error_method_failed;
    }
    return proxyIInteractionMode->setInteractionModes(modes);
}

// PID CONTROL INTERFACE METHODS
ReturnValue PassThroughControlBoard::getAvailablePids(int j,
                                                      std::vector<PidControlTypeEnum>& avail)
{
    if (!proxyIPidControl)
    {
        return ReturnValue::return_code::return_value_error_method_failed;
    }
    return proxyIPidControl->getAvailablePids(j, avail);
}

ReturnValue PassThroughControlBoard::setPid(const PidControlTypeEnum& pidtype, int j, const Pid& pid)
{
    if (!proxyIPidControl)
    {
        return ReturnValue::return_code::return_value_error_method_failed;
    }
    return proxyIPidControl->setPid(pidtype, j, pid);
}

ReturnValue PassThroughControlBoard::setPids(const PidControlTypeEnum& pidtype, const Pid* pids)
{
    if (!proxyIPidControl)
    {
        return ReturnValue::return_code::return_value_error_method_failed;
    }
    return proxyIPidControl->setPids(pidtype, pids);
}

ReturnValue PassThroughControlBoard::setPidReference(const PidControlTypeEnum& pidtype, int j, double ref)
{
    if (!proxyIPidControl)
    {
        return ReturnValue::return_code::return_value_error_method_failed;
    }
    return proxyIPidControl->setPidReference(pidtype, j, ref);
}

ReturnValue PassThroughControlBoard::setPidReferences(const PidControlTypeEnum& pidtype,
                                               const double* refs)
{
    if (!proxyIPidControl)
    {
        return ReturnValue::return_code::return_value_error_method_failed;
    }
    return proxyIPidControl->setPidReferences(pidtype, refs);
}

ReturnValue PassThroughControlBoard::setPidErrorLimit(const PidControlTypeEnum& pidtype,
                                               int j,
                                               double limit)
{
    if (!proxyIPidControl)
    {
        return ReturnValue::return_code::return_value_error_method_failed;
    }
    return proxyIPidControl->setPidErrorLimit(pidtype, j, limit);
}

ReturnValue PassThroughControlBoard::setPidErrorLimits(const PidControlTypeEnum& pidtype,
                                                const double* limits)
{
    if (!proxyIPidControl)
    {
        return ReturnValue::return_code::return_value_error_method_failed;
    }
    return proxyIPidControl->setPidErrorLimits(pidtype, limits);
}

ReturnValue PassThroughControlBoard::getPidError(const PidControlTypeEnum& pidtype, int j, double* err)
{
    if (!proxyIPidControl)
    {
        return ReturnValue::return_code::return_value_error_method_failed;
    }
    return proxyIPidControl->getPidError(pidtype, j, err);
}

ReturnValue PassThroughControlBoard::getPidErrors(const PidControlTypeEnum& pidtype, double* errs)
{
    if (!proxyIPidControl)
    {
        return ReturnValue::return_code::return_value_error_method_failed;
    }
    return proxyIPidControl->getPidErrors(pidtype, errs);
}

ReturnValue PassThroughControlBoard::getPidOutput(const PidControlTypeEnum& pidtype, int j, double* out)
{
    if (!proxyIPidControl)
    {
        return ReturnValue::return_code::return_value_error_method_failed;
    }
    return proxyIPidControl->getPidOutput(pidtype, j, out);
}

ReturnValue PassThroughControlBoard::getPidOutputs(const PidControlTypeEnum& pidtype, double* outs)
{
    if (!proxyIPidControl)
    {
        return ReturnValue::return_code::return_value_error_method_failed;
    }
    return proxyIPidControl->getPidOutputs(pidtype, outs);
}

ReturnValue PassThroughControlBoard::getPid(const PidControlTypeEnum& pidtype, int j, Pid* pid)
{
    if (!proxyIPidControl)
    {
        return ReturnValue::return_code::return_value_error_method_failed;
    }
    return proxyIPidControl->getPid(pidtype, j, pid);
}

ReturnValue PassThroughControlBoard::getPids(const PidControlTypeEnum& pidtype, Pid* pids)
{
    if (!proxyIPidControl)
    {
        return ReturnValue::return_code::return_value_error_method_failed;
    }
    return proxyIPidControl->getPids(pidtype, pids);
}

ReturnValue PassThroughControlBoard::getPidExtraInfo(const PidControlTypeEnum& pidtype,
                                                     int j,
                                                     PidExtraInfo& info)
{
    if (!proxyIPidControl)
    {
        return ReturnValue::return_code::return_value_error_method_failed;
    }
    return proxyIPidControl->getPidExtraInfo(pidtype, j, info);
}

ReturnValue PassThroughControlBoard::getPidExtraInfos(const PidControlTypeEnum& pidtype,
                                                      std::vector<PidExtraInfo>& info)
{
    if (!proxyIPidControl)
    {
        return ReturnValue::return_code::return_value_error_method_failed;
    }
    return proxyIPidControl->getPidExtraInfos(pidtype, info);
}

ReturnValue PassThroughControlBoard::getPidReference(const PidControlTypeEnum& pidtype, int j, double* ref)
{
    if (!proxyIPidControl)
    {
        return ReturnValue::return_code::return_value_error_method_failed;
    }
    return proxyIPidControl->getPidReference(pidtype, j, ref);
}

ReturnValue PassThroughControlBoard::getPidReferences(const PidControlTypeEnum& pidtype, double* refs)
{
    if (!proxyIPidControl)
    {
        return ReturnValue::return_code::return_value_error_method_failed;
    }
    return proxyIPidControl->getPidReferences(pidtype, refs);
}

ReturnValue PassThroughControlBoard::getPidErrorLimit(const PidControlTypeEnum& pidtype,
                                               int j,
                                               double* limit)
{
    if (!proxyIPidControl)
    {
        return ReturnValue::return_code::return_value_error_method_failed;
    }
    return proxyIPidControl->getPidErrorLimit(pidtype, j, limit);
}

ReturnValue PassThroughControlBoard::getPidErrorLimits(const PidControlTypeEnum& pidtype, double* limits)
{
    if (!proxyIPidControl)
    {
        return ReturnValue::return_code::return_value_error_method_failed;
    }
    return proxyIPidControl->getPidErrorLimits(pidtype, limits);
}

ReturnValue PassThroughControlBoard::resetPid(const PidControlTypeEnum& pidtype, int j)
{
    if (!proxyIPidControl)
    {
        return ReturnValue::return_code::return_value_error_method_failed;
    }
    return proxyIPidControl->resetPid(pidtype, j);
}

ReturnValue PassThroughControlBoard::disablePid(const PidControlTypeEnum& pidtype, int j)
{
    if (!proxyIPidControl)
    {
        return ReturnValue::return_code::return_value_error_method_failed;
    }
    return proxyIPidControl->disablePid(pidtype, j);
}

ReturnValue PassThroughControlBoard::enablePid(const PidControlTypeEnum& pidtype, int j)
{
    if (!proxyIPidControl)
    {
        return ReturnValue::return_code::return_value_error_method_failed;
    }
    return proxyIPidControl->enablePid(pidtype, j);
}

ReturnValue PassThroughControlBoard::setPidOffset(const PidControlTypeEnum& pidtype, int j, double v)
{
    if (!proxyIPidControl)
    {
        return ReturnValue::return_code::return_value_error_method_failed;
    }
    return proxyIPidControl->setPidOffset(pidtype, j, v);
}

ReturnValue PassThroughControlBoard::setPidFeedforward(const PidControlTypeEnum& pidtype,
                                                       int j,
                                                       double v)
{
    if (!proxyIPidControl)
    {
        return ReturnValue::return_code::return_value_error_method_failed;
    }
    return proxyIPidControl->setPidFeedforward(pidtype, j, v);
}

ReturnValue PassThroughControlBoard::getPidOffset(const PidControlTypeEnum& pidtype,
                                                  int j,
                                                  double& v)
{
    if (!proxyIPidControl)
    {
        return ReturnValue::return_code::return_value_error_method_failed;
    }
    return proxyIPidControl->getPidOffset(pidtype, j, v);
}

ReturnValue PassThroughControlBoard::getPidFeedforward(const PidControlTypeEnum& pidtype,
                                                       int j,
                                                       double& v)
{
    if (!proxyIPidControl)
    {
        return ReturnValue::return_code::return_value_error_method_failed;
    }
    return proxyIPidControl->getPidFeedforward(pidtype, j, v);
}

ReturnValue PassThroughControlBoard::isPidEnabled(const PidControlTypeEnum& pidtype,
                                                  int j,
                                                  bool& enabled)
{
    if (!proxyIPidControl)
    {
        return ReturnValue::return_code::return_value_error_method_failed;
    }
    return proxyIPidControl->isPidEnabled(pidtype, j, enabled);
}

// AXIS INFO
ReturnValue PassThroughControlBoard::getAxisName(int axis, std::string& name)
{
    if (!proxyIAxisInfo)
    {
        return ReturnValue::return_code::return_value_error_method_failed;
    }
    return proxyIAxisInfo->getAxisName(axis, name);
}
ReturnValue PassThroughControlBoard::getJointType(int axis, yarp::dev::JointTypeEnum& type)
{
    if (!proxyIAxisInfo)
    {
        return ReturnValue::return_code::return_value_error_method_failed;
    }
    return proxyIAxisInfo->getJointType(axis, type);
}

// AMPLIFIER CONTROL
ReturnValue PassThroughControlBoard::enableAmp(int j)
{
    if (!proxyIAmplifierControl)
    {
        return ReturnValue::return_code::return_value_error_method_failed;
    }
    return proxyIAmplifierControl->enableAmp(j);
}

ReturnValue PassThroughControlBoard::disableAmp(int j)
{
    if (!proxyIAmplifierControl)
    {
        return ReturnValue::return_code::return_value_error_method_failed;
    }
    return proxyIAmplifierControl->disableAmp(j);
}

ReturnValue PassThroughControlBoard::getAmpStatus(int* st)
{
    if (!proxyIAmplifierControl)
    {
        return ReturnValue::return_code::return_value_error_method_failed;
    }
    return proxyIAmplifierControl->getAmpStatus(st);
}

ReturnValue PassThroughControlBoard::getAmpStatus(int j, int* v)
{
    if (!proxyIAmplifierControl)
    {
        return ReturnValue::return_code::return_value_error_method_failed;
    }
    return proxyIAmplifierControl->getAmpStatus(j, v);
}

ReturnValue PassThroughControlBoard::getMaxCurrent(int j, double* v)
{
    if (!proxyIAmplifierControl)
    {
        return ReturnValue::return_code::return_value_error_method_failed;
    }
    return proxyIAmplifierControl->getMaxCurrent(j, v);
}

ReturnValue PassThroughControlBoard::setMaxCurrent(int j, double v)
{
    if (!proxyIAmplifierControl)
    {
        return ReturnValue::return_code::return_value_error_method_failed;
    }
    return proxyIAmplifierControl->setMaxCurrent(j, v);
}

ReturnValue PassThroughControlBoard::getNominalCurrent(int m, double* val)
{
    if (!proxyIAmplifierControl)
    {
        return ReturnValue::return_code::return_value_error_method_failed;
    }
    return proxyIAmplifierControl->getNominalCurrent(m, val);
}

ReturnValue PassThroughControlBoard::setNominalCurrent(int m, const double val)
{
    if (!proxyIAmplifierControl)
    {
        return ReturnValue::return_code::return_value_error_method_failed;
    }
    return proxyIAmplifierControl->setNominalCurrent(m, val);
}

ReturnValue PassThroughControlBoard::getPeakCurrent(int m, double* val)
{
    if (!proxyIAmplifierControl)
    {
        return ReturnValue::return_code::return_value_error_method_failed;
    }
    return proxyIAmplifierControl->getPeakCurrent(m, val);
}

ReturnValue PassThroughControlBoard::setPeakCurrent(int m, const double val)
{
    if (!proxyIAmplifierControl)
    {
        return ReturnValue::return_code::return_value_error_method_failed;
    }
    return proxyIAmplifierControl->setPeakCurrent(m, val);
}

ReturnValue PassThroughControlBoard::getPWM(int j, double* val)
{
    if (!proxyIAmplifierControl)
    {
        return ReturnValue::return_code::return_value_error_method_failed;
    }
    return proxyIAmplifierControl->getPWM(j, val);
}

ReturnValue PassThroughControlBoard::getPWMLimit(int j, double* val)
{
    if (!proxyIAmplifierControl)
    {
        return ReturnValue::return_code::return_value_error_method_failed;
    }
    return proxyIAmplifierControl->getPWMLimit(j, val);
}

ReturnValue PassThroughControlBoard::setPWMLimit(int j, const double val)
{
    if (!proxyIAmplifierControl)
    {
        return ReturnValue::return_code::return_value_error_method_failed;
    }
    return proxyIAmplifierControl->setPWMLimit(j, val);
}

ReturnValue PassThroughControlBoard::getPowerSupplyVoltage(int j, double* val)
{
    if (!proxyIAmplifierControl)
    {
        return ReturnValue::return_code::return_value_error_method_failed;
    }
    return proxyIAmplifierControl->getPowerSupplyVoltage(j, val);
}

// CONTROL CALIBRATION
ReturnValue PassThroughControlBoard::calibrateAxisWithParams(
    int axis, unsigned int type, double p1, double p2, double p3)
{
    if (!proxyIControlCalibration)
    {
        return ReturnValue::return_code::return_value_error_method_failed;
    }
    return proxyIControlCalibration->calibrateAxisWithParams(axis, type, p1, p2, p3);
}

ReturnValue PassThroughControlBoard::setCalibrationParameters(int axis,
                                                       const CalibrationParameters& params)
{
    if (!proxyIControlCalibration)
    {
        return ReturnValue::return_code::return_value_error_method_failed;
    }
    return proxyIControlCalibration->setCalibrationParameters(axis, params);
}

ReturnValue PassThroughControlBoard::calibrationDone(int j)
{
    if (!proxyIControlCalibration)
    {
        return ReturnValue::return_code::return_value_error_method_failed;
    }
    return proxyIControlCalibration->calibrationDone(j);
}

ReturnValue PassThroughControlBoard::setCalibrator(ICalibrator* c)
{
    if (!proxyIControlCalibration)
    {
        return ReturnValue::return_code::return_value_error_method_failed;
    }
    return proxyIControlCalibration->setCalibrator(c);
}

ReturnValue PassThroughControlBoard::calibrateRobot()
{
    if (!proxyIControlCalibration)
    {
        return ReturnValue::return_code::return_value_error_method_failed;
    }
    return proxyIControlCalibration->calibrateRobot();
}

ReturnValue PassThroughControlBoard::park(bool wait)
{
    if (!proxyIControlCalibration)
    {
        return ReturnValue::return_code::return_value_error_method_failed;
    }
    return proxyIControlCalibration->park(wait);
}

ReturnValue PassThroughControlBoard::abortCalibration()
{
    if (!proxyIControlCalibration)
    {
        return ReturnValue::return_code::return_value_error_method_failed;
    }
    return proxyIControlCalibration->abortCalibration();
}

ReturnValue PassThroughControlBoard::abortPark()
{
    if (!proxyIControlCalibration)
    {
        return ReturnValue::return_code::return_value_error_method_failed;
    }
    return proxyIControlCalibration->abortPark();
}

// CONTROL LIMITS
ReturnValue PassThroughControlBoard::setPosLimits(int axis, double min, double max)
{
    if (!proxyIControlLimits)
    {
        return ReturnValue::return_code::return_value_error_method_failed;
    }
    return proxyIControlLimits->setPosLimits(axis, min, max);
}

ReturnValue PassThroughControlBoard::getPosLimits(int axis, double* min, double* max)
{
    if (!proxyIControlLimits)
    {
        return ReturnValue::return_code::return_value_error_method_failed;
    }
    return proxyIControlLimits->getPosLimits(axis, min, max);
}
ReturnValue PassThroughControlBoard::setVelLimits(int axis, double min, double max)
{
    if (!proxyIControlLimits)
    {
        return ReturnValue::return_code::return_value_error_method_failed;
    }
    return proxyIControlLimits->setVelLimits(axis, min, max);
}
ReturnValue PassThroughControlBoard::getVelLimits(int axis, double* min, double* max)
{
    if (!proxyIControlLimits)
    {
        return ReturnValue::return_code::return_value_error_method_failed;
    }
    return proxyIControlLimits->getVelLimits(axis, min, max);
}

