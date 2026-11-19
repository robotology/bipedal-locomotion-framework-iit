/**
 * @file PassThroughControlBoard.h
 * @authors Ines Sorrentino
 * @copyright 2024 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef BIPEDAL_LOCOMOTION_FRAMEWORK_PASS_THROUGH_CONTROL_BOARD_H
#define BIPEDAL_LOCOMOTION_FRAMEWORK_PASS_THROUGH_CONTROL_BOARD_H

#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/IAmplifierControl.h>
#include <yarp/dev/IControlCalibration.h>
#include <yarp/dev/IControlLimits.h>
#include <yarp/dev/IControlMode.h>
#include <yarp/dev/ICurrentControl.h>
#include <yarp/dev/IEncoders.h>
#include <yarp/dev/IEncodersTimed.h>
#include <yarp/dev/IInteractionMode.h>
#include <yarp/dev/IMotorEncoders.h>
#include <yarp/dev/IMultipleWrapper.h>
#include <yarp/dev/IPWMControl.h>
#include <yarp/dev/IPositionControl.h>
#include <yarp/dev/IPositionDirect.h>
#include <yarp/dev/ITorqueControl.h>
#include <yarp/dev/IVelocityControl.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/IJointFault.h>
#include <yarp/dev/IMotor.h>

#include <vector>

#include <BipedalLocomotion/TextLogging/Logger.h>

namespace BipedalLocomotion
{
class PassThroughControlBoard;
}

/**
 *
 * \brief PassThroughControlBoard: base class for devices can be used to modify the behavior of a
 * controlboard device.
 *
 */
class BipedalLocomotion::PassThroughControlBoard : public yarp::dev::DeviceDriver,
                                                   public yarp::dev::IEncodersTimed,
                                                   public yarp::dev::IPositionControl,
                                                   public yarp::dev::IVelocityControl,
                                                   public yarp::dev::IPositionDirect,
                                                   public yarp::dev::IControlMode,
                                                   public yarp::dev::ITorqueControl,
                                                   public yarp::dev::IPWMControl,
                                                   public yarp::dev::IInteractionMode,
                                                   public yarp::dev::IPidControl,
                                                   public yarp::dev::ICurrentControl,
                                                   public yarp::dev::IMotorEncoders,
                                                   public yarp::dev::IAxisInfo,
                                                   public yarp::dev::IMultipleWrapper,
                                                   public yarp::dev::IAmplifierControl,
                                                   public yarp::dev::IControlCalibration,
                                                   public yarp::dev::IControlLimits,
                                                   public yarp::dev::IJointFault,
                                                   public yarp::dev::IMotor
{
protected:
    yarp::dev::IEncodersTimed* proxyIEncodersTimed;
    yarp::dev::IPositionControl* proxyIPositionControl;
    yarp::dev::IVelocityControl* proxyIVelocityControl;
    yarp::dev::IPositionDirect* proxyIPositionDirect;
    yarp::dev::IControlMode* proxyIControlMode;
    yarp::dev::ITorqueControl* proxyITorqueControl;
    yarp::dev::IPWMControl* proxyIOpenLoopControl;
    yarp::dev::IInteractionMode* proxyIInteractionMode;
    yarp::dev::IPidControl* proxyIPidControl;
    yarp::dev::ICurrentControl* proxyICurrentControl;
    yarp::dev::IMotorEncoders* proxyIMotorEncoders;
    yarp::dev::IAxisInfo* proxyIAxisInfo;
    yarp::dev::IAmplifierControl* proxyIAmplifierControl;
    yarp::dev::IControlCalibration* proxyIControlCalibration;
    yarp::dev::IControlLimits* proxyIControlLimits;
    yarp::dev::IMotor* proxyIMotor;
    yarp::dev::IJointFault* proxyIJointFault;
    void proxyIMotorEncoder(const double* vals);
    void resetPointers();

public:
    // CONSTRUCTOR
    PassThroughControlBoard();
    ~PassThroughControlBoard();

    // DEVICE DRIVER
    virtual bool open(yarp::os::Searchable& config);
    virtual bool close();

    // IMULTIPLEWRAPPER
    virtual bool attachAll(const yarp::dev::PolyDriverList& p);
    virtual bool detachAll();

    // GEARBOX RATIO
    virtual yarp::dev::ReturnValue getGearboxRatio(int m, double* val);
    virtual yarp::dev::ReturnValue setGearboxRatio(int m, const double val);

    // JOINT FAULT
    virtual yarp::dev::ReturnValue getLastJointFault(int j, int& fault, std::string& message);

    // ENCODERS
    virtual yarp::dev::ReturnValue getAxes(std::size_t& ax);
    virtual yarp::dev::ReturnValue resetEncoder(int j);
    virtual yarp::dev::ReturnValue resetEncoders();
    virtual yarp::dev::ReturnValue setEncoder(int j, double val);
    virtual yarp::dev::ReturnValue setEncoders(const double* vals);
    virtual yarp::dev::ReturnValue getEncoder(int j, double* v);
    virtual yarp::dev::ReturnValue getEncoders(double* encs);
    virtual yarp::dev::ReturnValue getEncoderSpeed(int j, double* sp);
    virtual yarp::dev::ReturnValue getEncoderSpeeds(double* spds);
    virtual yarp::dev::ReturnValue getEncoderAcceleration(int j, double* spds);
    virtual yarp::dev::ReturnValue getEncoderAccelerations(double* accs);

    // ENCODERS TIMED
    virtual yarp::dev::ReturnValue getEncodersTimed(double* encs, double* time);
    virtual yarp::dev::ReturnValue getEncoderTimed(int j, double* encs, double* time);

    // MOTOR ENCODERS
    virtual yarp::dev::ReturnValue getNumberOfMotorEncoders(int* num);
    virtual yarp::dev::ReturnValue resetMotorEncoder(int m);
    virtual yarp::dev::ReturnValue resetMotorEncoders();
    virtual yarp::dev::ReturnValue setMotorEncoderCountsPerRevolution(int m, const double cpr);
    virtual yarp::dev::ReturnValue getMotorEncoderCountsPerRevolution(int m, double* cpr);
    virtual yarp::dev::ReturnValue setMotorEncoder(int m, const double val);
    virtual yarp::dev::ReturnValue setMotorEncoders(const double* vals);
    virtual yarp::dev::ReturnValue getMotorEncoder(int m, double* v);
    virtual yarp::dev::ReturnValue getMotorEncoders(double* encs);
    virtual yarp::dev::ReturnValue getMotorEncodersTimed(double* encs, double* time);
    virtual yarp::dev::ReturnValue getMotorEncoderTimed(int m, double* encs, double* time);
    virtual yarp::dev::ReturnValue getMotorEncoderSpeed(int m, double* sp);
    virtual yarp::dev::ReturnValue getMotorEncoderSpeeds(double* spds);
    virtual yarp::dev::ReturnValue getMotorEncoderAcceleration(int m, double* acc);
    virtual yarp::dev::ReturnValue getMotorEncoderAccelerations(double* accs);
    virtual yarp::dev::ReturnValue getTemperatures(double *vals);
    virtual yarp::dev::ReturnValue getTemperature(int m, double* val);
    virtual yarp::dev::ReturnValue getTemperatureLimit(int m, double* temp);
    virtual yarp::dev::ReturnValue setTemperatureLimit(int m, const double temp);

    // POSITION CONTROL
    virtual yarp::dev::ReturnValue positionMove(int j, double ref);
    virtual yarp::dev::ReturnValue positionMove(const double* refs);
    virtual yarp::dev::ReturnValue relativeMove(int j, double delta);
    virtual yarp::dev::ReturnValue relativeMove(const double* deltas);
    virtual yarp::dev::ReturnValue checkMotionDone(int j, bool& flag);
    virtual yarp::dev::ReturnValue checkMotionDone(bool& flag);
    virtual yarp::dev::ReturnValue setTrajSpeed(int j, double sp);
    virtual yarp::dev::ReturnValue setTrajSpeeds(const double* spds);
    virtual yarp::dev::ReturnValue setTrajAcceleration(int j, double acc);
    virtual yarp::dev::ReturnValue setTrajAccelerations(const double* accs);
    virtual yarp::dev::ReturnValue getTrajSpeed(int j, double* ref);
    virtual yarp::dev::ReturnValue getTrajSpeeds(double* spds);
    virtual yarp::dev::ReturnValue getTrajAcceleration(int j, double* acc);
    virtual yarp::dev::ReturnValue getTrajAccelerations(double* accs);
    virtual yarp::dev::ReturnValue stop(int j);
    virtual yarp::dev::ReturnValue stop();

    // POSITION CONTROL
    virtual yarp::dev::ReturnValue positionMove(const int n_joint, const int* joints, const double* refs);
    virtual yarp::dev::ReturnValue relativeMove(const int n_joint, const int* joints, const double* deltas);
    virtual yarp::dev::ReturnValue checkMotionDone(const std::vector<int>& joints, bool& flag);
    virtual yarp::dev::ReturnValue setTrajSpeeds(const int n_joint, const int* joints, const double* spds);
    virtual yarp::dev::ReturnValue setTrajAccelerations(const int n_joint, const int* joints, const double* accs);
    virtual yarp::dev::ReturnValue getTrajSpeeds(const int n_joint, const int* joints, double* spds);
    virtual yarp::dev::ReturnValue getTrajAccelerations(const int n_joint, const int* joints, double* accs);
    virtual yarp::dev::ReturnValue stop(const int n_joint, const int* joints);
    virtual yarp::dev::ReturnValue getTargetPosition(const int joint, double* ref);
    virtual yarp::dev::ReturnValue getTargetPositions(double* refs);
    virtual yarp::dev::ReturnValue getTargetPositions(const int n_joint, const int* joints, double* refs);

    // VELOCITY CONTROL
    virtual yarp::dev::ReturnValue velocityMove(int j, double sp);
    virtual yarp::dev::ReturnValue velocityMove(const double* sp);

    // VELOCITY CONTROL
    virtual yarp::dev::ReturnValue velocityMove(const int n_joint, const int* joints, const double* spds);
    virtual yarp::dev::ReturnValue getTargetVelocity(const int joint, double* vel);
    virtual yarp::dev::ReturnValue getTargetVelocities(double* vels);
    virtual yarp::dev::ReturnValue getTargetVelocities(const int n_joint, const int* joints, double* vels);

    // POSITION DIRECT CONTROL
    virtual yarp::dev::ReturnValue setPosition(int j, double ref);
    virtual yarp::dev::ReturnValue setPositions(const int n_joint, const int* joints, const double* refs);
    virtual yarp::dev::ReturnValue setPositions(const double* refs);
    virtual yarp::dev::ReturnValue getRefPosition(const int joint, double* ref);
    virtual yarp::dev::ReturnValue getRefPositions(double* refs);
    virtual yarp::dev::ReturnValue getRefPositions(const int n_joint, const int* joints, double* refs);

    // CONTROL MODE
    virtual yarp::dev::ReturnValue
    getAvailableControlModes(int j, std::vector<yarp::dev::SelectableControlModeEnum>& avail);
    virtual yarp::dev::ReturnValue getControlMode(int j, yarp::dev::ControlModeEnum& mode);
    virtual yarp::dev::ReturnValue getControlModes(std::vector<yarp::dev::ControlModeEnum>& mode);
    virtual yarp::dev::ReturnValue
    getControlModes(const std::vector<int>& joints, std::vector<yarp::dev::ControlModeEnum>& mode);
    virtual yarp::dev::ReturnValue
    setControlMode(int j, yarp::dev::SelectableControlModeEnum mode);
    virtual yarp::dev::ReturnValue setControlModes(
        const std::vector<int>& joints,
        const std::vector<yarp::dev::SelectableControlModeEnum>& mode);
    virtual yarp::dev::ReturnValue
    setControlModes(const std::vector<yarp::dev::SelectableControlModeEnum>& mode);

    // TORQUE CONTROL
    virtual yarp::dev::ReturnValue getRefTorques(double* t);
    virtual yarp::dev::ReturnValue getRefTorque(int j, double* t);
    virtual yarp::dev::ReturnValue setRefTorques(const double* t);
    virtual yarp::dev::ReturnValue
    setRefTorques(const int n_joint, const int* joints, const double* t);
    virtual yarp::dev::ReturnValue setRefTorque(int j, double t);
    virtual yarp::dev::ReturnValue
    getMotorTorqueParams(int j, yarp::dev::MotorTorqueParameters* params);
    virtual yarp::dev::ReturnValue
    setMotorTorqueParams(int j, const yarp::dev::MotorTorqueParameters params);
    virtual yarp::dev::ReturnValue getTorque(int j, double* t);
    virtual yarp::dev::ReturnValue getTorques(double* t);
    virtual yarp::dev::ReturnValue getTorqueRange(int j, double* min, double* max);
    virtual yarp::dev::ReturnValue getTorqueRanges(double* min, double* max);

    // OPEN LOOP CONTROL
    virtual yarp::dev::ReturnValue getNumberOfMotors(int* n);
    virtual yarp::dev::ReturnValue setRefDutyCycle(int m, double ref);
    virtual yarp::dev::ReturnValue setRefDutyCycles(const double* refs);
    virtual yarp::dev::ReturnValue getRefDutyCycle(int m, double* ref);
    virtual yarp::dev::ReturnValue getRefDutyCycles(double* refs);
    virtual yarp::dev::ReturnValue getDutyCycle(int m, double* val);
    virtual yarp::dev::ReturnValue getDutyCycles(double* vals);

    // CURRENT CONTROL
    virtual yarp::dev::ReturnValue getCurrent(int m, double* curr);
    virtual yarp::dev::ReturnValue getCurrents(double* currs);
    virtual yarp::dev::ReturnValue getCurrentRange(int m, double* min, double* max);
    virtual yarp::dev::ReturnValue getCurrentRanges(double* min, double* max);
    virtual yarp::dev::ReturnValue setRefCurrents(const double* currs);
    virtual yarp::dev::ReturnValue setRefCurrent(int m, double curr);
    virtual yarp::dev::ReturnValue setRefCurrents(const int n_motor, const int* motors, const double* currs);
    virtual yarp::dev::ReturnValue getRefCurrents(double* currs);
    virtual yarp::dev::ReturnValue getRefCurrent(int m, double* curr);

    // AMPLIFIER CONTROL
    virtual yarp::dev::ReturnValue enableAmp(int j);
    virtual yarp::dev::ReturnValue disableAmp(int j);
    virtual yarp::dev::ReturnValue getAmpStatus(int* st);
    virtual yarp::dev::ReturnValue getAmpStatus(int j, int* v);
    virtual yarp::dev::ReturnValue getMaxCurrent(int j, double* v);
    virtual yarp::dev::ReturnValue setMaxCurrent(int j, double v);
    virtual yarp::dev::ReturnValue getNominalCurrent(int m, double* val);
    virtual yarp::dev::ReturnValue setNominalCurrent(int m, const double val);
    virtual yarp::dev::ReturnValue getPeakCurrent(int m, double* val);
    virtual yarp::dev::ReturnValue setPeakCurrent(int m, const double val);
    virtual yarp::dev::ReturnValue getPWM(int j, double* val);
    virtual yarp::dev::ReturnValue getPWMLimit(int j, double* val);
    virtual yarp::dev::ReturnValue setPWMLimit(int j, const double val);
    virtual yarp::dev::ReturnValue getPowerSupplyVoltage(int j, double* val);

    // INTERACTION MODE
    virtual yarp::dev::ReturnValue getInteractionMode(int axis, yarp::dev::InteractionModeEnum& mode);
    virtual yarp::dev::ReturnValue
    getInteractionModes(const std::vector<int>& joints, std::vector<yarp::dev::InteractionModeEnum>& modes);
    virtual yarp::dev::ReturnValue getInteractionModes(std::vector<yarp::dev::InteractionModeEnum>& modes);
    virtual yarp::dev::ReturnValue setInteractionMode(int axis, yarp::dev::InteractionModeEnum mode);
    virtual yarp::dev::ReturnValue
    setInteractionModes(const std::vector<int>& joints, const std::vector<yarp::dev::InteractionModeEnum>& modes);
    virtual yarp::dev::ReturnValue setInteractionModes(const std::vector<yarp::dev::InteractionModeEnum>& modes);

    // PID CONTROL INTERFACE METHODS
    virtual yarp::dev::ReturnValue
    getAvailablePids(int j, std::vector<yarp::dev::PidControlTypeEnum>& avail);
    virtual yarp::dev::ReturnValue
    setPid(const yarp::dev::PidControlTypeEnum& pidtype, int j, const yarp::dev::Pid& pid);
    virtual yarp::dev::ReturnValue setPids(const yarp::dev::PidControlTypeEnum& pidtype, const yarp::dev::Pid* pids);
    virtual yarp::dev::ReturnValue setPidReference(const yarp::dev::PidControlTypeEnum& pidtype, int j, double ref);
    virtual yarp::dev::ReturnValue setPidReferences(const yarp::dev::PidControlTypeEnum& pidtype, const double* refs);
    virtual yarp::dev::ReturnValue
    setPidErrorLimit(const yarp::dev::PidControlTypeEnum& pidtype, int j, double limit);
    virtual yarp::dev::ReturnValue
    setPidErrorLimits(const yarp::dev::PidControlTypeEnum& pidtype, const double* limits);
    virtual yarp::dev::ReturnValue getPidError(const yarp::dev::PidControlTypeEnum& pidtype, int j, double* err);
    virtual yarp::dev::ReturnValue getPidErrors(const yarp::dev::PidControlTypeEnum& pidtype, double* errs);
    virtual yarp::dev::ReturnValue getPidOutput(const yarp::dev::PidControlTypeEnum& pidtype, int j, double* out);
    virtual yarp::dev::ReturnValue getPidOutputs(const yarp::dev::PidControlTypeEnum& pidtype, double* outs);
    virtual yarp::dev::ReturnValue getPid(const yarp::dev::PidControlTypeEnum& pidtype, int j, yarp::dev::Pid* pid);
    virtual yarp::dev::ReturnValue getPids(const yarp::dev::PidControlTypeEnum& pidtype, yarp::dev::Pid* pids);
    virtual yarp::dev::ReturnValue
    getPidExtraInfo(const yarp::dev::PidControlTypeEnum& pidtype, int j, yarp::dev::PidExtraInfo& info);
    virtual yarp::dev::ReturnValue
    getPidExtraInfos(const yarp::dev::PidControlTypeEnum& pidtype,
                     std::vector<yarp::dev::PidExtraInfo>& info);
    virtual yarp::dev::ReturnValue getPidReference(const yarp::dev::PidControlTypeEnum& pidtype, int j, double* ref);
    virtual yarp::dev::ReturnValue getPidReferences(const yarp::dev::PidControlTypeEnum& pidtype, double* refs);
    virtual yarp::dev::ReturnValue
    getPidErrorLimit(const yarp::dev::PidControlTypeEnum& pidtype, int j, double* limit);
    virtual yarp::dev::ReturnValue getPidErrorLimits(const yarp::dev::PidControlTypeEnum& pidtype, double* limits);
    virtual yarp::dev::ReturnValue resetPid(const yarp::dev::PidControlTypeEnum& pidtype, int j);
    virtual yarp::dev::ReturnValue disablePid(const yarp::dev::PidControlTypeEnum& pidtype, int j);
    virtual yarp::dev::ReturnValue enablePid(const yarp::dev::PidControlTypeEnum& pidtype, int j);
    virtual yarp::dev::ReturnValue setPidOffset(const yarp::dev::PidControlTypeEnum& pidtype, int j, double v);
    virtual yarp::dev::ReturnValue
    setPidFeedforward(const yarp::dev::PidControlTypeEnum& pidtype, int j, double v);
    virtual yarp::dev::ReturnValue
    getPidOffset(const yarp::dev::PidControlTypeEnum& pidtype, int j, double& v);
    virtual yarp::dev::ReturnValue
    getPidFeedforward(const yarp::dev::PidControlTypeEnum& pidtype, int j, double& v);
    virtual yarp::dev::ReturnValue
    isPidEnabled(const yarp::dev::PidControlTypeEnum& pidtype, int j, bool& enabled);

    // AXIS INFO
    virtual yarp::dev::ReturnValue getAxisName(int axis, std::string& name);
    virtual yarp::dev::ReturnValue getJointType(int axis, yarp::dev::JointTypeEnum& type);

    // CONTROL CALIBRATION
    virtual yarp::dev::ReturnValue
    calibrateAxisWithParams(int axis, unsigned int type, double p1, double p2, double p3);
    virtual yarp::dev::ReturnValue setCalibrationParameters(int axis, const yarp::dev::CalibrationParameters& params);
    virtual yarp::dev::ReturnValue calibrationDone(int j);
    virtual yarp::dev::ReturnValue setCalibrator(yarp::dev::ICalibrator* c);
    virtual yarp::dev::ReturnValue calibrateRobot();
    virtual yarp::dev::ReturnValue park(bool wait = true);
    virtual yarp::dev::ReturnValue abortCalibration();
    virtual yarp::dev::ReturnValue abortPark();

    // CONTROL LIMITS
    virtual yarp::dev::ReturnValue setPosLimits(int axis, double min, double max);
    virtual yarp::dev::ReturnValue getPosLimits(int axis, double* min, double* max);
    virtual yarp::dev::ReturnValue setVelLimits(int axis, double min, double max);
    virtual yarp::dev::ReturnValue getVelLimits(int axis, double* min, double* max);
};

#endif /* CODYCO_PASS_THROUGHT_CONTROL_BOARD_H */
