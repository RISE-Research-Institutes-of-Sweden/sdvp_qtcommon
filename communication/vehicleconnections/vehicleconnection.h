/*
 *     Copyright 2022 RISE Research Institutes of Sweden AB, Safety and Transport   waywise@ri.se
 *     Published under GPLv3: https://www.gnu.org/licenses/gpl-3.0.html
 *
 * Abstract class to communicate with a vehicle using a remote VehicleConnection
 */

#ifndef VEHICLECONNECTION_H
#define VEHICLECONNECTION_H

#include <QObject>
#include <QDebug>
#include "core/coordinatetransforms.h"
#include "vehicles/vehiclestate.h"
#include "sensors/camera/gimbal.h"
#include "autopilot/waypointfollower.h"
#include "communication/parameterserver.h"
#include "autopilot/followpoint.h"

class FollowPoint;

class VehicleConnection : public QObject
{
    Q_OBJECT
public:
    enum class SystemComponent {
        Autopilot,
        OnboardComputer,
    };
    enum class ComponentAction {
        DoNothing,
        Reboot,
        Shutdown,
        RebootComponentAndKeepItInTheBootloaderUntilUpgraded,
    };
    enum class Result {
        Unknown,
        Success,
        Timeout,
        ConnectionError,
        WrongType,
        ParamNameTooLong,
        NoSystem,
        ParamValueTooLong,
    };
    enum class Gearbox {
        lowGear = 1000,
        highGear = 2000,
    };

    virtual QList<PosPoint> requestCurrentRouteFromVehicle() = 0;
    virtual void requestGotoENU(const xyz_t &xyz, bool changeAutopilotMode = false) = 0;
    virtual void requestVelocityAndYaw(const xyz_t &velocityENU, const double &yawDeg) = 0;
    virtual void requestArm() = 0;
    virtual void requestDisarm() = 0;
    virtual void requestTakeoff() = 0;
    virtual void requestLanding() = 0;
    virtual void requestPrecisionLanding() = 0;
    virtual void requestReturnToHome() = 0;
    virtual void requestManualControl() = 0;
    virtual void requestFollowPoint() = 0;
    virtual void setManualControl(double x, double y, double z, double r, uint16_t buttonStateMask) = 0;
    virtual void setActuatorOutput(int index, float value) = 0;
    virtual void requestGearSwitch(int gpioPin, Gearbox gear) = 0;
    virtual bool requestRebootOrShutdownOfSystemComponents(SystemComponent systemComponent, ComponentAction componentAction) = 0;
    virtual Result setIntParameterOnVehicle(std::string name, int32_t value) = 0;
    virtual Result setFloatParameterOnVehicle(std::string, float value) = 0;
    virtual Result setCustomParameterOnVehicle(std::string name, std::string value) = 0;
    virtual std::pair<Result, int32_t> getIntParameterFromVehicle(std::string name) const = 0;
    virtual std::pair<Result, float> getFloatParameterFromVehicle(std::string name) const = 0;
    virtual std::pair<Result, std::string> getCustomParameterFromVehicle(std::string name) const = 0;
    virtual ParameterServer::AllParameters getAllParametersFromVehicle() = 0;
    virtual void pollCurrentENUreference() = 0;

    void setWaypointFollowerConnectionLocal(const QSharedPointer<WaypointFollower> &waypointFollower);
    bool hasWaypointFollowerConnectionLocal();
    void setFollowPointConnectionLocal(const QSharedPointer<FollowPoint> &followPoint);
    bool hasFollowPointConnectionLocal();
    bool isAutopilotActive();
    void restartAutopilot();
    void startAutopilot();
    void pauseAutopilot();
    void stopAutopilot();
    void clearRoute(int id = 0);
    void appendToRoute(const QList<PosPoint> &route, int id = 0);
    void setRoute(const QList<PosPoint> &route, int id = 0);
    void setActiveAutopilotID(int id = 0);
    void startFollowPoint();
    void stopFollowPoint();
    void updatePointToFollowInEnuFrame(const PosPoint &point);

    QSharedPointer<VehicleState> getVehicleState() const;
    QSharedPointer<Gimbal> getGimbal() const;
    bool hasGimbal() const;

signals:
    void detectedGimbal(QSharedPointer<Gimbal> gimbal);
    void updatedBatteryState(float voltage, float percentRemaining);

protected:
    // Implement these as protected/private, they are used within the respective functions without "OnVehicle" in their names
    virtual bool isAutopilotActiveOnVehicle() {throw  std::logic_error("Function not implemented");};
    virtual void restartAutopilotOnVehicle() {throw  std::logic_error("Function not implemented");};
    virtual void startAutopilotOnVehicle() {throw  std::logic_error("Function not implemented");};
    virtual void pauseAutopilotOnVehicle() {throw  std::logic_error("Function not implemented");};
    virtual void stopAutopilotOnVehicle() {throw  std::logic_error("Function not implemented");};
    virtual void clearRouteOnVehicle(int id = 0) {Q_UNUSED(id) throw  std::logic_error("Function not implemented");};
    virtual void appendToRouteOnVehicle(const QList<PosPoint> &route, int id = 0) {Q_UNUSED(route )Q_UNUSED(id) throw  std::logic_error("Function not implemented");};
    virtual void setActiveAutopilotIDOnVehicle(int id = 0) {Q_UNUSED(id) throw  std::logic_error("Function not implemented");};
    virtual void startFollowPointOnVehicle() {throw  std::logic_error("Function not implemented");};
    virtual void stopFollowPointOnVehicle() {throw  std::logic_error("Function not implemented");};

    QSharedPointer<VehicleState> mVehicleState;
    QSharedPointer<Gimbal> mGimbal;
    QSharedPointer<WaypointFollower> mWaypointFollower;
    QSharedPointer<FollowPoint> mFollowPoint;

};

#endif // VEHICLECONNECTION_H
