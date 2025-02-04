/*
 *     Copyright 2022 RISE Research Institutes of Sweden AB, Safety and Transport   waywise@ri.se
 *     Published under GPLv3: https://www.gnu.org/licenses/gpl-3.0.html
 *
 * Implementation of VehicleConnection to communicate with MAVLINK-based vehicles using MAVSDK
 * NOTE: Only multicopters supported for now.
 */

#ifndef MAVSDKVEHICLECONNECTION_H
#define MAVSDKVEHICLECONNECTION_H

#include <QSharedPointer>
#include "waywise.h"
#include "vehicleconnection.h"
#include "vehicles/vehiclestate.h"
#include "core/coordinatetransforms.h"
#include "autopilot/gotowaypointfollower.h"
//#include "autopilot/purepursuitwaypointfollower.h"
#include "vehicles/copterstate.h"
#include "vehicles/carstate.h"
#include "vehicles/truckstate.h"
#include "sensors/camera/mavsdkgimbal.h"
#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/param/param.h>
#include <mavsdk/plugins/mavlink_passthrough/mavlink_passthrough.h>
#include <mavsdk/plugins/offboard/offboard.h>
#include <mavsdk/plugins/mission_raw/mission_raw.h>
#include <mavsdk/plugins/info/info.h>

class MavsdkVehicleConnection : public VehicleConnection
{
    Q_OBJECT
public:
    explicit MavsdkVehicleConnection(std::shared_ptr<mavsdk::System> system, MAV_TYPE vehicleType);
    ~MavsdkVehicleConnection() { mSystem->unsubscribe_component_discovered(mComponentDiscoveredHandle); };
    void setEnuReference(const llh_t &enuReference);
    void setHomeLlh(const llh_t &homeLlh);
    virtual QList<PosPoint> requestCurrentRouteFromVehicle() override;
    virtual void requestArm() override;
    virtual void requestDisarm() override;
    virtual void requestTakeoff() override;
    virtual void requestLanding() override;
    virtual void requestPrecisionLanding() override;
    virtual void requestReturnToHome() override;
    virtual void requestManualControl() override;
    virtual void requestFollowPoint() override;
    void requestGotoLlh(const llh_t &llh, bool changeFlightmodeToHold = false);
    virtual void requestGotoENU(const xyz_t &xyz, bool changeFlightmodeToHold = false) override;
    virtual void requestVelocityAndYaw(const xyz_t &velocityENU, const double &yawDeg) override;
    void inputRtcmData(const QByteArray &rtcmData);
    void sendLandingTargetLlh(const llh_t &landingTargetLlh);
    void sendLandingTargetENU(const xyz_t &landingTargetENU);
    void sendSetGpsOriginLlh(const llh_t &gpsOriginLlh);
    virtual void setActuatorOutput(int index, float value) override;
    virtual void setManualControl(double x, double y, double z, double r, uint16_t buttonStateMask) override;
    virtual void requestGearSwitch(int gpioPin, VehicleConnection::Gearbox gear) override;
    virtual bool requestRebootOrShutdownOfSystemComponents(VehicleConnection::SystemComponent systemComponent, VehicleConnection::ComponentAction componentAction) override;
    virtual VehicleConnection::Result setIntParameterOnVehicle(std::string name, int32_t value) override;
    virtual VehicleConnection::Result setFloatParameterOnVehicle(std::string, float value) override;
    virtual VehicleConnection::Result setCustomParameterOnVehicle(std::string name, std::string value) override;
    virtual std::pair<VehicleConnection::Result, int32_t> getIntParameterFromVehicle(std::string name) const override;
    virtual std::pair<VehicleConnection::Result, float> getFloatParameterFromVehicle(std::string name) const override;
    virtual std::pair<VehicleConnection::Result, std::string> getCustomParameterFromVehicle(std::string name) const override;
    virtual ParameterServer::AllParameters getAllParametersFromVehicle() override;
    virtual void pollCurrentENUreference() override;

    void setConvertLocalPositionsToGlobalBeforeSending(bool convertLocalPositionsToGlobalBeforeSending);

    MAV_TYPE getVehicleType() const;

signals:
    void gotVehicleENUreferenceLlh(const llh_t &enuReferenceLlh);
    void gotVehicleHomeLlh(const llh_t &homePositionLlh);
    void stopWaypointFollowerSignal(); // Used internally from MAVSDK callbacks (that live in other threads)
    void gotHeartbeat(const quint8 systemId);

private:
    MAV_TYPE mVehicleType;
    llh_t mEnuReference;
    llh_t mGpsGlobalOrigin; // reference for on-vehicle EKF (origin in NED, ENU frames on vehicle), polled once at startup
                            // do not use unless you really know what you are doing!
                            // Use mEnuReference instead.
    bool mConvertLocalPositionsToGlobalBeforeSending = false;
    std::shared_ptr<mavsdk::System> mSystem;
    mavsdk::System::ComponentDiscoveredHandle mComponentDiscoveredHandle;
    std::shared_ptr<mavsdk::Telemetry> mTelemetry;
    std::shared_ptr<mavsdk::Action> mAction;
    std::shared_ptr<mavsdk::Param> mParam;
    std::shared_ptr<mavsdk::MavlinkPassthrough> mMavlinkPassthrough;
    std::shared_ptr<mavsdk::Offboard> mOffboard;
    std::shared_ptr<mavsdk::MissionRaw> mMissionRaw;
    QSharedPointer<QTimer> mPosTimer;

    mavsdk::MissionRaw::MissionItem convertPosPointToMissionItem(const PosPoint& posPoint, int sequenceId, bool current = false);
    VehicleConnection::Result convertParamResult(mavsdk::Param::Result result) const;
    QString convertMissionRawResult(mavsdk::MissionRaw::Result result) const;
    QString convertMavlinkPassthroughResult(mavsdk::MavlinkPassthrough::Result result) const;
    void setupCarState(QSharedPointer<CarState> carState);
    void setupTruckState(QSharedPointer<TruckState> truckState);

    // VehicleConnection interface
protected:
    virtual bool isAutopilotActiveOnVehicle() override;
    virtual void restartAutopilotOnVehicle() override;
    virtual void startAutopilotOnVehicle() override;
    virtual void pauseAutopilotOnVehicle() override;
    virtual void stopAutopilotOnVehicle() override;
    virtual void clearRouteOnVehicle(int id) override;
    virtual void appendToRouteOnVehicle(const QList<PosPoint> &route, int id) override;
    virtual void setActiveAutopilotIDOnVehicle(int id) override;
    virtual void startFollowPointOnVehicle() override;
    virtual void stopFollowPointOnVehicle() override;
};

#endif // MAVSDKVEHICLECONNECTION_H
