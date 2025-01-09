/*
 *     Copyright 2022 Marvin Damschen   marvin.damschen@ri.se
 *               2022 Rickard Häll      rickard.hall@ri.se
 *     Published under GPLv3: https://www.gnu.org/licenses/gpl-3.0.html
 */

#ifndef MAVSDKVEHICLESERVER_H
#define MAVSDKVEHICLESERVER_H

#include <QObject>
#include <QSharedPointer>
#include <QTimer>
#include <QDateTime>
#include <QHostAddress>
#include <communication/vehicleserver.h>
#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action_server/action_server.h>
#include <mavsdk/plugins/mavlink_passthrough/mavlink_passthrough.h>
#include <mavsdk/plugins/mission_raw_server/mission_raw_server.h>
#include <mavsdk/plugins/telemetry_server/telemetry_server.h>
#include <mavsdk/server_component.h>
#include "communication/mavlinkparameterserver.h"
#include <mavsdk/plugins/mission_raw/mission_raw.h>

class MavsdkVehicleServer : public VehicleServer
{
    Q_OBJECT
public:
    explicit MavsdkVehicleServer(QSharedPointer<VehicleState> vehicleState, const QHostAddress controlTowerAddress = QHostAddress("127.0.0.1"),
                                 const unsigned controlTowerPort = 14540, const QAbstractSocket::SocketType controlTowerSocketType = QAbstractSocket::UdpSocket); // NOTE: currently, only UDP supported in mavsdk on vehicle side
    void setUbloxRover(QSharedPointer<UbloxRover> ubloxRover) override;
    void setWaypointFollower(QSharedPointer<WaypointFollower> waypointFollower) override;
    void setMovementController(QSharedPointer<MovementController> movementController) override;
    void setManualControlMaxSpeed(double manualControlMaxSpeed_ms) override;
    void setFollowPoint(QSharedPointer<FollowPoint> followPoint) override;
    double getManualControlMaxSpeed() const override;
    void sendGpsOriginLlh(const llh_t &gpsOriginLlh) override;
    void mavResult(const uint16_t command, MAV_RESULT result, MAV_COMPONENT compId);
    void on_logSent(const QString& message, const quint8& severity);
    void setRawGps(const mavsdk::TelemetryServer::RawGps rawGps) { mRawGps = rawGps; }
    void setGpsInfo(const mavsdk::TelemetryServer::GpsInfo gpsInfo) { mGpsInfo = gpsInfo; }

private:
    std::shared_ptr<mavsdk::Mavsdk> mMavsdk;
    std::shared_ptr<mavsdk::TelemetryServer> mTelemetryServer;
    mavsdk::TelemetryServer::RawGps mRawGps;
    mavsdk::TelemetryServer::GpsInfo mGpsInfo {0, mavsdk::TelemetryServer::FixType::NoGps};
    std::shared_ptr<mavsdk::ActionServer> mActionServer;
    std::shared_ptr<mavsdk::MissionRawServer> mMissionRawServer;
    std::shared_ptr<mavsdk::MavlinkPassthrough> mMavlinkPassthrough;
    QTimer mPublishMavlinkTimer;
    std::shared_ptr<mavsdk::Mavsdk> mTrailerMavsdk;
    std::shared_ptr<mavsdk::MavlinkPassthrough> mTrailerMavlinkPassthrough;

    QDateTime mMavsdkVehicleServerCreationTime = QDateTime::currentDateTime();
    ParameterServer *mParameterServer;

    const unsigned mCountdown_ms = 2000;

    void updateRawGpsAndGpsInfoFromUbx(const ubx_nav_pvt &pvt) override;
    void heartbeatTimeout() override;
    void heartbeatReset() override;
    PosPoint convertMissionItemToPosPoint(const mavsdk::MissionRawServer::MissionItem &item);
    void handleManualControlMessage(mavlink_manual_control_t manualControl);
    void sendMissionAck(quint8 type);
    double mManualControlMaxSpeed = 2.0; // [m/s]
    quint8 mSystemId = 1;
    void createMavsdkComponentForTrailer(const QHostAddress controlTowerAddress, const unsigned controlTowerPort, const QAbstractSocket::SocketType controlTowerSocketType);
};

#endif // MAVSDKVEHICLESERVER_H
