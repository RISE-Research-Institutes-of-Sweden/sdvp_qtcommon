/*
 *     Copyright 2022 Marvin Damschen   marvin.damschen@ri.se
 *               2022 Rickard Häll      rickard.hall@ri.se
 *     Published under GPLv3: https://www.gnu.org/licenses/gpl-3.0.html
 */
#include "mavsdkvehicleconnection.h"
#include <QDebug>
#include <QDateTime>

MavsdkVehicleConnection::MavsdkVehicleConnection(std::shared_ptr<mavsdk::System> system, MAV_TYPE vehicleType)
{
    mSystem = system;
    mVehicleType = vehicleType;
    mMavlinkPassthrough.reset(new mavsdk::MavlinkPassthrough(system));

    // Setup gimbal
    mSystem->subscribe_component_discovered([this](mavsdk::System::ComponentType){
        if (mSystem->has_gimbal() && mGimbal.isNull()) {
            mGimbal = QSharedPointer<MavsdkGimbal>::create(mSystem);
            emit detectedGimbal(mGimbal);
        }
    });
    if (mSystem->has_gimbal() && mGimbal.isNull()) // Gimbal might have been discovered before callback was registered
        mGimbal = QSharedPointer<MavsdkGimbal>::create(mSystem);

    switch (mVehicleType) {
    case MAV_TYPE_QUADROTOR:
        qDebug() << "MavsdkVehicleConnection: we are talking to a MAV_TYPE_QUADROTOR / PX4.";
        mVehicleState = QSharedPointer<CopterState>::create(mSystem->get_system_id());
        mVehicleState->setName("Copter " + QString::number(mSystem->get_system_id()));
        break;
    case MAV_TYPE_GROUND_ROVER:
        qDebug() << "MavsdkVehicleConnection: we are talking to a MAV_TYPE_GROUND_ROVER / WayWise.";
        // TODO: detect different rover types...
        mVehicleState = QSharedPointer<CarState>::create(mSystem->get_system_id());
        mVehicleState->setName("Car " + QString::number(mSystem->get_system_id()));
        break;
    default:
        qDebug() << "MavsdkVehicleConnection: unknown / unsupported vehicle type.";
        break;
    }

    // Set up telemetry plugin
    mTelemetry.reset(new mavsdk::Telemetry(mSystem));

    mTelemetry->subscribe_battery([this](mavsdk::Telemetry::Battery battery) {
       emit updatedBatteryState(battery.voltage_v, battery.remaining_percent);
    });

    mTelemetry->subscribe_armed([this](bool isArmed) {
       mVehicleState->setIsArmed(isArmed);
    });

    mTelemetry->subscribe_home([this](mavsdk::Telemetry::Position position) {
        llh_t llh = {position.latitude_deg, position.longitude_deg, position.absolute_altitude_m};
        xyz_t xyz = coordinateTransforms::llhToEnu(mEnuReference, llh);

        auto homePos = mVehicleState->getHomePosition();
        homePos.setX(xyz.x);
        homePos.setY(xyz.y);
        homePos.setHeight(xyz.z);
        mVehicleState->setHomePosition(homePos);

        emit gotVehicleHomeLlh({position.latitude_deg, position.longitude_deg, position.absolute_altitude_m});
    });

    if (mVehicleType == MAV_TYPE::MAV_TYPE_GROUND_ROVER) // assumption: rover = WayWise on vehicle side -> get NED (shared ENU ref), global pos otherwise
        mTelemetry->subscribe_position_velocity_ned([this](mavsdk::Telemetry::PositionVelocityNed positionVelocity_ned) {
            auto pos = mVehicleState->getPosition();

            xyz_t positionNED = {positionVelocity_ned.position.north_m, positionVelocity_ned.position.east_m, positionVelocity_ned.position.down_m};
            pos.setXYZ(coordinateTransforms::nedToENU(positionNED));

            mVehicleState->setPosition(pos);
        });
    else
        mTelemetry->subscribe_position([this](mavsdk::Telemetry::Position position) {
            llh_t llh = {position.latitude_deg, position.longitude_deg, position.absolute_altitude_m};
            xyz_t xyz = coordinateTransforms::llhToEnu(mEnuReference, llh);

            auto pos = mVehicleState->getPosition();
            pos.setX(xyz.x);
            pos.setY(xyz.y);
            pos.setHeight(xyz.z);

            mVehicleState->setPosition(pos);
        });

    mTelemetry->subscribe_heading([this](mavsdk::Telemetry::Heading heading) {
        auto pos = mVehicleState->getPosition();

        pos.setYaw(coordinateTransforms::yawNEDtoENU(heading.heading_deg));

        mVehicleState->setPosition(pos);
    });

    mTelemetry->subscribe_velocity_ned([this](mavsdk::Telemetry::VelocityNed velocity) {
        VehicleState::Velocity velocityCopter {velocity.east_m_s, velocity.north_m_s, -velocity.down_m_s};
        mVehicleState->setVelocity(velocityCopter);
    });

    if (mVehicleType == MAV_TYPE::MAV_TYPE_QUADROTOR) {
        mTelemetry->subscribe_landed_state([this](mavsdk::Telemetry::LandedState landedState) {
           mVehicleState.dynamicCast<CopterState>()->setLandedState(static_cast<CopterState::LandedState>(landedState));
        });
    }

    mTelemetry->subscribe_flight_mode([this](mavsdk::Telemetry::FlightMode flightMode) {
        mVehicleState->setFlightMode(static_cast<CopterState::FlightMode>(flightMode));

        if (flightMode != mavsdk::Telemetry::FlightMode::Offboard &&
                flightMode != mavsdk::Telemetry::FlightMode::Hold)
            if (hasWaypointFollowerConnectionLocal() && isAutopilotActive()) {
                emit stopWaypointFollowerSignal();
                qDebug() << "MavsdkVehicleConnection: connection-local WaypointFollower stopped by flightmode change (Note: can only be started in hold mode).";
            }

//            qDebug() << (int)flightMode << (mOffboard ? mOffboard->is_active() : false);
    });

    // poll update of GpsGlobalOrigin once
    mTelemetry->get_gps_global_origin_async([this](mavsdk::Telemetry::Result result, mavsdk::Telemetry::GpsGlobalOrigin gpsGlobalOrigin){
        if (result == mavsdk::Telemetry::Result::Success){
            mGpsGlobalOrigin = {gpsGlobalOrigin.latitude_deg, gpsGlobalOrigin.longitude_deg, gpsGlobalOrigin.altitude_m};
            emit gotVehicleGpsOriginLlh(mGpsGlobalOrigin);
        }
    });

    // Adaptive pure pursuit radius
    mMavlinkPassthrough->subscribe_message(MAVLINK_MSG_ID_NAMED_VALUE_FLOAT, [this](const mavlink_message_t &message) {
        mavlink_named_value_float_t mavMsg;
        mavlink_msg_named_value_float_decode(&message, &mavMsg);
        if (strcmp(mavMsg.name,"AR") == 0) {
            mavlink_msg_named_value_float_decode(&message, &mavMsg);
            mVehicleState->setAutopilotRadius(mavMsg.value);
        }
    });

    // Set up action plugin
    mAction.reset(new mavsdk::Action(mSystem));

    // Set up param plugin
    mParam.reset(new mavsdk::Param(mSystem));

// TODO: this should not happen here (blocking)
//    // Precision Landing: set required target tracking accuracy for starting approach
//    if (mParam->set_param_float("PLD_HACC_RAD", 5.0) != mavsdk::Param::Result::Success)
//        qDebug() << "Warning: failed to set PLD_HACC_RAD";

    // Necessary such that MAVSDK callbacks (from other threads) can stop WaypointFollower (QTimer)
    connect(this, &MavsdkVehicleConnection::stopWaypointFollowerSignal, this, &MavsdkVehicleConnection::stopAutopilot);
}

void MavsdkVehicleConnection::setEnuReference(const llh_t &enuReference)
{
    mEnuReference = enuReference;
}

void MavsdkVehicleConnection::setHomeLlh(const llh_t &homeLlh)
{
    // Set new home in llh via MAVLINK
    mavsdk::MavlinkPassthrough::CommandLong ComLong;
    ComLong.target_compid = mMavlinkPassthrough->get_target_compid();
    ComLong.target_sysid = mMavlinkPassthrough->get_target_sysid();
    ComLong.command = MAV_CMD_DO_SET_HOME;
    ComLong.param1 = 0;
    ComLong.param2 = 0;
    ComLong.param3 = 0;
    ComLong.param4 = NAN;
    ComLong.param5 = homeLlh.latitude;
    ComLong.param6 = homeLlh.longitude;
    ComLong.param7 = homeLlh.height;

    if (mMavlinkPassthrough->send_command_long(ComLong) == mavsdk::MavlinkPassthrough::Result::Success) {
        xyz_t xyz = coordinateTransforms::llhToEnu(mEnuReference, homeLlh);

        auto homePos = mVehicleState->getHomePosition();
        homePos.setX(xyz.x);
        homePos.setY(xyz.y);
        homePos.setHeight(xyz.z);
        mVehicleState->setHomePosition(homePos);
    }
}

void MavsdkVehicleConnection::requestArm()
{
    mAction->arm_async([](mavsdk::Action::Result res){
        if (res != mavsdk::Action::Result::Success)
            qDebug() << "Warning: MavsdkVehicleConnection's arm request failed.";
    });
}

void MavsdkVehicleConnection::requestDisarm()
{
    mAction->disarm_async([](mavsdk::Action::Result res){
        if (res != mavsdk::Action::Result::Success)
            qDebug() << "Warning: MavsdkVehicleConnection's disarm request failed.";
    });
}

void MavsdkVehicleConnection::requestTakeoff()
{
    if (mVehicleType == MAV_TYPE::MAV_TYPE_QUADROTOR) {
        mAction->takeoff_async([](mavsdk::Action::Result res){
            if (res != mavsdk::Action::Result::Success)
                qDebug() << "Warning: MavsdkVehicleConnection's takeoff request failed.";
        });
    } else
        qDebug() << "Warning: MavsdkVehicleConnection is trying to take off with an unknown/incompatible vehicle type, ignored.";
}

void MavsdkVehicleConnection::requestLanding()
{
    if (mVehicleType == MAV_TYPE::MAV_TYPE_QUADROTOR) {
        mAction->land_async([](mavsdk::Action::Result res){
            if (res != mavsdk::Action::Result::Success)
                qDebug() << "Warning: MavsdkVehicleConnection's land request failed.";
        });
    } else
        qDebug() << "Warning: MavsdkVehicleConnection is trying to land with an unknown/incompatible vehicle type, ignored.";
}

void MavsdkVehicleConnection::requestPrecisionLanding()
{
    mavsdk::MavlinkPassthrough::CommandLong ComLong;
    memset(&ComLong, 0, sizeof (ComLong));
    ComLong.target_compid = mMavlinkPassthrough->get_target_compid();
    ComLong.target_sysid = mMavlinkPassthrough->get_target_sysid();
    ComLong.command = MAV_CMD_DO_SET_MODE;
    ComLong.param1 = MAV_MODE_FLAG_SAFETY_ARMED | MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
    ComLong.param2 = 4; // PX4_CUSTOM_MAIN_MODE_AUTO
    ComLong.param3 = 9; // PX4_CUSTOM_SUB_MODE_AUTO_PRECLAND

    if (mMavlinkPassthrough->send_command_long(ComLong) != mavsdk::MavlinkPassthrough::Result::Success)
        qDebug() << "Warning: MavsdkVehicleConnection's precision land request failed.";
}

void MavsdkVehicleConnection::requestReturnToHome()
{
    if (mVehicleType == MAV_TYPE::MAV_TYPE_QUADROTOR) {
        mAction->return_to_launch_async([](mavsdk::Action::Result res){
            if (res != mavsdk::Action::Result::Success)
                qDebug() << "Warning: MavsdkVehicleConnection's return to home request failed.";
        });
    } else
        qDebug() << "Warning: MavsdkVehicleConnection is trying to land with an unknown/incompatible vehicle type, ignored.";
}

void MavsdkVehicleConnection::requestManualControl()
{
    mavsdk::MavlinkPassthrough::CommandLong ComLong;
    memset(&ComLong, 0, sizeof (ComLong));
    ComLong.target_compid = mMavlinkPassthrough->get_target_compid();
    ComLong.target_sysid = mMavlinkPassthrough->get_target_sysid();
    ComLong.command = MAV_CMD_DO_SET_MODE;
    ComLong.param1 = MAV_MODE_FLAG_SAFETY_ARMED | MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
    ComLong.param2 = 1; // PX4_CUSTOM_MAIN_MODE_MANUAL

    if (mMavlinkPassthrough->send_command_long(ComLong) != mavsdk::MavlinkPassthrough::Result::Success)
        qDebug() << "Warning: MavsdkVehicleConnection's mode change request failed.";
}

void MavsdkVehicleConnection::requestFollowPoint()
{
    if (isAutopilotActiveOnVehicle())
        pauseAutopilotOnVehicle();

    mavsdk::MavlinkPassthrough::CommandLong ComLong;
    memset(&ComLong, 0, sizeof (ComLong));
    ComLong.target_compid = mMavlinkPassthrough->get_target_compid();
    ComLong.target_sysid = mMavlinkPassthrough->get_target_sysid();
    ComLong.command = MAV_CMD_DO_SET_MODE;
    ComLong.param1 = MAV_MODE_FLAG_SAFETY_ARMED | MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
    ComLong.param2 = 4; // PX4_CUSTOM_MAIN_MODE_AUTO
    ComLong.param3 = 8; // PX4_CUSTOM_SUB_MODE_AUTO_FOLLOW_TARGET

    if (mMavlinkPassthrough->send_command_long(ComLong) != mavsdk::MavlinkPassthrough::Result::Success)
        qDebug() << "Warning: MavsdkVehicleConnection's follow point request failed.";
}

void MavsdkVehicleConnection::requestGotoLlh(const llh_t &llh, bool changeFlightmodeToHold)
{
    if (changeFlightmodeToHold) { // MAVSDK will change flightmode if necessary, not always desired
        mAction->goto_location_async(llh.latitude, llh.longitude, llh.height, NAN, [](mavsdk::Action::Result res){
            if (res != mavsdk::Action::Result::Success)
                qDebug() << "Warning: MavsdkVehicleConnection's goto request failed.";
        });
    } else {
        mavsdk::MavlinkPassthrough::CommandInt ComInt;
        memset(&ComInt, 0, sizeof (ComInt));
        ComInt.target_compid = mMavlinkPassthrough->get_target_compid();
        ComInt.target_sysid = mMavlinkPassthrough->get_target_sysid();
        ComInt.command = MAV_CMD_DO_REPOSITION;
        ComInt.param4 = NAN; // yaw in rad
        ComInt.x = int32_t(std::round(llh.latitude * 1e7));
        ComInt.y = int32_t(std::round(llh.longitude * 1e7));
        ComInt.z = llh.height;

        if (mMavlinkPassthrough->send_command_int(ComInt) != mavsdk::MavlinkPassthrough::Result::Success)
            qDebug() << "Warning: MavsdkVehicleConnection's reposition request failed.";
    }
}

void MavsdkVehicleConnection::requestGotoENU(const xyz_t &xyz, bool changeFlightmodeToHold)
{
    if (mConvertLocalPositionsToGlobalBeforeSending) {
        llh_t llh = coordinateTransforms::enuToLlh(mEnuReference, xyz);
        requestGotoLlh(llh, changeFlightmodeToHold);
    } else {
        qDebug() << "MavsdkVehicleConnection::requestGotoENU: sending local coordinates to vehicle without converting not implemented.";
    }
}

void MavsdkVehicleConnection::requestVelocityAndYaw(const xyz_t &velocityENU, const double &yawDeg)
{
    if (mOffboard == nullptr)
        mOffboard.reset(new mavsdk::Offboard(mSystem));

    if (!mOffboard->is_active()) {
        mOffboard->set_velocity_ned({});
        if (mOffboard->start() != mavsdk::Offboard::Result::Success) {
            qDebug() << "Warning: MavsdkVehicleConnection failed to start offboard mode";
            return;
        } else
            qDebug() << "MavsdkVehicleConnection: offboard mode started";
    }

    xyz_t velocityNED = coordinateTransforms::enuToNED(velocityENU);
    mOffboard->set_velocity_ned({(float)(velocityNED.x), (float)(velocityNED.y), (float)(velocityNED.z), (float)(coordinateTransforms::yawENUtoNED(yawDeg))});
}

void MavsdkVehicleConnection::inputRtcmData(const QByteArray &rtcmData)
{
    // See: https://github.com/mavlink/qgroundcontrol/blob/aba881bf8e3f2fdbf63ef0689a3bf0432f597759/src/GPS/RTCM/RTCMMavlink.cc#L24
    if (mMavlinkPassthrough == nullptr)
        return;

    static uint8_t sequenceId = 0;

    mavlink_message_t mavRtcmMsg;
    mavlink_gps_rtcm_data_t mavRtcmData;
    memset(&mavRtcmData, 0, sizeof(mavlink_gps_rtcm_data_t));
    if (rtcmData.length() < MAVLINK_MSG_GPS_RTCM_DATA_FIELD_DATA_LEN) {
        mavRtcmData.len = rtcmData.length();
        mavRtcmData.flags = (sequenceId & 0x1F) << 3;
        memcpy(mavRtcmData.data, rtcmData.data(), rtcmData.size());

        mavlink_msg_gps_rtcm_data_encode(mMavlinkPassthrough->get_our_sysid(), mMavlinkPassthrough->get_our_compid(), &mavRtcmMsg, &mavRtcmData);
        if (mMavlinkPassthrough->send_message(mavRtcmMsg) != mavsdk::MavlinkPassthrough::Result::Success)
            qDebug() << "Warning: could not send RTCM via MAVLINK.";
    } else { // rtcm data needs to be fragmented into multiple messages
        uint8_t fragmentId = 0;
        int numBytesProcessed = 0;
        while (numBytesProcessed < rtcmData.size()) {
            int fragmentLength = std::min(rtcmData.size() - numBytesProcessed, MAVLINK_MSG_GPS_RTCM_DATA_FIELD_DATA_LEN);
            mavRtcmData.flags = 1;                          // LSB set indicates message is fragmented
            mavRtcmData.flags |= fragmentId++ << 1;         // Next 2 bits are fragment id
            mavRtcmData.flags |= (sequenceId & 0x1F) << 3;  // Next 5 bits are sequence id
            mavRtcmData.len = fragmentLength;
            memcpy(mavRtcmData.data, rtcmData.data() + numBytesProcessed, fragmentLength);

            mavlink_msg_gps_rtcm_data_encode(mMavlinkPassthrough->get_our_sysid(), mMavlinkPassthrough->get_our_compid(), &mavRtcmMsg, &mavRtcmData);
            if (mMavlinkPassthrough->send_message(mavRtcmMsg) != mavsdk::MavlinkPassthrough::Result::Success)
                qDebug() << "Warning: could not send RTCM via MAVLINK.";
            numBytesProcessed += fragmentLength;
        }
    }

    sequenceId++;
}

void MavsdkVehicleConnection::sendLandingTargetLlh(const llh_t &landingTargetLlh)
{
    if (mMavlinkPassthrough == nullptr)
        return;

    // Note from https://docs.px4.io/master/en/advanced_features/precland.html#mission
    // The system must publish the coordinates of the target in the LANDING_TARGET message.
    // Note that PX4 requires LANDING_TARGET.frame to be MAV_FRAME_LOCAL_NED and only populates the fields x, y, and z.
    // The origin of the local NED frame [0,0] is the home position.

    // Note by Marvin: not home position, gps origin! I tried to update gps origin using mavlink_set_gps_global_origin_t,
    // but it is not updated while flying (PX4 1.12). Thus, we need to take their gps origin for calculating landing target in NED here.

    // From Llh to their ENU
    xyz_t landingTargetENUgpsOrigin = coordinateTransforms::llhToEnu({mGpsGlobalOrigin.latitude, mGpsGlobalOrigin.longitude, mGpsGlobalOrigin.height}, landingTargetLlh);

    // Send landing target message (with NED frame)
    mavlink_message_t mavLandingTargetMsg;
    mavlink_landing_target_t mavLandingTargetNED;
    memset(&mavLandingTargetNED, 0, sizeof(mavlink_landing_target_t));

    mavLandingTargetNED.position_valid = 1;
    mavLandingTargetNED.frame = MAV_FRAME_LOCAL_NED;
    mavLandingTargetNED.time_usec = QDateTime::currentDateTimeUtc().toMSecsSinceEpoch();

    xyz_t landingTargetNEDgpsOrigin = coordinateTransforms::enuToNED(landingTargetENUgpsOrigin);
    mavLandingTargetNED.x = landingTargetNEDgpsOrigin.x;
    mavLandingTargetNED.y = landingTargetNEDgpsOrigin.y;
    mavLandingTargetNED.z = landingTargetNEDgpsOrigin.z;

    mavlink_msg_landing_target_encode(mMavlinkPassthrough->get_our_sysid(), mMavlinkPassthrough->get_our_compid(), &mavLandingTargetMsg, &mavLandingTargetNED);
    if (mMavlinkPassthrough->send_message(mavLandingTargetMsg) != mavsdk::MavlinkPassthrough::Result::Success)
        qDebug() << "Warning: could not send LANDING_TARGET via MAVLINK.";

}

void MavsdkVehicleConnection::sendLandingTargetENU(const xyz_t &landingTargetENU)
{
    if (mMavlinkPassthrough == nullptr)
        return;

    llh_t landingTargetLlh = coordinateTransforms::enuToLlh(mEnuReference, landingTargetENU);
    sendLandingTargetLlh(landingTargetLlh);
}

void MavsdkVehicleConnection::sendSetGpsOriginLlh(const llh_t &gpsOriginLlh)
{
    if (mMavlinkPassthrough == nullptr)
        return;

    mavlink_message_t mavGpsGlobalOriginMsg;
    mavlink_set_gps_global_origin_t mavGpsGlobalOrigin;
    memset(&mavGpsGlobalOrigin, 0, sizeof(mavlink_set_gps_global_origin_t));

    mavGpsGlobalOrigin.latitude = (int) (gpsOriginLlh.latitude * 1e7);
    mavGpsGlobalOrigin.longitude = (int) (gpsOriginLlh.longitude * 1e7);
    mavGpsGlobalOrigin.altitude = (int) (gpsOriginLlh.height * 1e3);
    mavGpsGlobalOrigin.target_system = mMavlinkPassthrough->get_target_sysid();

    mavlink_msg_set_gps_global_origin_encode(mMavlinkPassthrough->get_our_sysid(), mMavlinkPassthrough->get_our_compid(), &mavGpsGlobalOriginMsg, &mavGpsGlobalOrigin);
    if (mMavlinkPassthrough->send_message(mavGpsGlobalOriginMsg) != mavsdk::MavlinkPassthrough::Result::Success)
        qDebug() << "Warning: could not send GPS_GLOBAL_ORIGIN via MAVLINK.";
    else
        qDebug() << "Sent GPS_GLOBAL_ORIGIN via MAVLINK:" << gpsOriginLlh.latitude << gpsOriginLlh.longitude;
}

void MavsdkVehicleConnection::setActuatorOutput(int index, float value)
{
    mAction->set_actuator_async(index, value, [](mavsdk::Action::Result res){
        if (res != mavsdk::Action::Result::Success)
            qDebug() << "Warning: MavsdkVehicleConnection's set_actuator request failed.";
    });
}

void MavsdkVehicleConnection::setManualControl(double x, double y, double z, double r, uint16_t buttonStateMask)
{
    mavlink_manual_control_t manual_control {};
    manual_control.x = (uint16_t) (x * 1000.0);
    manual_control.y = (uint16_t) (y * 1000.0);
    manual_control.z = (uint16_t) (z * 1000.0);
    manual_control.r = (uint16_t) (r * 1000.0);
    manual_control.buttons = buttonStateMask;

    mavlink_message_t message;
    mavlink_msg_manual_control_encode(mMavlinkPassthrough->get_our_sysid(), mMavlinkPassthrough->get_our_compid(), &message, &manual_control);

    if (mMavlinkPassthrough->send_message(message) != mavsdk::MavlinkPassthrough::Result::Success)
        qDebug() << "Warning: could not send MANUAL_CONTROL via MAVLINK.";
}

void MavsdkVehicleConnection::setConvertLocalPositionsToGlobalBeforeSending(bool convertLocalPositionsToGlobalBeforeSending)
{
    mConvertLocalPositionsToGlobalBeforeSending = convertLocalPositionsToGlobalBeforeSending;
}

MAV_TYPE MavsdkVehicleConnection::getVehicleType() const
{
    return mVehicleType;
}

mavsdk::MissionRaw::MissionItem MavsdkVehicleConnection::convertPosPointToMissionItem(const PosPoint& posPoint, int sequenceId, bool current) {
    mavsdk::MissionRaw::MissionItem missionItem = {};

    if (mVehicleType == MAV_TYPE_GROUND_ROVER) { // assumption: rover = WayWise on vehicle side
        missionItem.mission_type = MAV_MISSION_TYPE_MISSION;
        missionItem.frame = MAV_FRAME_LOCAL_ENU;
        missionItem.command = MAV_CMD_NAV_WAYPOINT;
        missionItem.seq = sequenceId;
        missionItem.current = current;
        missionItem.autocontinue = true;
        // TODO: does not follow MAV_CMD_NAV_WAYPOINT definition
        missionItem.param1 = posPoint.getSpeed();
        missionItem.param2 = posPoint.getAttributes();
        missionItem.param4 = NAN; // yaw
        missionItem.x = (int)(posPoint.getX() * 10e4);
        missionItem.y = (int)(posPoint.getY() * 10e4);
        missionItem.z = (float)posPoint.getHeight();
    } else     // TODO: PX4 only supports mission items in global frame / llh!
        throw std::logic_error("converting mission items to global frame not implemented");

    return missionItem;
}

bool MavsdkVehicleConnection::isAutopilotActiveOnVehicle()
{
    return mVehicleState->getFlightMode() == VehicleState::FlightMode::Mission;
}

void MavsdkVehicleConnection::restartAutopilotOnVehicle()
{ 
    if (!mMissionRaw)
        mMissionRaw.reset(new mavsdk::MissionRaw(mSystem));

    mMissionRaw->set_current_mission_item_async(0, [this](mavsdk::MissionRaw::Result res) {
        if (res != mavsdk::MissionRaw::Result::Success)
            qDebug() << "Warning: MavsdkVehicleConnection's set current mission item request failed.";
        else
            mMissionRaw->start_mission_async([](mavsdk::MissionRaw::Result res) {
                if (res != mavsdk::MissionRaw::Result::Success)
                    qDebug() << "Warning: MavsdkVehicleConnection's start mission request failed.";
            });
    });
}

void MavsdkVehicleConnection::startAutopilotOnVehicle()
{
    if (!mMissionRaw)
        mMissionRaw.reset(new mavsdk::MissionRaw(mSystem));

    mMissionRaw->start_mission_async([](mavsdk::MissionRaw::Result res) {
        if (res != mavsdk::MissionRaw::Result::Success)
            qDebug() << "Warning: MavsdkVehicleConnection's start mission request failed.";
    });
}

void MavsdkVehicleConnection::pauseAutopilotOnVehicle()
{
    if (!mMissionRaw)
        mMissionRaw.reset(new mavsdk::MissionRaw(mSystem));

    // Note: mavsdk::MissionRaw::pause_mission tries to transition to hold mode, but this is not supported by MAVSDK ActionServer (WayWise vehicle side)
    if (mVehicleType == MAV_TYPE_GROUND_ROVER) // assumption: rover = WayWise on vehicle side
        requestManualControl();
    else
        if (mMissionRaw->pause_mission() != mavsdk::MissionRaw::Result::Success)
            qDebug() << "Warning: MavsdkVehicleConnection's pause mission request failed.";
}

void MavsdkVehicleConnection::stopAutopilotOnVehicle()
{
    pauseAutopilotOnVehicle();

    mMissionRaw->set_current_mission_item_async(0, [](mavsdk::MissionRaw::Result res) {
        if (res != mavsdk::MissionRaw::Result::Success)
            qDebug() << "Warning: MavsdkVehicleConnection's set current mission item request failed.";
    });
}

void MavsdkVehicleConnection::clearRouteOnVehicle(int id)
{
    Q_UNUSED(id)
    if (!mMissionRaw)
        mMissionRaw.reset(new mavsdk::MissionRaw(mSystem));

    if (mMissionRaw->clear_mission() != mavsdk::MissionRaw::Result::Success)
        qDebug() << "Warning: MavsdkVehicleConnection's clear mission request failed.";
}

void MavsdkVehicleConnection::appendToRouteOnVehicle(const QList<PosPoint> &route, int id)
{
    Q_UNUSED(id)
    if (!mMissionRaw)
        mMissionRaw.reset(new mavsdk::MissionRaw(mSystem));

    // TODO: this does not actually append but replace
    std::vector<mavsdk::MissionRaw::MissionItem> missionItems;
    for (int i = 0; i < route.size(); i++)
        missionItems.push_back(convertPosPointToMissionItem(route.at(i), i, i == 0));

    mMissionRaw->upload_mission_async(missionItems, [](mavsdk::MissionRaw::Result res){
        if (res != mavsdk::MissionRaw::Result::Success)
            qDebug() << "Warning: MavsdkVehicleConnection's mission upload failed:" << (int)res;
    });
}

void MavsdkVehicleConnection::setActiveAutopilotIDOnVehicle(int id)
{
    mavsdk::MavlinkPassthrough::CommandLong ComLong;
    memset(&ComLong, 0, sizeof (ComLong));
    ComLong.target_compid = mMavlinkPassthrough->get_target_compid();
    ComLong.target_sysid = mMavlinkPassthrough->get_target_sysid();
    ComLong.command = MAV_CMD_DO_SET_MISSION_CURRENT;
    ComLong.param1 = -1;
    ComLong.param2 = 0;
    ComLong.param3 = id; // Autopilot ID

    if (mMavlinkPassthrough->send_command_long(ComLong) != mavsdk::MavlinkPassthrough::Result::Success)
        qDebug() << "Warning: could not send MISSION_SET_CURRENT via MAVLINK.";
}

mavsdk::Param::Result MavsdkVehicleConnection::setIntParameterOnVehicle(std::string name, int32_t value)
{
    return mParam->set_param_int(name, value);
}

mavsdk::Param::Result MavsdkVehicleConnection::setFloatParameterOnVehicle(std::string name, float value)
{
    return mParam->set_param_float(name, value);
}

mavsdk::Param::Result MavsdkVehicleConnection::setCustomParameterOnVehicle(std::string name, std::string value)
{
    return mParam->set_param_custom(name, value);
}

mavsdk::Param::AllParams MavsdkVehicleConnection::getAllParametersFromVehicle()
{
    return mParam->get_all_params();
}
