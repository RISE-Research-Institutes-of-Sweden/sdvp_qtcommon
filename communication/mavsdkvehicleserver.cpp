/*
 *     Copyright 2025 RISE Research Institutes of Sweden AB, Safety and Transport   waywise@ri.se
 *     Published under GPLv3: https://www.gnu.org/licenses/gpl-3.0.html
 */

#include <QDebug>
#include <QMetaMethod>
#include <future>
#include <algorithm>
#include <chrono>
#include <gpiod.hpp>
#include <unistd.h>

#include "mavsdkvehicleserver.h"
#include "logger/logger.h"
#include "communication/parameterserver.h"

MavsdkVehicleServer::MavsdkVehicleServer(QSharedPointer<VehicleState> vehicleState, const QHostAddress controlTowerAddress, const unsigned controlTowerPort, const QAbstractSocket::SocketType controlTowerSocketType) :
    VehicleServer(vehicleState)
{
    connect(&Logger::getInstance(), &Logger::logSent, this, &MavsdkVehicleServer::on_logSent);

    mVehicleState = vehicleState;
    mSystemId = mVehicleState->getId();

    mavsdk::Mavsdk::Configuration config = mavsdk::Mavsdk::Configuration{mavsdk::Mavsdk::ComponentType::Autopilot};
    config.set_always_send_heartbeats(true);
    config.set_system_id(mSystemId);
    mMavsdk.reset(new mavsdk::Mavsdk{config});

//    mavsdk::Mavsdk::Configuration customConfig = mavsdk::Mavsdk::Configuration{mavsdk::Mavsdk::ComponentType::Custom};
//    customConfig.set_system_id(mVehicleState->getId());
//    customConfig.set_always_send_heartbeats(true);
//    customConfig.set_component_id(100); // Should be 100 for camera
//    mMavsdk->set_configuration(customConfig);

    std::shared_ptr<mavsdk::ServerComponent> serverComponent = mMavsdk->server_component_by_type(mavsdk::Mavsdk::ComponentType::Autopilot);

    // Create server plugins
    MavlinkParameterServer::initialize(serverComponent);
    mParameterServer = MavlinkParameterServer::getInstance();
    mTelemetryServer.reset(new mavsdk::TelemetryServer(serverComponent));
    mActionServer.reset(new mavsdk::ActionServer(serverComponent));
    mMissionRawServer.reset(new mavsdk::MissionRawServer(serverComponent));

    // Allow the vehicle to change to auto mode (manual is always allowed) and arm, disable takeoff (only rover support for now)
    mActionServer->set_allowable_flight_modes({true, true, false});
    mActionServer->set_armable(true, true);
    mActionServer->set_allow_takeoff(false);

    // Publish vehicleState's telemetry info
    connect(&mPublishMavlinkTimer, &QTimer::timeout, [this](){
        mavsdk::TelemetryServer::PositionVelocityNed positionVelocityNed{{static_cast<float>(mVehicleState->getPosition(PosType::fused).getY()),
                                                                          static_cast<float>(mVehicleState->getPosition(PosType::fused).getX()),
                                                                          static_cast<float>(-mVehicleState->getPosition(PosType::fused).getHeight())},
                                                                         {static_cast<float>(mVehicleState->getVelocity().y),
                                                                          static_cast<float>(mVehicleState->getVelocity().x),
                                                                          static_cast<float>(-mVehicleState->getVelocity().z)}};

        mavsdk::TelemetryServer::VelocityNed velocity{static_cast<float>(mVehicleState->getVelocity().y),
                                                      static_cast<float>(mVehicleState->getVelocity().x),
                                                      static_cast<float>(-mVehicleState->getVelocity().z)};

        mavsdk::TelemetryServer::Heading heading{coordinateTransforms::yawENUtoNED(mVehicleState->getPosition(PosType::fused).getYaw())};

        mavsdk::TelemetryServer::Position homePositionLlh{};
        mavsdk::TelemetryServer::Position positionLlh{};

        if (!mGNSSReceiver.isNull()) {
            // publish gpsOrigin
            sendGpsOriginLlh(mGNSSReceiver->getEnuRef());

            //TODO: homePositionLlh should not be EnuRef
            homePositionLlh = {mGNSSReceiver->getEnuRef().latitude, mGNSSReceiver->getEnuRef().longitude, static_cast<float>(mGNSSReceiver->getEnuRef().height), 0};

            llh_t fusedPosGlobal = coordinateTransforms::enuToLlh(mGNSSReceiver->getEnuRef(), {mVehicleState->getPosition(PosType::fused).getXYZ()});
            positionLlh = {fusedPosGlobal.latitude, fusedPosGlobal.longitude, static_cast<float>(fusedPosGlobal.height), 0};
        }

        mTelemetryServer->publish_position(positionLlh, velocity, heading);
        mTelemetryServer->publish_home(homePositionLlh);
        mTelemetryServer->publish_position_velocity_ned(positionVelocityNed);
        mTelemetryServer->publish_raw_gps(mRawGps, mGpsInfo);
    });

    mActionServer->subscribe_flight_mode_change([this](mavsdk::ActionServer::Result res, mavsdk::ActionServer::FlightMode mode) {
        if  (res == mavsdk::ActionServer::Result::Success) {
            mVehicleState->setFlightMode((VehicleState::FlightMode) mode);

            switch (mode) {
            case mavsdk::ActionServer::FlightMode::Mission:
                emit startWaypointFollower(false);
                break;
            case mavsdk::ActionServer::FlightMode::Offboard:
                emit startFollowPoint();
                break;
            default:
                ;
            }
        }

        if (mode != mavsdk::ActionServer::FlightMode::Mission)
            if (!mWaypointFollower.isNull() && mWaypointFollower->isActive())
                emit pauseWaypointFollower();

        if (mode != mavsdk::ActionServer::FlightMode::Offboard)
            if (!mFollowPoint.isNull() && mFollowPoint->isActive())
                emit stopFollowPoint();
    });

    mMissionRawServer->subscribe_incoming_mission([this](mavsdk::MissionRawServer::Result res, mavsdk::MissionRawServer::MissionPlan plan) {
        if (res != mavsdk::MissionRawServer::Result::Success)
            qDebug() << "MavsdkVehicleServer: failed to receive mission," << (int)res;
        else {
            qDebug() << "MavsdkVehicleServer: got new mission with" << plan.mission_items.size() << "items.";

            if (!mWaypointFollower.isNull()) {
                QList<PosPoint> route;
                for (const auto &item : plan.mission_items)
                    route.append(convertMissionItemToPosPoint(item));

                mWaypointFollower->addRoute(route);
            } else
                qDebug() << "MavsdkVehicleServer: got new mission but no WaypointFollower is set to receive it.";

        }
    });

    mMissionRawServer->subscribe_clear_all([this](uint32_t val) {
        Q_UNUSED(val)
        if (!mWaypointFollower.isNull())
            emit clearRouteOnWaypointFollower();
        else
            qDebug() << "MavsdkVehicleServer: got clear mission request but no WaypointFollower is set to receive it.";
    });

    mMissionRawServer->subscribe_current_item_changed([this](mavsdk::MissionRawServer::MissionItem item) {
        if (!mWaypointFollower.isNull() && item.seq == 0)
            emit resetWaypointFollower();
        else if (item.seq != 0)
            qDebug() << "Warning: jumping to seq ID in mission not implemented in MavsdkVehicleServer / WaypointFollower.";
    });

    // Safety heartbeat
    mHeartbeat = false;
    mHeartbeatTimer.setSingleShot(true);
    connect(&mHeartbeatTimer, &QTimer::timeout, this, &MavsdkVehicleServer::heartbeatTimeout);
    connect(this, &MavsdkVehicleServer::resetHeartbeat, this, &MavsdkVehicleServer::heartbeatReset);

    mMavsdk->intercept_incoming_messages_async([this](mavlink_message_t &message){
        if (message.msgid == MAVLINK_MSG_ID_HEARTBEAT) { // TODO: make sure this is actually for us
            if (!mHeartbeat) {
                qDebug() << "MavsdkVehicleServer: got heartbeat, timeout was reset.";
            }
            emit resetHeartbeat();
        } else if (!mHeartbeat)
            return false; // Drop incoming messages until heartbeat restored

        static QList<PosPoint> currentRoute;

        switch (message.msgid) {
        case MAVLINK_MSG_ID_MANUAL_CONTROL:
        {
            mavlink_manual_control_t manual_control;
            mavlink_msg_manual_control_decode(&message, &manual_control);

            if (mVehicleState->getFlightMode() == VehicleState::FlightMode::Manual)
                handleManualControlMessage(manual_control);
            break;
        }
        case MAVLINK_MSG_ID_MISSION_REQUEST_LIST:
        {
            mavlink_mission_request_list_t missionRequestList;
            mavlink_msg_mission_request_list_decode(&message, &missionRequestList);

            if(missionRequestList.mission_type != MAV_MISSION_TYPE_MISSION) {
                sendMissionAck(MAV_MISSION_ERROR);
                break;
            }

            currentRoute = mWaypointFollower->getCurrentRoute();
            if (mMavlinkPassthrough->queue_message([&](MavlinkAddress mavlink_address, uint8_t channel) {
                mavlink_mission_count_t missionCount;
                memset(&missionCount, 0, sizeof(missionCount));

                missionCount.target_system = mMavlinkPassthrough->get_target_sysid();
                missionCount.target_component = mMavlinkPassthrough->get_target_compid();
                missionCount.count = currentRoute.size();
                missionCount.mission_type = MAV_MISSION_TYPE_MISSION;

                mavlink_address.system_id = mSystemId;
                mavlink_address.component_id = MAV_COMP_ID_AUTOPILOT1;

                mavlink_message_t mavMissionCountMsg;
                mavlink_msg_mission_count_encode_chan(mavlink_address.system_id, mavlink_address.component_id, channel, &mavMissionCountMsg, &missionCount);

                return mavMissionCountMsg;
            }) != mavsdk::MavlinkPassthrough::Result::Success)
                qWarning() << "Could not send MISSION_COUNT via MAVLINK.";

            break;
        }
        case MAVLINK_MSG_ID_MISSION_REQUEST_INT:
        {
            mavlink_mission_request_int_t missionRequestInt;
            mavlink_msg_mission_request_int_decode(&message, &missionRequestInt);

            if(missionRequestInt.mission_type != MAV_MISSION_TYPE_MISSION) {
                sendMissionAck(MAV_MISSION_ERROR);
                break;
            }

            PosPoint posPoint = currentRoute.at(missionRequestInt.seq);
            if (mMavlinkPassthrough->queue_message([&](MavlinkAddress mavlink_address, uint8_t channel) {
                mavlink_mission_item_int_t missionItemInt;
                memset(&missionItemInt, 0, sizeof(missionItemInt));

                missionItemInt.seq = missionRequestInt.seq;
                missionItemInt.frame = MAV_FRAME_LOCAL_ENU;
                missionItemInt.command = MAV_CMD_NAV_WAYPOINT;
                missionItemInt.current = false;
                missionItemInt.autocontinue = true;
                missionItemInt.param1 = posPoint.getSpeed();
                missionItemInt.param2 = posPoint.getAttributes();
                missionItemInt.param4 = NAN;    // yaw
                missionItemInt.x = (int)(posPoint.getX() * 10e4);
                missionItemInt.y = (int)(posPoint.getY() * 10e4);
                missionItemInt.z = (float)posPoint.getHeight();
                missionItemInt.mission_type = MAV_MISSION_TYPE_MISSION;

                mavlink_address.system_id = mSystemId;
                mavlink_address.component_id = MAV_COMP_ID_AUTOPILOT1;

                mavlink_message_t mavmissionItemIntMsg;
                mavlink_msg_mission_item_int_encode_chan(mavlink_address.system_id, mavlink_address.component_id, channel, &mavmissionItemIntMsg, &missionItemInt);

                return mavmissionItemIntMsg;
            }) != mavsdk::MavlinkPassthrough::Result::Success)
                qWarning() << "Could not send MISSION_ITEM_INT via MAVLINK.";
            break;
        }
        case MAVLINK_MSG_ID_MISSION_REQUEST:
        {
            mavlink_mission_request_t missionRequest;
            mavlink_msg_mission_request_decode(&message, &missionRequest);

            if(missionRequest.mission_type != MAV_MISSION_TYPE_MISSION) {
                sendMissionAck(MAV_MISSION_ERROR);
                break;
            }

            PosPoint posPoint = currentRoute.at(missionRequest.seq);
            if (mMavlinkPassthrough->queue_message([&](MavlinkAddress mavlink_address, uint8_t channel) {
                mavlink_mission_item_t missionItem;
                memset(&missionItem, 0, sizeof(missionItem));

                missionItem.seq = missionRequest.seq;
                missionItem.frame = MAV_FRAME_LOCAL_ENU;
                missionItem.command = MAV_CMD_NAV_WAYPOINT;
                missionItem.current = false;
                missionItem.autocontinue = true;
                missionItem.param1 = posPoint.getSpeed();
                missionItem.param2 = posPoint.getAttributes();
                missionItem.param4 = NAN;    // yaw
                missionItem.x = (int)(posPoint.getX() * 10e4);
                missionItem.y = (int)(posPoint.getY() * 10e4);
                missionItem.z = (float)posPoint.getHeight();
                missionItem.mission_type = MAV_MISSION_TYPE_MISSION;

                mavlink_address.system_id = mSystemId;
                mavlink_address.component_id = MAV_COMP_ID_AUTOPILOT1;

                mavlink_message_t mavmissionItemMsg;
                mavlink_msg_mission_item_encode_chan(mavlink_address.system_id, mavlink_address.component_id, channel, &mavmissionItemMsg, &missionItem);

                return mavmissionItemMsg;
            }) != mavsdk::MavlinkPassthrough::Result::Success)
                qWarning() << "Could not send MISSION_ITEM via MAVLINK.";
            break;
        }
        case MAVLINK_MSG_ID_MISSION_ACK:
        {
            mavlink_mission_ack_t missionAck;
            mavlink_msg_mission_ack_decode(&message, &missionAck);

            if((missionAck.mission_type == MAV_MISSION_TYPE_MISSION) && (missionAck.type != MAV_MISSION_ACCEPTED))
                sendMissionAck(MAV_MISSION_OPERATION_CANCELLED);
            break;
        }
        default:
            ;
        }

        return true;
    });

    // --- Things that we should implement/fix in MAVSDK follow from here :-)
    // Create mavlinkpassthrough plugin (TODO: should not be used on vehicle side...)
    mMavsdk->subscribe_on_new_system([this](){
        mMavlinkPassthrough.reset(new mavsdk::MavlinkPassthrough(mMavsdk->systems().at(0)));

        mMavlinkPassthrough->subscribe_message(MAVLINK_MSG_ID_COMMAND_INT, [this](const mavlink_message_t &message) {

            if(mavlink_msg_command_int_get_command(&message) == MAV_CMD_DO_SET_SERVO) {

                mavlink_command_int_t cmd;
                mavlink_msg_command_int_decode(&message, &cmd);

                /*
                 * Shuttle prototype GPIO configuration
                 *  Pin 22: High/low gear
                 *  Pin 27: T-lock servo (back)
                */
                unsigned int line_num = static_cast<int>(cmd.param1);
                int gear_pwm = static_cast<int>(cmd.param2);

                if(gear_pwm < 1000 || gear_pwm > 2000) {
                    mavResult(MAV_CMD_DO_SET_SERVO, MAV_RESULT_DENIED, MAV_COMP_ID_AUTOPILOT1);
                } else {
                    gpiod::chip chip;
                    QString chipName = "gpiochip4"; // Raspberry Pi 5
                    gpiod::line gearSwitch;

                    try {
                        chip.open(chipName.toStdString(), 3);
                        gearSwitch = chip.get_line(line_num);
                        gearSwitch.request({"servo", gpiod::line_request::DIRECTION_OUTPUT, 0});

                    } catch (const std::exception &e) {
                        qDebug() << "Error: " << e.what();
                    }

                    for(int i = 0; i < 50; ++i) {   // 50 pulses for 1 second (20ms period)
                        gearSwitch.set_value(1);
                        usleep(gear_pwm);
                        gearSwitch.set_value(0);
                        usleep(20000 - gear_pwm);
                        mavResult(MAV_CMD_DO_SET_SERVO, MAV_RESULT_ACCEPTED, MAV_COMP_ID_AUTOPILOT1);
                    }
                    gearSwitch.release();
                }
            }
        });

        mMavlinkPassthrough->subscribe_message(MAVLINK_MSG_ID_COMMAND_LONG, [this](const mavlink_message_t &message) {
            switch (mavlink_msg_command_long_get_command(&message)) {
            case MAV_CMD_DO_SET_MISSION_CURRENT: // used to switch between multiple autopilots
                static const QMetaMethod switchAutopilotIDSignal = QMetaMethod::fromSignal(&MavsdkVehicleServer::switchAutopilotID);
                if (isSignalConnected(switchAutopilotIDSignal))
                    emit switchAutopilotID(mavlink_msg_command_long_get_param3(&message));
                else
                    qDebug() << "Warning: got request to change autopilot id, but switchAutopilotID(..) signal is not connected.";
                break;
            case MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN:
                auto param2Value = mavlink_msg_command_long_get_param2(&message);
                if (param2Value == 1) {
                    mavResult(MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN, MAV_RESULT_ACCEPTED, MAV_COMP_ID_AUTOPILOT1);
                    emit shutdownOrRebootOnboardComputer(false);
                } else if (param2Value == 2) {
                    mavResult(MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN, MAV_RESULT_ACCEPTED, MAV_COMP_ID_AUTOPILOT1);
                    emit shutdownOrRebootOnboardComputer(true);
                } else
                    mavResult(MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN, MAV_RESULT_DENIED, MAV_COMP_ID_AUTOPILOT1);
                break;
            }
        });

        // Handle RTCM data
        mMavlinkPassthrough->subscribe_message(MAVLINK_MSG_ID_GPS_RTCM_DATA, [this](const mavlink_message_t &message) {
            mavlink_gps_rtcm_data_t mavRtcmData;
            mavlink_msg_gps_rtcm_data_decode(&message, &mavRtcmData);
            static const uint8_t maxMavlinkMessageLength = 180;
            static QByteArray fragments[4];
            static uint8_t lastFragment = 0;
            static uint8_t sequenceIdBuffer[4];

            QByteArray rtcmData;
            if ((mavRtcmData.flags & 0x1) != 0) {// LSB set indicates message is fragmented
                uint8_t sequenceId = ((mavRtcmData.flags >> 3) & 0x1F);  // 5 bits are sequence id
                uint8_t fragmentId = (mavRtcmData.flags >> 1) & 0x3;

                if (mavRtcmData.len < maxMavlinkMessageLength) // Last fragment
                    lastFragment = fragmentId;
                sequenceIdBuffer[fragmentId] = sequenceId;
                fragments[fragmentId].resize(std::min(mavRtcmData.len, maxMavlinkMessageLength));
                memcpy(fragments[fragmentId].data(), mavRtcmData.data, std::min(mavRtcmData.len, maxMavlinkMessageLength));

                if (lastFragment > 0 && std::all_of(sequenceIdBuffer, sequenceIdBuffer + lastFragment, [sequenceId](uint8_t x) { return x == sequenceId;})) { // Buffer is reconstructed with same sequence id
                    for (int i = 0; i <= lastFragment; i++)
                        rtcmData.append(fragments[i]);
                    emit rxRtcmData(rtcmData);
                    lastFragment = 0;
                }
            } else {
                rtcmData.resize(mavRtcmData.len);
                memcpy(rtcmData.data(), mavRtcmData.data, mavRtcmData.len);
                emit rxRtcmData(rtcmData);
            }
        });

        // Publish autopilot radius
        connect(&mPublishMavlinkTimer, &QTimer::timeout, [this](){
            if (mMavlinkPassthrough->queue_message([&](MavlinkAddress mavlink_address, uint8_t channel) {
                mavlink_message_t mavAutopilotRadiusmMsg;
                mavlink_named_value_float_t autopilotRadius;

                memset(&autopilotRadius, 0, sizeof(mavlink_named_value_float_t));

                autopilotRadius.time_boot_ms = QDateTime::currentMSecsSinceEpoch() - mMavsdkVehicleServerCreationTime.toMSecsSinceEpoch();
                autopilotRadius.value = mVehicleState->getAutopilotRadius();
                mavlink_address.system_id = mSystemId;
                mavlink_address.component_id = MAV_COMP_ID_AUTOPILOT1;

                strcpy(autopilotRadius.name, "AR");
                mavlink_msg_named_value_float_encode_chan(mavlink_address.system_id, mavlink_address.component_id, channel, &mavAutopilotRadiusmMsg, &autopilotRadius);

                return mavAutopilotRadiusmMsg;
            }) != mavsdk::MavlinkPassthrough::Result::Success)
                    qWarning() << "Could not send Autopilot Radius via MAVLINK.";
        });

        // Publish Autopilot lookahead and reference points
        connect(&mPublishMavlinkTimer, &QTimer::timeout, [this]() {
            if (mMavlinkPassthrough->queue_message([&](MavlinkAddress mavlink_address, uint8_t channel) {
                mavlink_message_t mavMsg;
                mavlink_position_target_local_ned_t autopilotPoints;

                memset(&autopilotPoints, 0, sizeof(mavlink_position_target_local_ned_t));

                autopilotPoints.time_boot_ms = QDateTime::currentMSecsSinceEpoch() - mMavsdkVehicleServerCreationTime.toMSecsSinceEpoch();

                QPointF autopilotTargetPointENU_XY = mVehicleState->getAutopilotTargetPoint();
                xyz_t autopilotTargetPointNED = coordinateTransforms::enuToNED({autopilotTargetPointENU_XY.x(), autopilotTargetPointENU_XY.y(), 0});
                autopilotPoints.x = autopilotTargetPointNED.x;
                autopilotPoints.y = autopilotTargetPointNED.y;

                autopilotPoints.type_mask = POSITION_TARGET_TYPEMASK_Z_IGNORE|
                                        POSITION_TARGET_TYPEMASK_VX_IGNORE |
                                        POSITION_TARGET_TYPEMASK_VY_IGNORE |
                                        POSITION_TARGET_TYPEMASK_VZ_IGNORE |
                                        POSITION_TARGET_TYPEMASK_AX_IGNORE |
                                        POSITION_TARGET_TYPEMASK_AY_IGNORE |
                                        POSITION_TARGET_TYPEMASK_AZ_IGNORE |
                                        POSITION_TARGET_TYPEMASK_YAW_IGNORE |
                                        POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE;

                mavlink_address.system_id = mSystemId;
                mavlink_address.component_id = MAV_COMP_ID_AUTOPILOT1;

                // Encode and send the POSITION_TARGET_LOCAL_NED message
                mavlink_msg_position_target_local_ned_encode_chan(mavlink_address.system_id,
                                                                mavlink_address.component_id,
                                                                channel,
                                                                &mavMsg,
                                                                &autopilotPoints);

                return mavMsg;
            }) != mavsdk::MavlinkPassthrough::Result::Success)
                qWarning() << "Could not send Autopilot Reference Point via MAVLINK.";
        });
    });

    mMavsdk->intercept_outgoing_messages_async([this](mavlink_message_t &message){
        switch (message.msgid) {
        case MAVLINK_MSG_ID_HEARTBEAT: // Fix some info in heartbeat s.th. MAVSDK / ControlTower detects vehicle correctly
            mavlink_heartbeat_t heartbeat;
            mavlink_msg_heartbeat_decode(&message, &heartbeat);
            switch (message.compid) {
            case MAV_COMP_ID_AUTOPILOT1:
                heartbeat.type = MAV_TYPE_GROUND_ROVER;
                heartbeat.autopilot = WAYWISE_MAVLINK_AUTOPILOT_ID;
                heartbeat.base_mode |= MAV_MODE_FLAG_SAFETY_ARMED | MAV_MODE_FLAG_CUSTOM_MODE_ENABLED; // Note: behave like PX4...
                break;
            case MAV_COMP_ID_CAMERA:
                heartbeat.type = MAV_TYPE_CAMERA;
                heartbeat.autopilot = MAV_AUTOPILOT_INVALID;
                break;
            default:
                qDebug() << "MavsdkVehicleServer: unsupported MAV_COMPONENT heartbeat: " << message.compid;
            }
            mavlink_msg_heartbeat_encode(message.sysid, message.compid, &message, &heartbeat);
            break;
//        case MAVLINK_MSG_ID_AUTOPILOT_VERSION:
//            mavlink_autopilot_version_t autopilot_version;
//            mavlink_msg_autopilot_version_decode(&message, &autopilot_version);
//            qDebug() << "MAVLINK_MSG_ID_AUTOPILOT_VERSION:" << autopilot_version.capabilities << autopilot_version.flight_sw_version << autopilot_version.middleware_sw_version <<
//                        autopilot_version.os_sw_version << autopilot_version.board_version << autopilot_version.flight_custom_version << autopilot_version.middleware_custom_version <<
//                        autopilot_version.os_custom_version << autopilot_version.vendor_id << autopilot_version.product_id << autopilot_version.uid << autopilot_version.uid2;
//            break;
        case MAVLINK_MSG_ID_PARAM_VALUE: // This message is sent when a parameter has been changed
            mavlink_param_value_t parameter;
            mavlink_msg_param_value_decode(&message, &parameter);
            if (parameter.param_type == MAV_PARAM_TYPE_REAL32) // Float
                mParameterServer->updateFloatParameter(parameter.param_id, (float) parameter.param_value);
            else if (parameter.param_type == MAV_PARAM_TYPE_INT32) // Int
            {
                mavlink_param_union_t param;
                param.param_float = parameter.param_value;
                mParameterServer->updateIntParameter(parameter.param_id, (int) param.param_int32);
            }
            break;
        default: ;
//            qDebug() << "out:" << message.msgid;
        }
        return true;

    });

    // Start publishing status on MAVLink (TODO: rate?)
    mPublishMavlinkTimer.start(100);

    mavsdk::ConnectionResult result;
    switch (controlTowerSocketType) {
    case QAbstractSocket::UdpSocket:
        result = mMavsdk->setup_udp_remote(controlTowerAddress.toString().toStdString(), controlTowerPort);
        break;
    default:
        qDebug() << "MavsdkVehicleServer initialized for unsupported controlTowerSocketType. Not connecting.";
        result = mavsdk::ConnectionResult::SocketError;
        break;
    }

    if (result == mavsdk::ConnectionResult::Success) {
        qDebug() << "MavsdkVehicleServer is listening on" << controlTowerAddress.toString() << "with port" << controlTowerPort;
        if (vehicleState->hasTrailingVehicle())
            createMavsdkComponentForTrailer(controlTowerAddress, controlTowerPort, controlTowerSocketType);
    }
}

void MavsdkVehicleServer::provideParametersToParameterServer() {
    ParameterServer::getInstance()->provideFloatParameter("MC_MAX_SPEED_MS", std::bind(&MavsdkVehicleServer::setManualControlMaxSpeed, this, std::placeholders::_1), std::bind(&MavsdkVehicleServer::getManualControlMaxSpeed, this));
    ParameterServer::getInstance()->provideIntParameter("VEH_WW_OBJ_TYPE",
        std::function<void(int)>([this](int value) {this->mVehicleState->setWaywiseObjectType(static_cast<WAYWISE_OBJECT_TYPE>(value));}),
        std::function<int(void)>([this]() {return static_cast<int>(this->mVehicleState->getWaywiseObjectType());})
    );
}

void MavsdkVehicleServer::heartbeatTimeout() {
    mHeartbeat = false;
    qDebug() << "MavsdkVehicleServer: heartbeat timed out";

    if (mWaypointFollower) {
        mWaypointFollower->stop();
     }

     if (mMovementController) {
              mMovementController->setDesiredSteering(0.0);
              mMovementController->setDesiredSpeed(0.0);
     }
}

void MavsdkVehicleServer::heartbeatReset()
{
    mHeartbeatTimer.start(mCountdown_ms);
    mHeartbeat = true;
}

void MavsdkVehicleServer::setUbloxRover(QSharedPointer<UbloxRover> ubloxRover)
{
    if (!mGNSSReceiver.isNull()) {
        QSharedPointer<UbloxRover> mUbloxRover = qSharedPointerDynamicCast<UbloxRover>(mGNSSReceiver);
        if (mUbloxRover) {
            disconnect(mUbloxRover.get(), &UbloxRover::txNavPvt, this, &MavsdkVehicleServer::updateRawGpsAndGpsInfoFromUbx);
        }
    }
    VehicleServer::setGNSSReceiver(ubloxRover);
    connect(ubloxRover.get(), &UbloxRover::txNavPvt, this, &MavsdkVehicleServer::updateRawGpsAndGpsInfoFromUbx);
}

void MavsdkVehicleServer::updateRawGpsAndGpsInfoFromUbx(const ubx_nav_pvt &pvt) {
    mRawGps.timestamp_us = pvt.i_tow*1000; // TODO
    mRawGps.latitude_deg = pvt.lat;
    mRawGps.longitude_deg = pvt.lon;
    mRawGps.absolute_altitude_m = pvt.height;
    mRawGps.hdop = pvt.p_dop; // hdop <= pdop
    mRawGps.vdop = pvt.p_dop; // vdop <= pdop
    mRawGps.velocity_m_s = sqrt(pvt.vel_n*pvt.vel_n + pvt.vel_e*pvt.vel_e + pvt.vel_d*pvt.vel_d);
    mRawGps.cog_deg = pvt.head_mot;
    mRawGps.altitude_ellipsoid_m = pvt.height;
    mRawGps.horizontal_uncertainty_m = pvt.h_acc;
    mRawGps.vertical_uncertainty_m = pvt.v_acc;
    mRawGps.velocity_uncertainty_m_s = pvt.s_acc;
    mRawGps.heading_uncertainty_deg = pvt.head_acc;
    mRawGps.yaw_deg = pvt.head_veh;

    mGpsInfo.num_satellites = pvt.num_sv;
    mGpsInfo.fix_type = mavsdk::TelemetryServer::FixType::NoFix;
    if (pvt.fix_type == 2)
        mGpsInfo.fix_type = mavsdk::TelemetryServer::FixType::Fix2D;
    else if (pvt.fix_type == 3 || pvt.fix_type == 4) {
        mGpsInfo.fix_type= mavsdk::TelemetryServer::FixType::Fix3D;
        if (pvt.carr_soln == 1)
            mGpsInfo.fix_type= mavsdk::TelemetryServer::FixType::RtkFloat;
        else if (pvt.carr_soln == 2)
            mGpsInfo.fix_type= mavsdk::TelemetryServer::FixType::RtkFixed;
    }

}

void MavsdkVehicleServer::setMavsdkRawGpsAndGpsInfo(const mavsdk::TelemetryServer::RawGps &rawGps, const mavsdk::TelemetryServer::GpsInfo &gpsInfo)
{
    mRawGps = rawGps;
    mGpsInfo = gpsInfo;
}

void MavsdkVehicleServer::setWaypointFollower(QSharedPointer<WaypointFollower> waypointFollower)
{
    mWaypointFollower = waypointFollower;
    connect(this, &MavsdkVehicleServer::startWaypointFollower, mWaypointFollower.get(), &WaypointFollower::startFollowingRoute);
    connect(this, &MavsdkVehicleServer::pauseWaypointFollower, mWaypointFollower.get(), &WaypointFollower::stop);
    connect(this, &MavsdkVehicleServer::resetWaypointFollower, mWaypointFollower.get(), &WaypointFollower::resetState);
    connect(this, &MavsdkVehicleServer::clearRouteOnWaypointFollower, mWaypointFollower.get(), &WaypointFollower::clearRoute);
}

void MavsdkVehicleServer::setMovementController(QSharedPointer<MovementController> movementController)
{
    mMovementController = movementController;
}

void MavsdkVehicleServer::setFollowPoint(QSharedPointer<FollowPoint> followPoint)
{
    mFollowPoint = followPoint;
    connect(this, &MavsdkVehicleServer::startFollowPoint, mFollowPoint.get(), &FollowPoint::startFollowPoint);
    connect(this, &MavsdkVehicleServer::stopFollowPoint, mFollowPoint.get(), &FollowPoint::stopFollowPoint);
}

PosPoint MavsdkVehicleServer::convertMissionItemToPosPoint(const mavsdk::MissionRawServer::MissionItem &item)
{
    PosPoint routePoint;

    routePoint.setX(item.x / 10e4);
    routePoint.setY(item.y / 10e4);
    routePoint.setHeight(item.z);
     // TODO: does not follow MAV_CMD_NAV_WAYPOINT definition
    routePoint.setSpeed(item.param1);
    routePoint.setAttributes(item.param2);

//    qDebug() << routePoint.getX() << routePoint.getY() << routePoint.getHeight();

    return routePoint;
}

void MavsdkVehicleServer::handleManualControlMessage(mavlink_manual_control_t manualControl)
{
    if (!mMovementController.isNull()) {
        mVehicleState->setFlightMode(VehicleState::FlightMode::Manual);

        if (!mWaypointFollower.isNull() && mWaypointFollower->isActive()) {
            qDebug() << "MavsdkVehicleServer: WaypointFollower stopped by manual control input.";
            mWaypointFollower->stop();
        }

        mMovementController->setDesiredSpeed((manualControl.x / 1000.0) * mManualControlMaxSpeed);
        mMovementController->setDesiredSteering(manualControl.r / 1000.0);
    } else
        qDebug() << "Warning: MavsdkVehicleServer got manual control message, but has no MovementController to talk to.";
}

void MavsdkVehicleServer::setManualControlMaxSpeed(double manualControlMaxSpeed_ms)
{
    mManualControlMaxSpeed = manualControlMaxSpeed_ms;
}

double MavsdkVehicleServer::getManualControlMaxSpeed() const
{
    return mManualControlMaxSpeed;
}

void MavsdkVehicleServer::mavResult(const uint16_t command, MAV_RESULT result, MAV_COMPONENT compId)
{
    if (mMavlinkPassthrough->queue_message([&](MavlinkAddress mavlink_address, uint8_t channel) {
        mavlink_message_t ackMsg;
        mavlink_command_ack_t commandAck;
        memset(&commandAck, 0, sizeof(commandAck));
        commandAck.command = command;
        commandAck.result = result;
        commandAck.progress = std::numeric_limits<uint8_t>::max();
        commandAck.result_param2 = 0;
        commandAck.target_system = mMavlinkPassthrough->get_target_sysid();
        commandAck.target_component = mMavlinkPassthrough->get_target_compid();

        mavlink_address.system_id = mSystemId;
        mavlink_address.component_id = compId;

        mavlink_msg_command_ack_encode_chan(mavlink_address.system_id, mavlink_address.component_id, channel, &ackMsg, &commandAck);
        return ackMsg;
    }) != mavsdk::MavlinkPassthrough::Result::Success)
            qWarning() << "Could not send ACK via MAVLINK.";
};

void MavsdkVehicleServer::sendGpsOriginLlh(const llh_t &gpsOriginLlh)
{
    if (mMavlinkPassthrough == nullptr)
        return;

    if (mMavlinkPassthrough->queue_message([&](MavlinkAddress mavlink_address, uint8_t channel) {
        mavlink_message_t mavGpsGlobalOriginMsg;
        mavlink_gps_global_origin_t mavGpsGlobalOrigin;
        memset(&mavGpsGlobalOrigin, 0, sizeof(mavlink_gps_global_origin_t));

        mavGpsGlobalOrigin.latitude = (int) (gpsOriginLlh.latitude * 1e7);
        mavGpsGlobalOrigin.longitude = (int) (gpsOriginLlh.longitude * 1e7);
        mavGpsGlobalOrigin.altitude = (int) (gpsOriginLlh.height * 1e3);

        mavlink_address.system_id = mSystemId;
        mavlink_address.component_id = MAV_COMP_ID_AUTOPILOT1;

        mavlink_msg_gps_global_origin_encode_chan(mavlink_address.system_id, mavlink_address.component_id, channel, &mavGpsGlobalOriginMsg, &mavGpsGlobalOrigin);
        return mavGpsGlobalOriginMsg;
    }) != mavsdk::MavlinkPassthrough::Result::Success)
            qWarning() << "Could not send GPS_GLOBAL_ORIGIN via MAVLINK.";
};

void MavsdkVehicleServer::on_logSent(const QString& message, const quint8& severity)
{
    struct logQueueItem {
        QString log;
        quint16 id;
        quint8 severity;
    };

    static QVector<logQueueItem> logQueue;

    logQueueItem item;

    item.log = message;
    item.severity = severity;

    logQueue.push_back(item);

    if(!mMavlinkPassthrough)
        return;

    for(const logQueueItem &item : logQueue) {

        static quint16 idCounter = 1;

        const int chunkSize = MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN - 8; // STATUSTEXT supports up to char[50], -8 adjustment to fit MAVLink header
        int numChunks = (item.log.length() + chunkSize - 1) / chunkSize;

        if(item.log.length() % chunkSize == 0)  // need extra message for null terminator, so to not overwrite any message character
            numChunks++;

        for(int chunkIndex = 0; chunkIndex < numChunks; chunkIndex++) {
            QString chunk = item.log.mid(chunkIndex * chunkSize, chunkSize);
            QByteArray chunkBytes = chunk.toUtf8();

            mavlink_statustext_t statusText;    // send text messages to ground-station or other MAVLink-enabled systems
            memset(&statusText, 1, sizeof(statusText));

            statusText.severity = item.severity;

            qstrcpy(statusText.text, chunkBytes.constData());

            if(chunkIndex == numChunks - 1)
                statusText.text[MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN - 1] = '\0'; // ensure null terminated

            if(numChunks > 1)
                statusText.id = idCounter;
            else
                statusText.id = 0;  // message can be omitted directly

            statusText.chunk_seq = chunkIndex;
            if (mMavlinkPassthrough->queue_message([&](MavlinkAddress mavlink_address, uint8_t channel) {
                mavlink_message_t mavLogMsg;
                mavlink_address.system_id = mSystemId;
                mavlink_address.component_id = MAV_COMP_ID_AUTOPILOT1;
                mavlink_msg_statustext_encode_chan(mavlink_address.system_id, mavlink_address.component_id, channel, &mavLogMsg, &statusText);

                return mavLogMsg;
            }) != mavsdk::MavlinkPassthrough::Result::Success)
                    qWarning() << "Could not send log (STATUSTEXT) via MAVLINK.";
        }

        if(idCounter == std::numeric_limits<typeof idCounter>::max())  // overflow avoidance
            idCounter = 1;
        else
            idCounter++;
    }

    logQueue.clear();
}

void MavsdkVehicleServer::sendMissionAck(quint8 type)
{
    if (mMavlinkPassthrough->queue_message([&](MavlinkAddress mavlink_address, uint8_t channel) {
        mavlink_mission_ack_t missionAck;
        mavlink_message_t mavMissionAckMsg;

        missionAck.target_system = mMavlinkPassthrough->get_target_sysid();
        missionAck.target_component = mMavlinkPassthrough->get_target_compid();
        missionAck.type = type;
        missionAck.mission_type = MAV_MISSION_TYPE_MISSION;

        mavlink_address.system_id = mSystemId;
        mavlink_address.component_id = MAV_COMP_ID_AUTOPILOT1;

        mavlink_msg_mission_ack_encode_chan(mavlink_address.system_id, mavlink_address.component_id, channel, &mavMissionAckMsg, &missionAck);

        return mavMissionAckMsg;
    }) != mavsdk::MavlinkPassthrough::Result::Success)
            qWarning() << "Could not send MISSION_ACK via MAVLINK.";
}

void MavsdkVehicleServer::createMavsdkComponentForTrailer(const QHostAddress controlTowerAddress, const unsigned controlTowerPort, const QAbstractSocket::SocketType controlTowerSocketType)
{
    auto trailerState = mVehicleState->getTrailingVehicle();

    mavsdk::Mavsdk::Configuration config =
        mavsdk::Mavsdk::Configuration{mavsdk::Mavsdk::ComponentType::Custom};
    config.set_system_id(mVehicleState->getId());
    config.set_always_send_heartbeats(true);
    config.set_component_id(trailerState->getId());
    mTrailerMavsdk.reset(new mavsdk::Mavsdk{config});

    mTrailerMavsdk->subscribe_on_new_system(
        [this]() {
            for (const auto & system : mTrailerMavsdk->systems()) {
                auto mavlinkPassthrough = new mavsdk::MavlinkPassthrough(system);
                mavlinkPassthrough->subscribe_message(
                    MAVLINK_MSG_ID_HEARTBEAT,
                    [this, mavlinkPassthrough, system](const mavlink_message_t & message) {
                        mavlink_heartbeat_t heartbeat;
                        mavlink_msg_heartbeat_decode(&message, &heartbeat);
                        // unsubscribe from further heartbeats by deleting passthrough
                        delete mavlinkPassthrough;
                        if ((MAV_TYPE) heartbeat.type == MAV_TYPE_GCS)
                            mTrailerMavlinkPassthrough.reset(new mavsdk::MavlinkPassthrough(system));
                    });
            }
        });

    mTrailerMavsdk->intercept_outgoing_messages_async(
        [trailerState](mavlink_message_t & message) {
            switch (message.msgid) {
            case MAVLINK_MSG_ID_HEARTBEAT: // Fix some info in heartbeat s.th. MAVSDK / ControlTower detects vehicle correctly
                mavlink_heartbeat_t heartbeat;
                mavlink_msg_heartbeat_decode(&message, &heartbeat);
                if (message.compid == trailerState->getId()) {
                    heartbeat.type = MAV_TYPE_ONBOARD_CONTROLLER;
                    heartbeat.autopilot = MAV_AUTOPILOT_INVALID;
                }
                mavlink_msg_heartbeat_encode(message.sysid, message.compid, &message, &heartbeat);
                break;
            default: ;
                //            qDebug() << "out:" << message.msgid;
            }
            return true;
        });

    mavsdk::ConnectionResult result;
    switch (controlTowerSocketType) {
    case QAbstractSocket::UdpSocket:
        result = mTrailerMavsdk->setup_udp_remote(controlTowerAddress.toString().toStdString(), controlTowerPort);
        break;
    default:
        qDebug() << "MavsdkVehicleServer initialized for unsupported controlTowerSocketType. Not connecting.";
        result = mavsdk::ConnectionResult::SocketError;
        break;
    }

    if (result == mavsdk::ConnectionResult::Success) {
        qDebug() << "Trailer component listening for MAVSDK connection.";

        connect(&mPublishMavlinkTimer, &QTimer::timeout, [this](){
            if (mTrailerMavlinkPassthrough && mTrailerMavlinkPassthrough->queue_message(
                [this](MavlinkAddress mavlink_address, uint8_t channel)->mavlink_message_t {
                    auto trailerState = mVehicleState->getTrailingVehicle();
                    mavlink_message_t trailerYawMsg;
                    mavlink_named_value_float_t trailerYaw;

                    trailerYaw.time_boot_ms = QDateTime::currentMSecsSinceEpoch() - mMavsdkVehicleServerCreationTime.toMSecsSinceEpoch();
                    trailerYaw.value = trailerState->getPosition(PosType::fused).getYaw();
                    mavlink_address.system_id = mVehicleState->getId();
                    mavlink_address.component_id = mVehicleState->getTrailingVehicle()->getId();

                    strcpy(trailerYaw.name, "TRLR_YAW");
                    mavlink_msg_named_value_float_encode_chan(mavlink_address.system_id, mavlink_address.component_id, channel, &trailerYawMsg, &trailerYaw);

                    return trailerYawMsg;
                }) != mavsdk::MavlinkPassthrough::Result::Success)
                    qWarning() << "Could not send Trailer Yaw via MAVLINK.";
        });
    }
}

