/*
 *     Copyright 2022 RISE Research Institutes of Sweden AB, Safety and Transport   waywise@ri.se
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

    // Set up param plugin
    mParam.reset(new mavsdk::Param(mSystem));

    switch (mVehicleType) {
    case MAV_TYPE_QUADROTOR:
        qDebug() << "MavsdkVehicleConnection: we are talking to a MAV_TYPE_QUADROTOR / PX4.";
        mVehicleState = QSharedPointer<CopterState>::create(mSystem->get_system_id());
        mVehicleState->setName("Copter " + QString::number(mSystem->get_system_id()));

        // Setup gimbal
        mComponentDiscoveredHandle = mSystem->subscribe_component_discovered([this](mavsdk::System::ComponentType component_type){
            if (component_type == mavsdk::System::ComponentType::GIMBAL && mSystem->has_gimbal() && mGimbal.isNull()) {
                mGimbal = QSharedPointer<MavsdkGimbal>::create(mSystem);
                emit detectedGimbal(mGimbal);
            }
        });
        if (mSystem->has_gimbal() && mGimbal.isNull()) // Gimbal might have been discovered before callback was registered
            mGimbal = QSharedPointer<MavsdkGimbal>::create(mSystem);

        break;
    case MAV_TYPE_GROUND_ROVER:
        {
            WAYWISE_OBJECT_TYPE mWaywiseObjectType = WAYWISE_OBJECT_TYPE_CAR;
            auto waywiseObjTypeParamResult = getIntParameterFromVehicle("VEH_WW_OBJ_TYPE");
            if (waywiseObjTypeParamResult.first == VehicleConnection::Result::Success) {
                mWaywiseObjectType = static_cast<WAYWISE_OBJECT_TYPE>(waywiseObjTypeParamResult.second);
                if (!(mWaywiseObjectType == WAYWISE_OBJECT_TYPE_TRUCK || mWaywiseObjectType == WAYWISE_OBJECT_TYPE_CAR)){
                    qDebug() << "VEH_WW_OBJ_TYPE param is set to unsupported type for MAV_TYPE_GROUND_ROVER. We asuume that it is a car type.";
                    mWaywiseObjectType = WAYWISE_OBJECT_TYPE_CAR;
                }
            }

            switch (mWaywiseObjectType) {
                case WAYWISE_OBJECT_TYPE_TRUCK:
                {
                    qDebug() << "MavsdkVehicleConnection: we are talking to a MAV_TYPE_GROUND_ROVER / WayWise Truck.";
                    mVehicleState = QSharedPointer<TruckState>::create(mSystem->get_system_id());
                    mVehicleState->setName("Truck " + QString::number(mSystem->get_system_id()));
                    QSharedPointer<TruckState> mTruckState = qSharedPointerDynamicCast<TruckState>(mVehicleState);
                    setupTruckState(mTruckState);
                    break;
                }
                default:
                {
                    qDebug() << "MavsdkVehicleConnection: we are talking to a MAV_TYPE_GROUND_ROVER / WayWise Car.";
                    mVehicleState = QSharedPointer<CarState>::create(mSystem->get_system_id());
                    mVehicleState->setName("Car " + QString::number(mSystem->get_system_id()));
                    QSharedPointer<CarState> mCarState = qSharedPointerDynamicCast<CarState>(mVehicleState);
                    setupCarState(mCarState);
                    break;
                }
            }

            if (system->has_autopilot())
            {
                auto paramResult = getIntParameterFromVehicle("PP_EGA_TYPE");
                if (paramResult.first == VehicleConnection::Result::Success) {
                    mVehicleState->setEndGoalAlignmentType(static_cast<AutopilotEndGoalAlignmentType>(paramResult.second));
                }
            }

            break;
        }
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
        xyz_t velocityNED {velocity.north_m_s, velocity.east_m_s, velocity.down_m_s};
        xyz_t velocityENU = coordinateTransforms::nedToENU(velocityNED);
        mVehicleState->setVelocity(velocityENU);
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
    MavsdkVehicleConnection::pollCurrentENUreference();


    mMavlinkPassthrough->subscribe_message(MAVLINK_MSG_ID_HEARTBEAT, [this](const mavlink_message_t &message) {
        Q_UNUSED(message)
        emit gotHeartbeat(mSystem->get_system_id());
    });

    mMavlinkPassthrough->subscribe_message(MAVLINK_MSG_ID_STATUSTEXT, [](const mavlink_message_t &message)
    {
        struct messageChunk {
            QString text;
            quint8 chunk_seq;
        };

        static QVector<messageChunk> messageChunks;
        static quint16 currentID = 0;

        mavlink_statustext_t statusText;
        mavlink_msg_statustext_decode(&message, &statusText);

        if(statusText.id != 0) {    // non-zero id -> statusText is a sequence of chunks that need to be assembled
            if(statusText.id != currentID) {
                if(!messageChunks.isEmpty()) {
                    std::sort(messageChunks.begin(), messageChunks.end(), [](const messageChunk &chunk1, const messageChunk &chunk2) {
                        return chunk1.chunk_seq < chunk2.chunk_seq;
                    });

                    QString messageOutput = "[Vehicle] ";

                    for(const messageChunk &chunk : messageChunks)
                        messageOutput.append(chunk.text);

                    switch (statusText.severity)
                    {
                    case MAV_SEVERITY_DEBUG:
                        qDebug() << messageOutput;
                        break;
                    case MAV_SEVERITY_INFO:
                        qInfo() << messageOutput;
                        break;
                    case MAV_SEVERITY_WARNING:
                        qWarning() << messageOutput;
                        break;
                    case MAV_SEVERITY_CRITICAL:
                        qCritical() << messageOutput;
                        break;
                    case MAV_SEVERITY_EMERGENCY:
                        qFatal("%s", qPrintable(messageOutput));
                        break;
                    }

                    messageChunks.clear();
                }
                currentID = statusText.id;
            }

            messageChunk chunk;
            chunk.text = statusText.text;
            chunk.chunk_seq = statusText.chunk_seq;
            messageChunks.push_back(chunk);
        }

        if(statusText.id == 0 || statusText.text[MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN - 1] == '\0') {  // we have received the complete statusText
            QString messageOutput = "[Vehicle] ";

            if(statusText.id == 0)  // id being zero indicates there is only one chunk
                messageOutput.append(statusText.text);
            else {
                std::sort(messageChunks.begin(), messageChunks.end(), [](const messageChunk &chunk1, const messageChunk &chunk2) {
                    return chunk1.chunk_seq < chunk2.chunk_seq;
                });

                for(const messageChunk &chunk : messageChunks)
                    messageOutput.append(chunk.text);

                messageChunks.clear();
            }

            switch (statusText.severity)
            {
            case MAV_SEVERITY_DEBUG:
                qDebug() << messageOutput;
                break;
            case MAV_SEVERITY_INFO:
                qInfo() << messageOutput;
                break;
            case MAV_SEVERITY_WARNING:
                qWarning() << messageOutput;
                break;
            case MAV_SEVERITY_CRITICAL:
                qCritical() << messageOutput;
                break;
            case MAV_SEVERITY_EMERGENCY:
                qFatal("%s", qPrintable(messageOutput));
                break;
            }
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

    // Autopilot Target Point
    mMavlinkPassthrough->subscribe_message(MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED, [this](const mavlink_message_t &mavMsg) {
        mavlink_position_target_local_ned_t autopilotPoints;
        mavlink_msg_position_target_local_ned_decode(&mavMsg, &autopilotPoints);
        xyz_t autopilotTargetPointENU = coordinateTransforms::nedToENU({autopilotPoints.x, autopilotPoints.y, 0});
        mVehicleState->setAutopilotTargetPoint(QPointF(autopilotTargetPointENU.x, autopilotTargetPointENU.y));
    });

    // Set up action plugin
    mAction.reset(new mavsdk::Action(mSystem));

// TODO: this should not happen here (blocking)
//    // Precision Landing: set required target tracking accuracy for starting approach
//    if (mParam->set_param_float("PLD_HACC_RAD", 5.0) != mavsdk::Param::Result::Success)
//        qDebug() << "Warning: failed to set PLD_HACC_RAD";

    // Necessary such that MAVSDK callbacks (from other threads) can stop WaypointFollower (QTimer)
    connect(this, &MavsdkVehicleConnection::stopWaypointFollowerSignal, this, &MavsdkVehicleConnection::stopAutopilot);
}

void MavsdkVehicleConnection::setupCarState(QSharedPointer<CarState> carState)
{
    auto vehicleParamResult = getFloatParameterFromVehicle("VEH_LENGTH");
    if (vehicleParamResult.first == VehicleConnection::Result::Success) {
        carState->setLength(vehicleParamResult.second);
    }
    vehicleParamResult = getFloatParameterFromVehicle("VEH_WIDTH");
    if (vehicleParamResult.first == VehicleConnection::Result::Success) {
        carState->setWidth(vehicleParamResult.second);
    }
    vehicleParamResult = getFloatParameterFromVehicle("VEH_WHLBASE");
    if (vehicleParamResult.first == VehicleConnection::Result::Success) {
        carState->setAxisDistance(vehicleParamResult.second);
    }

    vehicleParamResult = getFloatParameterFromVehicle("VEH_RA2CO_X");
    if (vehicleParamResult.first == VehicleConnection::Result::Success) {
        carState->setRearAxleToCenterOffset(vehicleParamResult.second);
    }
    vehicleParamResult = getFloatParameterFromVehicle("VEH_RA2REO_X");
    if (vehicleParamResult.first == VehicleConnection::Result::Success) {
        carState->setRearAxleToRearEndOffset(vehicleParamResult.second);
    }
    carState->setStateInitialized(true);
}

void MavsdkVehicleConnection::setupTruckState(QSharedPointer<TruckState> truckState)
{
    setupCarState(truckState);
    auto vehicleParamResult = getFloatParameterFromVehicle("VEH_RA2HO_X");
    if (vehicleParamResult.first == VehicleConnection::Result::Success) {
        truckState->setRearAxleToHitchOffset(vehicleParamResult.second);
    }
    truckState->setStateInitialized(true);

    auto trailerParamResult = getIntParameterFromVehicle("TRLR_COMP_ID");
    if (trailerParamResult.first == VehicleConnection::Result::Success) {
        int trailer_component_id = trailerParamResult.second;
        if (trailer_component_id>=0) {
            // Setup Trailer
            mavsdk::System::ComponentDiscoveredIdHandle mComponentDiscoveredIdHandle = mSystem->subscribe_component_discovered_id([this, truckState, trailer_component_id](mavsdk::System::ComponentType component_type, uint8_t component_id){
                if (component_type == mavsdk::System::ComponentType::UNKNOWN && component_id == (uint8_t) trailer_component_id) {
                    qDebug() << "Trailer discovered with system ID "<< mSystem->get_system_id() << " and component ID "<< component_id;
                    truckState->setTrailingVehicle(QSharedPointer<TrailerState>::create(component_id));
                    QSharedPointer<TrailerState> mTrailerState = truckState->getTrailingVehicle();
                    mTrailerState->setName("Trailer " + QString::number(mSystem->get_system_id()));

                    auto trailerParamResult = getFloatParameterFromVehicle("TRLR_LENGTH");
                    if (trailerParamResult.first == VehicleConnection::Result::Success) {
                        mTrailerState->setLength(trailerParamResult.second);
                    }
                    trailerParamResult = getFloatParameterFromVehicle("TRLR_WIDTH");
                    if (trailerParamResult.first == VehicleConnection::Result::Success) {
                        mTrailerState->setWidth(trailerParamResult.second);
                    }
                    trailerParamResult = getFloatParameterFromVehicle("TRLR_WHLBASE");
                    if (trailerParamResult.first == VehicleConnection::Result::Success) {
                        mTrailerState->setWheelBase(trailerParamResult.second);
                    }

                    trailerParamResult = getFloatParameterFromVehicle("TRLR_RA2CO_X");
                    if (trailerParamResult.first == VehicleConnection::Result::Success) {
                        mTrailerState->setRearAxleToCenterOffset(trailerParamResult.second);
                    }
                    trailerParamResult = getFloatParameterFromVehicle("TRLR_RA2REO_X");
                    if (trailerParamResult.first == VehicleConnection::Result::Success) {
                        mTrailerState->setRearAxleToRearEndOffset(trailerParamResult.second);
                    }
                    trailerParamResult = getFloatParameterFromVehicle("TRLR_RA2HO_X");
                    if (trailerParamResult.first == VehicleConnection::Result::Success) {
                        qDebug() << "Got trailer hitch offset: "<< trailerParamResult.second;
                        mTrailerState->setRearAxleToHitchOffset(trailerParamResult.second);
                    }
                    mTrailerState->setStateInitialized(true);

                    mMavlinkPassthrough->subscribe_message(MAVLINK_MSG_ID_NAMED_VALUE_FLOAT, [truckState, component_id](const mavlink_message_t &message) {
                        if (message.compid == component_id) {
                            mavlink_named_value_float_t mavMsg;
                            mavlink_msg_named_value_float_decode(&message, &mavMsg);
                            if (strcmp(mavMsg.name,"TRLR_YAW") == 0) {
                                mavlink_msg_named_value_float_decode(&message, &mavMsg);
                                truckState->setTrailerAngle(truckState->getPosition().getYaw() - mavMsg.value);
                            }
                        }
                    });
                }
            });
        }
    }
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

    auto result = mMavlinkPassthrough->send_command_long(ComLong);
    if (result != mavsdk::MavlinkPassthrough::Result::Success)
        qDebug() << "Warning: MavsdkVehicleConnection's precision land request failed (" << convertMavlinkPassthroughResult(result) << ")";
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

    auto result = mMavlinkPassthrough->send_command_long(ComLong);
    if (result != mavsdk::MavlinkPassthrough::Result::Success)
        qDebug() << "Warning: MavsdkVehicleConnection's mode change request failed (" << convertMavlinkPassthroughResult(result) << ")";
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
    ComLong.param2 = 6; // PX4_CUSTOM_MAIN_MODE_OFFBOARD
    ComLong.param3 = 8; // PX4_CUSTOM_SUB_MODE_AUTO_FOLLOW_TARGET

    auto result = mMavlinkPassthrough->send_command_long(ComLong);
    if (result != mavsdk::MavlinkPassthrough::Result::Success)
        qDebug() << "Warning: MavsdkVehicleConnection's follow point request failed (" << convertMavlinkPassthroughResult(result) << ")";
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

        auto result = mMavlinkPassthrough->send_command_int(ComInt);
        if (result != mavsdk::MavlinkPassthrough::Result::Success)
            qDebug() << "Warning: MavsdkVehicleConnection's reposition request failed (" << convertMavlinkPassthroughResult(result) << ")";
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

    if (rtcmData.length() < MAVLINK_MSG_GPS_RTCM_DATA_FIELD_DATA_LEN) {
        auto result = mMavlinkPassthrough->queue_message([&](MavlinkAddress mavlink_address, uint8_t channel) {
                mavlink_message_t mavRtcmMsg;
                mavlink_gps_rtcm_data_t mavRtcmData;
                memset(&mavRtcmData, 0, sizeof(mavlink_gps_rtcm_data_t));

                mavRtcmData.len = rtcmData.length();
                mavRtcmData.flags = (sequenceId & 0x1F) << 3;
                memcpy(mavRtcmData.data, rtcmData.data(), rtcmData.size());

                mavlink_msg_gps_rtcm_data_encode_chan(mavlink_address.system_id, mavlink_address.component_id, channel, &mavRtcmMsg, &mavRtcmData);
                return mavRtcmMsg;
            });
        if (result != mavsdk::MavlinkPassthrough::Result::Success)
            qWarning() << "Could not send RTCM via MAVLINK (" << convertMavlinkPassthroughResult(result) << ")";
    } else { // rtcm data needs to be fragmented into multiple messages
        uint8_t fragmentId = 0;
        int numBytesProcessed = 0;
        while (numBytesProcessed < rtcmData.size()) {
            int fragmentLength = std::min(rtcmData.size() - numBytesProcessed, MAVLINK_MSG_GPS_RTCM_DATA_FIELD_DATA_LEN);
            auto result = mMavlinkPassthrough->queue_message([&](MavlinkAddress mavlink_address, uint8_t channel) {
                    mavlink_message_t mavRtcmMsg;
                    mavlink_gps_rtcm_data_t mavRtcmData;
                    memset(&mavRtcmData, 0, sizeof(mavlink_gps_rtcm_data_t));

                    mavRtcmData.flags = 1;                          // LSB set indicates message is fragmented
                    mavRtcmData.flags |= fragmentId++ << 1;         // Next 2 bits are fragment id
                    mavRtcmData.flags |= (sequenceId & 0x1F) << 3;  // Next 5 bits are sequence id
                    mavRtcmData.len = fragmentLength;
                    memcpy(mavRtcmData.data, rtcmData.data() + numBytesProcessed, fragmentLength);

                    mavlink_msg_gps_rtcm_data_encode_chan(mavlink_address.system_id, mavlink_address.component_id, channel, &mavRtcmMsg, &mavRtcmData);
                    return mavRtcmMsg;
                });
            if (result != mavsdk::MavlinkPassthrough::Result::Success)
                qWarning() << "Could not send RTCM via MAVLINK (" << convertMavlinkPassthroughResult(result) << ")";
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
    auto result = mMavlinkPassthrough->queue_message([&](MavlinkAddress mavlink_address, uint8_t channel) {
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

            mavlink_msg_landing_target_encode_chan(mavlink_address.system_id, mavlink_address.component_id, channel, &mavLandingTargetMsg, &mavLandingTargetNED);
            return mavLandingTargetMsg;
        });
    if (result != mavsdk::MavlinkPassthrough::Result::Success)
        qWarning() << "Could not send LANDING_TARGET via MAVLINK (" << convertMavlinkPassthroughResult(result) << ")";
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

    auto result = mMavlinkPassthrough->queue_message([&](MavlinkAddress mavlink_address, uint8_t channel) {
            mavlink_message_t mavGpsGlobalOriginMsg;
            mavlink_set_gps_global_origin_t mavGpsGlobalOrigin;
            memset(&mavGpsGlobalOrigin, 0, sizeof(mavlink_set_gps_global_origin_t));

            mavGpsGlobalOrigin.latitude = (int) (gpsOriginLlh.latitude * 1e7);
            mavGpsGlobalOrigin.longitude = (int) (gpsOriginLlh.longitude * 1e7);
            mavGpsGlobalOrigin.altitude = (int) (gpsOriginLlh.height * 1e3);
            mavGpsGlobalOrigin.target_system = mMavlinkPassthrough->get_target_sysid();

            mavlink_msg_set_gps_global_origin_encode_chan(mavlink_address.system_id, mavlink_address.component_id, channel, &mavGpsGlobalOriginMsg, &mavGpsGlobalOrigin);
            return mavGpsGlobalOriginMsg;
        });
    if (result != mavsdk::MavlinkPassthrough::Result::Success)
        qDebug() << "Warning: could not send GPS_GLOBAL_ORIGIN via MAVLINK (" << convertMavlinkPassthroughResult(result) << ")";
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
    manual_control.target = mSystem->get_system_id();
    manual_control.x = (uint16_t) (x * 1000.0);
    manual_control.y = (uint16_t) (y * 1000.0);
    manual_control.z = (uint16_t) (z * 1000.0);
    manual_control.r = (uint16_t) (r * 1000.0);
    manual_control.buttons = buttonStateMask;

    auto result = mMavlinkPassthrough->queue_message([&](MavlinkAddress mavlink_address, uint8_t channel) {
            mavlink_message_t message;
            mavlink_msg_manual_control_encode_chan(mavlink_address.system_id, mavlink_address.component_id, channel, &message, &manual_control);
            return message;
        });
    if (result != mavsdk::MavlinkPassthrough::Result::Success)
        qDebug() << "Warning: could not send MANUAL_CONTROL via MAVLINK (" << convertMavlinkPassthroughResult(result) << ")";
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

    auto result = mMavlinkPassthrough->send_command_long(ComLong);
    if (result != mavsdk::MavlinkPassthrough::Result::Success)
        qDebug() << "Warning: could not send MISSION_SET_CURRENT via MAVLINK (" << convertMavlinkPassthroughResult(result) << ")";
}

void MavsdkVehicleConnection::requestGearSwitch(int gpioPin, VehicleConnection::Gearbox gear_pwm)
{
    if(!mMavlinkPassthrough)
        return;

    mavsdk::MavlinkPassthrough::CommandInt ComInt;
    memset(&ComInt, 0, sizeof (ComInt));
    ComInt.target_compid = mMavlinkPassthrough->get_target_compid();
    ComInt.target_sysid = mMavlinkPassthrough->get_target_sysid();
    ComInt.command = MAV_CMD_DO_SET_SERVO;
    ComInt.param1 = gpioPin;
    ComInt.param2 = static_cast<float>(gear_pwm);

    auto result = mMavlinkPassthrough->send_command_int(ComInt);
    if (result != mavsdk::MavlinkPassthrough::Result::Success)
        qDebug() << "Warning: MavsdkVehicleConnection's request to switch gear failed (" << convertMavlinkPassthroughResult(result) << ")";
}

bool MavsdkVehicleConnection::requestRebootOrShutdownOfSystemComponents(VehicleConnection::SystemComponent systemComponent, VehicleConnection::ComponentAction componentAction)
{
    mavsdk::MavlinkPassthrough::CommandLong ComLong;
    memset(&ComLong, 0, sizeof (ComLong));
    ComLong.target_compid = mMavlinkPassthrough->get_target_compid();
    ComLong.target_sysid = mMavlinkPassthrough->get_target_sysid();
    ComLong.command = MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN;
    ComLong.param1 = 0; // Autopilot
    ComLong.param2 = 0; // Onboard Computer
    ComLong.param3 = 0; // Component Action
    ComLong.param4 = 0; //MAVLink Component ID targeted in param3 (0 for all components)
    ComLong.param5 = 0; // Reserved (set to 0)
    ComLong.param6 = 0; // Reserved (set to 0)
    ComLong.param7 = -1; // WIP: ID (e.g camera ID -1 for all IDs)

    switch (systemComponent) {
    case VehicleConnection::SystemComponent::Autopilot:
        ComLong.param1 = (float)componentAction;
        break;
    case VehicleConnection::SystemComponent::OnboardComputer:
        ComLong.param2 = (float)componentAction;
        break;
    }

    auto result = mMavlinkPassthrough->send_command_long(ComLong);
    if (result != mavsdk::MavlinkPassthrough::Result::Success) {
        qDebug() << "Warning: could not send request for reboot or shutdown via MAVLINK (" << convertMavlinkPassthroughResult(result) << ")";
        return false;
    }
    return true;
}

VehicleConnection::Result MavsdkVehicleConnection::setIntParameterOnVehicle(std::string name, int32_t value)
{
    return convertParamResult(mParam->set_param_int(name, value));
}

VehicleConnection::Result MavsdkVehicleConnection::setFloatParameterOnVehicle(std::string name, float value)
{
    return convertParamResult(mParam->set_param_float(name, value));
}

VehicleConnection::Result MavsdkVehicleConnection::setCustomParameterOnVehicle(std::string name, std::string value)
{
    return convertParamResult(mParam->set_param_custom(name, value));
}

std::pair<VehicleConnection::Result, int32_t> MavsdkVehicleConnection::getIntParameterFromVehicle(std::string name) const
{
    auto intParameter =  mParam->get_param_int(name);

    return std::make_pair(convertParamResult(intParameter.first), intParameter.second);
};

std::pair<VehicleConnection::Result, float> MavsdkVehicleConnection::getFloatParameterFromVehicle(std::string name) const
{
    auto intParameter =  mParam->get_param_float(name);

    return std::make_pair(convertParamResult(intParameter.first), intParameter.second);
};

std::pair<VehicleConnection::Result, std::string> MavsdkVehicleConnection::getCustomParameterFromVehicle(std::string name) const
{
    auto intParameter =  mParam->get_param_custom(name);

    return std::make_pair(convertParamResult(intParameter.first), intParameter.second);
};

ParameterServer::AllParameters MavsdkVehicleConnection::getAllParametersFromVehicle()
{
    mavsdk::Param::AllParams mavsdkVehicleParameters = mParam->get_all_params();
    ParameterServer::IntParameter intParameter;
    ParameterServer::FloatParameter floatParameter;
    ParameterServer::CustomParameter customParameter;
    ParameterServer::AllParameters allParameters;

    for (const auto& vehicleParameter : mavsdkVehicleParameters.int_params) {
        intParameter.name = vehicleParameter.name;
        intParameter.value = vehicleParameter.value;
        intParameter.value = mParam->get_param_int(intParameter.name).second; //Remove this line when issue #72 is closed
        allParameters.intParameters.push_back(intParameter);
    }
    for (const auto& vehicleParameter : mavsdkVehicleParameters.float_params) {
        floatParameter.name = vehicleParameter.name;
        floatParameter.value = vehicleParameter.value;
        floatParameter.value = mParam->get_param_float(floatParameter.name).second; //Remove this line when issue #72 is closed
        allParameters.floatParameters.push_back(floatParameter);
    }
    for (const auto& vehicleParameter : mavsdkVehicleParameters.custom_params) {
        customParameter.name = vehicleParameter.name;
        customParameter.value = vehicleParameter.value;
        customParameter.value = mParam->get_param_custom(customParameter.name).second; //Remove this line when issue #72 is closed
        allParameters.customParameters.push_back(customParameter);
    }

    return allParameters;
}

void MavsdkVehicleConnection::pollCurrentENUreference()
{
    mTelemetry->get_gps_global_origin_async([this](mavsdk::Telemetry::Result result, mavsdk::Telemetry::GpsGlobalOrigin gpsGlobalOrigin){
        if (result == mavsdk::Telemetry::Result::Success){
            mGpsGlobalOrigin = {gpsGlobalOrigin.latitude_deg, gpsGlobalOrigin.longitude_deg, gpsGlobalOrigin.altitude_m};
            emit gotVehicleENUreferenceLlh(mGpsGlobalOrigin);
        }
    });
}

VehicleConnection::Result MavsdkVehicleConnection::convertParamResult(mavsdk::Param::Result result) const
{
    switch (result) {
    case mavsdk::Param::Result::Success:
        return VehicleConnection::Result::Success;
    case mavsdk::Param::Result::Unknown:
        return VehicleConnection::Result::Unknown;
    case mavsdk::Param::Result::WrongType:
        return VehicleConnection::Result::WrongType;
    case mavsdk::Param::Result::Timeout:
        return VehicleConnection::Result::Timeout;
    case mavsdk::Param::Result::ParamValueTooLong:
        return VehicleConnection::Result::ParamValueTooLong;
    case mavsdk::Param::Result::ParamNameTooLong:
        return VehicleConnection::Result::ParamNameTooLong;
    case mavsdk::Param::Result::NoSystem:
        return VehicleConnection::Result::NoSystem;
    case mavsdk::Param::Result::ConnectionError:
        return VehicleConnection::Result::ConnectionError;
    default:
        return VehicleConnection::Result::Unknown;
    }
}

QString MavsdkVehicleConnection::convertMissionRawResult(mavsdk::MissionRaw::Result result) const
{
    switch (result) {
        case mavsdk::MissionRaw::Result::Unknown:
            return "Unknown result.";
        case mavsdk::MissionRaw::Result::Error:
            return "Error.";
        case mavsdk::MissionRaw::Result::TooManyMissionItems:
            return "Too many mission items in the mission.";
        case mavsdk::MissionRaw::Result::Busy:
            return "Vehicle is busy.";
        case mavsdk::MissionRaw::Result::Timeout:
            return "Request timed out.";
        case mavsdk::MissionRaw::Result::InvalidArgument:
            return "Invalid argument.";
        case mavsdk::MissionRaw::Result::Unsupported:
            return "Mission downloaded from the system is not supported.";
        case mavsdk::MissionRaw::Result::NoMissionAvailable:
            return "No mission available on the system.";
        case mavsdk::MissionRaw::Result::TransferCancelled:
            return "Mission transfer (upload or download) has been cancelled.";
        case mavsdk::MissionRaw::Result::FailedToOpenQgcPlan:
            return "Failed to open the QGroundControl plan.";
        case mavsdk::MissionRaw::Result::FailedToParseQgcPlan:
            return "Failed to parse the QGroundControl plan.";
        case mavsdk::MissionRaw::Result::NoSystem:
            return "No system connected.";
        default:
            return "Unknown result.";
    }
}

QString MavsdkVehicleConnection::convertMavlinkPassthroughResult(mavsdk::MavlinkPassthrough::Result result) const
{
    switch (result) {
        case mavsdk::MavlinkPassthrough::Result::Unknown:
            return "Unknown error.";
        case mavsdk::MavlinkPassthrough::Result::Success:
            return "Success.";
        case mavsdk::MavlinkPassthrough::Result::ConnectionError:
            return "Connection Error.";
        case mavsdk::MavlinkPassthrough::Result::CommandNoSystem:
            return "System not available.";
        case mavsdk::MavlinkPassthrough::Result::CommandBusy:
            return "System is busy.";
        case mavsdk::MavlinkPassthrough::Result::CommandDenied:
            return "Command has been denied.";
        case mavsdk::MavlinkPassthrough::Result::CommandUnsupported:
            return "Command is not supported.";
        case mavsdk::MavlinkPassthrough::Result::CommandTimeout:
            return "A timeout happened.";
        case mavsdk::MavlinkPassthrough::Result::CommandTemporarilyRejected:
            return "Command has been rejected for now.";
        case mavsdk::MavlinkPassthrough::Result::CommandFailed:
            return "Command has failed.";
        case mavsdk::MavlinkPassthrough::Result::ParamWrongType:
            return "Wrong type for requested param.";
        case mavsdk::MavlinkPassthrough::Result::ParamNameTooLong:
            return "Param name too long.";
        case mavsdk::MavlinkPassthrough::Result::ParamValueTooLong:
            return "Param value too long.";
        case mavsdk::MavlinkPassthrough::Result::ParamNotFound:
            return "Param not found.";
        case mavsdk::MavlinkPassthrough::Result::ParamValueUnsupported:
            return "Param value unsupported.";
        default:
            return "Unknown result.";
    }  
}

QList<PosPoint> MavsdkVehicleConnection::requestCurrentRouteFromVehicle()
{    
    if (!mMissionRaw)
        mMissionRaw.reset(new mavsdk::MissionRaw(mSystem));

    std::pair<mavsdk::MissionRaw::Result, std::vector<mavsdk::MissionRaw::MissionItem>> result = mMissionRaw->download_mission();

    QList<PosPoint> currentRouteOnVehicle;

    if (result.first != mavsdk::MissionRaw::Result::Success) {
        qWarning() << "Mission download failed (" << convertMissionRawResult(result.first) << "), exiting.";
        return currentRouteOnVehicle;
    }

    qInfo() << "Mission downloaded, number of items: " << result.second.size();

    for(unsigned i = 0; i < result.second.size(); i++) {
        PosPoint routePoint;

        routePoint.setX(result.second.at(i).x / 10e4);
        routePoint.setY(result.second.at(i).y / 10e4);
        routePoint.setHeight(result.second.at(i).z);
        routePoint.setSpeed(result.second.at(i).param1);
        routePoint.setAttributes(result.second.at(i).param2);

        currentRouteOnVehicle.append(routePoint);
    }

    return currentRouteOnVehicle;
}

void MavsdkVehicleConnection::startFollowPointOnVehicle()
{
    if (mVehicleType == MAV_TYPE_GROUND_ROVER) // WayWise
        requestFollowPoint();
    else // PX4
        throw  std::logic_error("PX4 vehicles only support follow point on remote connection");
}

void MavsdkVehicleConnection::stopFollowPointOnVehicle()
{
    if (mVehicleType == MAV_TYPE_GROUND_ROVER) // WayWise
        pauseAutopilotOnVehicle();
    else // PX4
        throw  std::logic_error("PX4 vehicles only support follow point on remote connection");
}
