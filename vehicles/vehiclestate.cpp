/*
 *     Copyright 2012 Benjamin Vedder   benjamin@vedder.se
 *               2020 Marvin Damschen   marvin.damschen@ri.se
 *               2021 Lukas Wikander    lukas.wikander@astazero.com
 *               2022 Rickard Häll      rickard.hall@ri.se
 *     Published under GPLv3: https://www.gnu.org/licenses/gpl-3.0.html
 */
#include "vehiclestate.h"
#include <QDebug>

VehicleState::VehicleState(ObjectID_t id, Qt::GlobalColor color)
    : ObjectState (id, color)
{
    mTime = QTime();
    mLength = 0.8;
    mWidth = 0.335;
    mEnuReference = {57.71495867, 12.89134921, 219.0}; // AztaZero {57.7810, 12.7692, 0}, Klätterlabbet {57.6876, 11.9807, 0};

    for (int i = 0; i < (int)PosType::_LAST_; i++)
        switch((PosType) i) {
        case PosType::simulated: mPositionBySource[i].setType(PosType::simulated); break;
        case PosType::fused: mPositionBySource[i].setType(PosType::fused); break;
        case PosType::odom: mPositionBySource[i].setType(PosType::odom); break;
        case PosType::IMU: mPositionBySource[i].setType(PosType::IMU); break;
        case PosType::GNSS: mPositionBySource[i].setType(PosType::GNSS); break;
        case PosType::UWB: mPositionBySource[i].setType(PosType::UWB); break;
        case PosType::_LAST_: qDebug() << "This should not have happended."; break;

        }
}


void VehicleState::setPosition(PosPoint &point)
{
    mPositionBySource[(int)point.getType()] = point;

    emit positionUpdated();
}

void VehicleState::simulationStep(double dt_ms, PosType usePosType)
{
    double drivenDistance = getSpeed() * dt_ms / 1000;
    updateOdomPositionAndYaw(drivenDistance, usePosType);
}

std::array<float, 3> VehicleState::getGyroscopeXYZ() const
{
    return mGyroscopeXYZ;
}

void VehicleState::setGyroscopeXYZ(const std::array<float, 3> &gyroscopeXYZ)
{
    mGyroscopeXYZ = gyroscopeXYZ;
}

std::array<float, 3> VehicleState::getAccelerometerXYZ() const
{
    return mAccelerometerXYZ;
}

void VehicleState::setAccelerometerXYZ(const std::array<float, 3> &accelerometerXYZ)
{
    mAccelerometerXYZ = accelerometerXYZ;
}

double VehicleState::getSteering() const
{
    return mSteering;
}

void VehicleState::setSteering(double steering)
{
    if (abs(steering) > 1.0)
        steering = steering / abs(steering);

    mSteering = steering;
}

PosPoint VehicleState::getHomePosition() const
{
    return mHomePosition;
}

void VehicleState::setHomePosition(const PosPoint &homePosition)
{
    mHomePosition = homePosition;
}

llh_t VehicleState::getEnuRef() const
{
    return mEnuReference;
}

void VehicleState::setEnuRef(llh_t enuRef)
{
    mEnuReference = enuRef;
}

bool VehicleState::getIsArmed() const
{
    return mIsArmed;
}

void VehicleState::setIsArmed(bool isArmed)
{
    mIsArmed = isArmed;
}

PosPoint VehicleState::getPosition(PosType type) const
{
    return mPositionBySource[(int)type];
}

PosPoint VehicleState::getOffsetPosition(xyz_t offset, PosType type) const
{
    return getOffsetPosition(offset, getPosition(type).getYaw() * M_PI / 180.0, type);
}

PosPoint VehicleState::getOffsetPosition(xyz_t offset, double yaw_rad , PosType type) const
{
    PosPoint offsetPosition = getPosition(type);
    offsetPosition.transform(offset, yaw_rad);
    return offsetPosition;
}

VehicleState::FlightMode VehicleState::getFlightMode() const
{
    return mFlightMode;
}

void VehicleState::setFlightMode(const FlightMode &flightMode)
{
    mFlightMode = flightMode;
}

void VehicleState::setAutopilotRadius(double radius)
{
    mAutopilotRadius = radius;
}

double VehicleState::getAutopilotRadius()
{
    return mAutopilotRadius;
}

double VehicleState::getCurvatureToPointInVehicleFrame(const QPointF &point)
{
    // calc steering angle (pure pursuit)
    double distanceSquared = pow(point.x(), 2) + pow(point.y(), 2);
    double steeringAngleProportional = (2*point.y()) / distanceSquared;

    return -steeringAngleProportional;
}

double VehicleState::getCurvatureToPointInENU(const QPointF &point, PosType type)
{
    PosPoint vehiclePosition = mPositionBySource[(int)type];

    return getCurvatureToPointInVehicleFrame(coordinateTransforms::ENUToVehicleFrame(point, vehiclePosition.getXYZ(), vehiclePosition.getYaw()));
}

QSharedPointer<VehicleState> VehicleState::getTrailingVehicle() const
{
    return mTrailingVehicle;
}

void VehicleState::setTrailingVehicle(QSharedPointer<VehicleState> trailer)
{
    mTrailingVehicle = trailer;
}

bool VehicleState::hasTrailingVehicle() const
{
    return !mTrailingVehicle.isNull();
}
