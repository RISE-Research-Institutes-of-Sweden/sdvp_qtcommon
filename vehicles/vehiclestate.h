/*
 *     Copyright 2012 Benjamin Vedder   benjamin@vedder.se
 *               2020 Marvin Damschen   marvin.damschen@ri.se
 *               2021 Lukas Wikander    lukas.wikander@astazero.com
 *               2022 Rickard Häll      rickard.hall@ri.se
 *     Published under GPLv3: https://www.gnu.org/licenses/gpl-3.0.html
 * 
 * Specialisation of ObjectState for vehicles, storing all (dynamic and static) state
 */

#ifndef VEHICLESTATE_H
#define VEHICLESTATE_H

#include <QObject>
#include <QVector>
#include <QString>
#include <QSharedPointer>
#ifdef QT_GUI_LIB
#include <QPainter>
#endif

#include "core/pospoint.h"
#include "vehicles/objectstate.h"
#include <math.h>

class VehicleState : public ObjectState
{
    Q_OBJECT
public:
    // like MAVSDK FlightMode
    enum class FlightMode {
        Unknown,
        Ready,
        Takeoff,
        Hold,
        Mission,
        ReturnToLaunch,
        Land,
        Offboard,
        FollowMe,
        Manual,
        Altctl,
        Posctl,
        Acro,
        Stabilized,
        Rattitude
    };

    VehicleState(ObjectID_t id = 1, Qt::GlobalColor color = Qt::red);

    // Static state
    double getLength() const { return mLength; }
    virtual void setLength(double length) { mLength = length; }
    double getWidth() const { return mWidth; }
    void setWidth(double width) { mWidth = width; }
    double getMinAcceleration() const { return mMinAcceleration; }
    void setMinAcceleration(double minAcceleration) { mMinAcceleration = minAcceleration; }
    double getMaxAcceleration() const { return mMaxAcceleration; }
    void setMaxAcceleration(double maxAcceleration) { mMaxAcceleration = maxAcceleration; }

    xyz_t getRearAxleOffset() const { return mRearAxleOffset; }
    void setRearAxleOffset(xyz_t rearAxleOffset) { mRearAxleOffset = rearAxleOffset; }
    xyz_t getRearEndOffset() const { return mRearEndOffset; }
    void setRearEndOffset(xyz_t rearEndOffset) { mRearEndOffset = rearEndOffset; }
    xyz_t getHitchOffset() const { return mHitchOffset; }
    void setHitchOffset(xyz_t hitchOffset) { mHitchOffset = hitchOffset; }
    xyz_t getGnssOffset() const { return mGnssOffset; }
    void setGnssOffset(xyz_t gnssOffset) { mGnssOffset = gnssOffset; }
    xyz_t getImuOffset() const { return mImuOffset; }
    void setImuOffset(xyz_t imuOffset) { mImuOffset = imuOffset; }
    xyz_t getAutopilotCustomReferenceOffset() const { return mAutopilotCustomReferenceOffset; }
    void setAutopilotCustomReferenceOffset(xyz_t autopilotCustomReferenceOffset) { mAutopilotCustomReferenceOffset = autopilotCustomReferenceOffset; }

    // Dynamic state
    virtual PosPoint getPosition(PosType type) const;
    virtual PosPoint getPosition() const override { return getPosition(PosType::simulated); }
    virtual void setPosition(PosPoint &point) override;
    virtual QTime getTime() const override { return mTime; }
    virtual void setTime(const QTime &time) override { mTime = time; }
    FlightMode getFlightMode() const;
    void setFlightMode(const FlightMode &flightMode);
    double getSteering() const;
    virtual void setSteering(double steering);
    PosPoint getHomePosition() const;
    void setHomePosition(const PosPoint &homePosition);
    bool getIsArmed() const;
    void setIsArmed(bool isArmed);
    void setAutopilotRadius(double radius);
    double getAutopilotRadius();
    virtual double getCurvatureToPointInVehicleFrame(const QPointF &point);
    double getCurvatureToPointInENU(const QPointF &point, PosType type);

    // A vehicle can have a trailing vehicle
    // what this means needs to be defined in child classes
    QSharedPointer<VehicleState> getTrailingVehicle() const;
    void setTrailingVehicle(QSharedPointer<VehicleState> trailer);
    bool hasTrailingVehicle() const;

    void simulationStep(double dt_ms, PosType usePosType = PosType::simulated); // Take current state and simulate step forward for dt_ms milliseconds, update state accordingly
    virtual void updateOdomPositionAndYaw(double drivenDistance, PosType usePosType = PosType::odom) = 0;
    virtual double steeringCurvatureToSteering(double steeringCurvature) = 0;

    // For debugging and logging
    std::array<float, 3> getGyroscopeXYZ() const;
    void setGyroscopeXYZ(const std::array<float, 3> &gyroscopeXYZ);
    std::array<float, 3> getAccelerometerXYZ() const;
    void setAccelerometerXYZ(const std::array<float, 3> &accelerometerXYZ);


private:
    // Static state
    double mLength; // [m]
    double mWidth; // [m]
    // TODO: reasonable default values? set here or move?
    double mMinAcceleration = -5.0; // [m/s²]
    double mMaxAcceleration = 3.0; // [m/s²]

    xyz_t mRearAxleOffset;
    xyz_t mRearEndOffset;
    xyz_t mHitchOffset;
    xyz_t mGnssOffset;
    xyz_t mImuOffset;
    xyz_t mAutopilotCustomReferenceOffset;

    // Dynamic state
    double mSteering = 0.0; // [-1.0:1.0]
    PosPoint mPositionBySource[(int)PosType::_LAST_];
    PosPoint mApGoal;
    QTime mTime;
    PosPoint mHomePosition;
    bool mIsArmed = false;
    FlightMode mFlightMode = FlightMode::Unknown;
    double mAutopilotRadius = 0;

    QSharedPointer<VehicleState> mTrailingVehicle;

    std::array<float,3> mGyroscopeXYZ = std::array<float,3>({0.0, 0.0, 0.0}); // [deg/s]
    std::array<float,3> mAccelerometerXYZ = std::array<float,3>({0.0, 0.0, 0.0}); // [g]
};

#endif // VEHICLESTATE_H
