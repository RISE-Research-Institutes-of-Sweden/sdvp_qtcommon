/*
 *     Copyright 2024 RISE Sweden
 *
 *   Specific implementation of VehicleState for car-type (ackermann) vehicles, storing all (dynamic and static) state
 */

#ifndef TRUCKSTATE_H
#define TRUCKSTATE_H

#include "vehicles/carstate.h"
#include "vehicles/trailerstate.h"
#include <QSharedPointer>

class TruckState : public CarState
{
    Q_OBJECT
public:
    TruckState(ObjectID_t id = 1, Qt::GlobalColor color = Qt::blue);

    virtual void setLength(double length) override;
    double getCurvatureToPointInVehicleFrame(const QPointF &point) override;
    virtual void updateOdomPositionAndYaw(double drivenDistance, PosType usePosType = PosType::odom) override;
    void updateTrailingVehicleOdomPositionAndYaw(double drivenDistance, PosType usePosType = PosType::odom);

    double getPurePursuitForwardGain() const{ return mPurePursuitForwardGain;}
    void setPurePursuitForwardGain(double value){ mPurePursuitForwardGain = value;}

    double getPurePursuitReverseGain() const{ return mPurePursuitReverseGain;}
    void setPurePursuitReverseGain(double value){ mPurePursuitReverseGain = value;}

    QSharedPointer<TrailerState> getTrailingVehicle() const;
    void setTrailingVehicle(QSharedPointer<TrailerState> trailer);
    // Additional set/get state for angle of trailing vehicle towards us / ego vehicle
    double getTrailerAngleRadians() const { return mTrailerAngle_deg * (M_PI / 180.0); }
    double getTrailerAngleDegrees() const { return mTrailerAngle_deg; }
    void setTrailerAngle(double angle_deg);
    virtual void setPosition(PosPoint &point) override;

    bool getSimulateTrailer() const;
    void setSimulateTrailer(bool simulateTrailer);

#ifdef QT_GUI_LIB
    // Override or add drawing functions if needed (to draw a truck)
    virtual void draw(QPainter &painter, const QTransform &drawTrans, const QTransform &txtTrans, bool isSelected = true) override;
#endif

private:
    double mPurePursuitForwardGain = 1.0;
    double mPurePursuitReverseGain = -1.0;

    double mTrailerAngle_deg = 0.0;
    bool mSimulateTrailer = false;

    double getCurvatureWithTrailer(const QPointF &point);

};

#endif // TRUCKSTATE_H
