/*
 *     Copyright 2012 Benjamin Vedder   benjamin@vedder.se
 *               2020 Marvin Damschen   marvin.damschen@ri.se
 *               2021 Lukas Wikander    lukas.wikander@astazero.com
 *     Published under GPLv3: https://www.gnu.org/licenses/gpl-3.0.html
 * 
 * Specific implementation of VehicleState for car-type (ackermann) vehicles, storing all (dynamic and static) state
 */

#ifndef CARSTATE_H
#define CARSTATE_H

#include "vehicles/vehiclestate.h"

#include <QObject>
#include <QString>
#ifdef QT_GUI_LIB
#include <QPainter>
#endif
#include <cmath>

class CarState : public VehicleState
{
    Q_OBJECT
public:
    CarState(ObjectID_t id = 1, Qt::GlobalColor color = Qt::red);
#ifdef QT_GUI_LIB
    virtual void draw(QPainter &painter, const QTransform &drawTrans, const QTransform &txtTrans, bool isSelected = true) override;
    virtual QPainterPath getBoundingBox() const override;
#endif
    virtual void updateOdomPositionAndYaw(double drivenDistance, PosType usePosType = PosType::odom) override;
    virtual double steeringCurvatureToSteering(double steeringCurvature) override;
    virtual void provideParameters() override;

    // Static state
    virtual void setLength(double length) override;
    double getAxisDistance() const { return fabs(mAxisDistance) < 0.001 ? 0.8*getLength() : mAxisDistance; }
    void setAxisDistance(double axisDistance);
    double getRearOverhang() const { return mRearOverhang; }
    void setRearOverhang(double rearOverhang) { mRearOverhang = rearOverhang; }
    double getFrontOverhang() const { return mFrontOverhang; }
    void setFrontOverhang(double frontOverhang) { mFrontOverhang = frontOverhang; }

    inline double getMaxSteeringAngle() const { return mMaxSteeringAngle < M_PI/180.0 ? M_PI/4.0 : mMaxSteeringAngle; } // 45° assumed if unset
    void setMaxSteeringAngle(double steeringAngle_rad);
    void setMinTurnRadiusRear(double minTurnRadius_m);

    // Dynamic state
    void setSteering(double steering) override;
    inline double getTurnRadiusRear() const { return getAxisDistance() / -getSteering(); } // steering in [-1.0:1.0] as a simple approximation of tan(steering angle)
    inline double getTurnRadiusFront() const { return sqrt(pow(getAxisDistance(),2) + pow(getTurnRadiusRear(),2)); }
    inline double getTotalReactionTime() const { return 0.3; } // TODO: needs to be calculated/estimated
    double getBrakingDistance() const;
    double getBrakingDistance(double deceleration) const;
    double getThreeSecondsDistance() const { return 3.0*getSpeed(); } // Distance the vehicle can move within 3 seconds at current speed, Swedish "Tresekundersregeln"
    const QPointF getStoppingPointForTurnRadiusAndBrakingDistance(const double turnRadius, const double brakeDistance) const;
    const QPointF getStoppingPointForTurnRadius(const double turnRadius) const;
    inline double getMinTurnRadiusRear() const { return qMax(qMin(getAxisDistance() / tanf(getMaxSteeringAngle()), mMinTurnRadiusRear), pow(getSpeed(), 2)/(0.21*9.81)); }

private:
    double mRearOverhang = 0.0; //[m]
    double mFrontOverhang = 0.0; //[m]
    double mAxisDistance = 0.0; // [m]
    double mMaxSteeringAngle = 0.0; // [rad]
    double mMinTurnRadiusRear = std::numeric_limits<double>::infinity(); // [m]


};

#endif // CARSTATE_H
