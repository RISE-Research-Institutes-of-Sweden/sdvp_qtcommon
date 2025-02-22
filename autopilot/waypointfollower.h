/*
 *     Copyright 2022 RISE Research Institutes of Sweden AB, Safety and Transport   waywise@ri.se
 *     Published under GPLv3: https://www.gnu.org/licenses/gpl-3.0.html
 *
 * Abstract class to control autopilot
 */

#ifndef WAYPOINTFOLLOWER_H
#define WAYPOINTFOLLOWER_H

#include <QObject>
#include "core/pospoint.h"

class WaypointFollower : public QObject
{
    Q_OBJECT
public:

    virtual bool getRepeatRoute() const = 0;
    virtual void setRepeatRoute(bool value) = 0;

    virtual const PosPoint getCurrentGoal() = 0;

    virtual void clearRoute() = 0;
    virtual void addWaypoint(const PosPoint &point) = 0;
    virtual void addRoute(const QList<PosPoint>& route) = 0;

    virtual void startFollowingRoute(bool fromBeginning) = 0;
    virtual bool isActive() = 0;
    virtual void stop() = 0;
    virtual void resetState() = 0;

    virtual QList<PosPoint> getCurrentRoute() = 0;

signals:
    void deactivateEmergencyBrake();
    void activateEmergencyBrake();
};

#endif // WAYPOINTFOLLOWER_H
