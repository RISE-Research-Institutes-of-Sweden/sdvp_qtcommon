/*
 *     Copyright 2021 Marvin Damschen   marvin.damschen@ri.se
 *               2021 Rickard Häll      rickard.hall@ri.se
 *     Published under GPLv3: https://www.gnu.org/licenses/gpl-3.0.html
 *
 * Implementation of pure pursuit for following a list of waypoints ("Follow Route").
 * It can also follow a person or other vehicle when the point to follow is continously updated ("Follow Point").
 */

#ifndef PUREPURSUITWAYPOINTFOLLOWER_H
#define PUREPURSUITWAYPOINTFOLLOWER_H

#include <QObject>
#include <QSharedPointer>
#include <QPointF>
#include <QTimer>
#include "vehicles/vehiclestate.h"
#include "vehicles/controller/movementcontroller.h"
#include "communication/vehicleconnections/vehicleconnection.h"
#include "autopilot/waypointfollower.h"

enum class WayPointFollowerSTMstates {NONE, FOLLOW_POINT_FOLLOWING, FOLLOW_POINT_WAITING, FOLLOW_ROUTE_INIT, FOLLOW_ROUTE_GOTO_BEGIN, FOLLOW_ROUTE_FOLLOWING, FOLLOW_ROUTE_FINISHED};
struct WayPointFollowerState {
    WayPointFollowerSTMstates stmState = WayPointFollowerSTMstates::NONE;
    PosPoint currentGoal;
    int currentWaypointIndex;
    double purePursuitRadius = 1.0;
    // Follow Route
    int numWaypointsLookahead = 8;
    bool repeatRoute = false;
    // -- for flying vehicles
    double overrideAltitude = 0.0;
    // Follow Point
    PosPoint currentFollowPointInVehicleFrame; // independent of positioning
    double followPointSpeed = 1.0;
    double followPointDistance = 3.0;
    bool followPointTimedOut = true;
};

class PurepursuitWaypointFollower : public WaypointFollower
{
    Q_OBJECT
public:
    // WaypointFollower either works locally on a MovementController on the vehicle, or remotely on a VehicleConnection
    PurepursuitWaypointFollower(QSharedPointer<MovementController> movementController);
    PurepursuitWaypointFollower(QSharedPointer<VehicleConnection> vehicleConnection, PosType posTypeUsed);

    inline bool isOnVehicle() {return !mMovementController.isNull();}

    double getPurePursuitRadius() const;
    void setPurePursuitRadius(double value);

    virtual double getFollowPointSpeed() const;
    virtual void setFollowPointSpeed(double value);

    virtual bool getRepeatRoute() const override;
    virtual void setRepeatRoute(bool value) override;

    virtual const PosPoint getCurrentGoal() override;

    virtual void clearRoute() override;
    virtual void addWaypoint(const PosPoint &point) override;
    virtual void addRoute(const QList<PosPoint>& route) override;

    virtual void startFollowingRoute(bool fromBeginning) override;
    virtual bool isActive() override;
    virtual void stop() override;
    virtual void resetState() override;

    virtual void startFollowPoint();

    static double getCurvatureToPointInENU(QSharedPointer<VehicleState> vehicleState, const QPointF& point, PosType vehiclePosType = PosType::simulated);
    double getCurvatureToPointInENU(const QPointF& point);
    static double getCurvatureToPointInVehicleFrame(const QPointF& point);

    double getInterpolatedSpeed(const PosPoint &currentGoal, const PosPoint &lastWaypoint, const PosPoint &nextWaypoint);

    PosType getPosTypeUsed() const;
    void setPosTypeUsed(const PosType &posTypeUsed);

signals:
    void txDistOfRouteLeft(double meters);

public slots:
    void updateFollowPointInVehicleFrame(const PosPoint &point);

private:
    const unsigned mFollowPointTimeout_ms = 1000;
    QTimer mFollowPointHeartbeatTimer;

    PosPoint getCurrentVehiclePosition();
    void updateState();
    void updateControl(const PosPoint& goal);
    WayPointFollowerState mCurrentState;

    PosType mPosTypeUsed = PosType::fused; // The type of position (Odom, GNSS, UWB, ...) that should be used for planning
    QSharedPointer<MovementController> mMovementController;
    QSharedPointer<VehicleConnection> mVehicleConnection;
    QList <PosPoint> mWaypointList;
    unsigned mUpdateStatePeriod_ms = 50;
    QTimer mUpdateStateTimer;

    void holdPosition();
    void calculateDistanceOfRouteLeft();
};

#endif // PUREPURSUITWAYPOINTFOLLOWER_H
