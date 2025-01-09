/*
 *     Copyright 2021 Marvin Damschen   marvin.damschen@ri.se
 *               2021 Rickard Häll      rickard.hall@ri.se
 *     Published under GPLv3: https://www.gnu.org/licenses/gpl-3.0.html
 */
#include <cmath>
#include <QDebug>
#include <QLineF>
#include "purepursuitwaypointfollower.h"
#include "communication/parameterserver.h"
#include "core/geometry.h"

PurepursuitWaypointFollower::PurepursuitWaypointFollower(QSharedPointer<MovementController> movementController)
{
    mMovementController = movementController;
    mVehicleState = mMovementController->getVehicleState();
    connect(&mUpdateStateTimer, &QTimer::timeout, this, &PurepursuitWaypointFollower::updateState);

    // Provide system parameters to ControlTower
    if (ParameterServer::getInstance()) {
        ParameterServer::getInstance()->provideFloatParameter("PP_RADIUS", std::bind(&PurepursuitWaypointFollower::setPurePursuitRadius, this, std::placeholders::_1), std::bind(&PurepursuitWaypointFollower::getPurePursuitRadius, this));
        ParameterServer::getInstance()->provideFloatParameter("PP_ARC", std::bind(&PurepursuitWaypointFollower::setAdaptivePurePursuitRadiusCoefficient, this, std::placeholders::_1), std::bind(&PurepursuitWaypointFollower::getAdaptivePurePursuitRadiusCoefficient, this));
    }
}

PurepursuitWaypointFollower::PurepursuitWaypointFollower(QSharedPointer<VehicleConnection> vehicleConnection, PosType posTypeUsed)
{
    mVehicleConnection = vehicleConnection;
    mVehicleState = mVehicleConnection->getVehicleState();
    connect(&mUpdateStateTimer, &QTimer::timeout, this, &PurepursuitWaypointFollower::updateState);
    setPosTypeUsed(posTypeUsed);
}

void PurepursuitWaypointFollower::clearRoute()
{
    stop();
    mWaypointList.clear();
}

void PurepursuitWaypointFollower::addWaypoint(const PosPoint &point)
{
    mWaypointList.append(point);
}

void PurepursuitWaypointFollower::addRoute(const QList<PosPoint> &route)
{
    if (route.isEmpty()) {
        return;
    }

    if (!isActive()) {
        mWaypointList.clear();
        mWaypointList.append(route);
    } else {
        // Calculate closest point on new route to current vehicle position
        QPointF currentVehiclePositionXY = purePursuitTrackingPoint();

        int closestPointIndex = 0;
        double minDistance = std::numeric_limits<double>::max();

        for (int i = 0; i < route.size(); i++) {
            double distance = QLineF(currentVehiclePositionXY, route[i].getPoint()).length();
            if (distance < minDistance) {
                minDistance = distance;
                closestPointIndex = i;
            }
        }
        // Truncate the current route and append the new route from closest point onwards
        mWaypointList = mWaypointList.mid(0, mCurrentState.currentWaypointIndex);
        mWaypointList.append(route.mid(closestPointIndex));

        // Update current waypoint index
        while (mCurrentState.currentWaypointIndex < mWaypointList.size()) {
            if (QLineF(currentVehiclePositionXY, mWaypointList.at(mCurrentState.currentWaypointIndex).getPoint()).length() >= purePursuitRadius()) {
                break;
            }
            mCurrentState.currentWaypointIndex++;
        }
        if (mCurrentState.currentWaypointIndex >= mWaypointList.size()) {
            mCurrentState.currentWaypointIndex--;
            mCurrentState.stmState = WayPointFollowerSTMstates::FOLLOW_ROUTE_APPROACHING_GOAL;
        } else {
            mCurrentState.stmState = WayPointFollowerSTMstates::FOLLOW_ROUTE_FOLLOWING;
        }
    }
}

void PurepursuitWaypointFollower::startFollowingRoute(bool fromBeginning)
{
    // Activate emergency brake for follow route
    emit activateEmergencyBrake();
    mCurrentState.overrideAltitude = mVehicleState->getPosition(mPosTypeUsed).getHeight();
    qDebug() << "Note: WaypointFollower starts following route. Height info from route is ignored (staying at" << QString::number(mCurrentState.overrideAltitude, 'g', 2) << "m).";

    if (fromBeginning || mCurrentState.stmState == WayPointFollowerSTMstates::NONE)
        mCurrentState.stmState = WayPointFollowerSTMstates::FOLLOW_ROUTE_INIT;

    mUpdateStateTimer.start(mUpdateStatePeriod_ms);
}

bool PurepursuitWaypointFollower::isActive()
{
    return mUpdateStateTimer.isActive();
}

void PurepursuitWaypointFollower::holdPosition()
{
    if (isOnVehicle()) {
        mMovementController->setDesiredSteering(0.0);
        mMovementController->setDesiredSpeed(0.0);
    } else {
        mVehicleConnection->requestVelocityAndYaw({}, mVehicleState->getPosition(mPosTypeUsed).getYaw());
    }
}

void PurepursuitWaypointFollower::stop()
{
    if (mUpdateStateTimer.isActive()) {
        mUpdateStateTimer.stop();
    }
    mVehicleState->setAutopilotRadius(0);
    holdPosition();
    emit deactivateEmergencyBrake();
}

void PurepursuitWaypointFollower::resetState()
{
    mUpdateStateTimer.stop();
    mCurrentState.stmState = WayPointFollowerSTMstates::NONE;
    mCurrentState.currentWaypointIndex = mWaypointList.size();
    mCurrentState.startPointXY = QPointF();
}

void PurepursuitWaypointFollower::updateState()
{
    QPointF currentVehiclePositionXY = purePursuitTrackingPoint();

    switch (mCurrentState.stmState) {
    case WayPointFollowerSTMstates::NONE:
        qDebug() << "WARNING: WayPointFollower running uninitialized statemachine.";
        break;

    // FOLLOW_ROUTE: waypoints describe a route to be followed waypoint by waypoint
    case WayPointFollowerSTMstates::FOLLOW_ROUTE_INIT:
        if (mWaypointList.size()) {
            mCurrentState.currentWaypointIndex = 0;
            mCurrentState.currentGoal = mWaypointList.at(0);
            mCurrentState.stmState = WayPointFollowerSTMstates::FOLLOW_ROUTE_GOTO_BEGIN;
            mCurrentState.startPointXY = currentVehiclePositionXY;
        } else
            mCurrentState.stmState = WayPointFollowerSTMstates::FOLLOW_ROUTE_FINISHED;
        break;

    case WayPointFollowerSTMstates::FOLLOW_ROUTE_GOTO_BEGIN: {
        calculateDistanceOfRouteLeft(currentVehiclePositionXY);

        // draw straight line to first point and apply purePursuitRadius to find intersection
        QLineF carToStartLine(currentVehiclePositionXY, mWaypointList.at(0).getPoint());
        QVector<QPointF> intersections = geometry::findIntersectionsBetweenCircleAndLine(QPair<QPointF, double>(currentVehiclePositionXY, purePursuitRadius()), carToStartLine);

        if (intersections.size()) {
            mCurrentState.currentGoal.setXY(intersections[0].x(), intersections[0].y());
            updateControl(mCurrentState.currentGoal);
        } else { // first waypoint within circle -> start route
            if (mWaypointList.size() > 1) {
                mCurrentState.stmState = WayPointFollowerSTMstates::FOLLOW_ROUTE_FOLLOWING;
            } else {
                mCurrentState.stmState = WayPointFollowerSTMstates::FOLLOW_ROUTE_APPROACHING_GOAL;
                // qDebug() << "Approaching goal";
            }
        }
    } break;

    case WayPointFollowerSTMstates::FOLLOW_ROUTE_FOLLOWING: {
        calculateDistanceOfRouteLeft(currentVehiclePositionXY);

        QPointF currentWaypointPoint = mWaypointList.at(mCurrentState.currentWaypointIndex).getPoint();
        if (QLineF(currentVehiclePositionXY, currentWaypointPoint).length() < purePursuitRadius()) // consider previous waypoint as reached
            mCurrentState.currentWaypointIndex++;

        if (mCurrentState.currentWaypointIndex == mWaypointList.size()) {
            mCurrentState.currentWaypointIndex--;
            mCurrentState.stmState = WayPointFollowerSTMstates::FOLLOW_ROUTE_APPROACHING_GOAL;
            // qDebug() << "Approaching goal";
        } else {
            // --- Calculate current goal on route (which lies between two waypoints)
            // 1. Find intersection between circle around vehicle and route
            // look a number of points ahead and jump forward on route, if applicable
            // and take care of index wrap in case route is repeated
            QList<PosPoint> lookAheadWaypoints;
            if (mCurrentState.repeatRoute) {
                lookAheadWaypoints = mWaypointList.mid(mCurrentState.currentWaypointIndex - 1, mCurrentState.numWaypointsLookahead);

                const int lookaheadWaypointEndIndex = mCurrentState.currentWaypointIndex + mCurrentState.numWaypointsLookahead - 1;
                if (lookaheadWaypointEndIndex > mWaypointList.size()) // index wraparound
                    lookAheadWaypoints.append(mWaypointList.mid(0, lookaheadWaypointEndIndex % mWaypointList.size()));
                else if (mCurrentState.currentWaypointIndex == 0) // restarting from end to beginning
                    lookAheadWaypoints.prepend(mWaypointList.last());
            } else
                lookAheadWaypoints = mWaypointList.mid(mCurrentState.currentWaypointIndex - 1,  mCurrentState.numWaypointsLookahead);

            QVector<QPointF> intersections;
            for (int i = lookAheadWaypoints.size() - 1; i > 0; i--) { // step backwards through lookahead window until intersection is found
                QPointF iWaypoint = lookAheadWaypoints.at(i).getPoint();
                QLineF iLineSegment(lookAheadWaypoints.at(i-1).getPoint(), iWaypoint);

                intersections = geometry::findIntersectionsBetweenCircleAndLine(QPair<QPointF, double>(currentVehiclePositionXY, purePursuitRadius()), iLineSegment);
                if (intersections.size() > 0) {
                    mCurrentState.currentWaypointIndex = (i + mCurrentState.currentWaypointIndex - 1) % mWaypointList.size();
                    currentWaypointPoint = iWaypoint;
                    break;
                }
            }

            // 2. Set Goal depending on number of intersections found
            int previousWaypointIndex = mCurrentState.currentWaypointIndex - 1 >= 0 ? mCurrentState.currentWaypointIndex - 1 : mWaypointList.size() - 1;
            switch (intersections.size()) {
            case 0:
                // We seem to have left the route (e.g., because of high speed), reuse previous goal to  get back to route
                break;
            case 1:
                mCurrentState.currentGoal.setX(intersections[0].x());
                mCurrentState.currentGoal.setY(intersections[0].y());
                mCurrentState.currentGoal.setSpeed(getInterpolatedSpeed(mCurrentState.currentGoal, mWaypointList.at(previousWaypointIndex), mWaypointList.at(mCurrentState.currentWaypointIndex)));

                break;
            case 2:
                // Take intersection closest to current waypoint (most progress)
                if (QLineF(intersections[0], currentWaypointPoint).length()
                        < QLineF(intersections[1], currentWaypointPoint).length()) {
                    mCurrentState.currentGoal.setX(intersections[0].x());
                    mCurrentState.currentGoal.setY(intersections[0].y());
                    mCurrentState.currentGoal.setSpeed(getInterpolatedSpeed(mCurrentState.currentGoal, mWaypointList.at(previousWaypointIndex), mWaypointList.at(mCurrentState.currentWaypointIndex)));

                }
                else {
                    mCurrentState.currentGoal.setX(intersections[1].x());
                    mCurrentState.currentGoal.setY(intersections[1].y());
                    mCurrentState.currentGoal.setSpeed(getInterpolatedSpeed(mCurrentState.currentGoal, mWaypointList.at(previousWaypointIndex), mWaypointList.at(mCurrentState.currentWaypointIndex)));

                }
                break;
            default:
                break;
            }

            // 3. Determine closest waypoint to vehicle, it determines attributes
            PosPoint closestWaypoint;
            if (QLineF(currentVehiclePositionXY, mWaypointList.at(previousWaypointIndex).getPoint()).length()
                    < QLineF(currentVehiclePositionXY, mWaypointList.at(mCurrentState.currentWaypointIndex).getPoint()).length())
                closestWaypoint = mWaypointList.at(previousWaypointIndex);
            else
                closestWaypoint = mWaypointList.at(mCurrentState.currentWaypointIndex);
            mCurrentState.currentGoal.setAttributes(closestWaypoint.getAttributes());

            // 4. Update control for current goal
            updateControl(mCurrentState.currentGoal);
        }
    } break;
    case WayPointFollowerSTMstates::FOLLOW_ROUTE_APPROACHING_GOAL: {
        const PosPoint& goalPosPoint = mWaypointList.at(mCurrentState.currentWaypointIndex);
        QPointF goalPosPointXY = goalPosPoint.getPoint();
        QLineF carToGoalline(currentVehiclePositionXY, goalPosPointXY);
        double distanceToGoal = carToGoalline.length();
        if (distanceToGoal < mXYGoalThreshold) {
            if (!mCurrentState.repeatRoute) {
                mCurrentState.stmState = WayPointFollowerSTMstates::FOLLOW_ROUTE_FINISHED;
                qDebug() << "Goal reached with accuracy:" << distanceToGoal << "m.";
            } else {
                qDebug() << "Goal reached with accuracy:" << distanceToGoal << "m. Repeating the route...";
            }
        } else {
            QLineF lastWayPointToGoalline;
            lastWayPointToGoalline.setP2(goalPosPointXY);
            if (mCurrentState.currentWaypointIndex) {
                lastWayPointToGoalline.setP1(mWaypointList[mCurrentState.currentWaypointIndex - 1].getPoint());
            } else {
                lastWayPointToGoalline.setP1(mCurrentState.startPointXY);
            }
            double angleBetweenLines = carToGoalline.angleTo(lastWayPointToGoalline);
            if (angleBetweenLines > 180) { // Convert to -180 to 180 range
                angleBetweenLines -= 360;
            }

            if (fabs(angleBetweenLines) >= 90 && !mRetryAfterGoalOvershot) {
                stop();
                qDebug() << "Goal overshot! Stopped waypoint follower with distance to goal:" << distanceToGoal << "m.";
                return;
            } else {
                double purePursuitRadius_ = purePursuitRadius();
                double extensionDistance = std::clamp(purePursuitRadius_ - distanceToGoal, 0.0, purePursuitRadius_);
                double extensionRatio = (extensionDistance + lastWayPointToGoalline.length()) / lastWayPointToGoalline.length();
                QPointF extendedGoalPoint = lastWayPointToGoalline.pointAt(extensionRatio);

                mCurrentState.currentGoal.setX(extendedGoalPoint.x());
                mCurrentState.currentGoal.setY(extendedGoalPoint.y());
                mCurrentState.currentGoal.setSpeed(goalPosPoint.getSpeed());
                updateControl(mCurrentState.currentGoal);
            }
        }
    } break;
    case WayPointFollowerSTMstates::FOLLOW_ROUTE_FINISHED:
        mCurrentState.stmState = WayPointFollowerSTMstates::NONE;
        mCurrentState.currentWaypointIndex = mWaypointList.size();
        mCurrentState.startPointXY = QPointF();
        stop();
        break;

    default:
        break;
    }
}

void PurepursuitWaypointFollower::updateControl(const PosPoint &goal)
{
    if (isOnVehicle()) {
        mMovementController->setDesiredSteeringCurvature(mVehicleState->getCurvatureToPointInENU(goal.getPoint(), mPosTypeUsed));
        mMovementController->setDesiredSpeed(goal.getSpeed());
        mMovementController->setDesiredAttributes(goal.getAttributes());

        mVehicleState->setAutopilotLookAheadPoint(goal.getPoint());
    } else {
        // NOTE: we calculate in ENU coordinates
        xyz_t positionDifference = {goal.getX() - mVehicleConnection->getVehicleState()->getPosition(mPosTypeUsed).getX(),
                                    goal.getY() - mVehicleConnection->getVehicleState()->getPosition(mPosTypeUsed).getY(),
                                    mCurrentState.overrideAltitude - mVehicleState->getPosition(mPosTypeUsed).getHeight()};
        double positionDiffDistance = sqrtf(positionDifference.x*positionDifference.x + positionDifference.y*positionDifference.y + positionDifference.z*positionDifference.z);
        double velocityFactor = goal.getSpeed() / positionDiffDistance;

        double yawDeg = atan2(goal.getY() - mVehicleState->getPosition(mPosTypeUsed).getY(),
                              goal.getX() - mVehicleState->getPosition(mPosTypeUsed).getX()) * 180.0 / M_PI;

        mVehicleConnection->requestVelocityAndYaw({positionDifference.x*velocityFactor, positionDifference.y*velocityFactor, positionDifference.z*velocityFactor}, yawDeg);
    }
}

PosType PurepursuitWaypointFollower::getPosTypeUsed() const
{
    return mPosTypeUsed;
}

void PurepursuitWaypointFollower::setPosTypeUsed(const PosType &posTypeUsed)
{
    mPosTypeUsed = posTypeUsed;
}

double PurepursuitWaypointFollower::getPurePursuitRadius() const
{
    return mCurrentState.purePursuitRadius;
}

void PurepursuitWaypointFollower::setPurePursuitRadius(double value)
{
    mCurrentState.purePursuitRadius = value;
}

void PurepursuitWaypointFollower::setAdaptivePurePursuitRadiusCoefficient(double coefficient)
{
    mCurrentState.adaptivePurePursuitRadiusCoefficient = coefficient;
}

double PurepursuitWaypointFollower::getAdaptivePurePursuitRadiusCoefficient()
{
    return mCurrentState.adaptivePurePursuitRadiusCoefficient;
}

bool PurepursuitWaypointFollower::getRepeatRoute() const
{
    return mCurrentState.repeatRoute;
}

void PurepursuitWaypointFollower::setRepeatRoute(bool value)
{
    mCurrentState.repeatRoute = value;
}

const PosPoint PurepursuitWaypointFollower::getCurrentGoal()
{
    return mCurrentState.currentGoal;
}

double PurepursuitWaypointFollower::getInterpolatedSpeed(const PosPoint &currentGoal, const PosPoint &lastWaypoint, const PosPoint &nextWaypoint)
{
    // Linear interpolation
    double distanceToNextWaypoint = currentGoal.getDistanceTo(nextWaypoint);
    double distanceBetweenWaypoints = lastWaypoint.getDistanceTo(nextWaypoint);
    double x = distanceBetweenWaypoints - distanceToNextWaypoint;

    return lastWaypoint.getSpeed() + (nextWaypoint.getSpeed()-lastWaypoint.getSpeed())*(x/distanceBetweenWaypoints);
}

void PurepursuitWaypointFollower::calculateDistanceOfRouteLeft(const QPointF currentVehiclePositionXY)
{
    double distance = QLineF(currentVehiclePositionXY, mWaypointList.at(mCurrentState.currentWaypointIndex).getPoint()).length();

    for (int index = mCurrentState.currentWaypointIndex; index < mWaypointList.size()-1; index++) {
        distance += QLineF(mWaypointList.at(index).getPoint(), mWaypointList.at(index+1).getPoint()).length();
    }
    emit distanceOfRouteLeft(distance);
}

double PurepursuitWaypointFollower::purePursuitRadius()
{
    if (mCurrentState.adaptivePurePursuitRadius) {
        double dynamicRadius =  mCurrentState.adaptivePurePursuitRadiusCoefficient * mVehicleState->getSpeed();

        if (dynamicRadius > mCurrentState.purePursuitRadius)
            mVehicleState->setAutopilotRadius(dynamicRadius);
        else
            mVehicleState->setAutopilotRadius(mCurrentState.purePursuitRadius);
    } else
        mVehicleState->setAutopilotRadius(mCurrentState.purePursuitRadius);

    return mVehicleState->getAutopilotRadius();
}

void PurepursuitWaypointFollower::setAdaptivePurePursuitRadiusActive(bool adaptive)
{
    mCurrentState.adaptivePurePursuitRadius = adaptive;
}

QList<PosPoint> PurepursuitWaypointFollower::getCurrentRoute()
{
    return mWaypointList;
}

QPointF PurepursuitWaypointFollower::purePursuitTrackingPoint()
{
    QSharedPointer<VehicleState> trackingVehicleState = mVehicleState;
    if (mVehicleState->hasTrailingVehicle() && mVehicleState->getSpeed() < 0) { // position defined by trailer when backing (if exists)
        trackingVehicleState = mVehicleState->getTrailingVehicle();
    }

    PosPoint referencePosPoint;
    switch (mTrackingType) {
        case WayPointFollowerTrackingType::REAR_AXLE: {
            referencePosPoint = trackingVehicleState->getOffsetPosition(trackingVehicleState->getRearAxleOffset(), mPosTypeUsed);
        } break;
        case WayPointFollowerTrackingType::FRONT_REAR_END: {
            xyz_t offset = trackingVehicleState->getRearEndOffset();
            if(mVehicleState->getSpeed() >= 0) {
                offset.x += trackingVehicleState->getLength();
            }
            referencePosPoint = trackingVehicleState->getOffsetPosition(offset, mPosTypeUsed);
        } break;
        case WayPointFollowerTrackingType::CUSTOM_REFERENCE: {
            referencePosPoint = trackingVehicleState->getOffsetPosition(trackingVehicleState->getAutopilotCustomReferenceOffset(), mPosTypeUsed);
        } break;
        default:{
            referencePosPoint = trackingVehicleState->getPosition(mPosTypeUsed);
        } break;
    }

    mVehicleState->setAutopilotReferencePoint(referencePosPoint.getPoint());

    return mVehicleState->getAutopilotReferencePoint();
}