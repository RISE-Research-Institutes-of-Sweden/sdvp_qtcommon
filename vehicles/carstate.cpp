/*
 *     Copyright 2012 Benjamin Vedder   benjamin@vedder.se
 *               2020 Marvin Damschen   marvin.damschen@ri.se
 *               2021 Lukas Wikander    lukas.wikander@astazero.com
 *               2022 Rickard Häll      rickard.hall@ri.se
 *     Published under GPLv3: https://www.gnu.org/licenses/gpl-3.0.html
 * 
 * Specific implementation of VehicleState for car-type (ackermann) vehicles, storing all (dynamic and static) state
 */
#include "carstate.h"
#include <QDebug>
#include <QDateTime>

#include "communication/parameterserver.h"

CarState::CarState(ObjectID_t id, Qt::GlobalColor color) : VehicleState(id, color)
{
    ObjectState::setWaywiseObjectType(WAYWISE_OBJECT_TYPE_CAR);
}

void CarState::setLength(double length)
{
    VehicleState::setLength(length);

    xyz_t rearEndOffset = getRearEndOffset();
    rearEndOffset.x = - length / 2.0;
    setRearEndOffset(rearEndOffset);
}

void CarState::setAxisDistance(double axisDistance)
{
    mAxisDistance = axisDistance;

    xyz_t rearAxleOffset = getRearAxleOffset();
    rearAxleOffset.x = - axisDistance / 2.0;
    setRearAxleOffset(rearAxleOffset);
}


#ifdef QT_GUI_LIB
void CarState::draw(QPainter &painter, const QTransform &drawTrans, const QTransform &txtTrans, bool isSelected)
{
    PosPoint pos = getPosition();

    const double car_len = getLength() * 1000.0;
    const double car_w = getWidth() * 1000.0;
    const double car_corner = 0.02 * 1000.0;

    double x = pos.getX() * 1000.0;
    double y = pos.getY() * 1000.0;
    xyz_t rearAxleOffset = getRearAxleOffset();
    xyz_t rearEndOffset = getRearEndOffset();
    double rearAxleOffsetX = rearAxleOffset.x * 1000.0;
    const double rearAxleOffsetY = rearAxleOffset.y * 1000.0;
    double rearEndOffsetX = rearEndOffset.x * 1000.0;
    const double wheelbase = getAxisDistance() * 1000.0;

    painter.setTransform(drawTrans);

    QColor col_wheels;
    QColor col_bumper;
    QColor col_ap;
    QColor col_sigma = Qt::red;
    QColor col_hull = getColor();
    QColor col_center = Qt::blue;

    if (isSelected) {
        col_wheels = Qt::black;
        col_bumper = Qt::green;
        col_ap = getColor();
    } else {
        col_wheels = Qt::darkGray;
        col_bumper = Qt::lightGray;
        col_ap = Qt::lightGray;
    }

    // Draw standard deviation
    if (pos.getSigma() > 0.0) {
        QColor col = col_sigma;
        col.setAlphaF(0.2);
        painter.setBrush(QBrush(col));
        painter.drawEllipse(pos.getPointMm(), pos.getSigma() * 1000.0, pos.getSigma() * 1000.0);
    }

    // Draw car
    painter.save();
    painter.translate(x, y);
    painter.rotate(pos.getYaw());
    // Rear axle wheels
    const double wheel_diameter = car_len / 6.0;
    const double wheel_width = car_w / 12.0;
    painter.setBrush(QBrush(col_wheels));
    painter.drawRoundedRect(rearAxleOffsetX - wheel_diameter/2, rearAxleOffsetY - (car_w / 2 + wheel_width / 2), wheel_diameter, (car_w + wheel_width), car_corner / 3, car_corner / 3);
    // Front axle wheels
    painter.drawRoundedRect(rearAxleOffsetX + wheelbase - wheel_diameter/2, -(car_w / 2 + wheel_width / 2), wheel_diameter, (car_w + wheel_width), car_corner / 3, car_corner / 3);
    // Front bumper
    painter.setBrush(col_bumper);
    painter.drawRoundedRect(rearEndOffsetX, -((car_w - car_len / 20.0) / 2.0), car_len, car_w - car_len / 20.0, car_corner, car_corner);
    // Hull
    painter.setBrush(col_hull);
    painter.drawRoundedRect(rearEndOffsetX, -((car_w - car_len / 20.0) / 2.0), car_len - (car_len / 20.0), car_w - car_len / 20.0, car_corner, car_corner);
    painter.restore();

    // Center
    painter.setBrush(col_center);
    painter.drawEllipse(QPointF(x, y), car_w / 15.0, car_w / 15.0);

    // Turning radius
    if (getAutopilotRadius() > 0.001){
        painter.setPen(QPen(Qt::red, 40));
        painter.setBrush(Qt::transparent);
        painter.drawEllipse(QPointF(x, y), getAutopilotRadius()*1000.0, getAutopilotRadius()*1000.0);
        painter.setPen(Qt::black);
    }

    //        // GPS Location
    //        painter.setBrush(col_gps);
    //        painter.drawEllipse(QPointF(x_gps, y_gps), car_w / 15.0, car_w / 15.0);

    //        // Autopilot state
    //        LocPoint ap_goal = VehicleState->getApGoal();
    //        if (ap_goal.getRadius() > 0.0) {
    //            pen.setColor(col_ap);
    //            painter.setPen(pen);
    //            QPointF pm = pos.getPointMm();
    //            painter.setBrush(Qt::transparent);
    //            painter.drawEllipse(pm, ap_goal.getRadius() * 1000.0, ap_goal.getRadius() * 1000.0);

    //            QPointF p = ap_goal.getPointMm();
    //            pen.setWidthF(3.0 / mScaleFactor);
    //            painter.setBrush(col_gps);
    //            painter.drawEllipse(p, 10 / mScaleFactor, 10 / mScaleFactor);
    //        }

    //        painter.setPen(QPen(textColor));

    //        // Print data
    //        QTime t = QTime::fromMSecsSinceStartOfDay(VehicleState->getTime());
    //        QString solStr;
    //        if (!VehicleState->getLocationGps().getInfo().isEmpty()) {
    //            solStr = QString("Sol: %1\n").arg(VehicleState->getLocationGps().getInfo());
    //        }
    //        txt.sprintf("%s\n"
    //                    "%s"
    //                    "(%.3f, %.3f, %.0f)\n"
    //                    "%02d:%02d:%02d:%03d",
    //                    VehicleState->getName().toLocal8Bit().data(),
    //                    solStr.toLocal8Bit().data(),
    //                    pos.getX(), pos.getY(), angle,
    //                    t.hour(), t.minute(), t.second(), t.msec());

    if (getDrawStatusText()) {
        // Print data
        QString txt;
        QPointF pt_txt;
        QRectF rect_txt;

        QString flightModeStr;
        switch (getFlightMode()) {
            case FlightMode::Unknown: flightModeStr = "unknown"; break;
            case FlightMode::Ready: flightModeStr = "ready"; break;
            case FlightMode::Takeoff: flightModeStr = "takeoff"; break;
            case FlightMode::Hold: flightModeStr = "hold"; break;
            case FlightMode::Mission: flightModeStr = "mission"; break;
            case FlightMode::ReturnToLaunch: flightModeStr = "return to launch"; break;
            case FlightMode::Land: flightModeStr = "land"; break;
            case FlightMode::Offboard: flightModeStr = "offboard"; break;
            case FlightMode::FollowMe: flightModeStr = "follow me"; break;
            case FlightMode::Manual: flightModeStr = "manual"; break;
            case FlightMode::Altctl: flightModeStr = "altitude"; break;
            case FlightMode::Posctl: flightModeStr = "position"; break;
            case FlightMode::Acro: flightModeStr = "acro"; break;
            case FlightMode::Stabilized: flightModeStr = "stabilized"; break;
            case FlightMode::Rattitude: flightModeStr = "rattitude"; break;
        }

        QTextStream txtStream(&txt);
        txtStream.setRealNumberPrecision(3);
        txtStream << getName() << Qt::endl
                  << "(" << pos.getX() << ", " << pos.getY() << ", " << pos.getHeight() << ", " << (int)pos.getYaw() << ")" << Qt::endl
                  << "State: " << (getIsArmed() ? "armed" : "disarmed") << Qt::endl
                  << flightModeStr;

        pt_txt.setX(x + car_w + car_len * ((cos(getPosition().getYaw() * (M_PI/180.0)) + 1) / 3));
        pt_txt.setY(y);
        painter.setTransform(txtTrans);
        pt_txt = drawTrans.map(pt_txt);
        rect_txt.setCoords(pt_txt.x(), pt_txt.y() - 40,
                           pt_txt.x() + 400, pt_txt.y() + 65);
        painter.drawText(rect_txt, txt);
    }
}

QPainterPath CarState::getBoundingBox() const
{
    QPainterPath carBoundingBox;

    carBoundingBox.addRect(-mRearOverhang, -getWidth()/2, getLength(), getWidth());
    carBoundingBox.closeSubpath();

    return carBoundingBox;
}
#endif

void CarState::setSteering(double steering)
{
    steering = (steering > tanf(getMaxSteeringAngle())) ? tanf(getMaxSteeringAngle()) : steering;
    steering = (steering < -tanf(getMaxSteeringAngle())) ? -tanf(getMaxSteeringAngle()) : steering;
    VehicleState::setSteering(steering);
}

void CarState::setMaxSteeringAngle(double steeringAngle_rad) {
    mMaxSteeringAngle = fabs(steeringAngle_rad);
}

void CarState::setMinTurnRadiusRear(double minTurnRadius_m) {
    mMinTurnRadiusRear = fabs(minTurnRadius_m);
}

double CarState::getBrakingDistance() const {
    return getBrakingDistance(-getMinAcceleration());
}

double CarState::getBrakingDistance(double deceleration) const
{
    double brakingDistance = 0.0;
    if (getSpeed() != 0.0)
        brakingDistance = getSpeed() * getTotalReactionTime() -
                ((getSpeed() >= 0.0) ? 0.5 : -0.5) * pow(abs(getSpeed()) + getMaxAcceleration() * getTotalReactionTime(), 2.0) / -deceleration;

    return brakingDistance;

}

const QPointF CarState::getStoppingPointForTurnRadiusAndBrakingDistance(const double turnRadius, const double brakingDistance) const
{
    double x = fabs(turnRadius) * sin(brakingDistance/fabs(turnRadius));
    double y = turnRadius * (1 - cos(brakingDistance/fabs(turnRadius)));

    return QPointF(x, y);
}

const QPointF CarState::getStoppingPointForTurnRadius(const double turnRadius) const
{
    return getStoppingPointForTurnRadiusAndBrakingDistance(turnRadius, getBrakingDistance());
}

void CarState::updateOdomPositionAndYaw(double drivenDistance, PosType usePosType)
{
    PosPoint currentPosition = getPosition(usePosType);
    double yawRad = currentPosition.getYaw() / (180.0/M_PI);

    // Bicycle kinematic model with rear axle as reference point
    if (fabs(getSteering()) > 1e-6) { // Turning
        double turnRadiusRear = getTurnRadiusRear();
        double turnRadiusFront = getTurnRadiusFront();

        if (turnRadiusRear < 0.0) {
            turnRadiusFront = -turnRadiusFront;
        }

        double yawChange = drivenDistance / ((turnRadiusRear + turnRadiusFront) / 2.0);

        currentPosition.setX(currentPosition.getX() - turnRadiusRear * (sin(yawRad - yawChange) - sinf(yawRad)));
        currentPosition.setY(currentPosition.getY() - turnRadiusRear * (cos(yawRad + yawChange) - cosf(yawRad)));

        double newYaw_deg = (yawRad + yawChange) * (180.0/M_PI);

        // normalize
        while (newYaw_deg < 0.0)
            newYaw_deg += 360.0;
        while (newYaw_deg > 360.0)
            newYaw_deg -= 360.0;

        currentPosition.setYaw(newYaw_deg);
    } else { // Driving forward
        currentPosition.setX(currentPosition.getX() + cos(yawRad) * drivenDistance);
        currentPosition.setY(currentPosition.getY() + sin(yawRad) * drivenDistance);
    }

    currentPosition.setTime(QTime::currentTime().addSecs(-QDateTime::currentDateTime().offsetFromUtc()));
    setPosition(currentPosition);
}

double CarState::steeringCurvatureToSteering(double steeringCurvature)
{
    double steeringAngle_rad = atan(getAxisDistance() * steeringCurvature);
    if (abs(steeringAngle_rad) > getMaxSteeringAngle())
        steeringAngle_rad = getMaxSteeringAngle() * ((steeringAngle_rad > 0) ? 1.0 : -1.0);

    return steeringAngle_rad / getMaxSteeringAngle();
}

void CarState::provideParameters()
{
    ParameterServer::getInstance()->provideFloatParameter("VEH_LENGTH", std::bind(&CarState::setLength, this, std::placeholders::_1), std::bind(&CarState::getLength, this));
    ParameterServer::getInstance()->provideFloatParameter("VEH_WIDTH", std::bind(&CarState::setWidth, this, std::placeholders::_1), std::bind(&CarState::getWidth, this));
    ParameterServer::getInstance()->provideFloatParameter("VEH_WHLBASE", std::bind(&CarState::setAxisDistance, this, std::placeholders::_1), std::bind(&CarState::getAxisDistance, this));
}
