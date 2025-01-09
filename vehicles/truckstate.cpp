/*
 *     Copyright 2024 RISE Sweden
 *     Published under GPLv3: https://www.gnu.org/licenses/gpl-3.0.html
 *
 * Specific implementation of VehicleState for car-type (ackermann) vehicles, storing all (dynamic and static) state
 */

#include "truckstate.h"
#include <QDebug>
#include <QDateTime>

TruckState::TruckState(ObjectID_t id, Qt::GlobalColor color) : CarState(id, color)
{
    // Additional initialization if needed for the TruckState
    ObjectState::setWaywiseObjectType(WAYWISE_OBJECT_TYPE_TRUCK);
}

void TruckState::setLength(double length)
{
    VehicleState::setLength(length);

    xyz_t rearEndOffset = getRearEndOffset();
    rearEndOffset.x = - length / 2.0;
    setRearEndOffset(rearEndOffset);

    xyz_t hitchOffset = getHitchOffset();
    hitchOffset.x = - length / 4.0;
    setHitchOffset(hitchOffset);
}

double TruckState::getCurvatureToPointInVehicleFrame(const QPointF &point)
{
    if (hasTrailingVehicle())
        return getCurvatureWithTrailer(point);
    else
        return CarState::getCurvatureToPointInVehicleFrame(point);
}

void TruckState::updateOdomPositionAndYaw(double drivenDistance, PosType usePosType)
{
    updateTrailingVehicleOdomPositionAndYaw(drivenDistance, usePosType);
    CarState::updateOdomPositionAndYaw(drivenDistance, usePosType);
}

void TruckState::setPosition(PosPoint &point)
{
    CarState::setPosition(point);
    if (hasTrailingVehicle()){
        updateTrailingVehicleOdomPositionAndYaw(0, point.getType());
    }
}

bool TruckState::getSimulateTrailer() const
{
    return mSimulateTrailer;
}

void TruckState::setSimulateTrailer(bool simulateTrailer)
{
    mSimulateTrailer = simulateTrailer;
}

void TruckState::updateTrailingVehicleOdomPositionAndYaw(double drivenDistance, PosType usePosType)
{
    if(hasTrailingVehicle()) {
        auto hitchPosition = getPosition(usePosType);
        QSharedPointer<TrailerState> trailer = getTrailingVehicle();
        PosPoint currentTrailerPosition = trailer->getPosition(usePosType);

        double currYaw_rad = hitchPosition.getYaw() * M_PI / 180.0;

        double trailerYaw_rad = getTrailingVehicle()->getPosition(usePosType).getYaw() * M_PI / 180.0;
        if (mSimulateTrailer) { // We do not get external updates on the trailer angle -> simple estimation
            trailerYaw_rad = trailerYaw_rad + ((drivenDistance / getTrailingVehicle()->getWheelBase()) * sin(currYaw_rad - trailerYaw_rad));
            trailerYaw_rad = fmod(trailerYaw_rad + M_PI, 2 * M_PI) - M_PI;
            setTrailerAngle((currYaw_rad - trailerYaw_rad) * 180.0 / M_PI);
        } else {
            trailerYaw_rad = currYaw_rad - getTrailerAngleRadians();
            trailerYaw_rad = fmod(trailerYaw_rad + M_PI, 2 * M_PI) - M_PI;
        }

        double delta_x = (trailer->getWheelBase()) * cos(trailerYaw_rad); // trailer x difference from x of the truck wheelbase
        double delta_y = (trailer->getWheelBase()) * sin(trailerYaw_rad); // trailer y difference from y of the truck wheelbase

        currentTrailerPosition.setX(hitchPosition.getX() - delta_x);
        currentTrailerPosition.setY(hitchPosition.getY() - delta_y);
        currentTrailerPosition.setYaw(trailerYaw_rad * 180.0 / M_PI);
        currentTrailerPosition.setTime(QTime::currentTime().addSecs(-QDateTime::currentDateTime().offsetFromUtc()));
        trailer->setPosition(currentTrailerPosition);
    }
}

double TruckState::getCurvatureWithTrailer(const QPointF &pointInVehicleFrame)
{
    double trailerAngle_rad = getTrailerAngleRadians();
    trailerAngle_rad = fmod(trailerAngle_rad + M_PI, 2 * M_PI) - M_PI;
    double desiredTrailerAngle_rad;
    double l2 = getTrailingVehicle()->getWheelBase(); // trailer wheelbase in meters
    double gain;

    if (getSpeed() > 0){
        double theta_err =  atan2(pointInVehicleFrame.y(), pointInVehicleFrame.x());
        desiredTrailerAngle_rad = atan(2 * l2 * sin(theta_err)); // desired trailer angle towards truck
        gain = getPurePursuitForwardGain();
    } else {
        double trailerPositionInVehicleFrame_x = -(l2) * cos(-trailerAngle_rad);
        double trailerPositionInVehicleFrame_y = -(l2) * sin(-trailerAngle_rad);

        double theta_err =  atan2(pointInVehicleFrame.y() - trailerPositionInVehicleFrame_y,
                                 pointInVehicleFrame.x() - trailerPositionInVehicleFrame_x) - -trailerAngle_rad;
        desiredTrailerAngle_rad = atan(2 * l2 * sin(theta_err) / getAutopilotRadius());
        gain = getPurePursuitReverseGain();
    }
    double curve = gain * (trailerAngle_rad - desiredTrailerAngle_rad) - (sin(trailerAngle_rad) / (l2));
    return curve / cos(trailerAngle_rad);
}

QSharedPointer<TrailerState> TruckState::getTrailingVehicle() const {
    // NOTE: setTrailingVehicle / getTrailingVehicle hide the functions inherited from VehicleState.
    // This means, we know the trailing vehicle is a TrailerState here with reasonable certainty.
    // This is not perfect but I have no better solution right now.
    return qSharedPointerDynamicCast<TrailerState>(VehicleState::getTrailingVehicle());
}

void TruckState::setTrailingVehicle(QSharedPointer<TrailerState> trailer)
{
    VehicleState::setTrailingVehicle(trailer);
}

void TruckState::setTrailerAngle(double angle_deg)
{
    mTrailerAngle_deg = angle_deg;
}

#ifdef QT_GUI_LIB
void TruckState::draw(QPainter &painter, const QTransform &drawTrans, const QTransform &txtTrans, bool isSelected)
{
    PosPoint pos = getPosition();

    const double truck_len = getLength() * 1000.0;
    const double truck_w = getWidth() * 1000.0;
    const double truck_corner = 0.02 * 1000.0;

    double x = pos.getX() * 1000.0;
    double y = pos.getY() * 1000.0;
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

    // Draw truck
    painter.setBrush(QBrush(col_wheels));
    painter.save();
    painter.translate(x, y);
    painter.rotate(pos.getYaw());
    // Wheels
    painter.drawRoundedRect(-truck_len / 12.0,-(truck_w / 2), truck_len / 6.0, truck_w, truck_corner / 3, truck_corner / 3);
    painter.drawRoundedRect(truck_len - truck_len / 2.5,-(truck_w / 2), truck_len / 9.0, truck_w, truck_corner / 3, truck_corner / 3);
    // Front bumper
    painter.setBrush(col_bumper);
    painter.drawRoundedRect(-truck_len / 6.0, -((truck_w - truck_len / 20.0) / 2.0), truck_len, truck_w - truck_len / 20.0, truck_corner, truck_corner);
    // Hull
    painter.setBrush(col_hull);
    painter.drawRoundedRect(-truck_len / 6.0, -((truck_w - truck_len / 20.0) / 2.0), truck_len - (truck_len / 20.0), truck_w - truck_len / 20.0, truck_corner, truck_corner);

    if (hasTrailingVehicle())
        getTrailingVehicle()->drawTrailer(painter, drawTrans, pos, getTrailerAngleDegrees());

    painter.restore();

    painter.setBrush(col_center);
    painter.drawEllipse(QPointF(x, y), truck_w / 15.0, truck_w / 15.0);

    // Turning radius of truck
    painter.setPen(QPen(Qt::blue, 30));
    painter.setBrush(Qt::transparent);

    painter.drawEllipse(QPointF(x, y), getAutopilotRadius()*1000.0, getAutopilotRadius()*1000.0);
    painter.setPen(Qt::black);
    // Turning radius

    // TODO: needs cleanup
    double trailerYaw = 0.0, dx = 0.0, dy = 0.0;
    if (hasTrailingVehicle()) {
        double trailerAngle  = getTrailerAngleRadians();
        double currYaw_rad = getPosition().getYaw() * (M_PI/180.0);
        double trailerYaw = currYaw_rad- trailerAngle;
        double trailerAxis = getTrailingVehicle()->getWheelBase();
        double dx = trailerAxis * cos(trailerYaw);
        double dy = trailerAxis * sin(trailerYaw);
        double newX = (pos.getX() - dx) *1000.0;
        double newY = ( pos.getY() - dy) *1000.0;

        painter.setBrush(Qt::darkMagenta);
        painter.drawEllipse(QPointF(newX, newY), truck_w / 15.0, truck_w / 15.0);
        painter.setPen(QPen(Qt::darkMagenta, 20));
        painter.setBrush(Qt::transparent);
        painter.drawEllipse(QPointF(newX, newY), getAutopilotRadius()*1000.0, getAutopilotRadius()*1000.0);
        painter.setPen(Qt::black);
    }

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
                  << "(" << pos.getX() << ", " << pos.getY() << ", " << pos.getHeight() << ", " << pos.getYaw() << ")" << Qt::endl
                  << "State: " << (getIsArmed() ? "armed" : "disarmed") << Qt::endl
                  << flightModeStr << Qt::endl << Qt::endl;
        if (hasTrailingVehicle()) {
            txtStream << getTrailingVehicle()->getName() << Qt::endl
                    << "(" << pos.getX()- dx << ", " << pos.getY()- dy << ", " << pos.getHeight() << ", "
                    << trailerYaw * (180.0/M_PI)<< ")" << Qt::endl;
        }
        pt_txt.setX(x + truck_w + truck_len * ((cos(getPosition().getYaw() * (M_PI/180.0)) + 1) / 3));
        pt_txt.setY(y);
        painter.setTransform(txtTrans);
        pt_txt = drawTrans.map(pt_txt);
        rect_txt.setCoords(pt_txt.x(), pt_txt.y() - 40,
                           pt_txt.x() + 400, pt_txt.y() + 65);
        painter.drawText(rect_txt, txt);
    }

}

#endif
