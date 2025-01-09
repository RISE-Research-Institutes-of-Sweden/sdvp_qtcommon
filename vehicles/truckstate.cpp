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
        PosPoint truckHitchPosition = getOffsetPosition(getHitchOffset(), usePosType);
        QSharedPointer<TrailerState> trailer = getTrailingVehicle();
        PosPoint currentTrailerPosition = trailer->getPosition(usePosType);

        double currYaw_rad = truckHitchPosition.getYaw() * M_PI / 180.0;

        double trailerYaw_rad = trailer->getPosition(usePosType).getYaw() * M_PI / 180.0;
        if (mSimulateTrailer) { // We do not get external updates on the trailer angle -> simple estimation
            trailerYaw_rad = trailerYaw_rad + ((drivenDistance / trailer->getWheelBase()) * sin(currYaw_rad - trailerYaw_rad));
            trailerYaw_rad = std::clamp(fmod(trailerYaw_rad + M_PI, 2 * M_PI) - M_PI, -2 * M_PI / 3.0, 2 * M_PI / 3.0);
            setTrailerAngle((currYaw_rad - trailerYaw_rad) * 180.0 / M_PI);
        } else {
            trailerYaw_rad = currYaw_rad - getTrailerAngleRadians();
            trailerYaw_rad = fmod(trailerYaw_rad + M_PI, 2 * M_PI) - M_PI;
        }
        currentTrailerPosition.setXYZ(truckHitchPosition.getXYZ());
        xyz_t trailingVehicleHitchOffset = getTrailingVehicle()->getHitchOffset();
        currentTrailerPosition.transform(-trailingVehicleHitchOffset, trailerYaw_rad);
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
    xyz_t rearAxleOffset = getRearAxleOffset();
    xyz_t rearEndOffset = getRearEndOffset();
    double rearAxleOffsetX = rearAxleOffset.x * 1000.0;
    const double rearAxleOffsetY = rearAxleOffset.y * 1000.0;
    double rearEndOffsetX = rearEndOffset.x * 1000.0;
    const double wheelbase = getAxisDistance() * 1000.0;
    PosPoint hitchPos = getOffsetPosition(getHitchOffset());

    painter.setTransform(drawTrans);

    QColor col_wheels;
    QColor col_bumper;
    QColor col_ap;
    QColor col_sigma = Qt::red;
    QColor col_hull = getColor();
    QColor col_center = Qt::red;
    QColor col_hitch = Qt::magenta;

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
    painter.save();
    painter.translate(x, y);
    painter.rotate(pos.getYaw());
    // Rear axle wheels
    double wheel_diameter = truck_len / 6.0;
    double wheel_width = truck_w / 12.0;
    painter.setBrush(QBrush(col_wheels));
    painter.drawRoundedRect(rearAxleOffsetX - wheel_diameter/2, rearAxleOffsetY - (truck_w / 2 + wheel_width / 2), wheel_diameter, (truck_w + wheel_width), truck_corner / 3, truck_corner / 3);
    // Front axle wheels
    wheel_diameter = truck_len / 10.0;
    painter.drawRoundedRect(rearAxleOffsetX + wheelbase - wheel_diameter/2, -(truck_w / 2 + wheel_width / 2), wheel_diameter, (truck_w + wheel_width), truck_corner / 3, truck_corner / 3);
    // Front bumper
    painter.setBrush(col_bumper);
    painter.drawRoundedRect(rearEndOffsetX, -((truck_w - truck_len / 20.0) / 2.0), truck_len, truck_w - truck_len / 20.0, truck_corner, truck_corner);
    // Hull
    painter.setBrush(col_hull);
    painter.drawRoundedRect(rearEndOffsetX, -((truck_w - truck_len / 20.0) / 2.0), truck_len - (truck_len / 20.0), truck_w - truck_len / 20.0, truck_corner, truck_corner);

    if (hasTrailingVehicle())
    {
        getTrailingVehicle()->drawTrailer(painter, drawTrans);
    }

    painter.restore();

    painter.setBrush(col_center);
    painter.drawEllipse(QPointF(x, y), truck_w / 15.0, truck_w / 15.0);
    if (hasTrailingVehicle())
    {
        painter.setBrush(col_hitch);
        painter.drawEllipse(QPointF(hitchPos.getX()*1000.0, hitchPos.getY()*1000.0), truck_w / 20.0, truck_w / 20.0);
    }

    // Turning radius
    if (getAutopilotRadius() >0.001){
        painter.setPen(QPen(Qt::blue, 30));
        painter.setBrush(Qt::transparent);
        painter.drawEllipse(QPointF(x, y), getAutopilotRadius()*1000.0, getAutopilotRadius()*1000.0);

        painter.setPen(QPen(Qt::darkMagenta, 20));
        painter.setBrush(Qt::transparent);
        PosPoint trailerPos = getTrailingVehicle()->getPosition();
        painter.drawEllipse(QPointF(trailerPos.getX()*1000.0, trailerPos.getY()*1000.0), getAutopilotRadius()*1000.0, getAutopilotRadius()*1000.0);
    }
    painter.setPen(Qt::black);

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
            PosPoint trailerPos = getTrailingVehicle()->getPosition();
            txtStream << getTrailingVehicle()->getName() << Qt::endl
                    << "(" << trailerPos.getX() << ", " << trailerPos.getY() << ", " << trailerPos.getHeight() << ", "
                    << trailerPos.getYaw()<< ")" << Qt::endl;
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
