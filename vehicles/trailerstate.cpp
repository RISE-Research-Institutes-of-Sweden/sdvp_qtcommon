/*
 *     Copyright 2024 RISE Sweden
 *     Published under GPLv3: https://www.gnu.org/licenses/gpl-3.0.html
 *
 */

#include "trailerstate.h"
#include "communication/parameterserver.h"
#include <QDebug>

TrailerState::TrailerState(ObjectID_t id, Qt::GlobalColor color) : VehicleState (id, color)
{
    // Default values from Griffin
    setLength(0.96); // [m]
    setWidth(0.21); // [m]
    setWheelBase(0.64); // [m]

    ObjectState::setWaywiseObjectType(WAYWISE_OBJECT_TYPE_TRAILER);

}

void TrailerState::setLength(double length)
{
    VehicleState::setLength(length);

    xyz_t rearEndOffset = getRearEndOffset();
    rearEndOffset.x = - length / 2.0;
    setRearEndOffset(rearEndOffset);

    xyz_t hitchOffset = getHitchOffset();
    hitchOffset.x = length / 4.0;
    setHitchOffset(hitchOffset);
}

void TrailerState::setWheelBase(double value)
{
    mWheelBase = value;

    xyz_t rearAxleOffset = getRearAxleOffset();
    rearAxleOffset.x = - value / 2.0;
    setRearAxleOffset(rearAxleOffset);
}

void TrailerState::provideParameters()
{
    ParameterServer::getInstance()->provideIntParameter("TRLR_COMP_ID", std::bind(&TrailerState::setId, this, std::placeholders::_1, false), std::bind(&TrailerState::getId, this));
    ParameterServer::getInstance()->provideFloatParameter("TRLR_LENGTH", std::bind(&TrailerState::setLength, this, std::placeholders::_1), std::bind(&TrailerState::getLength, this));
    ParameterServer::getInstance()->provideFloatParameter("TRLR_WIDTH", std::bind(&TrailerState::setWidth, this, std::placeholders::_1), std::bind(&TrailerState::getWidth, this));
    ParameterServer::getInstance()->provideFloatParameter("TRLR_WHLBASE", std::bind(&TrailerState::setWheelBase, this, std::placeholders::_1), std::bind(&TrailerState::getWheelBase, this));
}

void TrailerState::updateOdomPositionAndYaw(double drivenDistance, PosType usePosType)
{
    // In our context, the trailer is passive.
    // Its position / state is updated by the truck / towing vehicle it is attached to
    Q_UNUSED(drivenDistance)
    Q_UNUSED(usePosType)
}

#ifdef QT_GUI_LIB
// draw on demand
void TrailerState::drawTrailer(QPainter &painter, const QTransform &drawTrans)
{
    if (!isStateInitialized())
        return;

    PosPoint pos = getPosition();
    double x = pos.getX() * 1000.0  ;// convert from m to mm (on the map objects are in mm)
    double y = pos.getY() * 1000.0  ;
    double trailer_len = getLength() * 1000.0;
    const double trailer_w = getWidth() * 1000.0;
    const double trailer_corner = 0.02 * 1000.0;
    xyz_t rearAxleOffset = getRearAxleOffset();
    xyz_t rearEndOffset = getRearEndOffset();
    double rearAxleOffsetX = rearAxleOffset.x * 1000.0;
    const double rearAxleOffsetY = rearAxleOffset.y * 1000.0;
    double rearEndOffsetX = rearEndOffset.x * 1000.0;
    xyz_t hitchOffset = getHitchOffset();

    painter.setTransform(drawTrans);
    painter.translate(x, y);
    painter.rotate(pos.getYaw());

    // Rear axle wheels
    painter.setBrush(QBrush(Qt::darkGray));
    double wheel_diameter = trailer_len / 9.0;
    double wheel_width = trailer_w / 12.0;
    painter.drawRoundedRect(rearAxleOffsetX - wheel_diameter/2, rearAxleOffsetY - (trailer_w / 2 + wheel_width / 2), wheel_diameter, (trailer_w + wheel_width), trailer_corner / 3, trailer_corner / 3);

    // Draw trailer
    // simple draw a rectangle representing the trailer
    painter.setBrush(getColor());
    painter.drawRoundedRect(rearEndOffsetX, -((trailer_w - trailer_len / 20.0) / 2.0), trailer_len - (trailer_len / 20.0), trailer_w - trailer_len / 20.0, trailer_corner, trailer_corner);

    // Center
    painter.setBrush(Qt::red);
    painter.drawEllipse(QPointF(0, 0), trailer_w / 15.0, trailer_w / 15.0);

    // Hitch
    painter.setBrush(Qt::black);
    painter.drawEllipse(QPointF(hitchOffset.x*1000.0, hitchOffset.y*1000.0), trailer_w / 9.0, trailer_w / 9.0);
}

void TrailerState::draw(QPainter &painter, const QTransform &drawTrans, const QTransform &txtTrans, bool isSelected){
    // draw a trailer indepentently

    (void)txtTrans;
    (void)isSelected;
    PosPoint pos = getPosition();
    double x = pos.getX() * 1000.0  ;// convert from m to mm (on the map objects are in mm)
    double y = pos.getY() * 1000.0  ;
    double trailer_len = getLength() * 1000.0;
    const double trailer_w = getWidth() * 1000.0;
    const double trailer_corner = 0.02 * 1000.0;

    painter.setTransform(drawTrans);
    painter.translate(x, y);
    painter.rotate(pos.getYaw() - this->mAngle + 180);

    // Wheels
    painter.setBrush(QBrush(Qt::darkGray));
    painter.drawRoundedRect(trailer_len - trailer_len / 2.5,-(trailer_w / 2), trailer_len / 9.0, trailer_w, trailer_corner / 3, trailer_corner / 3);

    // Draw trailer
    // simple draw a rectangle representing the trailer
    painter.setBrush(getColor());
    painter.drawRoundedRect(-trailer_len / 6.0, -((trailer_w - trailer_len / 20.0) / 2.0), trailer_len - (trailer_len / 20.0), trailer_w - trailer_len / 20.0, trailer_corner, trailer_corner);

}
#endif
