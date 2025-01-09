/*
    Copyright 2012 Benjamin Vedder	benjamin@vedder.se
              2020 Marvin Damschen  marvin.damschen@ri.se

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
    */

#include "pospoint.h"
#include <cmath>

PosPoint::PosPoint(double x, double y, double height, double roll, double pitch, double yaw, double speed,
                   double radius, double sigma, QTime time, int id, bool drawLine, quint32 attributes, PosType type) :
    mX(x), mY(y), mHeight(height), mRoll(roll), mPitch(pitch), mYaw(yaw), mSpeed(speed),
    mRadius(radius), mSigma(sigma), mTime(time), mId(id), mDrawLine(drawLine),
    mAttributes(attributes), mType(type)
{
}

PosPoint::PosPoint(const PosPoint &point) : QObject()
{
    *this = point;
}

double PosPoint::getX() const
{
    return mX;
}

double PosPoint::getY() const
{
    return mY;
}

double PosPoint::getHeight() const
{
    return mHeight;
}

xyz_t PosPoint::getXYZ() const
{
    return {mX, mY, mHeight};
}

double PosPoint::getRoll() const
{
    return mRoll;
}

double PosPoint::getPitch() const
{
    return mPitch;
}

double PosPoint::getYaw() const
{
    return mYaw;
}

double PosPoint::getSpeed() const
{
    return mSpeed;
}

QPointF PosPoint::getPoint() const
{
    return QPointF(mX, mY);
}

QPointF PosPoint::getPointMm() const
{
    return QPointF(mX * 1000.0, mY * 1000.0);
}

double PosPoint::getRadius() const
{
    return mRadius;
}

double PosPoint::getSigma() const
{
    return mSigma;
}

void PosPoint::setX(double x)
{
    mX = x;
}

void PosPoint::setY(double y)
{
    mY = y;
}

void PosPoint::setHeight(double height)
{
    mHeight = height;
}

void PosPoint::setXY(double x, double y)
{
    mX = x;
    mY = y;
}

void PosPoint::setXYZ(xyz_t xyz)
{
    mX = xyz.x;
    mY = xyz.y;
    mHeight = xyz.z;
}

void PosPoint::setTime(const QTime &time)
{
    mTime = time;
}

void PosPoint::setId(int id)
{
    mId = id;
}

QString PosPoint::getInfo() const
{
    return mInfo;
}

QTime PosPoint::getTime() const
{
    return mTime;
}

int PosPoint::getId() const
{
    return mId;
}

PosPoint &PosPoint::operator =(const PosPoint &point)
{
    mX = point.mX;
    mY = point.mY;
    mHeight = point.mHeight;
    mRoll = point.mRoll;
    mPitch = point.mPitch;
    mYaw = point.mYaw;
    mSpeed = point.mSpeed;
    mRadius = point.mRadius;
    mSigma = point.mSigma;
    mInfo = point.mInfo;
    mTime = point.mTime;
    mId = point.mId;
    mDrawLine = point.mDrawLine;
    mAttributes = point.mAttributes;
    mType = point.mType;
    return *this;
}

bool PosPoint::getDrawLine() const
{
    return mDrawLine;
}

quint32 PosPoint::getAttributes() const
{
    return mAttributes;
}

double PosPoint::getDistanceTo(const PosPoint &point) const
{
    return sqrt((point.mX - mX) * (point.mX - mX) +
                (point.mY - mY) * (point.mY - mY));
}

double PosPoint::getDistanceTo3d(const PosPoint &point) const
{
    return sqrt((point.mX - mX) * (point.mX - mX) +
                (point.mY - mY) * (point.mY - mY) +
                (point.mHeight - mHeight) * (point.mHeight - mHeight));
}

bool PosPoint::operator ==(const PosPoint &point)
{
    if (    mX == point.mX &&
            mY == point.mY &&
            mHeight == point.mHeight &&
            mRoll == point.mRoll &&
            mPitch == point.mPitch &&
            mYaw == point.mYaw &&
            mSpeed == point.mSpeed &&
            mRadius == point.mRadius &&
            mSigma == point.mSigma &&
            mInfo == point.mInfo &&
            mTime == point.mTime &&
            mId == point.mId &&
            mDrawLine == point.mDrawLine &&
            mAttributes == point.mAttributes) {
        return true;
    } else {
        return false;
    }
}

bool PosPoint::operator !=(const PosPoint &point)
{
    return !(operator==(point));
}

PosType PosPoint::getType() const
{
    return mType;
}

void PosPoint::setType(const PosType &type)
{
    mType = type;
}

void PosPoint::setInfo(const QString &info)
{
    mInfo = info;
}

void PosPoint::setRoll(double roll)
{
    mRoll = roll;
}

void PosPoint::setPitch(double pitch)
{
    mPitch = pitch;
}

void PosPoint::setYaw(double yaw)
{
    mYaw = yaw;
}

void PosPoint::setRollPitchYaw(double roll, double pitch, double yaw)
{
    mRoll = roll;
    mPitch = pitch;
    mYaw = yaw;
}

void PosPoint::setSpeed(double speed)
{
    mSpeed = speed;
}

void PosPoint::setRadius(double radius)
{
    mRadius = radius;
}

void PosPoint::setSigma(double sigma)
{
    mSigma = sigma;
}

void PosPoint::setDrawLine(bool drawLine)
{
    mDrawLine = drawLine;
}

void PosPoint::setAttributes(quint32 attributes)
{
    mAttributes = attributes;
}

void PosPoint::transform(xyz_t offset, double yaw_rad)
{
    double cos_yaw = cos(yaw_rad);
    double sin_yaw = sin(yaw_rad);
    double dx = offset.x * cos_yaw - offset.y * sin_yaw;
    double dy = offset.x * sin_yaw + offset.y * cos_yaw;
    setX(getX() + dx);
    setY(getY() + dy);
    setYaw(yaw_rad * 180.0 / M_PI);
}