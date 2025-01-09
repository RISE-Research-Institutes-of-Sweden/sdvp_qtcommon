/*
 *     Copyright 2016 - 2017 Benjamin Vedder	benjamin@vedder.se
 *               2021        Marvin Damschen    marvin.damschen@ri.se
 *               2024        Rickard Häll       rickard.hall@ri.se
 *     Published under GPLv3: https://www.gnu.org/licenses/gpl-3.0.html
 *
 * Header-only coordinate transformations, mainly ENU <-> latitude, longitude, height
 */

#ifndef COORDINATETRANSFORMS_H
#define COORDINATETRANSFORMS_H

#include <cmath>
#include <QPointF>

struct llh_t {
    double latitude;
    double longitude;
    double height;
};

struct xyz_t {
    double x;
    double y;
    double z;

    // Vector addition operator overload
    xyz_t operator+(const xyz_t& other) const {
        return {x + other.x, y + other.y, z + other.z};
    }

    // Scalar addition operator overload
    xyz_t operator+(const double scalar) const {
        return {x + scalar, y + scalar, z + scalar};
    }

    // Friend function for scalar addition from the left
    friend xyz_t operator+(const double scalar, const xyz_t& vec) {
        return vec + scalar;
    }

    // Vector subtraction operator overload
    xyz_t operator-(const xyz_t& other) const {
        return {x - other.x, y - other.y, z - other.z};
    }

    // Scalar subtraction operator overload
    xyz_t operator-(const double scalar) const {
        return {x - scalar, y - scalar, z - scalar};
    }

    // Friend function for scalar subtraction from the left
    friend xyz_t operator-(const double scalar, const xyz_t& vec) {
        return xyz_t{scalar, scalar, scalar} - vec;
    }

    // Unary minus operator overload
    xyz_t operator-() const {
        return {-x, -y, -z};
    }
};

namespace coordinateTransforms {

#define FE_WGS84        (1.0/298.257223563) // earth flattening (WGS84)
#define RE_WGS84        6378137.0           // earth semimajor axis (WGS84) (m)

inline xyz_t llhToXyz(llh_t llh)
{
    double sinp = sin(llh.latitude * M_PI / 180.0);
    double cosp = cos(llh.latitude * M_PI / 180.0);
    double sinl = sin(llh.longitude * M_PI / 180.0);
    double cosl = cos(llh.longitude * M_PI / 180.0);
    double e2 = FE_WGS84 * (2.0 - FE_WGS84);
    double v = RE_WGS84 / sqrt(1.0 - e2 * sinp * sinp);

    xyz_t res;
    res.x = (v + llh.height) * cosp * cosl;
    res.y = (v + llh.height) * cosp * sinl;
    res.z = (v * (1.0 - e2) + llh.height) * sinp;
    return res;
}

inline llh_t xyzToLlh(const xyz_t &xyz)
{
    double e2 = FE_WGS84 * (2.0 - FE_WGS84);
    double r2 = xyz.x * xyz.x + xyz.y * xyz.y;
    double za = xyz.z;
    double zk = 0.0;
    double sinp = 0.0;
    double v = RE_WGS84;

    while (fabs(za - zk) >= 1E-4) {
        zk = za;
        sinp = za / sqrt(r2 + za * za);
        v = RE_WGS84 / sqrt(1.0 - e2 * sinp * sinp);
        za = xyz.z + v * e2 * sinp;
    }

    llh_t res;
    res.latitude = (r2 > 1E-12 ? atan(za / sqrt(r2)) : (xyz.z > 0.0 ? M_PI / 2.0 : -M_PI / 2.0)) * 180.0 / M_PI;
    res.longitude = (r2 > 1E-12 ? atan2(xyz.y, xyz.x) : 0.0) * 180.0 / M_PI;
    res.height = sqrt(r2 + za * za) - v;
    return res;
}

inline void createEnuMatrix(double lat, double lon, double *enuMat)
{
    double so = sin(lon * M_PI / 180.0);
    double co = cos(lon * M_PI / 180.0);
    double sa = sin(lat * M_PI / 180.0);
    double ca = cos(lat * M_PI / 180.0);

    // ENU
    enuMat[0] = -so;
    enuMat[1] = co;
    enuMat[2] = 0.0;

    enuMat[3] = -sa * co;
    enuMat[4] = -sa * so;
    enuMat[5] = ca;

    enuMat[6] = ca * co;
    enuMat[7] = ca * so;
    enuMat[8] = sa;

    // NED
    //    enuMat[0] = -sa * co;
    //    enuMat[1] = -sa * so;
    //    enuMat[2] = ca;

    //    enuMat[3] = -so;
    //    enuMat[4] = co;
    //    enuMat[5] = 0.0;

    //    enuMat[6] = -ca * co;
    //    enuMat[7] = -ca * so;
    //    enuMat[8] = -sa;
}

inline xyz_t llhToEnu(const llh_t &iLlh, const llh_t &llh)
{
    xyz_t iXyz = llhToXyz(iLlh);
    xyz_t xyz = llhToXyz(llh);

    double enuMat[9];
    createEnuMatrix(iLlh.latitude, iLlh.longitude, enuMat);

    xyz_t dXyz = {xyz.x - iXyz.x, xyz.y - iXyz.y, xyz.z - iXyz.z};

    xyz.x = enuMat[0] * dXyz.x + enuMat[1] * dXyz.y + enuMat[2] * dXyz.y;
    xyz.y = enuMat[3] * dXyz.x + enuMat[4] * dXyz.y + enuMat[5] * dXyz.z;
    xyz.z = enuMat[6] * dXyz.x + enuMat[7] * dXyz.y + enuMat[8] * dXyz.z;

    return xyz;
}

inline llh_t enuToLlh(const llh_t &iLlh, const xyz_t &xyz)
{
    xyz_t iXyz = llhToXyz(iLlh);

    double enuMat[9];
    createEnuMatrix(iLlh.latitude, iLlh.longitude, enuMat);

    xyz_t temp_xyz = {
        enuMat[0] * xyz.x + enuMat[3] * xyz.y + enuMat[6] * xyz.z + iXyz.x,
        enuMat[1] * xyz.x + enuMat[4] * xyz.y + enuMat[7] * xyz.z + iXyz.y,
        enuMat[2] * xyz.x + enuMat[5] * xyz.y + enuMat[8] * xyz.z + iXyz.z
    };

    return xyzToLlh(temp_xyz);
}

inline xyz_t nedToENU(const xyz_t &xyzNED) {
    return {xyzNED.y, xyzNED.x, -xyzNED.z};
}

inline xyz_t enuToNED(const xyz_t &xyzENU) {
    return nedToENU(xyzENU);
}

inline double yawNEDtoENU(double yaw_degNED) {
    double yaw_degENU = 90.0 - yaw_degNED;

    // normalize to [-180.0:180.0[
    while (yaw_degENU < -180.0)
        yaw_degENU += 360.0;
    while (yaw_degENU >= 180.0)
        yaw_degENU -= 360.0;

    return yaw_degENU;
}

inline double yawENUtoNED(double yaw_degENU) {
    double yaw_degNED = 90.0 - yaw_degENU;

    // normalize to [0.0:360.0[
    while (yaw_degNED < 0.0)
        yaw_degNED += 360.0;
    while (yaw_degNED >= 360.0)
        yaw_degNED -= 360.0;

    return yaw_degNED;
}

inline QPointF ENUToVehicleFrame(const QPointF &point, const xyz_t &vehiclePosition, const double yawDegENU)
{
    // 1. transform point to vehicle frame
    QPointF pointInVehicleFrame;
    // translate
    pointInVehicleFrame.setX(point.x()-vehiclePosition.x);
    pointInVehicleFrame.setY(point.y()-vehiclePosition.y);
    // rotate
    double currYaw_rad = yawDegENU * M_PI / 180.0;
    const double newX = cos(-currYaw_rad)*pointInVehicleFrame.x() - sin(-currYaw_rad)*pointInVehicleFrame.y();
    const double newY = sin(-currYaw_rad)*pointInVehicleFrame.x() + cos(-currYaw_rad)*pointInVehicleFrame.y();
    pointInVehicleFrame.setX(newX);
    pointInVehicleFrame.setY(newY);

    return pointInVehicleFrame;
}
} // coordinateTransforms

#endif // COORDINATETRANSFORMS_H
