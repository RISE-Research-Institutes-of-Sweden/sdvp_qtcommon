/*
 *     Copyright 2021 Marvin Damschen   marvin.damschen@ri.se
 *               2021 Rickard Häll      rickard.hall@ri.se
 *     Published under GPLv3: https://www.gnu.org/licenses/gpl-3.0.html
 * 
 * Implements communication with a moving u-blox-based GNSS receiver
 */

#ifndef UBLOXROVER_H
#define UBLOXROVER_H

#include <QObject>
#include <QSharedPointer>
#include "ublox.h"
#include "core/coordinatetransforms.h"
#include "vehicles/vehiclestate.h"

class UbloxRover : public QObject
{
    Q_OBJECT
public:
    UbloxRover(QSharedPointer<VehicleState> vehicleState);
    bool connectSerial(const QSerialPortInfo &serialPortInfo);
    bool isSerialConnected();
    llh_t getEnuRef() const;
    void setEnuRef(llh_t enuRef);
    void writeRtcmToUblox(QByteArray data);
    void writeOdoToUblox(ubx_esf_datatype_enum dataType, uint32_t dataField);
    void saveOnShutdown();
    void setIMUOrientationOffset(double roll_deg, double pitch_deg, double yaw_deg);

signals:
    void updatedGNSSPositionAndYaw(QSharedPointer<VehicleState> vehicleState, double distanceMoved, bool fused);
    void txNavPvt(const ubx_nav_pvt &pvt);
    void gotNmeaGga(const QByteArray& nmeaGgaStr);

private:
    bool configureUblox();
    void updSosResponse(const ubx_upd_sos &sos);
    void updateGNSSPositionAndYaw(const ubx_nav_pvt &pvt);

    const int ms_per_day = 24 * 60 * 60 * 1000;

    llh_t mEnuReference;
    bool mEnuReferenceSet = false;

    Ublox mUblox;
    QSharedPointer<VehicleState> mVehicleState;
    struct {double rollOffset_deg, pitchOffset_deg, yawOffset_deg;} mIMUOrientationOffset;
};

#endif // UBLOXROVER_H
