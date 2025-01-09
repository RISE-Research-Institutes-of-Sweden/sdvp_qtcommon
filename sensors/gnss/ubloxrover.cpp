/*
 *     Copyright 2021 Marvin Damschen   marvin.damschen@ri.se
 *               2021 Rickard Häll      rickard.hall@ri.se
 *     Published under GPLv3: https://www.gnu.org/licenses/gpl-3.0.html
 */
#include "ubloxrover.h"
#include <QDebug>
#include <QDateTime>
#include <QLineF>
#include <cmath>

UbloxRover::UbloxRover(QSharedPointer<VehicleState> vehicleState)
{
    mVehicleState = vehicleState;
    mEnuReference = {57.6828, 11.9637, 0}; // AztaZero {57.7810, 12.7692, 0}, Klätterlabbet {57.6876, 11.9807, 0};

    // Use GNSS reception to update location
    connect(&mUblox, &Ublox::rxNavPvt, this, &UbloxRover::updateGNSSPositionAndYaw);

    // Save-on-shutdown feature
    connect(&mUblox, &Ublox::rxUpdSos, this, &UbloxRover::updSosResponse);

    // Fordward received NMEA GGA messages
    connect(&mUblox, &Ublox::rxNmeaGga, this, &UbloxRover::gotNmeaGga);

    // Automatic IMU orientation alignment feature
    connect(&mUblox, &Ublox::rxEsfAlg, this, [this](const ubx_esf_alg &alg){
        if (alg.autoMntAlgOn)
            setIMUOrientationOffset(alg.roll, alg.pitch, (coordinateTransforms::yawNEDtoENU(alg.yaw) - 180.0));

    //        static int count = 0;
    //        if (count++%5)
    //            return;
    //        qDebug() << "ESF-ALG data:"
    //                 << "\nStatus:" << alg.status
    //                 << "\nAuto mount alignmend enabled:" << alg.autoMntAlgOn
    //                 << "\nRoll, Pitch, Yaw:" << alg.roll << alg.pitch << alg.yaw;
    });

    // Print nav-pvt message
    // Search for nav-pvt in following document:
    // https://www.u-blox.com/sites/default/files/ZED-F9R_Interfacedescription_UBX-19056845.pdf
//    connect(&mUblox, &Ublox::rxNavPvt, this, [](const ubx_nav_pvt &pvt){
//        static int count = 0;
//        if (count++%5)
//            return;
//        qDebug() << "\nNAV-PVT data:"
//                 << "\nDate: " << pvt.year << pvt.month << pvt.day
//                 << "\nTime: " << pvt.hour << pvt.min << pvt.second
//                 << "\nFix Type: " << pvt.fix_type
//                 << "\nGNSS valid fix: " << pvt.gnss_fix_ok
//                 << "\nHeading valid: " << pvt.head_veh_valid
//                 << "\nNumber of satelites used: " << pvt.num_sv
//                 << "\nLongitude:" << pvt.lon << "Latitude:" << pvt.lat << "Height:" << pvt.height
//                 << "\nGround speed: " << pvt.g_speed << "m/s"
//                 << "\nHeading of motion: " << pvt.head_mot
//                 << "\nHeading of vehicle: " << pvt.head_veh
//                 << "\nDifferential corrections applied:" << pvt.diffsoln
//                 << "\nCarrier phase range solution status:" << pvt.carr_soln;
//    });

    // Print esf-meas
//    connect(&mUblox, &Ublox::rxEsfMeas, this, [](const ubx_esf_meas &meas){
//        static int count = 0;
//        if (count++%100)
//            return;
//        qDebug() << "ESF-MEAS data:"
//                 << "\ntimeTag: " << meas.time_tag
//                 << "\ntimeMarkSent: " << meas.time_mark_sent
//                 << "\ntimeMarkEdge: " << meas.time_mark_edge
//                 << "\ncalibTtagValid:" << meas.calib_t_tag_valid
//                 << "\nnumMeas:" << meas.num_meas
//                 << "\nid:" << meas.id;
//        for (int i = 0;i < meas.num_meas;i++) {
//            qDebug() << "\ndataType:" << meas.data_type[i]
//                     << "dataField:" << bin << meas.data_field[i];
//        }
//        if (meas.calib_t_tag_valid)
//            qDebug() << "\ncalibTtag:" << meas.calib_t_tag;
//    });

    // Print esf-status message
    // Search for sensor data type in following document for explanation:
    // https://www.u-blox.com/sites/default/files/ZED-F9R_Integrationmanual_UBX-20039643.pdf
//    connect(&mUblox, &Ublox::rxEsfStatus, this, [](const ubx_esf_status &status){
//        static int count = 0;
//        if (count++%5)
//            return;
//        qDebug() << "ESF-STATUS data:"
//                 << "\nVersion: " << status.version
//                 << "\nFusion mode: " << status.fusion_mode
//                 << "\nNumber of sensors: " << status.num_sens;
//                    ;
//        for (int i = 0;i < status.num_sens;i++) {
//            qDebug() << "Sensor data type: " << status.sensors[i].type
//                     << "\nSensor data used: " << status.sensors[i].used
//                     << "\nSensor data ready: " << status.sensors[i].ready
//                     << "\nSensor calibration status: " << status.sensors[i].calib_status
//                     << "\nSensor time status: " << status.sensors[i].time_status
//                     << "\nSensor observation freq: " << status.sensors[i].freq
//                     ;
//        }});

    // Print config recevied from valget
//    connect(&mUblox, &Ublox::rxCfgValget, this, [](const ubx_cfg_valget &valget){
//        qDebug() << "\n--- Polled UBX-CFG-VALGET ---"
//                 << "\nVersion:" << valget.version
//                 << "\nLayer:" << valget.layer
//                 << "\nPosition:" << valget.position;

//        for (int i=0; i<int(sizeof(valget.cfgData)/sizeof(*valget.cfgData)); i++) {
//            if (valget.key[i] != 0) {
//                qDebug() << "Key:" << hex << valget.key[i]
//                            << "Config:" << hex << valget.cfgData[i];
//            }
//        }
//        qDebug() << "--- End of poll ---\n";
//    });
}

bool UbloxRover::connectSerial(const QSerialPortInfo &serialPortInfo)
{
    if (mUblox.connectSerial(serialPortInfo)) {
        if (configureUblox())
            return true;
        else {
            mUblox.disconnectSerial();
            return false;
        }
    } else
        return false;
}

bool UbloxRover::isSerialConnected()
{
    return mUblox.isSerialConnected();
}

void UbloxRover::setEnuRef(llh_t enuRef)
{
    mEnuReference = enuRef;
    mEnuReferenceSet = true;
}

void UbloxRover::writeRtcmToUblox(QByteArray data)
{
    mUblox.writeRaw(data);
}

void UbloxRover::writeOdoToUblox(ubx_esf_datatype_enum dataType, uint32_t dataField)
{
    mUblox.ubloxOdometerInput(dataType, dataField);
}

void UbloxRover::setIMUOrientationOffset(double roll_deg, double pitch_deg, double yaw_deg)
{
    mIMUOrientationOffset = {roll_deg, pitch_deg, yaw_deg};
}

bool UbloxRover::configureUblox()
{
    // The u-blox receiver detects the previously stored data in the flash.
    // It restores the corresponding memory and reports the success of the operation
    mUblox.ubloxUpdSos(7);

    // The rate of NMEA and UBX protocol output messages are configurable
    // and it is possible to enable or disable single NMEA or UBX messages individually.
    // If the rate configuration value is zero, then the corresponding message will not be output.
    // Values greater than zero indicate how often the message is output.
    // TODO: convert to VALSET msg
    mUblox.ubxCfgMsg(UBX_CLASS_ESF, UBX_ESF_MEAS, 0);
    mUblox.ubxCfgMsg(UBX_CLASS_NAV, UBX_NAV_PVT, 1);
    mUblox.ubxCfgMsg(UBX_CLASS_ESF, UBX_ESF_STATUS, 1);
    mUblox.ubxCfgMsg(UBX_CLASS_ESF, UBX_ESF_ALG, 1);
    // make sure some messages are disabled
    mUblox.ubxCfgMsg(UBX_CLASS_NAV, UBX_NAV_SAT, 0);
    mUblox.ubxCfgMsg(UBX_CLASS_RXM, UBX_RXM_RAWX, 0);
    mUblox.ubxCfgMsg(UBX_CLASS_RXM, UBX_RXM_SFRBX, 0);
    mUblox.ubxCfgMsg(UBX_CLASS_NAV, UBX_NAV_SOL, 0);

    unsigned char buffer[512];
    int ind = 0;
    mUblox.ubloxCfgAppendMntalg(buffer, &ind, true); // enable auto mount alignment
    mUblox.ubloxCfgAppendRate(buffer, &ind, 100, 1, 0, 30); // nav prio mode
    if (!mUblox.ubloxCfgValset(buffer, ind, true, true, true)) { // try valset
        // setting auto mount alignment and nav prio failed -> this is F9P)
        bool result = true;
        result &= mUblox.ubxCfgRate(200, 1, 0);

        // Chip might have been used as base station, make sure to reconfigure.
        // TODO: this code is taken from "old sdvp" s.th. F9P behaves as we are used to,
        //       but needs to be revised, e.g., use VALSET msgs that work for F9R and F9P:
        //        ind = 0;
        //        mUblox.ubloxCfgAppendRate(buffer, &ind, 200, 1, 0);
        //        result &= mUblox.ubloxCfgValset(buffer, ind, true, true, true);

        // Disable RTCM output
        result &= mUblox.ubxCfgMsg(UBX_CLASS_RTCM3, UBX_RTCM3_1005, 0);
        result &= mUblox.ubxCfgMsg(UBX_CLASS_RTCM3, UBX_RTCM3_1074, 0);
        result &= mUblox.ubxCfgMsg(UBX_CLASS_RTCM3, UBX_RTCM3_1077, 0);
        result &= mUblox.ubxCfgMsg(UBX_CLASS_RTCM3, UBX_RTCM3_1084, 0);
        result &= mUblox.ubxCfgMsg(UBX_CLASS_RTCM3, UBX_RTCM3_1087, 0);
        result &= mUblox.ubxCfgMsg(UBX_CLASS_RTCM3, UBX_RTCM3_1094, 0);
        result &= mUblox.ubxCfgMsg(UBX_CLASS_RTCM3, UBX_RTCM3_1097, 0);
        result &= mUblox.ubxCfgMsg(UBX_CLASS_RTCM3, UBX_RTCM3_1124, 0);
        result &= mUblox.ubxCfgMsg(UBX_CLASS_RTCM3, UBX_RTCM3_1127, 0);
        result &= mUblox.ubxCfgMsg(UBX_CLASS_RTCM3, UBX_RTCM3_1230, 0);
        result &= mUblox.ubxCfgMsg(UBX_CLASS_RTCM3, UBX_RTCM3_4072_0, 0);
        result &= mUblox.ubxCfgMsg(UBX_CLASS_RTCM3, UBX_RTCM3_4072_1, 0);
        result &= mUblox.ubxCfgMsg(UBX_CLASS_NAV, UBX_NAV_SVIN, 0);

        // Enable NMEA GGA output (required by some NTRIP/RTCM servers)
        result &= mUblox.ubxCfgMsg(UBX_CLASS_NMEA, UBX_NMEA_GGA, 1);

        // Disable NMEA output (all but GGA)
        result &= mUblox.ubxCfgMsg(UBX_CLASS_NMEA, UBX_NMEA_GSV, 0);
        result &= mUblox.ubxCfgMsg(UBX_CLASS_NMEA, UBX_NMEA_GLL, 0);
        result &= mUblox.ubxCfgMsg(UBX_CLASS_NMEA, UBX_NMEA_GSA, 0);
        result &= mUblox.ubxCfgMsg(UBX_CLASS_NMEA, UBX_NMEA_RMC, 0);
        result &= mUblox.ubxCfgMsg(UBX_CLASS_NMEA, UBX_NMEA_VTG, 0);
        result &= mUblox.ubxCfgMsg(UBX_CLASS_NMEA, UBX_NMEA_GRS, 0);
        result &= mUblox.ubxCfgMsg(UBX_CLASS_NMEA, UBX_NMEA_GST, 0);
        result &= mUblox.ubxCfgMsg(UBX_CLASS_NMEA, UBX_NMEA_ZDA, 0);
        result &= mUblox.ubxCfgMsg(UBX_CLASS_NMEA, UBX_NMEA_GBS, 0);
        result &= mUblox.ubxCfgMsg(UBX_CLASS_NMEA, UBX_NMEA_DTM, 0);

        // Disable possible survey-in / fixed position
        ubx_cfg_tmode3 cfg_mode;
        memset(&cfg_mode, 0, sizeof(cfg_mode));
        cfg_mode.mode = 0;
        result &= mUblox.ubxCfgTmode3(&cfg_mode);

        // Stationary dynamic model
        ubx_cfg_nav5 nav5;
        memset(&nav5, 0, sizeof(ubx_cfg_nav5));
        nav5.apply_dyn = true;
        nav5.dyn_model = 4;
        result &= mUblox.ubxCfgNav5(&nav5);

        ind = 0;
        mUblox.ubloxCfgAppendEnableGps(buffer, &ind, true, true, true);
        mUblox.ubloxCfgAppendEnableGal(buffer, &ind, true, true, true);
        mUblox.ubloxCfgAppendEnableBds(buffer, &ind, true, true, true);
        mUblox.ubloxCfgAppendEnableGlo(buffer, &ind, true, true, true);
        result &= mUblox.ubloxCfgValset(buffer, ind, true, true, true);

        qDebug() << "UbloxRover: F9P configuration" << (result ? "was successful" : "reported an error");
    }

    return true;
}

void UbloxRover::updateGNSSPositionAndYaw(const ubx_nav_pvt &pvt)
{
    static bool initializationDone = false;
    static int leapSeconds_ms = 0;

    if (!initializationDone && pvt.valid_time) {
        leapSeconds_ms =  (pvt.i_tow % ms_per_day) - ((pvt.hour * 60 * 60 + pvt.min * 60 + pvt.second) * 1000);
        qDebug() << "UbloxRover: assuming" << leapSeconds_ms / 1000 << "seconds difference between GNSS time and UTC.";
        initializationDone = true;
    } else {
        PosPoint gnssPos = mVehicleState->getPosition(PosType::GNSS);

        llh_t llh = {pvt.lat, pvt.lon, pvt.height};
        xyz_t xyz = {0.0, 0.0, 0.0};

        if (!mEnuReferenceSet) {
            mEnuReference = llh;
            mEnuReferenceSet = true;
        } else
            xyz = coordinateTransforms::llhToEnu(mEnuReference, llh);

        // Position
        gnssPos.setXYZ(xyz);
        // Apply antenna offset to reference point (e.g., back axle) if set. Assumes fused yaw is updated.
        xyz_t gnssPositionOffset = mVehicleState->getGnssOffset();
        if (gnssPositionOffset.x != 0.0 || gnssPositionOffset.y != 0.0) {
            PosPoint fusedPos = mVehicleState->getPosition(PosType::fused);
            double fusedYaw_radENU = fusedPos.getYaw() * M_PI / 180.0;

            gnssPos.transform(-gnssPositionOffset, fusedYaw_radENU);
        }

        // Yaw --- based on last GNSS position if fusion (F9R) unavailable
        static xyz_t lastXyz;
        if(pvt.head_veh_valid) {
            double yaw_degENU = coordinateTransforms::yawNEDtoENU(pvt.head_veh) + mIMUOrientationOffset.yawOffset_deg;

            // normalize to [-180.0:180.0[
            while (yaw_degENU < -180.0)
                yaw_degENU += 360.0;
            while (yaw_degENU >= 180.0)
                yaw_degENU -= 360.0;

            gnssPos.setYaw(yaw_degENU);
        } else
            gnssPos.setYaw(atan2(xyz.y - lastXyz.y, xyz.x - lastXyz.x) * 180.0 / M_PI);

        // Time and speed
        gnssPos.setTime(QTime::fromMSecsSinceStartOfDay((pvt.i_tow % ms_per_day) - leapSeconds_ms));
//        qDebug() << "UbloxRover, gnssPos.getTime() - msSinceTodayUTC:" << QTime::currentTime().addSecs(-QDateTime::currentDateTime().offsetFromUtc()).msecsSinceStartOfDay() - gnssPos.getTime();
        gnssPos.setSpeed(pvt.g_speed);

        mVehicleState->setPosition(gnssPos);
        emit updatedGNSSPositionAndYaw(mVehicleState, QLineF(QPointF(lastXyz.x, lastXyz.y), gnssPos.getPoint()).length(), pvt.head_veh_valid);
        emit txNavPvt(pvt);


        lastXyz = xyz;
    }
}

llh_t UbloxRover::getEnuRef() const
{
    return mEnuReference;
}

void UbloxRover::updSosResponse(const ubx_upd_sos &sos)
{
    if(sos.cmd == 2){
        qDebug() << "\n--- Response from UBX-UPD-SOS ---"
                 << "\nBackup creation acknowledge"
                 << "\n0 = Not acknowledged"
                 << "\n1 = Acknowledged"
                 << "\nResponse:" << sos.response
                 << "\n--- End of response --\n";
    } else if(sos.cmd == 3) {
        qDebug() << "\n--- Response from UBX-UPD-SOS ---"
                 << "\nSystem restored from backup"
                 << "\n0 = Unknown"
                 << "\n1 = Failed restoring from backup"
                 << "\n2 = Restored from backup"
                 << "\n3 = Not restored (no backup)"
                 << "\nResponse:" << sos.response
                 << "\n--- End of response --\n";
        if(sos.response == 2) {
            // Once the u-blox receiver has started up it is recommended to delete the stored data
            mUblox.ubloxUpdSos(1);
        }
    }
}

void UbloxRover::saveOnShutdown()
{
    // With the UBX-CFG-RST message, the host commands the u-blox receiver to stop, specifying
    // a BBR mask 0 ("Hotstart") and a reset mode of 0x08 ("Controlled GNSS stop")
    mUblox.ubxCfgRst(8, 0);

    // The host commands the saving of the contents of BBR to the flash memory using the UBX-
    // UPD-SOS-BACKUP message
    mUblox.ubloxUpdSos(0);
}
