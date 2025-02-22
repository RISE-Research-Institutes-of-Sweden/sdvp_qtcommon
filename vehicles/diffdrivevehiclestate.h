/*
 *     Copyright 2021 RISE Research Institutes of Sweden AB, Safety and Transport   waywise@ri.se
 *     Published under GPLv3: https://www.gnu.org/licenses/gpl-3.0.html
 *
 * Specific implementation of VehicleState for differential vehicles, storing all (dynamic and static) state
 */

#ifndef DIFFDRIVEVEHICLESTATE_H
#define DIFFDRIVEVEHICLESTATE_H

#include "vehicles/vehiclestate.h"

class DiffDriveVehicleState : public VehicleState
{
public:
    DiffDriveVehicleState();

    virtual void setSteering(double steering) override;
    double getSpeed() const override;
    virtual void setSpeed(double speed) override;

    virtual void updateOdomPositionAndYaw(double drivenDistance, PosType usePosType = PosType::odom) override;
    virtual double steeringCurvatureToSteering(double steeringCurvature) override;

    double getSpeedLeft() const;
    void setSpeedLeft(double getSpeedLeft);

    double getSpeedRight() const;
    void setSpeedRight(double getSpeedRight);


private:
    double mSpeedLeft;
    double mSpeedRight;
};

#endif // DIFFDRIVEVEHICLESTATE_H
