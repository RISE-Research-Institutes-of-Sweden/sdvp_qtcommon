/*
 *     Copyright 2023 RISE Research Institutes of Sweden AB, Safety and Transport   waywise@ri.se
 *     Published under GPLv3: https://www.gnu.org/licenses/gpl-3.0.html
 */

#ifndef VEHICLEPARAMETERUI_H
#define VEHICLEPARAMETERUI_H

#include <QWidget>
#include <QDialog>
#include <QTableWidget>
#include "communication/vehicleconnections/vehicleconnection.h"

namespace Ui {
class VehicleParameterUI;
}

class VehicleParameterUI : public QDialog
{
    Q_OBJECT

public:
    explicit VehicleParameterUI(QWidget *parent = nullptr);
    ~VehicleParameterUI();

    void setCurrentVehicleConnection(const QSharedPointer<VehicleConnection> &currentVehicleConnection);

private slots:
    void on_getAllParametersFromVehicleButton_clicked();
    void on_setNewParametersOnVehicleButton_clicked();

private:
    void populateTableWithParameters();
    bool updateChangedParameters();

    Ui::VehicleParameterUI *ui;
    QSharedPointer<VehicleConnection> mCurrentVehicleConnection;
    ParameterServer::AllParameters mVehicleParameters;
    ParameterServer::AllParameters mControlTowerParameters;
};

#endif // VEHICLEPARAMETERUI_H
