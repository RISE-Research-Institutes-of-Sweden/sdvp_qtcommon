/*
 *     Copyright 2024 RISE Research Institutes of Sweden AB, Safety and Transport   waywise@ri.se
 *     Published under GPLv3: https://www.gnu.org/licenses/gpl-3.0.html
 */

#ifndef DRIVEUI_H
#define DRIVEUI_H

#include <QWidget>
#include <QKeyEvent>
#include <QTimer>
#include <QTableWidget>
#include "communication/vehicleconnections/vehicleconnection.h"
#include "userinterface/vehicleparameterui.h"

namespace Ui {
class DriveUI;
}

class DriveUI : public QWidget
{
    Q_OBJECT

public:
    explicit DriveUI(QWidget *parent = nullptr);
    ~DriveUI();

    void setCurrentVehicleConnection(const QSharedPointer<VehicleConnection> &currentVehicleConnection);
    void gotRouteForAutopilot(const QList<PosPoint>& route);

private slots:
    void on_apRestartButton_clicked();

    void on_apStartButton_clicked();

    void on_apPauseButton_clicked();

    void on_apStopButton_clicked();

    void on_apSetActiveIDButton_clicked();

    void on_vehicleParameterButton_clicked();

    void on_requestRebootButton_clicked();

    void on_requestShutdownButton_clicked();

    void on_pollENUrefButton_clicked();

    void on_lowGearButton_clicked();

    void on_highGearButton_clicked();

private:
    Ui::DriveUI *ui;

    QSharedPointer<VehicleParameterUI> mVehicleParameterUI;
    QSharedPointer<VehicleConnection> mCurrentVehicleConnection;
    struct {bool upPressed, downPressed, leftPressed, rightPressed;} mArrowKeyStates;
    struct {double throttle, steering;} mKeyControlState;
    QTimer mKeyControlTimer;

    double getMaxSignedStepFromValueTowardsGoal(double value, double goal, double maxStepSize);

    // QWidget interface
protected:
    virtual void keyPressEvent(QKeyEvent *event) override;
    virtual void keyReleaseEvent(QKeyEvent *event) override;
};

#endif // DRIVEUI_H
