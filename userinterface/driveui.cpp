/*
 *     Copyright 2025 RISE Research Institutes of Sweden AB, Safety and Transport   waywise@ri.se
 *     Published under GPLv3: https://www.gnu.org/licenses/gpl-3.0.html
 */

#include "driveui.h"
#include "ui_driveui.h"
#include <QInputDialog>

DriveUI::DriveUI(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::DriveUI)
{
    memset(&mArrowKeyStates, 0, sizeof (mArrowKeyStates));
    memset(&mKeyControlState, 0, sizeof (mKeyControlState));

    ui->setupUi(this);

    grabKeyboard();
    connect(&mKeyControlTimer, &QTimer::timeout, [this](){
        if (mArrowKeyStates.upPressed)
            mKeyControlState.throttle += getMaxSignedStepFromValueTowardsGoal(mKeyControlState.throttle, 1.0, 0.03);
        else if (mArrowKeyStates.downPressed)
            mKeyControlState.throttle += getMaxSignedStepFromValueTowardsGoal(mKeyControlState.throttle, -1.0, 0.03);
        else
            mKeyControlState.throttle += getMaxSignedStepFromValueTowardsGoal(mKeyControlState.throttle, 0.0, 0.03);

        if (mArrowKeyStates.leftPressed)
            mKeyControlState.steering += getMaxSignedStepFromValueTowardsGoal(mKeyControlState.steering, -1.0, 0.08);
        else if (mArrowKeyStates.rightPressed)
            mKeyControlState.steering += getMaxSignedStepFromValueTowardsGoal(mKeyControlState.steering, 1.0, 0.08);
        else
            mKeyControlState.steering += getMaxSignedStepFromValueTowardsGoal(mKeyControlState.steering, 0.0, 0.08);

//        qDebug() << mArrowKeyStates.upPressed << mArrowKeyStates.downPressed << mArrowKeyStates.leftPressed << mArrowKeyStates.rightPressed;
//        qDebug() << mKeyControlState.throttle << mKeyControlState.steering;
        ui->throttleBar->setValue(mKeyControlState.throttle * 100);
        ui->steeringBar->setValue(mKeyControlState.steering * 100);

        if (mCurrentVehicleConnection && mCurrentVehicleConnection->getVehicleState()->getFlightMode() == VehicleState::FlightMode::Manual)
            mCurrentVehicleConnection->setManualControl(mKeyControlState.throttle, 0, 0, mKeyControlState.steering, 0);
    });
    mKeyControlTimer.start(40);
}

DriveUI::~DriveUI()
{
    delete ui;
}

void DriveUI::setCurrentVehicleConnection(const QSharedPointer<VehicleConnection> &currentVehicleConnection)
{
    mCurrentVehicleConnection = currentVehicleConnection;
}

void DriveUI::gotRouteForAutopilot(const QList<PosPoint> &route)
{
    if (mCurrentVehicleConnection.isNull())
        return;

    bool safetyCheckFailed = false;

    bool speedPositive = route.at(0).getSpeed() >= 0.0;
    for (int i = 0; i < route.length(); i++) {
        if ((route.at(i).getSpeed() >= 0.0) != speedPositive) {
            safetyCheckFailed = true;
            qDebug () << "Warning: Route contains nodes with positive and negative speeds. Either one or the other is allowed within a single route. Route is discarded.";
            break;
        }
    }

    if (safetyCheckFailed)
        mCurrentVehicleConnection->clearRoute();
    else
        mCurrentVehicleConnection->setRoute(route);
}

void DriveUI::on_apRestartButton_clicked()
{
    if (mCurrentVehicleConnection) {
        if (ui->apExecuteRouteRadioButton->isChecked())
            mCurrentVehicleConnection->restartAutopilot();
        else
            mCurrentVehicleConnection->requestFollowPoint();
    }
}

void DriveUI::on_apStartButton_clicked()
{
    if (mCurrentVehicleConnection) {
        if (ui->apExecuteRouteRadioButton->isChecked())
            mCurrentVehicleConnection->startAutopilot();
        else
            mCurrentVehicleConnection->requestFollowPoint();
    }
}

void DriveUI::on_apPauseButton_clicked()
{
    if (mCurrentVehicleConnection)
        mCurrentVehicleConnection->pauseAutopilot();
}

void DriveUI::on_apStopButton_clicked()
{
    if (mCurrentVehicleConnection) {
        if (ui->apExecuteRouteRadioButton->isChecked())
            mCurrentVehicleConnection->stopAutopilot();
        else
            mCurrentVehicleConnection->pauseAutopilot();
    }
}

void DriveUI::keyPressEvent(QKeyEvent *event)
{
    switch (event->key()) {
    case Qt::Key_Up:
        mArrowKeyStates.upPressed = true;
        break;
    case Qt::Key_Down:
        mArrowKeyStates.downPressed = true;
        break;
    case Qt::Key_Left:
        mArrowKeyStates.leftPressed = true;
        break;
    case Qt::Key_Right:
        mArrowKeyStates.rightPressed = true;
        break;
    default:
        break;
    }
}

void DriveUI::keyReleaseEvent(QKeyEvent *event)
{
    switch (event->key()) {
    case Qt::Key_Up:
        mArrowKeyStates.upPressed = false;
        break;
    case Qt::Key_Down:
        mArrowKeyStates.downPressed = false;
        break;
    case Qt::Key_Left:
        mArrowKeyStates.leftPressed = false;
        break;
    case Qt::Key_Right:
        mArrowKeyStates.rightPressed = false;
        break;
    default:
        break;
    }
}

double DriveUI::getMaxSignedStepFromValueTowardsGoal(double value, double goal, double maxStepSize) {
    maxStepSize = abs(maxStepSize);

    if ((value < goal) && (value + maxStepSize) < goal)
        return maxStepSize;

    if ((value > goal) && (value - maxStepSize) > goal)
        return -maxStepSize;

    return goal - value;
}

void DriveUI::on_apSetActiveIDButton_clicked()
{
    bool gotOK;
    int apID = QInputDialog::getInt(this, tr("Set Active Autopilot ID..."),
                                 tr("Autopilot ID vehicle should switch to:"), 0, 0, 9, 1, &gotOK);
    if (gotOK)
        if (mCurrentVehicleConnection) {
                if (mCurrentVehicleConnection->isAutopilotActive())
                    mCurrentVehicleConnection->pauseAutopilot();
                mCurrentVehicleConnection->setActiveAutopilotID(apID);
        }
}

void DriveUI::on_vehicleParameterButton_clicked()
{
    if (mVehicleParameterUI.isNull())
        mVehicleParameterUI = QSharedPointer<VehicleParameterUI>::create(this);
    mVehicleParameterUI->setCurrentVehicleConnection(mCurrentVehicleConnection);
    mVehicleParameterUI->show();
    this->releaseKeyboard();
}

void DriveUI::on_requestRebootButton_clicked()
{
    if (mCurrentVehicleConnection)
        mCurrentVehicleConnection->requestRebootOrShutdownOfSystemComponents(VehicleConnection::SystemComponent::OnboardComputer,VehicleConnection::ComponentAction::Reboot);
}

void DriveUI::on_requestShutdownButton_clicked()
{
    if (mCurrentVehicleConnection)
        mCurrentVehicleConnection->requestRebootOrShutdownOfSystemComponents(VehicleConnection::SystemComponent::OnboardComputer,VehicleConnection::ComponentAction::Shutdown);
}


void DriveUI::on_pollENUrefButton_clicked()
{
    if (mCurrentVehicleConnection)
        mCurrentVehicleConnection->pollCurrentENUreference();
}

void DriveUI::on_lowGearButton_clicked()
{
    const int gpioPin = ui->pinSpinBox->value();

    if (mCurrentVehicleConnection)
        mCurrentVehicleConnection->requestGearSwitch(gpioPin, VehicleConnection::Gearbox::lowGear);
}

void DriveUI::on_highGearButton_clicked()
{
    const int gpioPin = ui->pinSpinBox->value();

    if (mCurrentVehicleConnection)
        mCurrentVehicleConnection->requestGearSwitch(gpioPin, VehicleConnection::Gearbox::highGear);
}
