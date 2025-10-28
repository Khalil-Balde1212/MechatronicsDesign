#include <drivebase/Drivebase.h>


namespace DriveBase
{
    void setRawSpeedLeft(double speed) {
        driveMode = DriveMode::RAW_CONTROL;
        motorLeft.enableRawPositionControl(false);
        motorLeft.enableSpeedControl(false);
        motorLeft.setSpeed(std::ceil(speed*4095));
    }

    void setRawSpeedRight(double speed) {
        driveMode = DriveMode::RAW_CONTROL;
        motorRight.enableRawPositionControl(false);
        motorRight.enableSpeedControl(false);
        motorRight.setSpeed(std::ceil(speed*4095));
    }

    void setRawSpeedLeftPivot(double speed) {
        driveMode = DriveMode::RAW_CONTROL;
        motorPivotFront.enableRawPositionControl(false);
        motorPivotFront.enableSpeedControl(false);
        motorPivotFront.setSpeed(std::ceil(speed*4095));
    }

    void setRawSpeedRightPivot(double speed) {
        driveMode = DriveMode::RAW_CONTROL;
        motorPivotRear.enableRawPositionControl(false);
        motorPivotRear.enableSpeedControl(false);
        motorPivotRear.setSpeed(std::ceil(speed*4095));
    }





    void setPositionRight(long positionTicks) {
        driveMode = DriveMode::HEADING_CONTROL;
        motorRight.enableSpeedControl(false);
        motorRight.enableRawPositionControl(true);
        motorRight.setTargetPosition(positionTicks);
    }

    void setPositionLeft(long positionTicks) {
        driveMode = DriveMode::HEADING_CONTROL;
        motorLeft.enableSpeedControl(false);
        motorLeft.enableRawPositionControl(true);
        motorLeft.setTargetPosition(positionTicks);
    }

    void setPositionLeftPivot(long positionTicks) {
        driveMode = DriveMode::HEADING_CONTROL;
        motorPivotFront.enableSpeedControl(false);
        motorPivotFront.enableRawPositionControl(true);
        motorPivotFront.setTargetPosition(positionTicks);
    }

    void setPositionRightPivot(long positionTicks) {
        driveMode = DriveMode::HEADING_CONTROL;
        motorPivotRear.enableSpeedControl(false);
        motorPivotRear.enableRawPositionControl(true);
        motorPivotRear.setTargetPosition(positionTicks);
    }
}