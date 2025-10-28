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
        motorPivotLeft.enableRawPositionControl(false);
        motorPivotLeft.enableSpeedControl(false);
        motorPivotLeft.setSpeed(std::ceil(speed*4095));
    }

    void setRawSpeedRightPivot(double speed) {
        driveMode = DriveMode::RAW_CONTROL;
        motorPivotRight.enableRawPositionControl(false);
        motorPivotRight.enableSpeedControl(false);
        motorPivotRight.setSpeed(std::ceil(speed*4095));
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
        motorPivotLeft.enableSpeedControl(false);
        motorPivotLeft.enableRawPositionControl(true);
        motorPivotLeft.setTargetPosition(positionTicks);
    }

    void setPositionRightPivot(long positionTicks) {
        driveMode = DriveMode::HEADING_CONTROL;
        motorPivotRight.enableSpeedControl(false);
        motorPivotRight.enableRawPositionControl(true);
        motorPivotRight.setTargetPosition(positionTicks);
    }
}