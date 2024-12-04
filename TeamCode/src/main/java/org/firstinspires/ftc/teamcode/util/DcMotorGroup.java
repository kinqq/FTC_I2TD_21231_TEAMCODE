package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class DcMotorGroup {
    private DcMotorEx[] motors;

    // Constructor using varargs to allow any number of motors
    public DcMotorGroup(DcMotorEx... motors) {
        this.motors = motors;
    }

    // Set zero power behavior for all motors
    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        for (DcMotorEx motor : motors) {
            motor.setZeroPowerBehavior(zeroPowerBehavior);
        }
    }

    // Set mode for all motors
    public void setMode(DcMotor.RunMode runMode) {
        for (DcMotorEx motor : motors) {
            motor.setMode(runMode);
        }
    }

    // Set target position for all motors
    public void setTargetPosition(int targetPosition) {
        for (DcMotorEx motor : motors) {
            motor.setTargetPosition(targetPosition);
        }
    }

    // Set power for all motors
    public void setPower(double power) {
        for (DcMotorEx motor : motors) {
            motor.setPower(power);
        }
    }

    // Set tolerance for target position for all motors
    public void setTolerance(int tolerance) {
        for (DcMotorEx motor : motors) {
            motor.setTargetPositionTolerance(tolerance);
        }
    }

    // Check if all motors are at their target positions
    public boolean atTargetPosition() {
        for (DcMotorEx motor : motors) {
            if (Math.abs(motor.getCurrentPosition() - motor.getTargetPosition()) >= motor.getTargetPositionTolerance()) {
                return false;
            }
        }
        return true;
    }
}
