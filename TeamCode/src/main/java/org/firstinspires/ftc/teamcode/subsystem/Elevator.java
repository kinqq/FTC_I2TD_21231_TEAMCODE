package org.firstinspires.ftc.teamcode.subsystem;

import static org.firstinspires.ftc.teamcode.util.Constants.*;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Elevator {
    private final DcMotorEx leftEle, rightEle;
    public final DcMotorEx leftRot;
    private boolean isRigging = false;

    public Elevator(HardwareMap hardwareMap) {
        leftEle = hardwareMap.get(DcMotorEx.class, "leftEle");
        rightEle = hardwareMap.get(DcMotorEx.class, "rightEle");
        leftRot = hardwareMap.get(DcMotorEx.class, "leftRot");

        leftEle.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightEle.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftEle.setPower(0);
        rightEle.setPower(0);
        leftRot.setPower(0);

        rightEle.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public class Rotate implements Action {
        private boolean initialized = false;
        private final int pos;

        public Rotate(int _pos) {
            pos = _pos;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            // Giving a generous value of 50 ticks error
            // Combining with initialize if statement will just delay and schedule the action
//            if (Math.abs(leftEle.getCurrentPosition() - ELE_BOT) > 50) {
//                packet.addLine("WARNING: Arm pivot was commanded to move while elevator was raised.");
//                return false;
//            }

            if (!initialized) {
                leftRot.setTargetPosition(pos);
                if (pos - leftRot.getCurrentPosition() > 0) {
                    leftRot.setPower(0.3);
                }
                else {
                    leftRot.setPower(0.5);
                }

                leftRot.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                initialized = true;
            }

            double leftRotError = Math.abs(leftRot.getCurrentPosition() - leftRot.getTargetPosition());
            if (leftRotError <= 10) leftRot.setPower(1);
            return !(leftRotError < 10);
        }
    }

    public class Elevate implements Action {
        private boolean initialized = false;
        private final int pos;

        public Elevate(int _pos) {
            pos = _pos;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (isRigging) return false;
            if (!initialized) {
                leftEle.setTargetPosition(pos);
                rightEle.setTargetPosition(pos);

                leftEle.setPower(1);
                rightEle.setPower(1);

                leftEle.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightEle.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                leftEle.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                rightEle.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

                initialized = true;
            }

            double leftEleError = Math.abs(leftEle.getCurrentPosition() - leftEle.getTargetPosition());
            double rightEleError = Math.abs(rightEle.getCurrentPosition() - rightEle.getTargetPosition());

            return !(leftEleError < 10 || rightEleError < 10);
        }
    }

    public void initRot() {
        leftRot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void initEle() {
        leftEle.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightEle.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public Action rotateUp() {
        return new Rotate(ROT_UP);
    }

    public Action rotateDown() {
        return new Rotate(ROT_DOWN);
    }

    public Action rotateGrab() {
        return new Rotate(ROT_GRAB);
    }

    public Action rotateTo(int pos) {
        return new Rotate(pos);
    }

    public int getPivotTarget() {
        return leftRot.getTargetPosition();
    }

    public Action elevate(int pos) {
        return new Elevate(pos);
    }

    public void rigging() {
        leftEle.setPower(-1);
        rightEle.setPower(-1);

        leftEle.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightEle.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        isRigging = true;
    }

    public void stopRigging() {
        leftEle.setPower(0);
        rightEle.setPower(0);

        leftEle.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightEle.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftEle.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightEle.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
}
