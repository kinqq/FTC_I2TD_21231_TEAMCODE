package org.firstinspires.ftc.teamcode;

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
    private final DcMotorEx leftRot, rightRot;

    public Elevator(HardwareMap hardwareMap) {
        leftEle = hardwareMap.get(DcMotorEx.class, "leftEle");
        rightEle = hardwareMap.get(DcMotorEx.class, "rightEle");
        leftRot = hardwareMap.get(DcMotorEx.class, "leftRot");
        rightRot = hardwareMap.get(DcMotorEx.class, "rightRot");

        leftEle.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightEle.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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
            if (!initialized) {
                leftRot.setTargetPosition(pos);
                rightRot.setTargetPosition(pos);

                leftRot.setPower(0.8);
                rightRot.setPower(0.8);

                leftRot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightRot.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                initialized = true;
            }

            double leftRotError = Math.abs(leftRot.getCurrentPosition() - leftRot.getTargetPosition());
            double rightRotError = Math.abs(rightRot.getCurrentPosition() - rightRot.getTargetPosition());

            return !(leftRotError < 10 || rightRotError < 10);
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
            if (!initialized) {
                leftEle.setTargetPosition(pos);
                rightEle.setTargetPosition(pos);

                leftEle.setPower(0.8);
                rightEle.setPower(0.8);

                leftEle.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightEle.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                initialized = true;
            }

            double leftEleError = Math.abs(leftEle.getCurrentPosition() - leftEle.getTargetPosition());
            double rightEleError = Math.abs(rightEle.getCurrentPosition() - rightEle.getTargetPosition());

            return !(leftEleError < 10 || rightEleError < 10);
        }
    }

    public void initRot() {
        leftRot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
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

    public Action rotateTo(int pos) {
        return new Rotate(pos);
    }

    public Action elevate(int pos) {
        return new Elevate(pos);
    }
}
