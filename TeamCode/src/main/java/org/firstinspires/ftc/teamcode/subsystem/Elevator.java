package org.firstinspires.ftc.teamcode.subsystem;

import static org.firstinspires.ftc.teamcode.util.Constants.*;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Elevator {
    private final DcMotorEx leftEle, rightEle;
    private final DcMotorEx leftRot;
    public boolean isRigging = false;
    private PIDController rotController;
    private PIDFController eleController;

    private static class rotPIDF {
        public static double pUp = 0.0025, i = 0, d = 0, f = 0.2, pDown = 0.001;
    }

    private static class elePIDF {
        public static double p = 0.005, i = 0, d = 0, f = 0.00004;
    }



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

        rotController = new PIDController(rotPIDF.pUp, rotPIDF.i, rotPIDF.d);
        eleController = new PIDFController(elePIDF.p, elePIDF.i, elePIDF.d, elePIDF.f);
    }

    public class Rotate implements Action {
        private boolean initialized = false;
        private final int pos;

        public Rotate(int _pos) {
            pos = _pos;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
//            if (!initialized) {
//                leftRot.setTargetPosition(pos);
//                if (pos - leftRot.getCurrentPosition() > 0) {
//                    leftRot.setPower(0.3);
//                }
//                else {
//                    leftRot.setPower(0.5);
//                }
//
//                leftRot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//                initialized = true;
//            }
            rotatePIDF(pos);

            return !rotationAtPosition();
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
//            if (!initialized) {
//                leftEle.setTargetPosition(pos);
//                rightEle.setTargetPosition(pos);
//
//                leftEle.setPower(1);
//                rightEle.setPower(1);
//
//                leftEle.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                rightEle.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//                leftEle.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//                rightEle.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//                initialized = true;
//            }
            elevatePIDF(pos);

            return !elevatorAtPosition();
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

    public Action elevate(int pos) {
        return new Elevate(pos);
    }

    public void elevatePIDF(int target) {
        int currentPosition = leftEle.getCurrentPosition();
        double pid = eleController.calculate(currentPosition, target);

        leftEle.setPower(pid);
        rightEle.setPower(pid);
    }

    public void rotatePIDF(int target) {
        int currentPosition = leftRot.getCurrentPosition();

        // When going down
        if (currentPosition - target < 0)
            rotController.setP(rotPIDF.pDown);
        else
            rotController.setP(rotPIDF.pUp);

        double pid = rotController.calculate(currentPosition, target);
        double TICKS_PER_DEG = 537.7 * 5 / 360.0;
        double ff = -Math.sin(Math.toRadians(currentPosition / TICKS_PER_DEG)) * rotPIDF.f;
        double power = pid + ff;

        leftRot.setPower(power);
    }

    public boolean elevatorAtPosition() {
        double leftEleError = Math.abs(leftEle.getCurrentPosition() - leftEle.getTargetPosition());
        double rightEleError = Math.abs(rightEle.getCurrentPosition() - rightEle.getTargetPosition());

        return leftEleError < 10 || rightEleError < 10;
    }

    public boolean rotationAtPosition() {
        double leftRotError = Math.abs(leftRot.getCurrentPosition() - leftRot.getTargetPosition());
        return leftRotError < 10;
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
