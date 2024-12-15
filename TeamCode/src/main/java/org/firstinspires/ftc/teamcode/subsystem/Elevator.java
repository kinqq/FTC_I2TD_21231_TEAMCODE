package org.firstinspires.ftc.teamcode.subsystem;

import static org.firstinspires.ftc.teamcode.util.Constants.*;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
public class Elevator {
    private final DcMotorEx leftEle, rightEle;
    private final DcMotorEx leftRot;
    public boolean isRigging = false;
    private PIDController rotController;
    private PIDFController eleController;

    public static class rotPIDF {
        public static double pUp = 0.006, i = 0, d = 0, f = 0.10, pDown = 0.0015;
    }

    public static class elePIDF {
        public static double p = 0.006, i = 0, d = 0, f = 0.00012;
    }

    public Elevator(HardwareMap hardwareMap) {
        leftEle = hardwareMap.get(DcMotorEx.class, "leftEle");
        rightEle = hardwareMap.get(DcMotorEx.class, "rightEle");
        leftRot = hardwareMap.get(DcMotorEx.class, "leftRot");

        // Reset encoders to zero
//        leftEle.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        rightEle.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        leftRot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

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

    public class RotateUp implements Action {
        private boolean initialized = false;
        private int target;

        public RotateUp(int target) {
            this.target = target;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                leftRot.setPower(-0.8);
                leftRot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                initialized = true;
            }

            double pos = leftRot.getCurrentPosition();
            packet.put("liftPos", pos);
            if (pos - 10 > target) {
                return true;
            } else {
                leftRot.setPower(0);
                leftRot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                return false;
            }
        }
    }
    public Action rotateUp(int target) {
        return new RotateUp(target);
    }

    public class ElevateUp implements Action {
        private boolean initialized = false;
        private int target;

        public ElevateUp(int target) {
            this.target = target;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                leftEle.setPower(1);
                rightEle.setPower(1);
                leftEle.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                rightEle.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                initialized = true;
            }

            double pos = leftEle.getCurrentPosition();
            if (pos + 5 < target) {
                return true;
            } else {
                leftEle.setPower(0);
                rightEle.setPower(0);
                leftEle.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                rightEle.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                return false;
            }
        }
    }
    public Action elevateUp(int target) {
        return new ElevateUp(target);
    }

    public class ElevateDown implements Action {
        private boolean initialized = false;
        private int target;
        public ElevateDown(int target) {
            this.target = target;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                leftEle.setPower(-1);
                rightEle.setPower(-1);
                leftEle.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                rightEle.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                initialized = true;
            }

            double pos = leftEle.getCurrentPosition();
            if (pos - 10 > target) {
                return true;
            } else {
                leftEle.setPower(0);
                rightEle.setPower(0);
                leftEle.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                rightEle.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                return false;
            }
        }
    }
    public Action elevateDown(int target) {
        return new ElevateDown(target);
    }

    public class RotateDown implements Action {
        private boolean initialized = false;
        private int target;

        public RotateDown(int target) {
            this.target = target;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                leftRot.setPower(0.4);
                leftRot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                initialized = true;
            }

            double pos = leftRot.getCurrentPosition();
            packet.put("liftPos", pos);
            if (pos + 5 < target) {
                return true;
            } else {
                leftRot.setPower(0);
                return false;
            }
        }
    }
    public Action rotateDown(int target) {
        return new RotateDown(target);
    }

    public class Rotate implements Action {
        private boolean initialized = false;
        private final int pos;
        ElapsedTime runtime = new ElapsedTime();

        public Rotate(int _pos) {
            pos = _pos;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                runtime.reset();
                leftRot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                leftRot.setTargetPosition(pos);
                leftRot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                leftRot.setPower(0.7);
                initialized = true;
            }

            boolean atPosition = Math.abs(leftRot.getCurrentPosition() - pos) < 10;
            if (atPosition) {
                leftRot.setPower(0);
                leftRot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }
            return !atPosition && runtime.seconds() < 3;
        }
    }

    public class Elevate implements Action {
        private boolean initialized = false;
        private final int pos;
        ElapsedTime runtime = new ElapsedTime();

        public Elevate(int _pos) {
            pos = _pos;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                runtime.reset();
                initialized = true;
            }
            packet.put("eleTarget", pos);
            elevatePIDF(pos);

            boolean elevatorAtPosition = Math.abs(leftEle.getCurrentPosition() - pos) < 10;
            if (elevatorAtPosition) {
                leftEle.setPower(0);
                rightEle.setPower(0);
            }
            return !elevatorAtPosition && runtime.seconds() < 3;
        }
    }

    public void initRot() {
        leftRot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void initEle() {
        leftEle.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightEle.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
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

        leftEle.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightEle.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public Action rotate(int target) {
        return new Rotate(target);
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

        leftRot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void rigging() {
        leftEle.setPower(-1);
        rightEle.setPower(-1);

        leftEle.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightEle.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftRot.setPower(0);
        leftRot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        isRigging = true;
    }

    public void stopRigging() {
        leftEle.setPower(0);
        rightEle.setPower(0);

        leftEle.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightEle.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftEle.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightEle.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftRot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
}
