package org.firstinspires.ftc.teamcode.teleop;

import static org.firstinspires.ftc.teamcode.teleop.RobotMap.*;
import static org.firstinspires.ftc.teamcode.util.Constants.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
//import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.util.GoBildaPinpointDriver;

@TeleOp(name = "DriveControl", group = "TeleOp")
@Config
public class DriveControl extends OpMode {
    public enum RotStatus {
        ROT_UP,
        ROT_DOWN,
        ROT_GRAB,
    }

    public static RotStatus rotStatus = RotStatus.ROT_UP;
    public static double rotFactor = 1.1;
    public static double speed = 1;

    public static double SAFE_MODE = 0.5;
    public static double PRECISION_MODE = 0.25;
    public static double NORMAL_MODE = 1.0;
    private final double ticks_in_degree = 537.7 * 5 / 360.0;
    final float[] hsvValues = new float[3];

    public static double p = 0.0015, i = 0, d = 0.00001, f = -0.10;

//    private PIDController controller;

    Gamepad previousGamepad1 = new Gamepad();
    Gamepad previousGamepad2 = new Gamepad();

    int elePos = ELE_BOT;
    int eleCorrection = 0;
    int rotPos = ROT_UP;
    double rotPow = 0;

    boolean isDroppingSpecimen = false;
    boolean isEleStalling = true;
    ElapsedTime rotTimer = new ElapsedTime();
    ElapsedTime speedTimer = new ElapsedTime();

    @Override
    public void init() {
        //init hardware
        initRobot(hardwareMap);

        odo.setOffsets(0, 0);
        odo.setEncoderResolution(org.firstinspires.ftc.teamcode.util.GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(org.firstinspires.ftc.teamcode.util.GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);

        odo.recalibrateIMU();
        odo.resetPosAndIMU();

//        controller = new PIDController(p, i, d);

        rotTimer.reset();
        speedTimer.reset();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        //update log
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void init_loop() {
        if (gamepad2.start) {
            eleLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            eleRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }

    @Override
    public void loop() {
        // Elevator
        if (gamepad2.a) elePos = ELE_BOT;
        if (gamepad2.b) elePos = ELE_HIGH_RUNG;
        if (gamepad2.x) elePos = ELE_MID;
        if (gamepad2.y) elePos = ELE_HIGH;

        // reset elevator position
        if (gamepad2.left_trigger > 0.5) {
            eleCorrection += 3;
        }
        if (gamepad2.right_trigger > 0.5) {
            eleCorrection -= 3;
        }

        if (gamepad2.start) {
            eleLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            eleRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        eleLeftMotor.setPower(1);
        eleRightMotor.setPower(1);

        eleLeftMotor.setTargetPosition(elePos + eleCorrection);
        eleLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        eleRightMotor.setTargetPosition(elePos + eleCorrection);
        eleRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

//      TODO: Check if power direction is correct.
        if (gamepad1.left_bumper && gamepad1.dpad_down) {
            rotLeftMotor.setPower(0);
            rotRightMotor.setPower(0);

            eleLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            eleRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            eleLeftMotor.setPower(-1);
            eleRightMotor.setPower(-1);
        }

//      Elevator rotation
        if (gamepad2.dpad_up && !previousGamepad2.dpad_up) {
            if (rotStatus == RotStatus.ROT_GRAB) {
                rotStatus = RotStatus.ROT_DOWN;
                rotPos = Constants.ROT_DOWN;
                rotPow = 0.5;
            } else if (rotStatus == RotStatus.ROT_DOWN) {
                rotStatus = RotStatus.ROT_UP;
                rotPos = Constants.ROT_UP;
                rotPow = 0.5;
            }
            rotTimer.reset();
        }
        if (gamepad2.dpad_down && !previousGamepad2.dpad_down) {
            if (rotStatus == RotStatus.ROT_UP) {
                rotStatus = RotStatus.ROT_DOWN;
                rotPos = Constants.ROT_DOWN;
                rotPow = 0.3;
            } else if (rotStatus == RotStatus.ROT_DOWN) {
                rotStatus = RotStatus.ROT_GRAB;
                rotPos = Constants.ROT_GRAB;
                rotPow = 0.1;
            }
            rotTimer.reset();
        }

        if ((rotStatus == RotStatus.ROT_UP && elePos != ELE_BOT)
                || (rotStatus == RotStatus.ROT_UP && Math.abs(rotLeftMotor.getCurrentPosition() - rotPos) < 50)
                || (rotStatus == RotStatus.ROT_GRAB && Math.abs(rotLeftMotor.getCurrentPosition() - rotPos) < 50)
        ) {
            rotLeftMotor.setPower(0);
            rotRightMotor.setPower(0);
            telemetry.addData("rotation", "Phew...");
        } else {
            rotLeftMotor.setTargetPosition(rotPos);
            rotRightMotor.setTargetPosition(rotPos);

            rotLeftMotor.setPower(rotPow);
            rotRightMotor.setPower(rotPow);

            rotLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rotRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            telemetry.addData("rotation", "Working...");
        }

//      Grabber servo
        if (gamepad2.left_bumper) {
            grabber.setPosition(GRABBER_CLOSE);
        }

        if (gamepad2.right_bumper) {
            grabber.setPosition(GRABBER_OPEN);
        }

//      Specimen hang process
//      When press left stick "DOWN"
        if (gamepad2.left_stick_y > 0.9 && !isDroppingSpecimen && !(previousGamepad2.left_stick_y > 0.9)) {
//            isDroppingSpecimen = true;
            elePos = ELE_DROP_SPECIMEN;
        }

//        if (isDroppingSpecimen && Math.abs(eleLeftMotor.getCurrentPosition() - (elePos + eleCorrection)) < 10) {
//            grabber.setPosition(GRABBER_OPEN);
//            isDroppingSpecimen = false;
//        }

//      Color sensor and LED
        assert colorSensor != null;

        colorSensor.setGain(10.0F);
        NormalizedRGBA colors = colorSensor.getNormalizedColors();

//        Color.colorToHSV(colors.toColor(), hsvValues);
//        HSV hsv = new HSV(hsvValues[0], hsvValues[1], hsvValues[2]);
//
//        if (hsv.getColorCategory().equals("Red"))
//            led.setRed();
//        if (hsv.getColorCategory().equals("Blue"))
//            led.setBlue();
//        if (hsv.getColorCategory().equals("Yellow"))
//            led.setYellow();
//        if (hsv.getColorCategory().equals("N/A"))
//            led.turnOff();
        led.setRainbow();

//      Drivetrain speed
        if (gamepad1.left_bumper && !previousGamepad1.left_bumper) {
            speed = speed != SAFE_MODE ? SAFE_MODE : NORMAL_MODE;
            speedTimer.reset();
        }

        if (gamepad1.right_bumper && !previousGamepad1.right_bumper) {
            speed = speed != PRECISION_MODE ? PRECISION_MODE : NORMAL_MODE;
            speedTimer.reset();
        }

//      IMU reset
        if (gamepad1.start) {
            odo.resetPosAndIMU();
        }

//      Drivetrain
        double y = gamepad1.left_stick_y; // Remember, Y stick value is reversed
        double x = -gamepad1.left_stick_x;
        double rx = -gamepad1.right_stick_x;

//      double botHeading = odo.getPosition().getHeading(AngleUnit.RADIANS);
        double botHeading = 0;

        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        rotX = rotX * rotFactor;  // Counteract imperfect strafing

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);

        double frontLeftPower = (rotY + rotX + rx) / denominator * speed;
        double backLeftPower = (rotY - rotX + rx) / denominator * speed;
        double frontRightPower = (rotY - rotX - rx) / denominator * speed;
        double backRightPower = (rotY + rotX - rx) / denominator * speed;

        frontLeftMotor.setPower(frontLeftPower);
        backLeftMotor.setPower(backLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backRightMotor.setPower(backRightPower);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("elePos", elePos + eleCorrection);
        telemetry.addData("actualElePos", eleLeftMotor.getCurrentPosition());
        telemetry.addData("eleTol", eleLeftMotor.getTargetPositionTolerance());

        telemetry.addData("fr", frontLeftPower);
        telemetry.addData("bl", backLeftPower);
        telemetry.addData("fr", frontRightPower);
        telemetry.addData("br", backRightPower);

        telemetry.addData("botHeading", botHeading);
        telemetry.addData("rot", rotStatus);
        telemetry.addData("eleStalling", isEleStalling);

        telemetry.update();

        previousGamepad1.copy(gamepad1);
        previousGamepad2.copy(gamepad2);
    }

    @Override
    public void stop() {
        super.stop();
    }
}