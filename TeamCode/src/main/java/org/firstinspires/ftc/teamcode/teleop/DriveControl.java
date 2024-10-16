package org.firstinspires.ftc.teamcode.teleop;

import static org.firstinspires.ftc.teamcode.teleop.RobotMap.*;
import static org.firstinspires.ftc.teamcode.util.Constants.*;

import android.graphics.Color;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.util.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.util.HSV;

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

    private PIDController controller;

    Gamepad previousGamepad1 = new Gamepad();
    Gamepad previousGamepad2 = new Gamepad();

    int elePos = 0;
    int eleCorrection = 0;
    int rotPos = ROT_UP;
    double rotPow = 0;

    boolean isDroppingSpecimen = false;
    ElapsedTime time = new ElapsedTime();

    @Override
    public void init() {
        //init hardware
        initRobot(hardwareMap);

        odo.setOffsets(0, 0);
        odo.setEncoderResolution(org.firstinspires.ftc.teamcode.util.GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(org.firstinspires.ftc.teamcode.util.GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);

        odo.recalibrateIMU();
        odo.resetPosAndIMU();

        controller = new PIDController(p, i, d);

        time.reset();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        //update log
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {
        odo.update();

        // Elevator
        if (gamepad2.a) elePos = ELE_BOT;
        if (gamepad2.b) elePos = ELE_LOW;
        if (gamepad2.x) elePos = ELE_MID;
        if (gamepad2.y) elePos = ELE_HIGH;

        // reset elevator position
        if (gamepad2.back) {
            if (gamepad2.a) {
                eleCorrection += 1;
            }
            if (gamepad2.b) {
                eleCorrection -= 1;
            }
        }

        if (gamepad2.start) {
            eleLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            eleRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        eleLeftMotor.setTargetPosition(elePos + eleCorrection);
        eleLeftMotor.setPower(1);
        eleLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        eleRightMotor.setTargetPosition(elePos + eleCorrection);
        eleRightMotor.setPower(1);
        eleRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

//      TODO: Check if power direction is correct.
        if (gamepad1.left_bumper && gamepad1.dpad_down) {
            eleLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            eleRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            eleLeftMotor.setPower(-1);
            eleRightMotor.setPower(-1);
        }

//      Elevator rotation
        if (gamepad2.dpad_up && time.seconds() > 0.5) {
            if (rotStatus == RotStatus.ROT_GRAB) {
                rotStatus = RotStatus.ROT_DOWN;
                rotPos = Constants.ROT_DOWN;
                rotPow = 0.4;
            }
            else if (rotStatus == RotStatus.ROT_DOWN) {
                rotStatus = RotStatus.ROT_UP;
                rotPos = Constants.ROT_UP;
                rotPow = 0.4;
            }
            time.reset();
        }
        if (gamepad2.dpad_down && time.seconds() > 0.5) {
            if (rotStatus == RotStatus.ROT_UP) {
                rotStatus = RotStatus.ROT_DOWN;
                rotPos = Constants.ROT_DOWN;
                rotPow = 0.2;
            }
            else if (rotStatus == RotStatus.ROT_DOWN) {
                rotStatus = RotStatus.ROT_GRAB;
                rotPos = Constants.ROT_GRAB;
                rotPow = 0.1;
            }
            time.reset();
        }

//        controller.setPID(p, i, d);
//        int arm_pos = rotLeftMotor.getCurrentPosition();
//        double pid = controller.calculate(arm_pos, rotPos);
//        double ff = Math.sin(Math.toRadians((300 + arm_pos) / ticks_in_degree)) * f;
//        double power = pid + ff;
        rotLeftMotor.setTargetPosition(rotPos);
        rotRightMotor.setTargetPosition(rotPos);

        rotLeftMotor.setPower(rotPow);
        rotRightMotor.setPower(rotPow);

        rotLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rotRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

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
            isDroppingSpecimen = true;
            elePos = ELE_DROP_SPECIMEN;
        }

        if (isDroppingSpecimen && Math.abs(eleLeftMotor.getCurrentPosition() - (elePos + eleCorrection)) < 10) {
            grabber.setPosition(GRABBER_OPEN);
            isDroppingSpecimen = false;
        }

//      Color sensor and LED
        assert colorSensor != null;

        colorSensor.setGain(10.0F);
        NormalizedRGBA colors = colorSensor.getNormalizedColors();

        Color.colorToHSV(colors.toColor(), hsvValues);
        HSV hsv = new HSV(hsvValues[0], hsvValues[1], hsvValues[2]);

        if (hsv.getColorCategory().equals("Red"))
            led.setRed();
        if (hsv.getColorCategory().equals("Blue"))
            led.setBlue();
        if (hsv.getColorCategory().equals("Yellow"))
            led.setYellow();
        if (hsv.getColorCategory().equals("N/A"))
            led.turnOff();

//      Drivetrain speed
        if (gamepad1.left_bumper && !previousGamepad1.left_bumper) {
            speed = speed != SAFE_MODE ? SAFE_MODE : NORMAL_MODE;
        }

        if (gamepad1.right_bumper && !previousGamepad1.right_bumper) {
            speed = speed != PRECISION_MODE ? PRECISION_MODE : NORMAL_MODE;
        }

//      IMU reset
        if (gamepad1.start) {
            odo.resetPosAndIMU();
        }

//      Drivetrain
        double y = gamepad1.left_stick_y; // Remember, Y stick value is reversed
        double x = -gamepad1.left_stick_x;
        double rx = -gamepad1.right_stick_x;

        double botHeading = odo.getPosition().getHeading(AngleUnit.RADIANS);

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

        telemetry.addData("fr", frontLeftPower);
        telemetry.addData("bl", backLeftPower);
        telemetry.addData("fr", frontRightPower);
        telemetry.addData("br", backRightPower);

        telemetry.addData("botHeading", botHeading);

        telemetry.addData("rot", rotStatus);

        telemetry.update();

        gamepad1.copy(previousGamepad1);
        gamepad2.copy(previousGamepad2);
    }

    @Override
    public void stop() {
        super.stop();
    }
}