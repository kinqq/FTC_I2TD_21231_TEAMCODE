package org.firstinspires.ftc.teamcode.test;

import static org.firstinspires.ftc.teamcode.test.TestMap.backLeftMotor;
import static org.firstinspires.ftc.teamcode.test.TestMap.backRightMotor;
import static org.firstinspires.ftc.teamcode.test.TestMap.eleLeftMotor;
import static org.firstinspires.ftc.teamcode.test.TestMap.eleRightMotor;
import static org.firstinspires.ftc.teamcode.test.TestMap.frontLeftMotor;
import static org.firstinspires.ftc.teamcode.test.TestMap.frontRightMotor;
import static org.firstinspires.ftc.teamcode.test.TestMap.imu;
import static org.firstinspires.ftc.teamcode.test.TestMap.initTestRobot;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "TestDrive", group = "Test")
@Config
public class TestDrive extends OpMode {
    public static double rotOva = 1.06;
    public static double speed = 1;

    public static double SAFE_MODE = 0.5;
    public static double PRECISION_MODE = 0.25;
    public static double NORMAL_MODE = 1.0;

    Gamepad previousGamepad1 = new Gamepad();
    Gamepad previousGamepad2 = new Gamepad();

     int elePos = 0;
    int eleTargetPos = 0;

    @Override
    public void init() {
        //init hardware
        initTestRobot(hardwareMap);
        //update log
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {
        //main
        double y = gamepad1.left_stick_y; // Remember, Y stick value is reversed
        double x = -gamepad1.left_stick_x;
        double rx = -gamepad1.right_stick_x;

        if (gamepad2.a) {
            elePos = 0;
        }
        if (gamepad2.b) elePos = 1200;
        if (gamepad2.x) elePos = 2000;
        if (gamepad2.y) elePos = 2600;

        eleLeftMotor.setTargetPosition(elePos);
        eleLeftMotor.setPower(1);
        eleLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        eleRightMotor.setTargetPosition(elePos);
        eleRightMotor.setPower(1);
        eleRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    if (gamepad2.start) {
        eleLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        eleRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

        if (gamepad1.left_bumper && !previousGamepad1.left_bumper) {
            speed = speed != SAFE_MODE ? SAFE_MODE : NORMAL_MODE;
        }

        if (gamepad1.right_bumper && !previousGamepad1.right_bumper) {
            speed = speed != PRECISION_MODE ? PRECISION_MODE : NORMAL_MODE;
        }

        if (gamepad1.start) {
            imu.resetYaw();
        }

        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        rotX = rotX * rotOva;  // Counteract imperfect strafing

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

        telemetry.addLine("RENNIE!");

        telemetry.update();

        gamepad1.copy(previousGamepad1);
        gamepad2.copy(previousGamepad2);

    }

    @Override
    public void stop() {
        super.stop();
    }
}
