package org.firstinspires.ftc.teamcode.test;

import static org.firstinspires.ftc.teamcode.test.TestMap.*;
import static org.firstinspires.ftc.teamcode.util.Constants.*;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

@TeleOp(name = "TestDrive", group = "Test")
@Config
public class TestDrive extends OpMode {
    public static double speed = 1;

    Gamepad previousGamepad1 = new Gamepad();
    Gamepad previousGamepad2 = new Gamepad();

    int elePos = 0;
    int rotPos = 0;

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

        if (gamepad2.a) elePos = ELE_BOT;
        if (gamepad2.b) elePos = ELE_HIGH_RUNG;
        if (gamepad2.x) elePos = ELE_MID;
        if (gamepad2.y) elePos = ELE_HIGH;

        eleLeftMotor.setTargetPosition(elePos);
        eleLeftMotor.setPower(0.5);
        eleLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        eleRightMotor.setTargetPosition(elePos);
        eleRightMotor.setPower(0.5);
        eleRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        if (gamepad2.dpad_up) rotPos = 40;
        if (gamepad2.dpad_down) rotPos = 945; //오바해서 내려가지 않게

        rotLeftMotor.setTargetPosition(rotPos);
        rotLeftMotor.setPower(0.5);
        rotLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rotRightMotor.setTargetPosition(rotPos);
        rotRightMotor.setPower(0.5);
        rotRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        if (gamepad1.left_bumper && !previousGamepad1.left_bumper) {
            speed = speed != SAFE_MODE ? SAFE_MODE : NORMAL_MODE;
        }

        if (gamepad1.right_bumper && !previousGamepad1.right_bumper) {
            speed = speed != PRECISION_MODE ? PRECISION_MODE : NORMAL_MODE;
        }

        if (gamepad2.left_bumper) {
            grabber.setPosition(0.006);
        }

        if (gamepad2.right_bumper) {
            grabber.setPosition(0.285);
        }

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

        double frontLeftPower = (y + x + rx) / denominator * speed;
        double backLeftPower = (y - x + rx) / denominator * speed;
        double frontRightPower = (y - x - rx) / denominator * speed;
        double backRightPower = (y + x - rx) / denominator * speed;

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

        telemetry.addData("elePos", elePos);
        telemetry.addData("rotPos", rotPos);

        telemetry.addData("rotPoss", rotLeftMotor.getCurrentPosition());


        telemetry.update();

        gamepad1.copy(previousGamepad1);
        gamepad2.copy(previousGamepad2);
    }

    @Override
    public void stop() {
        super.stop();
    }
}
