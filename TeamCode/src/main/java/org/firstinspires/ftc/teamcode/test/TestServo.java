package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Test", group = "Test")
@Config
public class TestServo extends LinearOpMode {
    @Override
    public void runOpMode() {
        waitForStart();

        Servo test1 = hardwareMap.get(Servo.class,"grabber");

        double servoPosition = test1.getPosition();

        while (opModeIsActive()) {
            if (gamepad1.a) {
                servoPosition += 0.01;
            }

            if (gamepad1.b) {
                servoPosition -= 0.01;
            }

            if (gamepad1.x) {
                servoPosition += 0.005;
            }

            if (gamepad1.y) {
                servoPosition -= 0.005;
            }

            test1.setPosition(servoPosition);

            telemetry.addData("pos", servoPosition);
            telemetry.update();
        }

    }
}