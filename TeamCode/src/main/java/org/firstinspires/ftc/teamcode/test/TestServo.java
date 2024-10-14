package org.firstinspires.ftc.teamcode.test;

import static org.firstinspires.ftc.teamcode.teleop.RobotMap.*;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.ftc.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.SwitchableLight;

@TeleOp(name = "TestServo", group = "Test")
@Config
public class TestServo extends OpMode {
    Servo servo;
    double servoPos = 0;
    double GRABBER_CLOSE = 0.006;
    double GRABBER_OPEN  = 0.285;

    @Override
    public void init() {
        servo = hardwareMap.get(Servo.class, "grabber");
        servoPos = servo.getPosition();
    }

    @Override
    public void loop() {
        if (gamepad1.a) {
            servoPos += 0.001;
        }
        if (gamepad1.b) {
            servoPos -= 0.001;
        }
        if (gamepad1.dpad_up) {
            servoPos += 0.005;
        }
        if (gamepad1.dpad_down) {
            servoPos -= 0.005;
        }

        if (gamepad1.dpad_left) {
            servoPos = GRABBER_CLOSE;
        }
        if (gamepad1.dpad_right) {
            servoPos = GRABBER_OPEN;
        }

        if (gamepad1.x) {
            servo.setDirection(servo.getDirection() == Servo.Direction.FORWARD ? Servo.Direction.REVERSE : Servo.Direction.FORWARD);
        }
        servo.setPosition(servoPos);

        telemetry.addData("servoPos", servoPos);
    }

    @Override
    public void stop() {
        super.stop();
    }
}