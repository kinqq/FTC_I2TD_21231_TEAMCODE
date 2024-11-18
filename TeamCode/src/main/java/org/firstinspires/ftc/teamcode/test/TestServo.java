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

import org.firstinspires.ftc.robotcore.external.Const;
import org.firstinspires.ftc.teamcode.util.Constants;

@TeleOp(name = "TestServo", group = "Test")
@Config
public class TestServo extends OpMode {
    Servo servo;
    public static String deviceName = "pitch";
    double servoPos = 0;
    double GRABBER_CLOSE = 0.006;
    double GRABBER_OPEN  = 0.285;

    @Override
    public void init() {
        servo = hardwareMap.get(Servo.class, deviceName);
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

        if (gamepad2.dpad_up) {
            servoPos = Constants.PITCH_FORWARD;
        }
        if (gamepad2.dpad_down) {
            servoPos = Constants.PITCH_BACKWARD;
        }


        servo.setPosition(servoPos);

        telemetry.addData("servoPos", servoPos);
        telemetry.addData("deviceName", deviceName);

        telemetry.update();
    }

    @Override
    public void stop() {
        super.stop();
    }
}