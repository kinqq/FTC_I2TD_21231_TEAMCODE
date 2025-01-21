package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.util.Constants;

@TeleOp(name = "TestServo", group = "Test")
@Config
public class TestServo extends OpMode {
    ServoImplEx servo;
    public static String deviceName = "grabber";
    public static int lowPwm = 600, highPwm = 2400;
    public static double servoPos = 0;

    @Override
    public void init() {
        servo = (ServoImplEx) hardwareMap.get(Servo.class, deviceName);
        servo.setPwmRange(new PwmControl.PwmRange(lowPwm, highPwm));
        servoPos = servo.getPosition();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
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

        if (gamepad1.x) {
            servo.setDirection(servo.getDirection() == Servo.Direction.FORWARD ? Servo.Direction.REVERSE : Servo.Direction.FORWARD);
        }

        servo.setPosition(servoPos);

        telemetry.addData("servoPos", servoPos);
        telemetry.addData("servoDir", servo.getDirection());
        telemetry.addData("deviceName", deviceName);

        telemetry.update();
    }

    @Override
    public void stop() {
        super.stop();
    }
}