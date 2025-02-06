package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

@TeleOp(name = "TestServo", group = "Test")
@Config
public class TestServo extends OpMode {
    ServoImplEx servo1, servo2;
    DcMotor motor1, motor2;
    public static String servoName1 = "pitch";
    public static String servoName2 = "pivot";
    public static String motorName1 = "leftRot";
    public static String motorName2 = "leftEle";
    public static int lowPwm = 600, highPwm = 2400;
    public static double servoPos1 = 0, servoPos2 = 0, motorPow1 = 1, motorPow2 = 1;
    public static int motorPos1 = 0, motorPos2;


    @Override
    public void init() {
        servo1 = (ServoImplEx) hardwareMap.get(Servo.class, servoName1);
        servo2 = (ServoImplEx) hardwareMap.get(Servo.class, servoName2);
        servo1.setPwmRange(new PwmControl.PwmRange(lowPwm, highPwm));
        servo2.setPwmRange(new PwmControl.PwmRange(lowPwm, highPwm));
        servoPos1 = servo1.getPosition();
        servoPos2 = servo2.getPosition();

        motor1 = hardwareMap.get(DcMotor.class, motorName1);
        motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor2 = hardwareMap.get(DcMotor.class, motorName2);
        motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void loop() {
        if (gamepad1.a) {
            servoPos1 += 0.001;
        }
        if (gamepad1.b) {
            servoPos1 -= 0.001;
        }
        if (gamepad1.dpad_up) {
            servoPos1 += 0.005;
        }
        if (gamepad1.dpad_down) {
            servoPos1 -= 0.005;
        }

        if (gamepad2.a) {
            servoPos2 += 0.001;
        }
        if (gamepad2.b) {
            servoPos2 -= 0.001;
        }
        if (gamepad2.dpad_up) {
            servoPos2 += 0.005;
        }
        if (gamepad2.dpad_down) {
            servoPos2 -= 0.005;
        }

        servo1.setPosition(servoPos1);
        servo2.setPosition(servoPos2);
        motor1.setTargetPosition(motorPos1);
        motor1.setPower(motorPow1);
        motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor2.setTargetPosition(motorPos2);
        motor2.setPower(motorPow2);
        motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        telemetry.addData("servoPos1", servoPos1);
        telemetry.addData("servoPos2", servoPos2);
        telemetry.addData("servoName1", servoName1);
        telemetry.addData("servoName2", servoName2);
        telemetry.addData("motorPos1", motor1.getCurrentPosition());
        telemetry.addData("motorPos2", motor2.getCurrentPosition());

        telemetry.update();
    }

    @Override
    public void stop() {
        super.stop();
    }
}