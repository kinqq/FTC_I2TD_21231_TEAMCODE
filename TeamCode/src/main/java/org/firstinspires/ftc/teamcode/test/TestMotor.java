package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name="TestMotor", group = "Test")
public class TestMotor extends LinearOpMode {
    @Override
    public void runOpMode() {
        waitForStart();

        DcMotor t0 = hardwareMap.get(DcMotor.class,"t0");
        DcMotor t1 = hardwareMap.get(DcMotor.class,"t1");
        DcMotor t2 = hardwareMap.get(DcMotor.class,"t2");
        DcMotor t3 = hardwareMap.get(DcMotor.class,"t3");

        DcMotor motor = t0;
        int pos = 0;

        while (opModeIsActive()) {
            if (gamepad1.a) {
                motor = t0;
                motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }
            if (gamepad1.b) {
                motor = t1;
                motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }
            if (gamepad1.x) {
                motor = t2;
                motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }
            if (gamepad1.y) {
                motor = t3;
                motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }

            if (gamepad1.dpad_up) {
                pos = 200;
            }

            if (gamepad1.dpad_down) {
                pos = -200;
            }

            if (gamepad1.dpad_left) {
                pos = 800;
            }

            if (gamepad1.dpad_right) {
                pos = -800;
            }

            motor.setTargetPosition(pos);
            motor.setPower(1);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

    }
}