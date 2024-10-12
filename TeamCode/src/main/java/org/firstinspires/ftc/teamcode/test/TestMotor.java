package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Config
@TeleOp(name="TestMotor", group = "Test")
public class TestMotor extends LinearOpMode {
    public static String deviceName = "leftRot";

    @Override
    public void runOpMode() {
        waitForStart();

        DcMotor motor = hardwareMap.get(DcMotor.class, deviceName);
        motor.setDirection(DcMotorSimple.Direction.REVERSE);

        int pos = 0;
        double pow = 0.5;

        while (opModeIsActive()) {
            if (gamepad1.dpad_up) {
                pos += 1;
            }

            if (gamepad1.dpad_down) {
                pos -= 1;
            }

            if (gamepad1.a) {
                pow += 0.01;
            }

            if (gamepad1.b) {
                pow -= 0.01;
            }

            if (!gamepad1.x) {
                motor.setTargetPosition(pos);
                motor.setPower(pow);
                motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
        }
    }
}