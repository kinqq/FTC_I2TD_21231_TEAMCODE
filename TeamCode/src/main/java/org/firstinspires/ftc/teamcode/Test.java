package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name="Test", group = "Test")
public class Test extends LinearOpMode {
    @Override
    public void runOpMode() {
        waitForStart();

        DcMotor test1 = hardwareMap.get(DcMotor.class,"fr");
        while (opModeIsActive()) {
            if (gamepad1.a) {
                test1.setPower(1);
                test1.setDirection(DcMotorSimple.Direction.REVERSE);
            }

            if (gamepad1.b) {
                test1.setPower(0);
            }

            if (gamepad1.x) {
                test1.setPower(0.5);
            }

            if (gamepad1.y) {
                test1.setPower(0.2);
            }
        }

    }
}