package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
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
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        int pos = 0;
        double pow = 0.5;



        while (opModeIsActive()) {
            if (gamepad1.dpad_up) {
                pos += 5;
            }

            if (gamepad1.dpad_down) {
                pos -= 5;
            }

            if (gamepad1.a) {
                pow += 0.01;
            }

            if (gamepad1.b) {
                pow -= 0.01;
            }
            if (gamepad1.y) {
                motor.setDirection(DcMotorSimple.Direction.REVERSE);
            }

            if (!gamepad1.x) {
                motor.setTargetPosition(pos);
                motor.setPower(pow);
                motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }

            telemetry.addData("motorName", deviceName);
            telemetry.addData("motorDir", motor.getDirection());
            telemetry.addData("targetPos", pos);
            telemetry.addData("currPos", motor.getCurrentPosition());
            telemetry.addData("targetPow", pow);
            telemetry.update();
        }
    }
}