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
    public static int pos = 0, pos2 = 0;
    public static double pow = 0.5, pow2 = 0.5;
    public static boolean isReversed = false;

    @Override
    public void runOpMode() {
        waitForStart();

        DcMotor motor = hardwareMap.get(DcMotor.class, deviceName);
        DcMotor motor2 = hardwareMap.get(DcMotor.class, "leftEle");
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


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

            if (gamepad2.dpad_up) {
                pos2 += 5;
            }

            if (gamepad2.dpad_down) {
                pos2 -= 5;
            }

            if (gamepad2.a) {
                pow2 += 0.01;
            }

            if (gamepad2.b) {
                pow2 -= 0.01;
            }

            if (gamepad1.y) isReversed = !isReversed;
            if (isReversed) {
                motor.setDirection(DcMotorSimple.Direction.REVERSE);
            }
            else {
                motor.setDirection(DcMotorSimple.Direction.FORWARD);
            }

            if (!gamepad1.x) {
                motor.setTargetPosition(pos);
                motor.setPower(pow);
                motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            if (!gamepad2.x) {
                motor2.setTargetPosition(pos2);
                motor2.setPower(pow2);
                motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }

            telemetry.addData("motorName", deviceName);
            telemetry.addData("motorDir", motor.getDirection());
            telemetry.addData("targetPos", pos);
            telemetry.addData("currPos", motor.getCurrentPosition());
            telemetry.addData("targetPow", pow);


            telemetry.addData("targetPos2", pos2);
            telemetry.addData("currPos2", motor2.getCurrentPosition());
            telemetry.addData("targetPow2", pow2);
            telemetry.update();
        }
    }
}