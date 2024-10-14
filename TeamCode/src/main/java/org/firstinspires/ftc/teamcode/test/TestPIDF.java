package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Config
@TeleOp(name = "TestPIDF", group = "Test")
public class TestPIDF extends OpMode {
    private PIDController controller, controller2;
    public static double p = 0, i = 0, d = 0, f = 0;
    public static double p2 = 0, i2 = 0, d2 = 0, f2 = 0;


    public static int target = 100;

    private final double ticks_in_degree = 537.7 / 360.0;

    private DcMotorEx arm_motor1, arm_motor2;

    @Override
    public void init() {
        controller = new PIDController(p, i, d);
        controller2 = new PIDController(p2, i2, d2);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        arm_motor1 = hardwareMap.get(DcMotorEx.class, "leftRot");
        arm_motor2 = hardwareMap.get(DcMotorEx.class, "rightRot");

        if (gamepad1.start) {
            arm_motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            arm_motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            arm_motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            arm_motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    @Override
    public void loop() {
        controller.setPID(p, i, d);
        int arm_pos = arm_motor1.getCurrentPosition();
        double pid = controller.calculate(arm_pos, target);
        double ff = Math.cos(Math.toRadians(target / ticks_in_degree)) * f;
        double power = pid + ff;
        arm_motor1.setPower(power);

        controller2.setPID(p, i, d);
        int arm_pos2 = -arm_motor1.getCurrentPosition();
        double pid2 = controller2.calculate(arm_pos2, -target);
        double ff2 = Math.cos(Math.toRadians(-target / ticks_in_degree)) * f;
        double power2 = pid2 + ff2;
        arm_motor2.setPower(power2);

        telemetry.addData("pos1", arm_pos);
        telemetry.addData("pos2", arm_motor2.getCurrentPosition());

        telemetry.addData("pow1", power);
        telemetry.addData("pow2", arm_motor2.getPower());

        telemetry.addData("target", target);
        telemetry.update();

    }
}