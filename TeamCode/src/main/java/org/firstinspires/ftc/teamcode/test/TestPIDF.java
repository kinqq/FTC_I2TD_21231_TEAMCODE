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
    public static double pUp = 0.0025, i = 0, d = 0, f = 0.2, pDown = 0.001;
    public static int target = 0;
    private final double ticks_in_degree = 537.7 * 5 / 360.0;
    private DcMotorEx arm_motor1, arm_motor2;

    @Override
    public void init() {
        controller = new PIDController(pUp, i, d);
        controller2 = new PIDController(pUp, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        arm_motor1 = hardwareMap.get(DcMotorEx.class, "leftRot");
        arm_motor2 = hardwareMap.get(DcMotorEx.class, "rightRot");

        arm_motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm_motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        arm_motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        arm_motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void loop() {
        controller.setPID(pUp, i, d);
        int arm_pos = arm_motor1.getCurrentPosition();
        // When going down
        if (arm_pos - target < 0) {
            controller.setP(pDown);
        }
        double pid = controller.calculate(arm_pos, target);
        double ff = -Math.sin(Math.toRadians(arm_pos / ticks_in_degree)) * f;
        double power = pid + ff;
        arm_motor1.setPower(power);
        arm_motor2.setPower(power);

        telemetry.addData("pos1", arm_pos);
        telemetry.addData("pos2", arm_motor2.getCurrentPosition());

        telemetry.addData("pow1", power);
        telemetry.addData("pow2", arm_motor2.getPower());

        telemetry.addData("target", target);
        telemetry.update();

    }
}