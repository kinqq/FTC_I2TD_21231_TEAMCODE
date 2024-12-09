package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Config
@TeleOp(name = "TestElePIDF", group = "Test")
public class TestElePIDF extends OpMode {
    private PIDController controller;
    public static double p = 0.005, i = 0.0001, d = 0.0001;

    public static int target = 0;
    public static String leftEleName = "leftEle";
    public static String rightEleName = "rightEle";

    private DcMotorEx leftEle, rightEle;

    @Override
    public void init() {
        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        leftEle = hardwareMap.get(DcMotorEx.class, leftEleName);
        rightEle = hardwareMap.get(DcMotorEx.class, rightEleName);

        leftEle.setDirection(DcMotorSimple.Direction.FORWARD);
        rightEle.setDirection(DcMotorSimple.Direction.REVERSE);

        leftEle.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftEle.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftEle.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightEle.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightEle.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightEle.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void loop() {
        controller.setPID(p, i, d);
        int arm_pos = leftEle.getCurrentPosition();
        double pid = controller.calculate(arm_pos, target);

        leftEle.setPower(pid);
        rightEle.setPower(pid);

        telemetry.addData("pos", arm_pos);
        telemetry.addData("pow", pid);
        telemetry.addData("target", target);
        telemetry.update();
    }
}