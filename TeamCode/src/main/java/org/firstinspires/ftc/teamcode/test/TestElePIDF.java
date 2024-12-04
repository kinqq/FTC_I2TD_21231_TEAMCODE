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
    public static double p = 0.0015, i = 0, d = 0.00001;

    public static int target = 0;
    public static String deviceName = "rightEle";

    private DcMotorEx eleMotor;

    @Override
    public void init() {
        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        eleMotor = hardwareMap.get(DcMotorEx.class, deviceName);

        eleMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        eleMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        eleMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void loop() {
        // 312 rpm -- 0.005, 0.0001, 0.0001
        //
        controller.setPID(p, i, d);
        int arm_pos = eleMotor.getCurrentPosition();
        double pid = controller.calculate(arm_pos, target);
        eleMotor.setPower(pid);

        telemetry.addData("pos", arm_pos);
        telemetry.addData("pow", pid);
        telemetry.addData("target", target);
        telemetry.update();
    }
}