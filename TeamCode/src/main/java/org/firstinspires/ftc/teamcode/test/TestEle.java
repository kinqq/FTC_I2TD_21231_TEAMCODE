package org.firstinspires.ftc.teamcode.test;

import static org.firstinspires.ftc.teamcode.teleop.RobotMap.*;
import static org.firstinspires.ftc.teamcode.util.Constants.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
//import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

@TeleOp(name = "TestEle", group = "Test")
@Config
public class TestEle extends OpMode {
    Gamepad previousGamepad1 = new Gamepad();
    Gamepad previousGamepad2 = new Gamepad();

    int elePos = ELE_BOT;
    public int eleCorrection = 0;

    @Override
    public void init() {
        //init hardware
        initRobot(hardwareMap);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        //update log
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void init_loop() {
        if (gamepad2.start) {
            eleLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            eleRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }

    @Override
    public void loop() {
        // Elevator
        if (gamepad2.a) elePos = ELE_BOT;
        if (gamepad2.b) elePos = ELE_HIGH_CHAMBER;
        if (gamepad2.x) elePos = ELE_MID;
        if (gamepad2.y) elePos = ELE_HIGH;

        eleLeftMotor.setTargetPosition(elePos + eleCorrection);
        eleRightMotor.setTargetPosition(elePos + eleCorrection);
        eleLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        eleRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        eleLeftMotor.setPower(1);
        eleRightMotor.setPower(1);

        telemetry.addData("elePos", elePos + eleCorrection);
        telemetry.addData("actualElePos", eleLeftMotor.getCurrentPosition());
        telemetry.addData("eleTol", eleLeftMotor.getTargetPositionTolerance());

        telemetry.addData("ltrig", (int)gamepad2.left_trigger * 10);
        telemetry.addData("rtrig", (int)gamepad2.right_trigger * 10);
eleRightMotor.getPower();
        telemetry.update();

        previousGamepad1.copy(gamepad1);
        previousGamepad2.copy(gamepad2);
    }

    @Override
    public void stop() {
        super.stop();
    }
}