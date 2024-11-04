package org.firstinspires.ftc.teamcode.test;

import static org.firstinspires.ftc.teamcode.test.TestMap.*;
import static org.firstinspires.ftc.teamcode.util.Constants.*;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.wpilibcontroller.ArmFeedforward;
import com.arcrobotics.ftclib.controller.wpilibcontroller.ElevatorFeedforward;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.GamepadKeys.Button;
import com.arcrobotics.ftclib.gamepad.ToggleButtonReader;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

@TeleOp(name = "TestDrive", group = "Test")
@Config
public class TestDrive extends OpMode {
    private GamepadEx driver1, driver2;
    private int armPos = ROT_UP;
    private double armPow;
    private int elePos = 0;
    private EleControlMode eleControlMode = EleControlMode.BASKET;
    private double maxSpeed = 1.0;
    private boolean isOdoDrivingEnabled = false;

    @Override
    public void init() {
        //init hardware
        initTestRobot(hardwareMap);

        driver1 = new GamepadEx(gamepad1);
        driver2 = new GamepadEx(gamepad2);

        //update log
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {
        odo.update();
        driver1.readButtons();
        driver2.readButtons();

        if (driver1.wasJustPressed(Button.A)) isOdoDrivingEnabled = !isOdoDrivingEnabled;
        if (driver1.wasJustPressed(Button.LEFT_BUMPER)) maxSpeed = maxSpeed != 0.6 ? 0.6 : 1;
        if (driver1.wasJustPressed(Button.RIGHT_BUMPER)) maxSpeed = maxSpeed != 0.3 ? 0.3 : 1;

        if (driver1.wasJustPressed(Button.START)) odo.resetPosAndIMU();

        double heading = isOdoDrivingEnabled ? odo.getHeading() * 180 / Math.PI : 0;
        drive.setMaxSpeed(maxSpeed);
        drive.driveFieldCentric(-gamepad1.left_stick_x, gamepad1.left_stick_y, -gamepad1.right_stick_x, heading);

        if (driver2.wasJustPressed(Button.DPAD_DOWN)) {
            armPos = armPos == ROT_UP ? ROT_DOWN : ROT_GRAB;
            armPow = armPos == ROT_UP ? 0.2 : 0.1;
        }
        if (driver2.wasJustPressed(Button.DPAD_UP)) {
            armPos = armPos == ROT_GRAB ? ROT_DOWN : ROT_UP;
            armPow = armPos == ROT_GRAB ? 0.3 : 0.25;
        }
        rotate(armPos, armPow);

        if (driver2.wasJustPressed(Button.A)) elePos += 10;
        if (driver2.wasJustPressed(Button.B)) elePos -= 10;
        if (driver2.wasJustPressed(Button.X)) elePos += 50;
        if (driver2.wasJustPressed(Button.Y)) elePos -= 50;
        elevate(elePos);

        if (driver2.wasJustPressed(Button.LEFT_BUMPER)) grabber.turnToAngle(10.8);
        if (driver2.wasJustPressed(Button.RIGHT_BUMPER)) grabber.turnToAngle(585);

        telemetry.addData("armPos", armPos);
        telemetry.addData("elePos", elePos);
        telemetry.addData("maxSpd", maxSpeed);
        telemetry.addData("odoDri", isOdoDrivingEnabled);
        telemetry.addData("odoDeg", odo.getHeading());
        telemetry.addData("eleMod", eleControlMode);

        telemetry.update();
    }

    @Override
    public void stop() {
        super.stop();
    }
}
