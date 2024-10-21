package org.firstinspires.ftc.teamcode.test;

import static org.firstinspires.ftc.teamcode.test.TestMap.*;
import static org.firstinspires.ftc.teamcode.util.Constants.*;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.wpilibcontroller.ArmFeedforward;
import com.arcrobotics.ftclib.controller.wpilibcontroller.ElevatorFeedforward;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys.Button;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "TestDrive", group = "Test")
@Config
public class TestDrive extends OpMode {
    private GamepadEx driver1, driver2;
//    private ArmFeedforward armff;
//    private ElevatorFeedforward eleff;
    private int armPos = 0;
    private int elePos = 0;
    private double maxSpeed = 1.0;

    @Override
    public void init() {
        //init hardware
        initTestRobot(hardwareMap);

        driver1 = new GamepadEx(gamepad1);
        driver2 = new GamepadEx(gamepad2);
//        armff = new ArmFeedforward(aS, aCos, aV, aA);
//        eleff = new ElevatorFeedforward(eS, eG, eV, eA);

        //update log
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {
        if (driver1.wasJustPressed(Button.LEFT_BUMPER)) maxSpeed = maxSpeed != 0.6 ? 0.6 : 1;
        if (driver1.wasJustPressed(Button.RIGHT_BUMPER)) maxSpeed = maxSpeed != 0.3 ? 0.3 : 1;
        drive.setMaxSpeed(maxSpeed);
        drive.driveRobotCentric(driver1.getLeftX(), driver1.getLeftY(), driver1.getRightY());

        if (driver1.wasJustPressed(Button.DPAD_DOWN)) armPos -= 1;
        if (driver1.wasJustPressed(Button.DPAD_UP)) armPos += 1;
        rotate(armPos, 0.5);

        if (driver2.wasJustPressed(Button.A)) elePos = ELE_LOW;
        if (driver2.wasJustPressed(Button.B)) elePos = ELE_HIGH_CHAMBER;
        if (driver2.wasJustPressed(Button.X)) elePos = ELE_MID;
        if (driver2.wasJustPressed(Button.Y)) elePos = ELE_HIGH;
        if (driver2.getLeftY() > 0.8) elePos = ELE_DROP_SPECIMEN;
        elevate(elePos);

        if (driver2.wasJustPressed(Button.LEFT_BUMPER)) grabber.turnToAngle(GRABBER_CLOSE * 360 * 5);
        if (driver2.wasJustPressed(Button.RIGHT_BUMPER)) grabber.turnToAngle(GRABBER_OPEN * 360 * 5);

        telemetry.addData("armPos", armPos);
        telemetry.addData("elePos", elePos);
        telemetry.addData("maxSpd", maxSpeed);
    }

    @Override
    public void stop() {
        super.stop();
    }
}
