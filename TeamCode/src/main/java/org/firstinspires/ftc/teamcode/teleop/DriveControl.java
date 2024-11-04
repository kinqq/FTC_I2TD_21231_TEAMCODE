package org.firstinspires.ftc.teamcode.teleop;

import static org.firstinspires.ftc.teamcode.teleop.RobotMap.*;
import static org.firstinspires.ftc.teamcode.util.Constants.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "DriveControl", group = "TeleOp")
@Config
public class DriveControl extends OpMode {
    private GamepadEx driver1, driver2;

    private int armPos = ROT_UP;
    private double armPow;
    private int elePos = ELE_BOT;
    private EleControlMode eleControlMode = EleControlMode.BASKET;
    private double maxSpeed = 1.0;
    private boolean isOdoDrivingEnabled = false;

    @Override
    public void init() {
        //init hardware
        initRobot(hardwareMap);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        driver1 = new GamepadEx(gamepad1);
        driver2 = new GamepadEx(gamepad2);

        //update log
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void init_loop() {
        if (gamepad2.start) {
            eleMotors.stopAndResetEncoder();
        }
    }

    @Override
    public void loop() {
        odo.update();
        driver1.readButtons();
        driver2.readButtons();

        if (driver1.wasJustPressed(GamepadKeys.Button.A))
            isOdoDrivingEnabled = !isOdoDrivingEnabled;
        if (driver1.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER))
            maxSpeed = maxSpeed != 0.6 ? 0.6 : 1;
        if (driver1.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER))
            maxSpeed = maxSpeed != 0.3 ? 0.3 : 1;

        if (driver1.wasJustPressed(GamepadKeys.Button.START)) odo.resetPosAndIMU();
        if (driver2.wasJustPressed(GamepadKeys.Button.START)) eleMotors.stopAndResetEncoder();

        double heading = isOdoDrivingEnabled ? odo.getHeading() * 180 / Math.PI : 0;
        drive.setMaxSpeed(maxSpeed);
        drive.driveFieldCentric(-gamepad1.left_stick_x, gamepad1.left_stick_y, -gamepad1.right_stick_x, heading);

        if (driver2.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
            armPos = armPos == ROT_UP ? ROT_DOWN : ROT_GRAB;
            armPow = armPos == ROT_UP ? 0.2 : 0.1;
        }
        if (driver2.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
            armPos = armPos == ROT_GRAB ? ROT_DOWN : ROT_UP;
            armPow = armPos == ROT_GRAB ? 0.3 : 0.25;
        }
        rotate(armPos, armPow);

        if (driver2.wasJustPressed(GamepadKeys.Button.Y)) {
            toggleEleControlMode();
            if (eleControlMode == EleControlMode.CHAMBER) gamepad2.rumbleBlips(1);
            if (eleControlMode == EleControlMode.BASKET) gamepad2.rumbleBlips(2);
        }

        if (eleControlMode == EleControlMode.CHAMBER) {
            if (driver2.wasJustPressed(GamepadKeys.Button.A)) {
                elePos = ELE_BOT;
            }
            if (driver2.wasJustPressed(GamepadKeys.Button.B)) {
                elePos = elePos == ELE_CHAMBER_LOW ? ELE_CHAMBER_LOW_DROP : ELE_CHAMBER_LOW;
            }
            if (driver2.wasJustPressed(GamepadKeys.Button.X)) {
                elePos = elePos == ELE_CHAMBER_HIGH ? ELE_CHAMBER_HIGH_DROP : ELE_CHAMBER_HIGH;
            }
        } else {
            if (driver2.wasJustPressed(GamepadKeys.Button.A)) {
                elePos = ELE_BOT;
                // pitchPos = PitchPosition.FORWARD
            }
            if (driver2.wasJustPressed(GamepadKeys.Button.B)) {
                elePos = ELE_BASKET_LOW;
                // if (armPos == ROT_DOWN) pitchPos = PitchPosition.FORWARD
                // else pitchPos = PitchPosition.BACKWARD
            }
            if (driver2.wasJustPressed(GamepadKeys.Button.X)) {
                elePos = ELE_BASKET_HIGH;
                // if (armPos == ROT_DOWN) pitchPos = PitchPosition.FORWARD
                // else pitchPos = PitchPosition.BACKWARD
            }
        }
        elevate(elePos);

        if (driver2.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) grabber.turnToAngle(10.8);
        if (driver2.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) grabber.turnToAngle(585);

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

    public void toggleEleControlMode() {
        eleControlMode = eleControlMode == EleControlMode.CHAMBER ? EleControlMode.BASKET : EleControlMode.CHAMBER;
    }
}