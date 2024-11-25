package org.firstinspires.ftc.teamcode.teleop;

import static org.firstinspires.ftc.teamcode.teleop.RobotMap.*;
import static org.firstinspires.ftc.teamcode.teleop.RobotMap.rotate;
import static org.firstinspires.ftc.teamcode.util.Constants.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.TriggerReader;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.Delayed;

@TeleOp(name = "DriveControl", group = "TeleOp")
@Config
public class DriveControl extends OpMode {
    private GamepadEx driver1, driver2;
    private FtcDashboard dash = FtcDashboard.getInstance();
    private List<Action> runningActions = new ArrayList<>();

    private int armPos = ROT_DOWN;
    private double armPow;
    private double elePos = ELE_BOT;
    private double pitchPos = PITCH_FORWARD, rollPos = ROLL_0;
    private EleControlMode eleControlMode = EleControlMode.BASKET;
    private double maxSpeed = 1.0;
    private boolean isOdoDrivingEnabled = true;

    TriggerReader driver2LeftTrigger, driver2RightTrigger;

    @Override
    public void init() {
        //init hardware
        initRobot(hardwareMap);

        telemetry = new MultipleTelemetry(telemetry, dash.getTelemetry());

        driver1 = new GamepadEx(gamepad1);
        driver2 = new GamepadEx(gamepad2);
        driver2LeftTrigger = new TriggerReader(
                driver2, GamepadKeys.Trigger.LEFT_TRIGGER
        );
        driver2RightTrigger = new TriggerReader(
                driver2, GamepadKeys.Trigger.RIGHT_TRIGGER
        );

        //update log
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void init_loop() {
        if (gamepad2.start) {
            eleMotors.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        if (gamepad2.back) {
            rotMotors.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }

    @Override
    public void loop() {
        odo.update();
        driver1.readButtons();
        driver2.readButtons();
        driver2LeftTrigger.readValue();
        driver2RightTrigger.readValue();
        TelemetryPacket packet = new TelemetryPacket();


        if (driver1.wasJustPressed(GamepadKeys.Button.BACK))
            isOdoDrivingEnabled = !isOdoDrivingEnabled;
        if (driver1.isDown(GamepadKeys.Button.LEFT_BUMPER))
            maxSpeed = NORMAL_MODE;
        else if (driver1.isDown(GamepadKeys.Button.RIGHT_BUMPER))
            maxSpeed = PRECISION_MODE;
        else
            maxSpeed = SAFE_MODE;

        if (driver1.wasJustPressed(GamepadKeys.Button.START)) odo.resetPosAndIMU();
        if (driver2.wasJustPressed(GamepadKeys.Button.START))
            eleMotors.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        double heading = isOdoDrivingEnabled ? -odo.getHeading() * 180 / Math.PI : 0;
        drive.setMaxSpeed(maxSpeed);
        drive.driveFieldCentric(-gamepad1.left_stick_x, gamepad1.left_stick_y, -gamepad1.right_stick_x, heading);

        if (driver2.wasJustPressed(GamepadKeys.Button.DPAD_DOWN) && elePos == ELE_BOT) {
            armPos = ROT_DOWN;
            armPow = 0.5;
        }
        if (driver2.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
            armPos = ROT_UP;
            armPow = 0.7;

            elePos = ELE_BOT;
        }
        rotate(armPos, armPow);

        if (driver2.wasJustPressed(GamepadKeys.Button.Y)) {
            toggleEleControlMode();
            if (eleControlMode == EleControlMode.CHAMBER) gamepad2.rumble(400);
            if (eleControlMode == EleControlMode.BASKET) gamepad2.rumble(800);
        }

        if (armPos != ROT_UP) {
            if (driver2.wasJustPressed(GamepadKeys.Button.A)) {
                elePos = ELE_BOT;
            }
            if (driver2.wasJustPressed(GamepadKeys.Button.B)) {
                elePos = ELE_CHAMBER_LOW;
            }
            if (driver2.wasJustPressed(GamepadKeys.Button.X)) {
                elePos = ELE_CHAMBER_HIGH;
            }
            elePos += driver2.getLeftY() * 35;
            if (elePos > 2200) elePos = 2200;
            if (elePos < 0) elePos = 0;
        }
        if (eleControlMode == EleControlMode.CHAMBER) {
            if (driver2.wasJustPressed(GamepadKeys.Button.A)) {
                elePos = ELE_BOT;
                armPos = ROT_UP;
//                pitchPos = PITCH_FORWARD;
                rollPos = ROLL_0;
                grab();
            }
            if (driver2.wasJustPressed(GamepadKeys.Button.B)) {
                elePos = ELE_BOT;
                armPos = ROT_GRAB;
                armPow = 0.3;
                pitchPos = PITCH_GRAB;
                rollPos = ROLL_0;
                release();
            }
            if (driver2.wasJustPressed(GamepadKeys.Button.X)) {
                elePos = elePos == ELE_CHAMBER_HIGH ? ELE_CHAMBER_HIGH_DROP : ELE_CHAMBER_HIGH;
                pitchPos = PITCH_BACKWARD;
                rollPos = ROLL_180;
            }
        } else if (eleControlMode == EleControlMode.BASKET) {
            if (driver2.wasJustPressed(GamepadKeys.Button.A)) {
                elePos = ELE_BOT;
                pitchPos = PITCH_FORWARD;
                rollPos = ROLL_0;
                grab();
            }
            if (driver2.wasJustPressed(GamepadKeys.Button.B)) {
                elePos = ELE_BASKET_LOW;
                pitchPos = PITCH_UP;
                rollPos = ROLL_0;
            }
            if (driver2.wasJustPressed(GamepadKeys.Button.X)) {
                elePos = ELE_BASKET_HIGH;
                pitchPos = PITCH_UP;
                rollPos = ROLL_0;
            }
        }
        elevate((int) elePos, 1);
        pitch(pitchPos);

        if ((elePos == ELE_CHAMBER_HIGH_DROP || elePos == ELE_CHAMBER_LOW_DROP) && eleMotors.atTargetPosition()) {
//            release();
//            elePos = ELE_BOT;
        }

        if (driver2.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) grab();
        if (driver2.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
            release();
            if (eleControlMode == EleControlMode.BASKET && (elePos == ELE_BASKET_LOW || elePos == ELE_BASKET_HIGH)) {
                pitchPos = PITCH_BACKWARD;
            }
        }

        if (driver2LeftTrigger.wasJustPressed()) {
            if (rollPos == ROLL_0) rollPos = ROLL_NEG_45;
            else if (rollPos == ROLL_NEG_45) rollPos = ROLL_90;
            else if (rollPos == ROLL_45) rollPos = ROLL_0;
            else if (rollPos == ROLL_90) rollPos = ROLL_45;
        }
        if (driver2RightTrigger.wasJustPressed()) {
            if (rollPos == ROLL_0) rollPos = ROLL_45;
            else if (rollPos == ROLL_NEG_45) rollPos = ROLL_0;
            else if (rollPos == ROLL_45) rollPos = ROLL_90;
            else if (rollPos == ROLL_90) rollPos = ROLL_NEG_45;
        }
        roll(rollPos);

        /* Example code for addaing action
        if (gamepad1.a) {
            runningActions.add(new SequentialAction(
                    new SleepAction(0.5),
                    new InstantAction(() -> servo.setPosition(0.5))
            ));
        }
        */

        List<Action> newActions = new ArrayList<>();
        for (Action action : runningActions) {
            action.preview(packet.fieldOverlay());
            if (action.run(packet)) {
                newActions.add(action);
            }
        }
        runningActions = newActions;

        dash.sendTelemetryPacket(packet);

        telemetry.addData("armPos", armPos);
        telemetry.addData("armLft", rotLeftMotor.getCurrentPosition());
        telemetry.addData("elePos", elePos);
        telemetry.addData("eleLft", eleLeftMotor.getCurrentPosition());
        telemetry.addData("eleRit", eleRightMotor.getCurrentPosition());
        telemetry.addData("rolPos", rollPos);
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