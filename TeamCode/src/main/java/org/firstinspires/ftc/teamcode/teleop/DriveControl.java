package org.firstinspires.ftc.teamcode.teleop;

import static org.firstinspires.ftc.teamcode.teleop.RobotMap.*;
import static org.firstinspires.ftc.teamcode.util.Constants.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.TriggerReader;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "DriveControl", group = "TeleOp")
public class DriveControl extends OpMode {
    private GamepadEx driver1, driver2;
    private FtcDashboard dash = FtcDashboard.getInstance();
    private List<Action> runningActions = new ArrayList<>();

    private int elePos = ELE_BOT, rotPos = ROT_UP;
    private double rollAngle = 0;
    private EleControlMode eleControlMode = EleControlMode.BASKET;
    private double maxSpeed = NORMAL_MODE;

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
        if (gamepad2.back) {
            elevator.initRot();
        }
        if (gamepad2.start) {
            elevator.initEle();
        }
    }

    @Override
    public void start() {
        grabber.grabber.setPosition(GRABBER_CLOSE);
        grabber.pitch.setPosition(PITCH_SUBMERSIBLE);
        grabber.roll.setPosition(ROLL_TICK_ON_ZERO);
        grabber.pivot.setPosition(PIVOT_SUBMERSIBLE);
    }

    @Override
    public void loop() {
        driver1.readButtons();
        driver2.readButtons();
        driver2LeftTrigger.readValue();
        driver2RightTrigger.readValue();
        TelemetryPacket packet = new TelemetryPacket();

        if (driver1.isDown(GamepadKeys.Button.A))
            maxSpeed = PRECISION_MODE;
        else
            maxSpeed = NORMAL_MODE;

        rotPos += (int) (-driver1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) * 10 + driver1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) * 10);
        elePos += (driver1.getButton(GamepadKeys.Button.LEFT_BUMPER) ? -10 : 0) + (driver1.getButton(GamepadKeys.Button.RIGHT_BUMPER) ? 10 : 0);

        if (driver1.wasJustPressed(GamepadKeys.Button.START)) drive.resetImu();
        if (driver2.wasJustPressed(GamepadKeys.Button.START)) elevator.initEle();
        if (driver1.wasJustPressed(GamepadKeys.Button.BACK)) drive.toggleFieldCentricDrive();
        if (driver2.wasJustPressed(GamepadKeys.Button.BACK)) elevator.initRot();

        drive.setMaxPower(maxSpeed);
        drive.drive(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);

        if (driver2.wasJustPressed(GamepadKeys.Button.DPAD_DOWN) && elePos == ELE_BOT) {
            runningActions.add(new ParallelAction(grabber.readySampleGrab(), grabber.release()));
            rotPos = ROT_DOWN;
        }
        if (driver2.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
            rotPos = ROT_UP;
        }

        if (gamepad1.dpad_left) {
            rotPos = ROT_HANG_DOWN;
        }
        if (gamepad1.dpad_right) {
            elePos = ELE_HANG;
        }

        if (gamepad1.dpad_down) {
            elevator.rigging();
        }

        if (gamepad1.dpad_up) {
            elevator.stopRigging();
        }

        if (driver2.wasJustPressed(GamepadKeys.Button.Y)) {
            toggleEleControlMode();
            if (eleControlMode == EleControlMode.CHAMBER) gamepad2.rumble(400);
            if (eleControlMode == EleControlMode.BASKET) gamepad2.rumble(800);
        }

        if (rotPos == ROT_DOWN) {
            if (driver2.wasJustPressed(GamepadKeys.Button.A))
                elePos = ELE_BOT;
            if (driver2.wasJustPressed(GamepadKeys.Button.B))
                elePos = ELE_CHAMBER_LOW;
            if (driver2.wasJustPressed(GamepadKeys.Button.X))
                elePos = ELE_CHAMBER_HIGH;
            elePos += (int) (driver2.getLeftY() * 45.0);
            elePos = Range.clip(elePos, 0, 1450);
        } else if (eleControlMode == EleControlMode.CHAMBER) {
            if (driver2.wasJustPressed(GamepadKeys.Button.A)) {
                elePos = ELE_BOT;
                rotPos = ROT_UP;
                rollAngle = 0;
                runningActions.add(grabber.grab());
            }
            if (driver2.wasJustPressed(GamepadKeys.Button.B)) {
//                elePos = ELE_BOT;
//                rotPos = ROT_SPECIMEN;
//                rollAngle = 0;
//                runningActions.add(
//                        grabber.readySpecimenGrab()
//                );

                elePos = ELE_BOT;
                rotPos = ROT_UP;
                rollAngle = 0;
                runningActions.add(
                        grabber.readySpecimenGrab()
                );
            }
            if (driver2.wasJustPressed(GamepadKeys.Button.X)) {
//                elePos = elePos == ELE_CHAMBER_HIGH ? ELE_CHAMBER_HIGH_DROP : ELE_CHAMBER_HIGH;
//                rollAngle = 180;
//                rotPos = ROT_UP;
//                runningActions.add(grabber.readySpecimenClip());

                elePos = ELE_CLIP;
                rollAngle = 180;
                rotPos = ROT_CLIP;
                runningActions.add(grabber.readySpecimenClip());
            }
        } else if (eleControlMode == EleControlMode.BASKET) {
            if (driver2.wasJustPressed(GamepadKeys.Button.A)) {
                elePos = ELE_BOT;
                rollAngle = 0;
                runningActions.add(
                        grabber.readySampleGrab()
                );
            }
            if (driver2.wasJustPressed(GamepadKeys.Button.B)) {
                elePos = ELE_BASKET_LOW;
                rollAngle = 0;
                runningActions.add(grabber.basketReady());
            }
            if (driver2.wasJustPressed(GamepadKeys.Button.X)) {
                elePos = ELE_BASKET_HIGH;
                rollAngle = 0;
                runningActions.add(grabber.basketReady());
            }
        }

        if (!elevator.isRigging) {
            elevator.elevatePIDF(elePos);
            elevator.rotatePIDF(rotPos);
        }

        if (driver2LeftTrigger.wasJustPressed()) {
            if (rotPos == ROT_DOWN) {
                runningActions.add(grabber.performSampleGrab());
            }
            else {
                runningActions.add(grabber.grab());
            }
        }
        if (driver2RightTrigger.wasJustPressed()) {
            if (elePos == ELE_BASKET_LOW || elePos == ELE_BASKET_HIGH)
                runningActions.add(grabber.basketDeposit());
            else
                runningActions.add(grabber.release());
        }

        if (driver2.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER))
            rollAngle += 30;
        if (driver2.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER))
            rollAngle -= 30;

        rollAngle = rollAngle % 360;
        runningActions.add(grabber.roll(rollAngle));

        List<Action> newActions = new ArrayList<>();
        for (Action action : runningActions) {
            action.preview(packet.fieldOverlay());
            if (action.run(packet)) {
                newActions.add(action);
            }
        }
        runningActions = newActions;

        dash.sendTelemetryPacket(packet);

        telemetry.addLine("-------DRIVE CONTROL-------");
        telemetry.addData("CONTROL MODE", eleControlMode);

        telemetry.addLine("-------MOTOR POSITION-------");
        telemetry.addData("elePosition", elePos);
        telemetry.addData("eleEncoder", elevator.getElevatorPosition());
        telemetry.addData("rotPosition", rotPos);
        telemetry.addData("rotEncoder", elevator.getRotationPosition());

        telemetry.addLine("-------ROBOT POSITION-------");
        telemetry.addData("robotX", drive.getPosition().getX(DistanceUnit.MM));
        telemetry.addData("robotY", drive.getPosition().getY(DistanceUnit.MM));
        telemetry.addData("robotH", drive.getPosition().getHeading(AngleUnit.DEGREES));

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