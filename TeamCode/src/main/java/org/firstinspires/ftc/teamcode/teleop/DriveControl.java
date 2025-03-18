package org.firstinspires.ftc.teamcode.teleop;

import static org.firstinspires.ftc.teamcode.teleop.RobotMap.*;
import static org.firstinspires.ftc.teamcode.util.Constants.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.TriggerReader;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Drawing;

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
    private double oldTime = 0;
    private boolean isClipped = true;
    private ElapsedTime timer = new ElapsedTime();
    private ElapsedTime timer2 = new ElapsedTime();
    private ElapsedTime timer3 = new ElapsedTime();

    private int imuCnt = 0;

    private enum RiggingState {
        READY,
        INIT_ELE,
        INIT_ROT,
        FIRST_CLIMB,
        FIRST_TRANSFER,
        SECOND_EXT,
        SECOND_HANG,
        SECOND_HOOK,
        SECOND_TOUCH,
        SECOND_CLIMB_READY,
        SECOND_CLIMB,
        FINISHED,
    }

    RiggingState riggingState = RiggingState.READY;

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

        drive.resetTargetHeading();
        timer.reset();
        timer2.reset();
        timer3.reset();
    }

    @Override
    public void loop() {
        driver1.readButtons();
        driver2.readButtons();
        driver2LeftTrigger.readValue();
        driver2RightTrigger.readValue();
        TelemetryPacket packet = new TelemetryPacket();

        if (driver1.isDown(GamepadKeys.Button.RIGHT_BUMPER))
            maxSpeed = PRECISION_MODE;
        else
            maxSpeed = NORMAL_MODE;

        rotPos += (int) (-driver1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) * 10 + driver1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) * 10);
        elePos += (driver1.getButton(GamepadKeys.Button.B) ? -10 : 0) + (driver1.getButton(GamepadKeys.Button.A) ? 10 : 0);

        if (driver1.wasJustPressed(GamepadKeys.Button.START)) {
            drive.resetImu();
            imuCnt++;
        }
        if (driver2.wasJustPressed(GamepadKeys.Button.START)) elevator.initEle();
        if (driver1.wasJustPressed(GamepadKeys.Button.BACK)) drive.toggleFieldCentricDrive();
        if (driver2.wasJustPressed(GamepadKeys.Button.BACK)) elevator.initRot();

        drive.setMaxPower(maxSpeed);
        if (!elevator.isRigging)
            drive.drive(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);
        else drive.turnOff();

        if (driver1.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
            drive.toggleHeadingLock();
        }

        if (driver2.wasJustPressed(GamepadKeys.Button.DPAD_DOWN) && elePos == ELE_BOT) {
            runningActions.add(new ParallelAction(grabber.readySampleGrab(), grabber.release()));
            rotPos = ROT_DOWN;
        }
        if (driver2.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
            rotPos = ROT_UP;
        }

        switch (riggingState) {
            case READY:
                elevator.isRigging = false;
                if (driver1.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) {
                    riggingState = RiggingState.INIT_ELE;
                    elePos = ELE_HANG + 80;
                    elevator.isRigging = true;
                }
                break;
            case INIT_ELE:
                if (driver1.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) {
                    riggingState = RiggingState.INIT_ROT;
                    rotPos = ROT_HANG_DOWN + 15;
                }
                break;
            case INIT_ROT:
                if (driver1.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) {
                    riggingState = RiggingState.FIRST_CLIMB;
                    elePos = 180;
                    rotPos = 350;
                }
                break;
            case FIRST_CLIMB:
                if (driver1.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) {
                    riggingState = RiggingState.FIRST_TRANSFER;
                    rotPos = 170;
                }
                break;
            case FIRST_TRANSFER:
                if (driver1.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) {
                    riggingState = RiggingState.SECOND_EXT;
                    elePos = 1900;
                    rotPos = 0;
                }
                break;
            case SECOND_EXT:
                if (driver1.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) {
                    riggingState = RiggingState.SECOND_HANG;
                    rotPos = 350;
                }
                break;
            case SECOND_HANG:
                if (driver1.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) {
                    riggingState = RiggingState.SECOND_HOOK;
                    elePos = 1500;
                }
                break;
            case SECOND_HOOK:
                if (driver1.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) {
                    riggingState = RiggingState.SECOND_TOUCH;
                    elePos = 1700;
                    rotPos = ROT_DOWN + 60;
                }
                if (driver1.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) {
                    riggingState = RiggingState.SECOND_EXT;
                    elePos = 1900;
                    rotPos = 0;
                }
                break;
            case SECOND_TOUCH:
                if (driver1.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) {
                    riggingState = RiggingState.SECOND_CLIMB_READY;
                    elePos = 1450;
                }
                break;
            case SECOND_CLIMB_READY:
                if (driver1.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) {
                    riggingState = RiggingState.SECOND_CLIMB;
                    elePos = ELE_BOT;
                    rotPos = ROT_UP;
                }
                break;
            case SECOND_CLIMB:
                if (driver1.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT))
                    riggingState = RiggingState.FINISHED;
                break;
            default:
                riggingState = RiggingState.READY;
        }

        // turn of heading lock while climbing
        if (elevator.isRigging && drive.isHeadingLockEnabled()) {
            drive.toggleHeadingLock();
        }

        if (driver2.wasJustPressed(GamepadKeys.Button.Y)) {
            toggleEleControlMode();
            if (eleControlMode == EleControlMode.CHAMBER) gamepad2.rumble(400);
            if (eleControlMode == EleControlMode.BASKET) gamepad2.rumble(800);
        }

        if (rotPos == ROT_DOWN && !elevator.isRigging) {
            if (driver2.wasJustPressed(GamepadKeys.Button.A))
                elePos = ELE_BOT;
            if (driver2.wasJustPressed(GamepadKeys.Button.B))
                elePos = ELE_CHAMBER_LOW;
            if (driver2.wasJustPressed(GamepadKeys.Button.X))
                elePos = ELE_CHAMBER_HIGH;
            elePos += (int) (driver2.getLeftY() * 50.0);
            elePos = Range.clip(elePos, 0, 1300);

            if (timer2.seconds() > 0.4) {
                grabber.pitch.setPosition(PITCH_SUBMERSIBLE + 0.03 / 500 * elePos);
                grabber.pivot.setPosition(PIVOT_SUBMERSIBLE + 0.03 / 500 * elePos);
            }
        } else if (eleControlMode == EleControlMode.CHAMBER) {
            if (driver2.wasJustPressed(GamepadKeys.Button.A)) {
                elePos = ELE_BOT;
                rotPos = ROT_UP;
                rollAngle = 0;
                runningActions.add(grabber.readySpecimenGrab());
            }
            if (driver2.wasJustPressed(GamepadKeys.Button.B)) {
                elePos = ELE_BOT;
                rotPos = 180;
                rollAngle = 0;
                runningActions.add(grabber.readySpecimenGrab());
                isClipped = false;
            }
            if (driver2.wasJustPressed(GamepadKeys.Button.X) && timer.seconds() > 0.3) {
                if (elePos == ELE_BOT)
                    runningActions.add(grabber.readySpecimenClip());
                else {
                    if (isClipped) {
                        runningActions.add(grabber.readySpecimenClip());
                        isClipped = false;
                    }
                    else {
                        runningActions.add(grabber.performSpecimenClip());
                        isClipped = true;
                    }
                }
                elePos = 130;
                rotPos = ROT_UP;
                rollAngle = 180;
                timer.reset();
            }
        } else if (eleControlMode == EleControlMode.BASKET) {
            if (driver2.wasJustPressed(GamepadKeys.Button.A)) {
                elePos = ELE_BOT;
                rollAngle = 0;

                if (rotPos == ROT_DOWN) runningActions.add(grabber.basketReady());
                else runningActions.add(grabber.readySampleGrab());
            }
            if (driver2.wasJustPressed(GamepadKeys.Button.B)) {
                elePos = ELE_BASKET_LOW;
                rollAngle = 90;
                timer3.reset();
                runningActions.add(grabber.basketReady());
            }
            if (driver2.wasJustPressed(GamepadKeys.Button.X)) {
                elePos = ELE_BASKET_HIGH;
                rollAngle = 90;
                timer3.reset();
                runningActions.add(grabber.basketReady());
            }
        }

        // ELEVATOR / ROTATION
        if (riggingState == RiggingState.FINISHED) {
            elevator.stopRigging();
        }
        else if (!elevator.isRigging) {
            elevator.elevatePIDF(elePos);
            elevator.rotatePIDF(rotPos);
        }
        else {
            elevator.elevateTo(elePos);
            elevator.rotateTo(rotPos);
        }

        // GRABBER
        if (driver2LeftTrigger.wasJustPressed()) {
            if (rotPos == ROT_DOWN) {
                timer2.reset();
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

        if ((elePos == ELE_BASKET_LOW || elePos == ELE_BASKET_HIGH) && timer3.seconds() > 1.0) {
            runningActions.add(grabber.basketDepositReady());
        }

        // ROLL
        if (driver2.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER))
            rollAngle += 30;
        if (driver2.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER))
            rollAngle -= 30;

        rollAngle = (rollAngle + 180) % 360 - 180;
        rollAngle = Range.clip(rollAngle, -180, 180);
        grabber.roll.setPosition(ROLL_TICK_ON_ZERO + ROLL_TICK_PER_DEG * rollAngle);
        grabber.updateElePos(elePos);

        List<Action> newActions = new ArrayList<>();
        for (Action action : runningActions) {
            action.preview(packet.fieldOverlay());
            if (action.run(packet)) {
                newActions.add(action);
            }
        }
        runningActions = newActions;

        dash.sendTelemetryPacket(packet);

        double newTime = getRuntime();
        double loopTime = newTime - oldTime;
        double frequency = 1 / loopTime;
        oldTime = newTime;

        telemetry.addData("Target Heading", drive.targetHeading);
        telemetry.addData("IMU", imuCnt);
        telemetry.addData("RiggingState", riggingState);
        telemetry.addData("Roll Angle", rollAngle);
        telemetry.addData("Roll Tick", grabber.roll.getPosition());
        telemetry.addLine("-------DRIVE CONTROL-------");
        telemetry.addData("CONTROL MODE", eleControlMode);
        telemetry.addData("FIELD CENTRIC", drive.isFieldCentricDriveEnabled());
        telemetry.addData("HEADING LOCK", drive.isHeadingLockEnabled());
        telemetry.addData("Frequency (Hz)", frequency);

        telemetry.addLine("-------MOTOR POSITION-------");
        telemetry.addData("elePosition", elePos);
        telemetry.addData("eleEncoder", elevator.getElevatorPosition());
        telemetry.addData("rotPosition", rotPos);
        telemetry.addData("rotEncoder", elevator.getRotationPosition());

        telemetry.addLine("-------ROBOT POSITION-------");
        telemetry.addData("robotX", drive.getPosition().getX(DistanceUnit.MM));
        telemetry.addData("robotY", drive.getPosition().getY(DistanceUnit.MM));
        telemetry.addData("robotH", drive.getPosition().getHeading(AngleUnit.DEGREES));

        Drawing.drawRobot(packet.fieldOverlay(),
                new Pose2d(
                        new Vector2d(drive.getPosition().getX(DistanceUnit.INCH),
                                drive.getPosition().getY(DistanceUnit.INCH)),
                        drive.getPosition().getHeading(AngleUnit.RADIANS)
                )
        );

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