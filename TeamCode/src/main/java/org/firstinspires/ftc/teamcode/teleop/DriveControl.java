package org.firstinspires.ftc.teamcode.teleop;

import static org.firstinspires.ftc.teamcode.teleop.RobotMap.*;
import static org.firstinspires.ftc.teamcode.util.Constants.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.TriggerReader;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "DriveControl", group = "TeleOp")
@Config
public class DriveControl extends OpMode {
    private GamepadEx driver1, driver2;
    private FtcDashboard dash = FtcDashboard.getInstance();
    private List<Action> runningActions = new ArrayList<>();

    private int elePos = ELE_BOT;
    private double rollAngle = 0;
    private EleControlMode eleControlMode = EleControlMode.BASKET;
    private double maxSpeed = 1.0;

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
    public void loop() {
        driver1.readButtons();
        driver2.readButtons();
        driver2LeftTrigger.readValue();
        driver2RightTrigger.readValue();
        TelemetryPacket packet = new TelemetryPacket();

        if (driver1.wasJustPressed(GamepadKeys.Button.BACK))
            drive.toggleFieldCentricDrive();

        if (driver1.isDown(GamepadKeys.Button.LEFT_BUMPER))
            maxSpeed = NORMAL_MODE;
        else if (driver1.isDown(GamepadKeys.Button.RIGHT_BUMPER))
            maxSpeed = PRECISION_MODE;
        else
            maxSpeed = SAFE_MODE;

        if (driver1.wasJustPressed(GamepadKeys.Button.START)) drive.resetImu();
        if (driver2.wasJustPressed(GamepadKeys.Button.START))
            elevator.initEle();

        drive.setMaxPower(maxSpeed);
        drive.drive(-gamepad1.left_stick_x, gamepad1.left_stick_y, -gamepad1.right_stick_x);

        if (driver2.wasJustPressed(GamepadKeys.Button.DPAD_DOWN) && elePos == ELE_BOT) {
            runningActions.add(elevator.rotateDown());
        }
        if (driver2.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
            runningActions.add(elevator.rotateUp());
            // we had elevator-bottom code here...
        }

        if (driver2.wasJustPressed(GamepadKeys.Button.Y)) {
            toggleEleControlMode();
            if (eleControlMode == EleControlMode.CHAMBER) gamepad2.rumble(400);
            if (eleControlMode == EleControlMode.BASKET) gamepad2.rumble(800);
        }

        if (elevator.getPivotTarget() != ROT_UP) {
            if (driver2.wasJustPressed(GamepadKeys.Button.A))
                elePos = ELE_BOT;
            if (driver2.wasJustPressed(GamepadKeys.Button.B))
                elePos = ELE_CHAMBER_LOW;
            if (driver2.wasJustPressed(GamepadKeys.Button.X))
                elePos = ELE_CHAMBER_HIGH;
            elePos += (int) (driver2.getLeftY() * 35.0);
            if (elePos > 2200) elePos = 2200;
            if (elePos < 0) elePos = 0;
        }
        if (eleControlMode == EleControlMode.CHAMBER) {
            if (driver2.wasJustPressed(GamepadKeys.Button.A)) {
                elePos = ELE_BOT;
                runningActions.add(elevator.rotateUp());
                runningActions.add(grabber.roll(0));
                runningActions.add(grabber.grab());
            }
            if (driver2.wasJustPressed(GamepadKeys.Button.B)) {
                elePos = ELE_BOT;
                runningActions.add(new ParallelAction(
                        elevator.rotateGrab(),
                        grabber.pitchGrab(),
                        grabber.roll(0),
                        grabber.release()
                ));
            }
            if (driver2.wasJustPressed(GamepadKeys.Button.X)) {
                elePos = elePos == ELE_CHAMBER_HIGH ? ELE_CHAMBER_HIGH_DROP : ELE_CHAMBER_HIGH;
                runningActions.add(new ParallelAction(grabber.pitchBackward(), grabber.roll(180)));
            }
        } else if (eleControlMode == EleControlMode.BASKET) {
            if (driver2.wasJustPressed(GamepadKeys.Button.A)) {
                elePos = ELE_BOT;
                runningActions.add(new ParallelAction(
                        grabber.pitchForward(),
                        grabber.roll(0),
                        grabber.grab()
                ));
            }
            if (driver2.wasJustPressed(GamepadKeys.Button.B)) {
                elePos = ELE_BASKET_LOW;
                runningActions.add(new ParallelAction(
                        grabber.pitchUp(),
                        grabber.roll(0)
                ));
            }
            if (driver2.wasJustPressed(GamepadKeys.Button.X)) {
                elePos = ELE_BASKET_HIGH;
                runningActions.add(new ParallelAction(
                        grabber.pitchUp(),
                        grabber.roll(0)
                ));
            }
        }
        runningActions.add(elevator.elevate(elePos));

        // Auto release specimen -- activate when clipping is reliable
//        if ((elePos == ELE_CHAMBER_HIGH_DROP || elePos == ELE_CHAMBER_LOW_DROP) && eleMotors.atTargetPosition()) {
//            release();
//            elePos = ELE_BOT;
//        }

        if (driver2.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER))
            runningActions.add(grabber.grab());
        if (driver2.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
            runningActions.add(new SequentialAction(grabber.pitchBackward(), grabber.release()));
        }

        if (driver2LeftTrigger.wasJustPressed())
            rollAngle += 30;
        if (driver2RightTrigger.wasJustPressed())
            rollAngle -= 30;
        rollAngle = (rollAngle + 360) % 360;
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

        telemetry.addData("elePos", elePos);
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