package org.firstinspires.ftc.teamcode.autonomous;


import static org.firstinspires.ftc.teamcode.util.Constants.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystem.Elevator;
import org.firstinspires.ftc.teamcode.subsystem.Grabber;
import org.firstinspires.ftc.teamcode.PinpointDrive;

@Autonomous(group = "Auto")
@Config
public class AutoRRTest extends LinearOpMode {
    enum Alliance {
        BLUE,
        RED
    }

    enum Position {
        LEFT,
        RIGHT
    }

    public Alliance alliance = Alliance.BLUE;
    public Position position = Position.LEFT;

    @Override
    public void runOpMode() {
        PinpointDrive drive = new PinpointDrive(hardwareMap, new Pose2d(0, 0, 0));
        Elevator elevator = new Elevator(hardwareMap);
        Grabber grabber = new Grabber(hardwareMap);

        Pose2d leftInitialPose = new Pose2d(15, 63, Math.toRadians(90));
        Pose2d rightInitialPose = new Pose2d(-15, 63, Math.toRadians(90));

        elevator.initEle();
        elevator.initRot();

        grabber.grabber.setPosition(GRABBER_CLOSE);
        grabber.pitch.setPosition(PITCH_BACKWARD);
        grabber.roll.setPosition(ROLL_TICK_ON_ZERO);

        // Wait until start and set up parameters
        while (!opModeIsActive() && !isStopRequested()) {
            if (gamepad1.left_bumper) {
                alliance = Alliance.BLUE;
            }
            if (gamepad1.right_bumper) {
                alliance = Alliance.RED;
            }
            if (gamepad1.left_trigger > 0.25) {
                position = Position.LEFT;
            }
            if (gamepad1.right_trigger > 0.25) {
                position = Position.RIGHT;
            }
            if (gamepad2.left_trigger > 0.25) {
                grabber.grabber.setPosition(GRABBER_CLOSE);
                grabber.pitch.setPosition(PITCH_BACKWARD);
            }
            if (gamepad2.right_trigger > 0.25) {
                grabber.grabber.setPosition(GRABBER_OPEN);
                grabber.pitch.setPosition(PITCH_BACKWARD);
            }

            telemetry.addData("Alliance", alliance);
            telemetry.addData("Position", position);
            telemetry.update();
        }

        if (position == Position.LEFT)
            drive = new PinpointDrive(hardwareMap, leftInitialPose);
        if (position == Position.RIGHT)
            drive = new PinpointDrive(hardwareMap, rightInitialPose);

        if (position == Position.LEFT) {
            Vector2d CHAMBER_POSE = new Vector2d(3, 27);
            Vector2d BACKUP_POSE = new Vector2d(3, 40);
            Vector2d FIRST_SAMPLE_POSE = new Vector2d(49, 39.5);
            double FIRST_SAMPLE_HEADING = Math.toRadians(270);
            Vector2d BASKET_POSE = new Vector2d(59, 59);
            double BASKET_HEADING = Math.toRadians(-135);
            Vector2d SECOND_SAMPLE_POSE = new Vector2d(58.6, 39.5);
            double SECOND_SAMPLE_HEADING = Math.toRadians(270);
            Vector2d THIRD_SAMPLE_POSE = new Vector2d(52.5, 24.5);
            double THIRD_SAMPLE_HEADING = Math.toRadians(0);
            Pose2d PARKING_POSE = new Pose2d(22, 10, Math.toRadians(180));
            double PARKING_TANGENT = Math.toRadians(180);

            TrajectoryActionBuilder traj1, traj2, traj3, traj4, traj5, traj6, traj7, traj8;

            traj1 = drive.actionBuilder(leftInitialPose).strafeTo(CHAMBER_POSE);
            traj2 = traj1.endTrajectory().fresh().strafeTo(BACKUP_POSE).strafeToLinearHeading(FIRST_SAMPLE_POSE, FIRST_SAMPLE_HEADING);
            traj3 = traj2.endTrajectory().fresh().strafeToLinearHeading(BASKET_POSE, BASKET_HEADING);
            traj4 = traj3.endTrajectory().fresh().strafeToLinearHeading(SECOND_SAMPLE_POSE, SECOND_SAMPLE_HEADING);
            traj5 = traj4.endTrajectory().fresh().strafeToLinearHeading(BASKET_POSE, BASKET_HEADING);
            traj6 = traj5.endTrajectory().fresh().strafeToLinearHeading(THIRD_SAMPLE_POSE, THIRD_SAMPLE_HEADING);
            traj7 = traj6.endTrajectory().fresh().strafeToLinearHeading(BASKET_POSE, BASKET_HEADING);
            traj8 = traj7.endTrajectory().fresh().splineToLinearHeading(PARKING_POSE, PARKING_TANGENT);

            Actions.runBlocking(new SequentialAction(
                    // Clip Pre-loaded Specimen
                    new ParallelAction(
                            traj1.build(),
                            grabber.pitchBackward(),
                            grabber.grab(),
                            elevator.rotateUp(ROT_UP),
                            elevator.elevateUp(ELE_CHAMBER_HIGH)
                    ),
                    elevator.elevateDown(ELE_CHAMBER_HIGH_DROP),
                    grabber.release(),
                    // Go to first sample
                    new ParallelAction(
                            elevator.elevateDown(ELE_BOT),
                            grabber.pitchForward(),
                            traj2.build()
                    ),
                    elevator.rotateDown(ROT_DOWN),
                    // Grab first sample and go to basket
                    grabber.grab(),
                    elevator.rotateUp(ROT_UP),
                    new ParallelAction(
                            traj3.build(),
                            elevator.rotateUp(ROT_UP),
                            grabber.pitchUp(),
                            new SequentialAction(
                                    new SleepAction(1),
                                    elevator.elevateUp(ELE_BASKET_HIGH)
                            )
                    ),
                    // Release first sample
                    new SequentialAction(
                            grabber.pitchBackward(),
                            grabber.release()
                    ),
                    // Go to second sample
                    new ParallelAction(
                            grabber.pitchForward(),
                            grabber.roll(0),
                            elevator.elevateDown(ELE_BOT),
                            traj4.build()
                    ),
                    elevator.rotateDown(ROT_DOWN),
                    grabber.grab(),
                    elevator.rotateUp(ROT_UP),
                    new ParallelAction(
                            traj5.build(),
                            elevator.rotateUp(ROT_UP),
                            grabber.pitchUp(),
                            grabber.roll(0),
                            new SequentialAction(
                                    new SleepAction(1),
                                    elevator.elevateUp(ELE_BASKET_HIGH)
                            )
                    ),
                    // Release second sample
                    new SequentialAction(
                            grabber.pitchBackward(),
                            grabber.release()
                    ),
                    // Go to third sample
                    new ParallelAction(
                            grabber.pitchForward(),
                            elevator.elevate(ELE_BOT),
                            grabber.roll(90),
                            traj6.build()
                    ),
                    elevator.rotateDown(ROT_DOWN),
                    grabber.grab(),
                    elevator.rotateUp(ROT_UP),
                    new ParallelAction(
                            traj7.build(),
                            elevator.rotateUp(ROT_UP),
                            grabber.pitchUp(),
                            grabber.roll(0),
                            new SequentialAction(
                                new SleepAction(1),
                                elevator.elevate(ELE_BASKET_HIGH)
                            )
                    ),
                    // Release third sample
                    new SequentialAction(
                            grabber.pitchBackward(),
                            grabber.release()
                    ),
                    // Go park
                    new ParallelAction(
                            grabber.pitchBackward(),
                            elevator.elevateDown(400),
                            traj8.build()
                    ),
                    elevator.rotateDown(170)
            ));
        }
        if (position == Position.RIGHT) {
            Vector2d PRELOAD_CHAMBER_POSE = new Vector2d(-3, 29.4);
            Vector2d FIRST_SAMPLE_POSE = new Vector2d(-31, 40);
            double FIRST_SAMPLE_HEADING = Math.toRadians(-135);
            double FIRST_SAMPLE_DEPOSIT_HEADING = Math.toRadians(140);
            Vector2d SECOND_SAMPLE_POSE = new Vector2d(-40, 40);
            double SECOND_SAMPLE_HEADING = Math.toRadians(-140);
            Vector2d SECOND_SAMPLE_DEPOSIT_POSE = new Vector2d(-44, 50);
            double CLIP_HEADING = Math.toRadians(90);
            Vector2d HP_SPECIMEN_POSE = new Vector2d(-44, 58);
            Vector2d HP_SPECIMEN_CLIP = new Vector2d(-1, 29.4);
            Vector2d FIRST_SPECIMEN_POSE = new Vector2d(-44, 58);
            Vector2d FIRST_SPECIMEN_CLIP = new Vector2d(1, 29.4);
            Vector2d SECOND_SPECIMEN_POSE = new Vector2d(-44, 58);
            Vector2d SECOND_SPECIMEN_CLIP = new Vector2d(3, 29.4);
            Vector2d PARKING_POSE = new Vector2d(-40, 60);

            TrajectoryActionBuilder traj1, traj2, traj3, traj4, traj5, traj6, traj7, traj8, traj9, traj10, traj11, traj12;

            traj1 = drive.actionBuilder(rightInitialPose).strafeTo(PRELOAD_CHAMBER_POSE);
            traj2 = traj1.endTrajectory().fresh().splineTo(FIRST_SAMPLE_POSE, FIRST_SAMPLE_HEADING);
            traj3 = traj2.endTrajectory().fresh().turnTo(FIRST_SAMPLE_DEPOSIT_HEADING);
            traj4 = traj3.endTrajectory().fresh().strafeToLinearHeading(SECOND_SAMPLE_POSE, SECOND_SAMPLE_HEADING);
            traj5 = traj4.endTrajectory().fresh().strafeToLinearHeading(SECOND_SAMPLE_DEPOSIT_POSE, CLIP_HEADING);
            traj6 = traj5.endTrajectory().fresh().strafeTo(HP_SPECIMEN_POSE);
            traj7 = traj6.endTrajectory().fresh().strafeTo(HP_SPECIMEN_CLIP);
            traj8 = traj7.endTrajectory().fresh().strafeTo(FIRST_SPECIMEN_POSE);
            traj9 = traj8.endTrajectory().fresh().strafeTo(FIRST_SPECIMEN_CLIP);
            traj10 = traj9.endTrajectory().fresh().strafeTo(SECOND_SPECIMEN_POSE);
            traj11 = traj10.endTrajectory().fresh().strafeTo(SECOND_SPECIMEN_CLIP);
            traj12 = traj11.endTrajectory().fresh().strafeTo(PARKING_POSE);

            Actions.runBlocking(
                    new SequentialAction(
                            // Clip preload specimen
                            new ParallelAction(
                                    elevator.rotateUp(ROT_UP),
                                    elevator.elevate(ELE_CHAMBER_HIGH),
                                    grabber.pitchBackward(),
                                    grabber.grab(),
                                    traj1.build()
                            ),
                            elevator.elevate(ELE_CHAMBER_HIGH_DROP),
                            grabber.release(),
                            new ParallelAction(
                                    traj2.build(),
                                    elevator.rotateDown(ROT_DOWN)
                            ),
                            elevator.elevate(800),
                            grabber.grab(),
                            traj3.build(),
                            grabber.release(),
                            traj4.build(),
                            grabber.grab(),
                            // Go to observation zone
                            new ParallelAction(
                                    traj5.build(),
                                    elevator.rotateGrab(),
                                    grabber.pitchGrab()
                            ),
                            grabber.release(),
                            traj6.build(),
                            grabber.grab(),
                            // Go clip first specimen
                            new ParallelAction(
                                    traj7.build(),
                                    elevator.rotateUp(ROT_UP),
                                    elevator.elevate(ELE_CHAMBER_HIGH),
                                    grabber.pitchBackward(),
                                    grabber.roll(180)
                            ),
                            elevator.elevate(ELE_CHAMBER_HIGH_DROP),
                            grabber.release(),
                            // Go grab second specimen
                            new ParallelAction(
                                    traj8.build(),
                                    elevator.rotateGrab(),
                                    grabber.pitchGrab(),
                                    grabber.roll(0)
                            ),
                            grabber.grab(),
                            // Go clip second specimen
                            new ParallelAction(
                                    traj9.build(),
                                    elevator.rotateUp(ROT_UP),
                                    elevator.elevate(ELE_CHAMBER_HIGH),
                                    grabber.pitchBackward(),
                                    grabber.roll(180)
                            ),
                            elevator.elevate(ELE_CHAMBER_HIGH_DROP),
                            grabber.release(),
                            // Go grab third specimen
                            new ParallelAction(
                                    traj10.build(),
                                    elevator.rotateGrab(),
                                    grabber.pitchGrab(),
                                    grabber.roll(0)
                            ),
                            grabber.grab(),
                            // Go clip third specimen
                            new ParallelAction(
                                    traj11.build(),
                                    elevator.rotateUp(ROT_UP),
                                    elevator.elevate(ELE_CHAMBER_HIGH),
                                    grabber.pitchBackward(),
                                    grabber.roll(180)
                            ),
                            elevator.elevate(ELE_CHAMBER_HIGH_DROP),
                            grabber.release(),
                            // Park
                            new ParallelAction(
                                    traj12.build(),
                                    elevator.elevate(ELE_BOT)
                            )
                    )
            );
        }

        telemetry.addData("Path", "Execution complete");
        telemetry.update();
    }
}