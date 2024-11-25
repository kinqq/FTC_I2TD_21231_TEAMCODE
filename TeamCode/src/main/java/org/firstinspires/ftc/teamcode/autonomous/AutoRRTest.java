package org.firstinspires.ftc.teamcode.autonomous;


import static org.firstinspires.ftc.teamcode.util.Constants.*;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Elevator;
import org.firstinspires.ftc.teamcode.Grabber;
import org.firstinspires.ftc.teamcode.MecanumDrive;
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

        Pose2d leftInitialPose = new Pose2d(15, 63, Math.toRadians(-90));
        Pose2d rightInitialPose = new Pose2d(-15, 63, Math.toRadians(-90));

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
            if (gamepad2.back) {
                elevator.initRot();
            }
            if (gamepad2.start) {
                elevator.initEle();
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
            Vector2d CHAMBER_POSE = new Vector2d(8, 32);
            Vector2d FIRST_SAMPLE_POSE = new Vector2d(50, 42);
            double FIRST_SAMPLE_HEADING = Math.toRadians(270);
            Vector2d BASKET_POSE = new Vector2d(59, 59);
            double BASKET_HEADING = Math.toRadians(-135);
            Vector2d SECOND_SAMPLE_POSE = new Vector2d(59, 42);
            double SECOND_SAMPLE_HEADING = Math.toRadians(270);
            Vector2d THIRD_SAMPLE_POSE = new Vector2d(54, 33);
            double THIRD_SAMPLE_HEADING = Math.toRadians(-30);
            Pose2d PARKING_POSE = new Pose2d(30, 10, Math.toRadians(180));
            double PARKING_TANGENT = Math.toRadians(180);

            TrajectoryActionBuilder traj1, traj2, traj3, traj4, traj5, traj6, traj7, traj8;

            traj1 = drive.actionBuilder(leftInitialPose).strafeTo(CHAMBER_POSE);
            traj2 = traj1.endTrajectory().fresh().strafeToLinearHeading(FIRST_SAMPLE_POSE, FIRST_SAMPLE_HEADING);
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
                            elevator.elevate(ELE_CHAMBER_HIGH),
                            elevator.rotateTo(160)
                    ),
                    grabber.release(),
                    // Go to first sample
                    new ParallelAction(
                            elevator.elevate(ELE_BOT),
                            traj2.build(),
                            new SequentialAction(
                                    new SleepAction(1.5),
                                    new ParallelAction(
                                            elevator.rotateDown(),
                                            elevator.elevate(600)
                                    )
                            )
                    ),
                    // Grab first sample and go to basket
                    grabber.grab(),
                    elevator.rotateUp(),
                    new ParallelAction(
                            traj3.build(),
                            elevator.elevate(ELE_BASKET_HIGH),
                            elevator.rotateUp(),
                            grabber.pitchUp()
                    ),
                    // Release first sample
                    new ParallelAction(
                            grabber.pitchBackward(),
                            grabber.release()
                    ),
                    // Go to second sample
                    new ParallelAction(
                            grabber.pitchForward(),
                            elevator.elevate(ELE_BOT),
                            traj4.build(),
                            new SequentialAction(
                                    new SleepAction(1),
                                    new ParallelAction(
                                            elevator.rotateDown(),
                                            elevator.elevate(600)
                                    )
                            )
                    ),
                    grabber.grab(),
                    elevator.rotateUp(),
                    new ParallelAction(
                            traj5.build(),
                            elevator.elevate(ELE_BASKET_HIGH),
                            elevator.rotateUp(),
                            grabber.pitchUp()
                    ),
                    // Release second sample
                    new ParallelAction(
                            grabber.pitchBackward(),
                            grabber.release()
                    ),
                    // Go to third sample
                    new ParallelAction(
                            grabber.pitchForward(),
                            elevator.elevate(ELE_BOT),
                            traj6.build(),
                            new SequentialAction(
                                    new SleepAction(1),
                                    new ParallelAction(
                                            elevator.rotateDown(),
                                            elevator.elevate(600)
                                    )
                            )
                    ),
                    grabber.grab(),
                    elevator.rotateUp(),
                    new ParallelAction(
                            traj7.build(),
                            elevator.elevate(ELE_BASKET_HIGH),
                            elevator.rotateUp(),
                            grabber.pitchUp()
                    ),
                    // Release third sample
                    new ParallelAction(
                            grabber.pitchBackward(),
                            grabber.release()
                    ),
                    // Go park
                    new ParallelAction(
                            grabber.pitchUp(),
                            elevator.elevate(ELE_BOT),
                            traj8.build(),
                            new SequentialAction(
                                    new SleepAction(2),
                                    elevator.rotateDown()
                            )
                    )
            ));
        }
        if (position == Position.RIGHT) {
            TrajectoryActionBuilder traj1, traj2, traj3, traj4, traj5, traj6, traj7, traj8, traj9, traj10, traj11, traj12, traj13, traj14, traj15, traj16;

            traj1 = drive.actionBuilder(rightInitialPose).strafeTo(new Vector2d(-8, 34));
            traj2 = traj1.endTrajectory().fresh().strafeToLinearHeading(new Vector2d(-31, 40), Math.toRadians(-130));
            traj3 = traj2.endTrajectory().fresh().turnTo(Math.toRadians(140));
            traj4 = traj3.endTrajectory().fresh().strafeToLinearHeading(new Vector2d(-40, 40), Math.toRadians(-140));
            traj5 = traj4.endTrajectory().fresh().turnTo(Math.toRadians(120));
            traj6 = traj5.endTrajectory().fresh().turnTo(Math.toRadians(-150));
            traj7 = traj6.endTrajectory().fresh().strafeToLinearHeading(new Vector2d(-44, 50), Math.toRadians(90));
            traj8 = traj7.endTrajectory().fresh().strafeToLinearHeading(new Vector2d(-44, 54), Math.toRadians(90));
            traj9 = traj8.endTrajectory().fresh().strafeToLinearHeading(new Vector2d(-8, 32), Math.toRadians(90));
            traj10 = traj9.endTrajectory().fresh().strafeToLinearHeading(new Vector2d(-44, 54), Math.toRadians(90));
            traj11 = traj10.endTrajectory().fresh().strafeToLinearHeading(new Vector2d(-8, 32), Math.toRadians(90));
            traj12 = traj11.endTrajectory().fresh().strafeToLinearHeading(new Vector2d(-44, 54), Math.toRadians(90));
            traj13 = traj12.endTrajectory().fresh().strafeToLinearHeading(new Vector2d(-8, 32), Math.toRadians(90));
            traj14 = traj13.endTrajectory().fresh().strafeToLinearHeading(new Vector2d(-44, 54), Math.toRadians(90));
            traj15 = traj14.endTrajectory().fresh().strafeToLinearHeading(new Vector2d(-8, 32), Math.toRadians(90));
            traj16 = traj15.endTrajectory().fresh().strafeToLinearHeading(new Vector2d(-34, 48), Math.toRadians(145));

            Actions.runBlocking(
                    new SequentialAction(
                            new ParallelAction(
                                    traj1.build(),
                                    elevator.elevate(ELE_CHAMBER_HIGH),
                                    elevator.rotateTo(160)
                            ),
                            grabber.release(),
                            new ParallelAction(
                                    elevator.elevate(ELE_BOT),
                                    traj2.build(),
                                    new SequentialAction(
                                            new SleepAction(1.5),
                                            new ParallelAction(
                                                    elevator.rotateDown(),
                                                    elevator.elevate(600)
                                            )
                                    )
                            ),
                            grabber.grab(),
                            new ParallelAction(
                                    traj3.build(),
                                    elevator.elevate(1600)
                            ),
                            grabber.release(),
                            new ParallelAction(
                                    traj4.build(),
                                    elevator.elevate(600)
                            ),
                            grabber.grab(),
                            new ParallelAction(
                                    traj5.build(),
                                    elevator.elevate(1600)
                            ),
                            grabber.release(),
                            new ParallelAction(
                                    traj6.build(),
                                    elevator.elevate(1000)
                            ),
                            grabber.grab(),
                            new ParallelAction(
                                    traj7.build(),
                                    elevator.rotateTo(ROT_GRAB),
                                    grabber.pitchTo(PITCH_GRAB)
                            ),
                            grabber.release(),
                            traj8.build(),
                            grabber.grab(),
                            new ParallelAction(
                                    traj9.build(),
                                    elevator.rotateUp(),
                                    elevator.elevate(ELE_CHAMBER_HIGH),
                                    grabber.pitchBackward(),
                                    grabber.roll(180)
                            ),
                            elevator.elevate(ELE_CHAMBER_HIGH_DROP),
                            grabber.release(),
                            new ParallelAction(
                                    traj10.build(),
                                    elevator.rotateTo(ROT_GRAB),
                                    grabber.pitchTo(PITCH_GRAB),
                                    grabber.roll(0)
                            ),
                            grabber.grab(),
                            new ParallelAction(
                                    traj11.build(),
                                    elevator.rotateUp(),
                                    elevator.elevate(ELE_CHAMBER_HIGH),
                                    grabber.pitchBackward(),
                                    grabber.roll(180)
                            ),
                            elevator.elevate(ELE_CHAMBER_HIGH_DROP),
                            grabber.release(),
                            new ParallelAction(
                                    traj12.build(),
                                    elevator.rotateTo(ROT_GRAB),
                                    grabber.pitchTo(PITCH_GRAB),
                                    grabber.roll(0)
                            ),
                            grabber.grab(),
                            new ParallelAction(
                                    traj13.build(),
                                    elevator.rotateUp(),
                                    elevator.elevate(ELE_CHAMBER_HIGH),
                                    grabber.pitchBackward(),
                                    grabber.roll(180)
                            ),
                            elevator.elevate(ELE_CHAMBER_HIGH_DROP),
                            grabber.release(),
                            new ParallelAction(
                                    traj14.build(),
                                    elevator.rotateTo(ROT_GRAB),
                                    grabber.pitchTo(PITCH_GRAB),
                                    grabber.roll(0)
                            ),
                            grabber.grab(),
                            new ParallelAction(
                                    traj15.build(),
                                    elevator.rotateUp(),
                                    elevator.elevate(ELE_CHAMBER_HIGH),
                                    grabber.pitchBackward(),
                                    grabber.roll(180)
                            ),
                            elevator.elevate(ELE_CHAMBER_HIGH_DROP),
                            grabber.release(),
                            new ParallelAction(
                                    traj16.build(),
                                    elevator.rotateDown(),
                                    elevator.elevate(1000)
                            )
                    )
            );
        }

        telemetry.addData("Path", "Execution complete");
        telemetry.update();
    }
}