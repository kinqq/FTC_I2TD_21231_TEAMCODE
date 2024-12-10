package org.firstinspires.ftc.teamcode.autonomous;


import static org.firstinspires.ftc.teamcode.util.Constants.*;

import com.acmerobotics.dashboard.config.Config;
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
    public Position position = Position.RIGHT;

    @Override
    public void runOpMode() {
        PinpointDrive drive = new PinpointDrive(hardwareMap, new Pose2d(0, 0, 0));
        Elevator elevator = new Elevator(hardwareMap);
        Grabber grabber = new Grabber(hardwareMap);

        Pose2d leftInitialPose = new Pose2d(15, 63, Math.toRadians(90));
        Pose2d rightInitialPose = new Pose2d(-15, 63, Math.toRadians(90));

        grabber.grab();
        grabber.pitchBackward();

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
            Vector2d CHAMBER_POSE = new Vector2d(3, 29.4);
            Pose2d FIRST_SAMPLE_POSE = new Pose2d(48, 39, Math.toRadians(270));
            double FIRST_SAMPLE_TANGENT = Math.toRadians(15);
            Vector2d BASKET_POSE = new Vector2d(59, 59);
            double BASKET_HEADING = Math.toRadians(-135);
            Vector2d SECOND_SAMPLE_POSE = new Vector2d(58.6, 39);
            double SECOND_SAMPLE_HEADING = Math.toRadians(270);
            Vector2d THIRD_SAMPLE_POSE = new Vector2d(58, 36.5);
            double THIRD_SAMPLE_HEADING = Math.toRadians(-45);
            Pose2d PARKING_POSE = new Pose2d(22, 10, Math.toRadians(180));
            double PARKING_TANGENT = Math.toRadians(180);

            TrajectoryActionBuilder traj1, traj2, traj3, traj4, traj5, traj6, traj7, traj8;

            traj1 = drive.actionBuilder(leftInitialPose).strafeTo(CHAMBER_POSE);
            traj2 = traj1.endTrajectory().fresh().splineToLinearHeading(FIRST_SAMPLE_POSE, FIRST_SAMPLE_TANGENT);
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
                            elevator.rotateUp(),
                            elevator.elevate(ELE_CHAMBER_HIGH)
                    ),
                    elevator.elevate(ELE_CHAMBER_HIGH_DROP),
                    grabber.release(),
                    // Go to first sample
                    new ParallelAction(
                            elevator.elevate(ELE_BOT),
                            traj2.build()
                    ),
                    elevator.rotateDown(),
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
                    new SequentialAction(
                            grabber.pitchBackward(),
                            grabber.release()
                    ),
                    // Go to second sample
                    new ParallelAction(
                            grabber.pitchForward(),
                            elevator.elevate(ELE_BOT),
                            traj4.build()
                    ),
                    elevator.rotateDown(),
                    grabber.grab(),
                    elevator.rotateUp(),
                    new ParallelAction(
                            traj5.build(),
                            elevator.elevate(ELE_BASKET_HIGH),
                            elevator.rotateUp(),
                            grabber.pitchUp()
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
                            traj6.build()
                    ),
                    elevator.rotateDown(),
                    grabber.grab(),
                    elevator.rotateUp(),
                    new ParallelAction(
                            traj7.build(),
                            elevator.elevate(ELE_BASKET_HIGH),
                            elevator.rotateUp(),
                            grabber.pitchUp()
                    ),
                    // Release third sample
                    new SequentialAction(
                            grabber.pitchBackward(),
                            grabber.release()
                    ),
                    // Go park
                    new ParallelAction(
                            grabber.pitchBackward(),
                            elevator.elevate(400),
                            traj8.build()
                    )
            ));
        }
        if (position == Position.RIGHT) {
            TrajectoryActionBuilder traj1, traj2, traj3, traj4, traj5, traj6, traj7, traj8, traj9, traj10, traj11;

            traj1 = drive.actionBuilder(rightInitialPose).strafeTo(new Vector2d(-3, 29.4));
            traj2 = traj1.endTrajectory().fresh().strafeToConstantHeading(new Vector2d(-15, 40))
                    .splineToConstantHeading(new Vector2d(-45, 10), Math.toRadians(90))
                    .splineToConstantHeading(new Vector2d(-45, 56), Math.toRadians(90))
                    .splineToConstantHeading(new Vector2d(-45, 25), Math.toRadians(-90))
                    .splineToConstantHeading(new Vector2d(-54, 15), Math.toRadians(90))
                    .splineToConstantHeading(new Vector2d(-54, 56), Math.toRadians(90))
                    .splineToConstantHeading(new Vector2d(-54, 25), Math.toRadians(-90))
                    .splineToConstantHeading(new Vector2d(-63, 15), Math.toRadians(90))
                    .splineToConstantHeading(new Vector2d(-63, 56), Math.toRadians(90));
            traj3 = traj2.endTrajectory().fresh().strafeToLinearHeading(new Vector2d(-44, 54), Math.toRadians(90));
            traj4 = traj3.endTrajectory().fresh().strafeToLinearHeading(new Vector2d(-3, 29.4), Math.toRadians(90));
            traj5 = traj4.endTrajectory().fresh().strafeToLinearHeading(new Vector2d(-44, 54), Math.toRadians(90));
            traj6 = traj5.endTrajectory().fresh().strafeToLinearHeading(new Vector2d(-3, 29.4), Math.toRadians(90));
            traj7 = traj6.endTrajectory().fresh().strafeToLinearHeading(new Vector2d(-44, 54), Math.toRadians(90));
            traj8 = traj7.endTrajectory().fresh().strafeToLinearHeading(new Vector2d(-3, 29.4), Math.toRadians(90));
            traj9 = traj8.endTrajectory().fresh().strafeToLinearHeading(new Vector2d(-44, 54), Math.toRadians(90));
            traj10 = traj9.endTrajectory().fresh().strafeToLinearHeading(new Vector2d(-3, 29.4), Math.toRadians(90));
            traj11 = traj10.endTrajectory().fresh().strafeToLinearHeading(new Vector2d(-34, 48), Math.toRadians(145));

            Actions.runBlocking(
                    new SequentialAction(
                            // Clip preload specimen
                            new ParallelAction(
                                    elevator.rotateUp(),
                                    elevator.elevate(ELE_CHAMBER_HIGH),
                                    grabber.pitchBackward(),
                                    grabber.grab(),
                                    traj1.build()
                            ),
                            elevator.elevate(ELE_CHAMBER_HIGH_DROP),
                            grabber.release(),
                            traj2.build(),
                            // Go to observation zone
                            new ParallelAction(
                                    traj3.build(),
                                    elevator.rotateGrab(),
                                    grabber.pitchGrab()
                            ),
                            grabber.grab(),
                            // Go clip first specimen
                            new ParallelAction(
                                    traj4.build(),
                                    elevator.rotateUp(),
                                    elevator.elevate(ELE_CHAMBER_HIGH),
                                    grabber.pitchBackward(),
                                    grabber.roll(180)
                            ),
                            elevator.elevate(ELE_CHAMBER_HIGH_DROP),
                            grabber.release(),
                            // Go grab second specimen
                            new ParallelAction(
                                    traj5.build(),
                                    elevator.rotateGrab(),
                                    grabber.pitchGrab(),
                                    grabber.roll(0)
                            ),
                            grabber.grab(),
                            // Go clip second specimen
                            new ParallelAction(
                                    traj6.build(),
                                    elevator.rotateUp(),
                                    elevator.elevate(ELE_CHAMBER_HIGH),
                                    grabber.pitchBackward(),
                                    grabber.roll(180)
                            ),
                            elevator.elevate(ELE_CHAMBER_HIGH_DROP),
                            grabber.release(),
                            // Go grab third specimen
                            new ParallelAction(
                                    traj7.build(),
                                    elevator.rotateGrab(),
                                    grabber.pitchGrab(),
                                    grabber.roll(0)
                            ),
                            grabber.grab(),
                            // Go clip third specimen
                            new ParallelAction(
                                    traj8.build(),
                                    elevator.rotateUp(),
                                    elevator.elevate(ELE_CHAMBER_HIGH),
                                    grabber.pitchBackward(),
                                    grabber.roll(180)
                            ),
                            elevator.elevate(ELE_CHAMBER_HIGH_DROP),
                            grabber.release(),
                            // Go grab fourth specimen
                            new ParallelAction(
                                    traj9.build(),
                                    elevator.rotateGrab(),
                                    grabber.pitchGrab(),
                                    grabber.roll(0)
                            ),
                            grabber.grab(),
                            // Go clip fourth specimen
                            new ParallelAction(
                                    traj10.build(),
                                    elevator.rotateUp(),
                                    elevator.elevate(ELE_CHAMBER_HIGH),
                                    grabber.pitchBackward(),
                                    grabber.roll(180)
                            ),
                            elevator.elevate(ELE_CHAMBER_HIGH_DROP),
                            grabber.release(),
                            // Park
                            new ParallelAction(
                                    traj11.build(),
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