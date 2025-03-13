package org.firstinspires.ftc.teamcode.autonomous;


import static org.firstinspires.ftc.teamcode.util.Constants.*;

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.PinpointDrive;
import org.firstinspires.ftc.teamcode.subsystem.Elevator;
import org.firstinspires.ftc.teamcode.subsystem.Grabber;
import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous(group = "Auto")
public class AUTO_RIGHT extends LinearOpMode {
    @Override
    public void runOpMode() {
        Elevator elevator = new Elevator(hardwareMap);
        Grabber grabber = new Grabber(hardwareMap);
        Pose2d initialPose = new Pose2d(-8, 63, Math.toRadians(-90));

        PinpointDrive drive = new PinpointDrive(hardwareMap, initialPose);

        elevator.initEle();
        elevator.initRot();

        elevator.rotateTo(ROT_CLIP);

        grabber.grabber.setPosition(GRABBER_CLOSE);
        grabber.pitch.setPosition(PITCH_BACKWARD);
        grabber.pivot.setPosition(PIVOT_CLIP);
        grabber.roll.setPosition(ROLL_TICK_ON_ZERO + ROLL_TICK_PER_DEG * 180);

        // Wait until start and set up parameters
        while (!opModeIsActive() && !isStopRequested()) {
            if (gamepad2.left_trigger > 0.25) {
                grabber.grabber.setPosition(GRABBER_CLOSE);
            }
            if (gamepad2.right_trigger > 0.25) {
                grabber.grabber.setPosition(GRABBER_OPEN);
            }

            telemetry.addLine("-------Initialized-------");
            telemetry.addLine(">>>> Press ▶ to start. <<<<");
            telemetry.update();
        }

        Vector2d PRELOAD_SPECIMEN_CLIP = new Vector2d(10, 27);
        Vector2d HP_SPECIMEN_POSE = new Vector2d(-37, 61);
        Vector2d HP_SPECIMEN_CLIP_CONTROL = new Vector2d(-5, 50);
        Vector2d HP_SPECIMEN_CLIP = new Vector2d(7, 26);
        Vector2d GRAB_CONTROL = new Vector2d(-8, 40);
        Vector2d FIRST_SPECIMEN_CLIP_CONTROL = new Vector2d(-6, 50);
        Vector2d FIRST_SPECIMEN_CLIP = new Vector2d(4, 26);
        Vector2d SECOND_SPECIMEN_CLIP_CONTROL = new Vector2d(-7, 48);
        Vector2d SECOND_SPECIMEN_CLIP = new Vector2d(1, 26);
        Vector2d THIRD_SPECIMEN_CLIP_CONTROL = new Vector2d(-5.5, 48);
        Vector2d THIRD_SPECIMEN_CLIP = new Vector2d(-3, 26);
        Vector2d PARKING_POSE = new Vector2d(-40, 60);
        double PARKING_HEADING = Math.toRadians(180);

        TrajectoryActionBuilder traj1, traj2, traj3, traj4, traj5, traj6, traj7, traj8, traj9, traj10, traj11, traj12;

        traj1 = drive.actionBuilder(initialPose).strafeTo(PRELOAD_SPECIMEN_CLIP);
        traj2 = traj1.endTrajectory().fresh().setTangent(Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(-35, 36), Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(-35, 20), Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(-45, 12), Math.toRadians(90))
//                    .waitSeconds(0)
                .strafeTo(new Vector2d(-47, 52))
                    .waitSeconds(0)
                .splineToConstantHeading(new Vector2d(-45, 25), Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(-55, 15), Math.toRadians(90))
//                    .waitSeconds(0)
                .strafeTo(new Vector2d(-55, 52))
                    .waitSeconds(0)
                .splineToConstantHeading(new Vector2d(-55, 25), Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(-60.5, 15), Math.toRadians(90))
                .strafeTo(new Vector2d(-60.5, 62));
        traj3 = traj2.endTrajectory().fresh()
                .strafeToConstantHeading(HP_SPECIMEN_CLIP_CONTROL)
                .splineToConstantHeading(HP_SPECIMEN_CLIP, Math.toRadians(-90));
        traj4 = traj3.endTrajectory().fresh()
                .strafeToConstantHeading(GRAB_CONTROL)
                .splineToConstantHeading(HP_SPECIMEN_POSE, Math.toRadians(90));
        traj5 = traj4.endTrajectory().fresh()
                .strafeToConstantHeading(FIRST_SPECIMEN_CLIP_CONTROL)
                .splineToConstantHeading(FIRST_SPECIMEN_CLIP, Math.toRadians(-90));
        traj6 = traj5.endTrajectory().fresh()
                .strafeToConstantHeading(GRAB_CONTROL)
                .splineToConstantHeading(HP_SPECIMEN_POSE, Math.toRadians(90));
        traj7 = traj6.endTrajectory().fresh()
                .strafeToConstantHeading(SECOND_SPECIMEN_CLIP_CONTROL)
                .splineToConstantHeading(SECOND_SPECIMEN_CLIP, Math.toRadians(-90));
        traj8 = traj7.endTrajectory().fresh()
                .strafeToConstantHeading(GRAB_CONTROL)
                .splineToConstantHeading(HP_SPECIMEN_POSE, Math.toRadians(90));
        traj9 = traj8.endTrajectory().fresh()
                .strafeToConstantHeading(THIRD_SPECIMEN_CLIP_CONTROL)
                .splineToConstantHeading(THIRD_SPECIMEN_CLIP, Math.toRadians(-90));
        traj10 = traj9.endTrajectory().fresh()
                .strafeToLinearHeading(PARKING_POSE, PARKING_HEADING);

        Actions.runBlocking(
                new SequentialAction(
                        // Clip preload specimen
                        new ParallelAction(
                                elevator.rotate(ROT_CLIP + 15), // -20
                                elevator.elevate(ELE_CLIP + 20), // +75
                                grabber.readySpecimenClip(),
                                grabber.grab(),
                                traj1.build()
                        ),
                        new ParallelAction(
                                grabber.release(),
                                grabber.pitchUp()
                        ),
                        new ParallelAction(
                                traj2.build(),
                                grabber.readySpecimenGrab(),
                                elevator.elevate(ELE_CLIP-200),
                                new SequentialAction(
                                        new SleepAction(0.2),
                                        elevator.rotate(ROT_UP),
                                        elevator.elevate(ELE_BOT)
                                )
                        ),
                        grabber.grab(),
                        new ParallelAction(
                                traj3.build(),
                                elevator.elevate(ELE_CLIP), // -35
                                elevator.rotate(ROT_CLIP + 50), // -15
                                grabber.readySpecimenClip()
                        ),
                        new ParallelAction(
                                grabber.release(),
                                grabber.pitchUp()
                        ),
                        // Grab 1st Spec
                        new ParallelAction(
                                traj4.build(),
                                grabber.readySpecimenGrab(),
                                elevator.elevate(ELE_CLIP-200),
                                new SequentialAction(
                                        new SleepAction(0.2),
                                        elevator.rotate(ROT_UP),
                                        elevator.elevate(ELE_BOT)
                                )
                        ),
                        grabber.grab(),
                        new ParallelAction(
                                traj5.build(),
                                elevator.elevate(ELE_CLIP), // -45
                                elevator.rotate(ROT_CLIP + 54),
                                grabber.readySpecimenClip()
                        ),
                        new ParallelAction(
                                grabber.release(),
                                grabber.pitchUp()
                        ),
                        // Grab 2nd Spec
                        new ParallelAction(
                                traj6.build(),
                                grabber.readySpecimenGrab(),
                                elevator.elevate(ELE_CLIP-200),
                                new SequentialAction(
                                        new SleepAction(0.2),
                                        elevator.rotate(ROT_UP),
                                        elevator.elevate(ELE_BOT)
                                )
                        ),
                        grabber.grab(),
                        new ParallelAction(
                                traj7.build(),
                                elevator.elevate(ELE_CLIP), // -45
                                elevator.rotate(ROT_CLIP + 56),
                                grabber.readySpecimenClip()
                        ),
                        new ParallelAction(
                                grabber.release(),
                                grabber.pitchUp()
                        ),
                        new ParallelAction(
                                traj8.build(),
                                grabber.readySpecimenGrab(),
                                elevator.elevate(ELE_CLIP-200),
                                new SequentialAction(
                                        new SleepAction(0.2),
                                        elevator.rotate(ROT_UP),
                                        elevator.elevate(ELE_BOT)
                                )
                        ),
                        grabber.grab(),
                        new ParallelAction(
                                traj9.build(),
                                elevator.elevate(ELE_CLIP),
                                elevator.rotate(ROT_CLIP + 56),
                                grabber.readySpecimenClip()
                        ),
                        grabber.release(),
                        new ParallelAction(
                                traj10.build(),
                                grabber.readySpecimenGrab(),
                                elevator.elevate(ELE_CLIP-200),
                                new SequentialAction(
                                        new SleepAction(0.2),
                                        elevator.rotate(ROT_UP),
                                        elevator.elevate(ELE_BOT)
                                )
                        )
                )
        );

        telemetry.addData("Path", "Execution complete");
        telemetry.update();
    }
}