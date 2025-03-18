package org.firstinspires.ftc.teamcode.autonomous;


import static org.firstinspires.ftc.teamcode.util.Constants.*;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.Arclength;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.PosePath;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.command.ParallelRaceGroup;
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
        Pose2d initialPose = new Pose2d(8, 63, Math.toRadians(-90));

        PinpointDrive drive = new PinpointDrive(hardwareMap, initialPose);

        elevator.initEle();
        elevator.initRot();

        elevator.rotateTo(ROT_CLIP);

        grabber.grabber.setPosition(GRABBER_CLOSE);
        grabber.pitch.setPosition(PITCH_BACKWARD);
        grabber.pivot.setPosition(PIVOT_CLIP);
        grabber.roll.setPosition(ROLL_TICK_ON_ZERO + 180 * ROLL_TICK_PER_DEG);

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

        Vector2d PRELOAD_SPECIMEN_CLIP = new Vector2d(6, 27.5);
        Vector2d HP_SPECIMEN_POSE = new Vector2d(-37, 62.3);
        Vector2d HP_SPECIMEN_CLIP_CONTROL = new Vector2d(-5, 50);
        Vector2d HP_SPECIMEN_CLIP = new Vector2d(5, 26.5);
        Vector2d GRAB_CONTROL = new Vector2d(-8, 40);
        Vector2d FIRST_SPECIMEN_CLIP_CONTROL = new Vector2d(-6, 50);
        Vector2d FIRST_SPECIMEN_CLIP = new Vector2d(3, 26.5);
        Vector2d SECOND_SPECIMEN_CLIP_CONTROL = new Vector2d(-7, 48);
        Vector2d SECOND_SPECIMEN_CLIP = new Vector2d(1, 26.5);
        Vector2d THIRD_SPECIMEN_CLIP_CONTROL = new Vector2d(-5.5, 48);
        Vector2d THIRD_SPECIMEN_CLIP = new Vector2d(-1, 26.5);
        Vector2d PARKING_POSE = new Vector2d(-40, 60);
        double PARKING_HEADING = Math.toRadians(180);

        TrajectoryActionBuilder traj1, traj2, traj3, traj4, traj5, traj6, traj7, traj8, traj9, traj10, traj11, traj12, traj13, traj14, traj15, traj16;

        traj1 = drive.actionBuilder(initialPose).strafeTo(PRELOAD_SPECIMEN_CLIP);
        traj2 = traj1.endTrajectory().fresh().setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(-35, 36, Math.toRadians(-115)), Math.toRadians(-120));
        traj3 = traj2.endTrajectory().fresh()
            .turnTo(Math.toRadians(105));
        traj4 = traj3.endTrajectory().fresh()
            .turnTo(Math.toRadians(-135));
        traj5 = traj4.endTrajectory().fresh()
            .turnTo(Math.toRadians(135));
        traj6 = traj5.endTrajectory().fresh()
            .strafeToLinearHeading(new Vector2d(-40, 32), Math.toRadians(-140));
        traj7 = traj6.endTrajectory().fresh().turnTo(Math.toRadians(135));
        traj8 = traj7.endTrajectory().fresh().strafeToLinearHeading(HP_SPECIMEN_POSE, Math.toRadians(90));

        traj9 = traj8.endTrajectory().fresh()
                .strafeTo(HP_SPECIMEN_CLIP);
        traj10 = traj9.endTrajectory().fresh()
                .strafeTo(HP_SPECIMEN_POSE);
        traj11 = traj10.endTrajectory().fresh()
                .strafeTo(FIRST_SPECIMEN_CLIP);
        traj12 = traj11.endTrajectory().fresh()
                .strafeTo(HP_SPECIMEN_POSE);
        traj13 = traj12.endTrajectory().fresh()
                .strafeTo(SECOND_SPECIMEN_CLIP);
        traj14 = traj13.endTrajectory().fresh()
                .strafeTo(HP_SPECIMEN_POSE);
        traj15 = traj14.endTrajectory().fresh()
                .strafeTo(THIRD_SPECIMEN_CLIP);
        traj16 = traj15.endTrajectory().fresh()
                .strafeToLinearHeading(PARKING_POSE, PARKING_HEADING);

        Actions.runBlocking(
                new SequentialAction(
                        // Clip preload specimen
                        new ParallelAction(
                                elevator.rotate(ROT_CLIP + 15),
                                elevator.elevate(ELE_CLIP + 20),
                                grabber.readySpecimenClipFront(),
                                grabber.grab(),
                                traj1.build()
                        ),
                        new ParallelAction(
                                grabber.release(),
                                grabber.pitchUp()
                        ),
                        new ParallelAction(
                            traj2.build(),
                            elevator.elevate(ELE_CLIP - 110),
                            new SequentialAction(
                                elevator.rotate(ROT_UP),
                                elevator.elevate(ELE_BOT),
                                grabber.sweep()
                            ),
                            new SequentialAction(
                                new SleepAction(2),
                                new ParallelAction(
                                    elevator.rotate(ROT_DOWN),
                                    new SequentialAction(
                                        new SleepAction(0.3),
                                        elevator.elevate(540)
                                    )
                                )
                            )
                        ),
                        traj3.build(),
                        new ParallelAction(
                            traj4.build(),
                            new SequentialAction(
                                new SleepAction(0.4),
                                elevator.elevate(1300)
                            )
                        ),
                        traj5.build(),
                        new ParallelAction(
                            traj6.build(),
                            elevator.elevate(1100)
                        ),
                        elevator.elevate(1300),
                        traj7.build(),
                        new ParallelAction(
                            traj8.build(),
                            grabber.readySpecimenGrab(),
                            elevator.elevate(ELE_BOT),
                            elevator.rotate(180)
                        ),
                        grabber.grab(),
                        new ParallelAction(
                                traj9.build(),
                                elevator.rotate(ROT_UP),
                                elevator.elevate(130),
                                grabber.readySpecimenClip()
                        ),
                        grabber.performSpecimenClip(),
                        // Grab 1st Spec
                        new ParallelAction(
                                traj10.build(),
                                grabber.readySpecimenGrab(),
                                elevator.elevate(ELE_BOT),
                                elevator.rotate(180)
                        ),
                        grabber.grab(),
                        new ParallelAction(
                                traj11.build(),
                                elevator.elevate(130),
                                elevator.rotate(ROT_UP),
                                grabber.readySpecimenClip()
                        ),
                        grabber.performSpecimenClip(),
                        // Grab 2nd Spec
                        new ParallelAction(
                                traj12.build(),
                                grabber.readySpecimenGrab(),
                                elevator.elevate(ELE_BOT),
                                elevator.rotate(180)
                        ),
                        grabber.grab(),
                        new ParallelAction(
                                traj13.build(),
                                elevator.elevate(130),
                                elevator.rotate(ROT_UP),
                                grabber.readySpecimenClip()
                        ),
                        grabber.performSpecimenClip(),
                        new ParallelAction(
                                traj14.build(),
                                grabber.readySpecimenGrab(),
                                elevator.elevate(ELE_BOT),
                                elevator.rotate(180)
                        ),
                        grabber.grab(),
                        new ParallelAction(
                                traj15.build(),
                                elevator.elevate(130),
                                elevator.rotate(ROT_UP),
                                grabber.readySpecimenClip()
                        ),
                        grabber.performSpecimenClip(),
                        grabber.release(),
                        new ParallelAction(
                                traj16.build(),
                                grabber.readySampleGrab(),
                                elevator.elevate(ELE_BOT),
                                elevator.rotate(ROT_DOWN)
                        )
                )
        );

        telemetry.addData("Path", "Execution complete");
        telemetry.update();
    }
}