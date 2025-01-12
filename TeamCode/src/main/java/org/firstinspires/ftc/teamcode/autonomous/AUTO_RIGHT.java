package org.firstinspires.ftc.teamcode.autonomous;


import static org.firstinspires.ftc.teamcode.util.Constants.*;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
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
public class AUTO_RIGHT extends LinearOpMode {
    @Override
    public void runOpMode() {
        Elevator elevator = new Elevator(hardwareMap);
        Grabber grabber = new Grabber(hardwareMap);
        Pose2d initialPose = new Pose2d(15, 63, Math.toRadians(-90));

        PinpointDrive drive = new PinpointDrive(hardwareMap, initialPose);

        elevator.initEle();
        elevator.initRot();

        grabber.grabber.setPosition(GRABBER_CLOSE);
        grabber.pitch.setPosition(PITCH_FORWARD);
        grabber.roll.setPosition(ROLL_TICK_ON_ZERO);

        // Wait until start and set up parameters
        while (!opModeIsActive() && !isStopRequested()) {
            if (gamepad2.left_trigger > 0.25) {
                grabber.grabber.setPosition(GRABBER_CLOSE);
                grabber.pitch.setPosition(PITCH_FORWARD);
            }
            if (gamepad2.right_trigger > 0.25) {
                grabber.grabber.setPosition(GRABBER_OPEN);
                grabber.pitch.setPosition(PITCH_FORWARD);
            }

            telemetry.addLine("-------Initialized-------");
            telemetry.addLine(">>>> Press ▶ to start. <<<<");
            telemetry.update();
        }

        Vector2d PRELOAD_SPECIMEN_CLIP = new Vector2d(-3, 34);
        Vector2d HP_SPECIMEN_POSE = new Vector2d(-37, 54);
        Vector2d HP_SPECIMEN_CLIP = new Vector2d(-1, 29.4);
        Vector2d FIRST_SPECIMEN_CLIP = new Vector2d(1, 29.4);
        Vector2d SECOND_SPECIMEN_CLIP = new Vector2d(3, 29.4);
        Vector2d THIRD_SPECIMEN_CLIP = new Vector2d(3, 29.4);
        Vector2d PARKING_POSE = new Vector2d(-40, 60);
        double PARKING_HEADING = Math.toRadians(140);

        TrajectoryActionBuilder traj1, traj2, traj3, traj4, traj5, traj6, traj7, traj8, traj9, traj10, traj11, traj12;

        traj1 = drive.actionBuilder(initialPose).strafeTo(PRELOAD_SPECIMEN_CLIP);
        traj2 = traj1.endTrajectory().fresh().setTangent(Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(-35, 36), Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(-35, 12), Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(-42, 12), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(-42, 56), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(-42, 25), Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(-51, 15), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(-51, 56), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(-51, 25), Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(-61, 15), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(-61, 56), Math.toRadians(90));
        traj3 = traj2.endTrajectory().fresh().strafeToLinearHeading(HP_SPECIMEN_POSE, Math.toRadians(90));
        traj4 = traj3.endTrajectory().fresh().strafeTo(HP_SPECIMEN_CLIP);
        traj5 = traj4.endTrajectory().fresh().strafeTo(HP_SPECIMEN_POSE);
        traj6 = traj5.endTrajectory().fresh().strafeTo(FIRST_SPECIMEN_CLIP);
        traj7 = traj6.endTrajectory().fresh().strafeTo(HP_SPECIMEN_POSE);
        traj8 = traj7.endTrajectory().fresh().strafeTo(SECOND_SPECIMEN_CLIP);
        traj9 = traj8.endTrajectory().fresh().strafeTo(HP_SPECIMEN_POSE);
        traj10 = traj9.endTrajectory().fresh().strafeTo(THIRD_SPECIMEN_CLIP);
        traj11 = traj10.endTrajectory().fresh().strafeToLinearHeading(PARKING_POSE, PARKING_HEADING);

        Actions.runBlocking(
                new SequentialAction(
                        // Clip preload specimen
                        new ParallelAction(
                                elevator.rotateDown(ROT_CLIP),
                                elevator.elevate(ELE_CLIP),
                                grabber.pitchClip(),
                                grabber.grab(),
                                traj1.build()
                        ),
                        grabber.release(),
                        new ParallelAction(
                                traj2.build(),
                                elevator.rotateUp(ROT_UP),
                                elevator.elevateDown(ELE_BOT)
                        ),
                        // Grab HP Spec
                        traj3.build(),
                        new ParallelAction(
                                elevator.rotateDown(ROT_GRAB),
                                grabber.pitchGrab(),
                                grabber.roll(0)
                        ),
                        grabber.grab(),
                        new ParallelAction(
                                traj4.build(),
                                elevator.elevateUp(ELE_CHAMBER_HIGH),
                                grabber.pitchBackward(),
                                grabber.roll(180)
                        ),
                        elevator.elevateDown(ELE_CHAMBER_HIGH_DROP),
                        grabber.release(),

                        // Grab 1st Spec
                        new ParallelAction(
                                traj5.build(),
                                elevator.rotateDown(ROT_GRAB),
                                grabber.pitchGrab(),
                                grabber.roll(0)
                        ),
                        grabber.grab(),
                        new ParallelAction(
                                traj6.build(),
                                elevator.elevateUp(ELE_CHAMBER_HIGH),
                                grabber.pitchBackward(),
                                grabber.roll(180)
                        ),
                        elevator.elevateDown(ELE_CHAMBER_HIGH_DROP),
                        grabber.release(),

                        // Grab 2nd Spec
                        new ParallelAction(
                                traj7.build(),
                                elevator.rotateDown(ROT_GRAB),
                                grabber.pitchGrab(),
                                grabber.roll(0)
                        ),
                        grabber.grab(),
                        new ParallelAction(
                                traj8.build(),
                                elevator.elevateUp(ELE_CHAMBER_HIGH),
                                grabber.pitchBackward(),
                                grabber.roll(180)
                        ),
                        elevator.elevateDown(ELE_CHAMBER_HIGH_DROP),
                        grabber.release(),

                        // Grab 3rd Spec
                        new ParallelAction(
                                traj9.build(),
                                elevator.rotateDown(ROT_GRAB),
                                grabber.pitchGrab(),
                                grabber.roll(0)
                        ),
                        grabber.grab(),
                        new ParallelAction(
                                traj10.build(),
                                elevator.elevateUp(ELE_CHAMBER_HIGH),
                                grabber.pitchBackward(),
                                grabber.roll(180)
                        ),
                        elevator.elevateDown(ELE_CHAMBER_HIGH_DROP),
                        grabber.release(),

                        // Park
                        new ParallelAction(
                                traj11.build(),
                                elevator.elevate(ELE_BOT),
                                grabber.pitchForward(),
                                grabber.release()
                        )
                )
        );

        telemetry.addData("Path", "Execution complete");
        telemetry.update();
    }
}