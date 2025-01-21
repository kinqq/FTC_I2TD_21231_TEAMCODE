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
public class AUTO_LEFT_0_4 extends LinearOpMode {
    @Override
    public void runOpMode() {
        Pose2d initialPose = new Pose2d(39, 62, Math.toRadians(-180));
        Elevator elevator = new Elevator(hardwareMap);
        Grabber grabber = new Grabber(hardwareMap);

        PinpointDrive drive = new PinpointDrive(hardwareMap, initialPose);

        elevator.initEle();
        elevator.initRot();

        grabber.grabber.setPosition(GRABBER_CLOSE);
        grabber.pitch.setPosition(PITCH_BACKWARD);
        grabber.roll.setPosition(ROLL_TICK_ON_ZERO);

        // Wait until start and set up parameters
        while (!opModeIsActive() && !isStopRequested()) {
            if (gamepad2.left_trigger > 0.25) {
                grabber.grabber.setPosition(GRABBER_CLOSE);
                grabber.pitch.setPosition(PITCH_BACKWARD);
            }
            if (gamepad2.right_trigger > 0.25) {
                grabber.grabber.setPosition(GRABBER_OPEN);
                grabber.pitch.setPosition(PITCH_BACKWARD);
            }

            telemetry.addLine("-------Initialized-------");
            telemetry.addLine(">>>> Press ▶ to start. <<<<");
            telemetry.update();
        }

        Vector2d FIRST_SAMPLE_POSE = new Vector2d(49, 39.5);
        double FIRST_SAMPLE_HEADING = Math.toRadians(270);
        Vector2d BASKET_POSE = new Vector2d(59.5, 59.5);
        double BASKET_HEADING = Math.toRadians(-135);
        Vector2d SECOND_SAMPLE_POSE = new Vector2d(58.6, 39.5);
        double SECOND_SAMPLE_HEADING = Math.toRadians(270);
        Vector2d THIRD_SAMPLE_POSE = new Vector2d(57, 35);
        double THIRD_SAMPLE_HEADING = Math.toRadians(-45);
        Vector2d SUBMERSIBLE_POSE = new Vector2d(22, 10);
        double SUBMERSIBLE_TANGENT = Math.toRadians(180);

        TrajectoryActionBuilder traj1, traj2, traj3, traj4, traj5, traj6, traj7, traj8, traj9;

        traj1 = drive.actionBuilder(initialPose).strafeToLinearHeading(BASKET_POSE, BASKET_HEADING);
        traj2 = traj1.endTrajectory().fresh().strafeToLinearHeading(FIRST_SAMPLE_POSE, FIRST_SAMPLE_HEADING);
        traj3 = traj2.endTrajectory().fresh().strafeToLinearHeading(BASKET_POSE, BASKET_HEADING);
        traj4 = traj3.endTrajectory().fresh().strafeToLinearHeading(SECOND_SAMPLE_POSE, SECOND_SAMPLE_HEADING);
        traj5 = traj4.endTrajectory().fresh().strafeToLinearHeading(BASKET_POSE, BASKET_HEADING);
        traj6 = traj5.endTrajectory().fresh().strafeToLinearHeading(THIRD_SAMPLE_POSE, THIRD_SAMPLE_HEADING);
        traj7 = traj6.endTrajectory().fresh().strafeToLinearHeading(BASKET_POSE, BASKET_HEADING);
        traj8 = traj7.endTrajectory().fresh().splineTo(SUBMERSIBLE_POSE, SUBMERSIBLE_TANGENT);
        traj9 = traj8.endTrajectory().fresh().setTangent(0).strafeToLinearHeading(BASKET_POSE, BASKET_HEADING);

        Actions.runBlocking(new SequentialAction(
                // Go to basket
                new ParallelAction(
                        traj1.build(),
                        elevator.rotateUp(ROT_UP),
                        grabber.pitchUp(),
                        new SequentialAction(
                                new SleepAction(0.3),
                                elevator.elevateUp(ELE_BASKET_HIGH)
                        )
                ),
                // Release preload sample
                new SequentialAction(
                        grabber.pitchBackward(),
                        grabber.release()
                ),
                // Go to first sample
                new ParallelAction(
                        elevator.elevateDown(ELE_BOT),
                        grabber.pitchForward(),
                        traj2.build(),
                        new SequentialAction(
                                new SleepAction(1),
                                elevator.rotateDown(ROT_DOWN)
                        )
                ),
                // Grab first sample and go to basket
                grabber.grab(),
                elevator.rotateUp(ROT_UP),
                new ParallelAction(
                        traj3.build(),
                        elevator.rotateUp(ROT_UP),
                        grabber.pitchUp(),
                        new SequentialAction(
                                new SleepAction(0.5),
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
                                new SleepAction(0.5),
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

        telemetry.addData("Path", "Execution complete");
        telemetry.update();
    }
}