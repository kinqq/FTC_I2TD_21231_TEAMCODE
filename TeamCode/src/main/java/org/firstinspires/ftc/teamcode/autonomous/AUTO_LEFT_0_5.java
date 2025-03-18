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

@Autonomous(group = "Auto")
public class AUTO_LEFT_0_5 extends LinearOpMode {
    @Override
    public void runOpMode() {
        Pose2d initialPose = new Pose2d(39, 62, Math.toRadians(-180));
        Elevator elevator = new Elevator(hardwareMap);
        Grabber grabber = new Grabber(hardwareMap);

        double xOffset = 0, yOffset = 0, hOffset = 0;

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

            if (gamepad1.dpad_left) xOffset -= 0.0004;
            if (gamepad1.dpad_right) xOffset += 0.0004;
            if (gamepad1.dpad_down) yOffset -= 0.0004;
            if (gamepad1.dpad_up) yOffset += 0.0004;
            if (gamepad1.b) hOffset -= 0.001;
            if (gamepad1.a) hOffset += 0.001;

            telemetry.addLine("-------Initialized-------");
            telemetry.addData("xOffset", xOffset + "in");
            telemetry.addData("yOffset", yOffset + "in");
            telemetry.addData("hOffset", hOffset + "deg");
            telemetry.addLine(">>>> Press ▶ to start. <<<<");
            telemetry.update();
        }

        Vector2d FIRST_SAMPLE_POSE = new Vector2d(47.3, 38.5);
        double FIRST_SAMPLE_HEADING = Math.toRadians(270);
        Vector2d BASKET_POSE = new Vector2d(57, 57);
        Pose2d BASKET_POSE_SPLINE = new Pose2d(56.5, 56.5, Math.toRadians(-135));
        double BASKET_HEADING = Math.toRadians(-135);
        double BASKET_TANGENT = Math.toRadians(45);
//        Vector2d SECOND_SAMPLE_POSE = new Vector2d(57.9, 38.5);
        Vector2d SECOND_SAMPLE_POSE = new Vector2d(57.5, 38.5);
        double SECOND_SAMPLE_HEADING = Math.toRadians(270);
        Vector2d THIRD_SAMPLE_POSE = new Vector2d(58.2, 35);
        double THIRD_SAMPLE_HEADING = Math.toRadians(-45);
        Pose2d SUBMERSIBLE_POSE = new Pose2d(23, 0 - xOffset, Math.toRadians(180));
        double SUBMERSIBLE_TANGENT = Math.toRadians(180);

        TrajectoryActionBuilder traj1, traj2, traj3, traj4, traj5, traj6, traj7, traj8, traj9;

        traj1 = drive.actionBuilder(initialPose).strafeToLinearHeading(BASKET_POSE, BASKET_HEADING);
        traj2 = traj1.endTrajectory().fresh().strafeToLinearHeading(FIRST_SAMPLE_POSE, FIRST_SAMPLE_HEADING);
        traj3 = traj2.endTrajectory().fresh().strafeToLinearHeading(BASKET_POSE, BASKET_HEADING);
        traj4 = traj3.endTrajectory().fresh().strafeToLinearHeading(SECOND_SAMPLE_POSE, SECOND_SAMPLE_HEADING);
        traj5 = traj4.endTrajectory().fresh().strafeToLinearHeading(BASKET_POSE, BASKET_HEADING);
        traj6 = traj5.endTrajectory().fresh().strafeToLinearHeading(THIRD_SAMPLE_POSE, THIRD_SAMPLE_HEADING);
        traj7 = traj6.endTrajectory().fresh().strafeToLinearHeading(BASKET_POSE, BASKET_HEADING);
        traj8 = traj7.endTrajectory().fresh()
                .setTangent(BASKET_HEADING)
                .splineToLinearHeading(SUBMERSIBLE_POSE, SUBMERSIBLE_TANGENT);
        traj9 = traj8.endTrajectory().fresh()
                .setTangent(0)
                .splineToLinearHeading(BASKET_POSE_SPLINE, BASKET_TANGENT);

        Actions.runBlocking(new SequentialAction(
                // Go to basket
                new ParallelAction(
                    traj1.build(),
                    elevator.rotate(ROT_UP),
                    grabber.basketReady(),
                    new SequentialAction(
                        new SleepAction(0.2),
                        elevator.elevate(ELE_BASKET_HIGH),
                        grabber.basketDepositReady()
                    )
                ),
                // Release preload sample
                grabber.basketDeposit(),

                // Go to first sample
                new ParallelAction(
                        traj2.build(),
                        elevator.elevate(ELE_BOT),
                        grabber.readySampleGrab()
                ),
                elevator.rotateDown(ROT_DOWN),
                new SleepAction(0.2),
                // Grab first sample and go to basket
                grabber.performSampleGrab(),
                new ParallelAction(
                        traj3.build(),
                        elevator.rotate(ROT_UP),
                        grabber.basketReady(),
                        new SequentialAction(
                                new SleepAction(0.6),
                                elevator.elevate(ELE_BASKET_HIGH),
                                grabber.basketDepositReady()
                        )
                ),
                // Release first sample
                grabber.basketDeposit(),
                // Go to second sample
                new ParallelAction(
                        traj4.build(),
                        grabber.readySampleGrab(),
                        elevator.elevate(ELE_BOT)
                ),
                elevator.rotateDown(ROT_DOWN),
                new SleepAction(0.2),
                grabber.performSampleGrab(),
                new ParallelAction(
                        traj5.build(),
                        elevator.rotate(ROT_UP),
                        grabber.basketReady(),
                        new SequentialAction(
                                new SleepAction(0.6),
                                elevator.elevate(ELE_BASKET_HIGH),
                                grabber.basketDepositReady()
                        )
                ),
                // Release second sample
                grabber.basketDeposit(),
                // Go to third sample
                new ParallelAction(
                        grabber.readySampleGrab(),
                        grabber.roll(-45),
                        elevator.elevate(ELE_BOT),
                        traj6.build()
                ),
                elevator.rotateDown(ROT_DOWN),
                grabber.performSampleGrab(),
                new ParallelAction(
                        traj7.build(),
                        elevator.rotate(ROT_UP),
                        grabber.basketReady(),
                        new SequentialAction(
                                new SleepAction(0.6),
                                elevator.elevate(ELE_BASKET_HIGH),
                                grabber.basketDepositReady()
                        )
                ),
                // Release third sample
                grabber.basketDeposit(),
                // Go submersible
                new ParallelAction(
                        traj8.build(),
                        grabber.readySampleGrab(),
                        grabber.roll(hOffset),
                        elevator.elevate(ELE_BOT),
                        new SequentialAction(
                                new SleepAction(2.4),
                                elevator.rotateDown(ROT_DOWN)
                        )
                ),
                elevator.elevate(ELE_BOT + (int)(yOffset * 83)),
                grabber.performSampleGrab(),
                elevator.elevate(ELE_BOT),
                new ParallelAction(
                        traj9.build(),
                        elevator.rotate(ROT_UP),
                        grabber.basketReady(),
                        new SequentialAction(
                                new SleepAction(0.6),
                                elevator.elevate(ELE_BASKET_HIGH),
                                grabber.basketDepositReady()
                        )
                ),
                grabber.basketDeposit(),
                elevator.elevate(ELE_BOT)
        ));

        telemetry.addData("Path", "Execution complete");
        telemetry.update();
    }
}