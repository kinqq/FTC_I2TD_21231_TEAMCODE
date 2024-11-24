package org.firstinspires.ftc.teamcode.autonomous;


import static org.firstinspires.ftc.teamcode.util.Constants.*;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
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
        Pose2d initialPose = new Pose2d(15, 63, Math.toRadians(-90));

        PinpointDrive drive = new PinpointDrive(hardwareMap, initialPose);
        Elevator elevator = new Elevator(hardwareMap);
        Grabber grabber = new Grabber(hardwareMap);

        TrajectoryActionBuilder traj1, traj2, traj3, traj4, traj5, traj6, traj7, traj8, traj9;

        traj1 = drive.actionBuilder(initialPose).strafeTo(new Vector2d(8, 32));
        traj2 = traj1.endTrajectory().fresh().strafeToLinearHeading(new Vector2d(50, 42), Math.toRadians(270));
        traj3 = traj2.endTrajectory().fresh().strafeToLinearHeading(new Vector2d(60, 60), Math.toRadians(-135));
        traj4 = traj3.endTrajectory().fresh().strafeToLinearHeading(new Vector2d(58.6, 42), Math.toRadians(270));
        traj5 = traj4.endTrajectory().fresh().strafeToLinearHeading(new Vector2d(60, 60), Math.toRadians(-135));
        traj6 = traj5.endTrajectory().fresh().strafeToLinearHeading(new Vector2d(54, 33), Math.toRadians(-30));
        traj7 = traj6.endTrajectory().fresh().strafeToLinearHeading(new Vector2d(60, 60), Math.toRadians(-135));
        traj8 = traj7.endTrajectory().fresh().splineToLinearHeading(new Pose2d(30, 10, Math.toRadians(180)), Math.toRadians(180));


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
        }

//        Actions.runBlocking(new ParallelAction(traj1.build(), elevator.elevate(ELE_CHAMBER_HIGH)));
//        Actions.runBlocking(new SequentialAction(
//                        elevator.elevate(ELE_CHAMBER_HIGH_DROP),
//                        grabber.release(),
//                        new ParallelAction(
//                                traj2.build(),
//                                elevator.rotateDown()
//                        )
//
//                )
//        );
        Actions.runBlocking(new SequentialAction(
                new ParallelAction(
                        traj1.build(),
                        elevator.elevate(ELE_CHAMBER_HIGH),
                        elevator.rotateTo(160)
                ),
                grabber.release(),
                new ParallelAction(
                        elevator.elevate(ELE_BOT),
                        traj2.build()
                ),
                new ParallelAction(
                        elevator.rotateDown(),
                        elevator.elevate(600)
                ),
                grabber.grab(),
                elevator.rotateUp(),
                new ParallelAction(
                        traj3.build(),
                        elevator.elevate(ELE_BASKET_HIGH),
                        elevator.rotateUp()
                ),
                grabber.release(),
                new ParallelAction(
                        elevator.elevate(ELE_BOT),
                        traj4.build()
                ),
                elevator.rotateDown()
//                traj5.build(),
//                traj6.build(),
//                traj7.build(),
//                traj8.build()
        ));


        telemetry.addData("Path", "Execution complete");
        telemetry.update();
    }
}