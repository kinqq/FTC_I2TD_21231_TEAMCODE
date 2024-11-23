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
        Pose2d initialPose = new Pose2d(32, 63, Math.toRadians(90));

        PinpointDrive drive = new PinpointDrive(hardwareMap, initialPose);
        Elevator elevator = new Elevator(hardwareMap);
        Grabber grabber = new Grabber(hardwareMap);

        TrajectoryActionBuilder traj1 = drive.actionBuilder(initialPose).strafeTo(new Vector2d(10, 35));
        TrajectoryActionBuilder traj2 = traj1.endTrajectory().strafeToLinearHeading(new Vector2d(48, 42), Math.toRadians(270));

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
            telemetry.addData("Alliance", alliance);
            telemetry.addData("Position", position);
        }

        Actions.runBlocking(new ParallelAction(traj1.build(), elevator.elevate(ELE_CHAMBER_HIGH)));

        Actions.runBlocking(new SequentialAction(
                        elevator.elevate(ELE_CHAMBER_HIGH_DROP),
                        grabber.release(),
                        new ParallelAction(
                                traj2.build(),
                                elevator.rotateDown()
                        )
                
                )
        );


        telemetry.addData("Path", "Execution complete");
        telemetry.update();
    }
}