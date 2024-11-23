package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

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
        PinpointDrive drive = new PinpointDrive(hardwareMap, new Pose2d(32, 63, Math.toRadians(90)));

//         Action Traj2 = drive.actionBuilder(drive.pose)
//                    .lineToY(40)
//                        .turn(Math.toRadians(90))
//                        .lineToX(-40)
//                        .turn(Math.toRadians(90))
//                        .lineToY(-40)
//                        .turn(Math.toRadians(90))
//                        .lineToX(40)
//                        .turn(Math.toRadians(90))
//                 .build();

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

        Actions.runBlocking(drive.actionBuilder(new Pose2d(32, 63, Math.toRadians(90)))
                .strafeTo(new Vector2d(10, 35))
                .waitSeconds(1)
                .strafeToLinearHeading(new Vector2d(48, 42), Math.toRadians(270))
                .waitSeconds(1)
                .strafeToLinearHeading(new Vector2d(55, 55), Math.toRadians(-135))
                .waitSeconds(1)
                .strafeToLinearHeading(new Vector2d(58.6, 42), Math.toRadians(270))
                .waitSeconds(1)
                .strafeToLinearHeading(new Vector2d(55, 55), Math.toRadians(-135))
                .waitSeconds(1)
                .strafeToLinearHeading(new Vector2d(60, 35), Math.toRadians(-45))
                .waitSeconds(1)
                .strafeToLinearHeading(new Vector2d(55, 55), Math.toRadians(-135))
                .waitSeconds(1)
                .splineToSplineHeading(new Pose2d(25, 15, Math.toRadians(180)), Math.toRadians(180))
                .build()
        );

        telemetry.addData("Path", "Execution complete");
        telemetry.update();
    }
}