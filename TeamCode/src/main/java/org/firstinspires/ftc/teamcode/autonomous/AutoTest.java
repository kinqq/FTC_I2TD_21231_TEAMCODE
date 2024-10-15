package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous(group = "Test")
@Config
public class AutoTest extends LinearOpMode {
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
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(40, -40, Math.PI / 2));
        Action Traj1 = drive.actionBuilder(drive.pose)
                .splineTo(new Vector2d(30, 30), Math.PI / 3)
                .turn(4 * Math.PI / 6)
                .splineTo(new Vector2d(0, 0), 0)
                .build();

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

        Actions.runBlocking(new SequentialAction(
                Traj1
        ));

        telemetry.addData("Path", "Execution complete");
        telemetry.update();
    }
}