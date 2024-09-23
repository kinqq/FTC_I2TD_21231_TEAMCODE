package org.firstinspires.ftc.teamcode.autonomous;

import androidx.annotation.NonNull;

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
public class AutoTest extends LinearOpMode {
     @Override
    public void runOpMode() {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(40, -40, Math.PI / 2));
        Action Traj1 = drive.actionBuilder(drive.pose)
                .splineTo(new Vector2d(30, 30), Math.PI / 3)
                .turn(4 * Math.PI / 6)
                .splineTo(new Vector2d(0, 0), 0)
                .build();

         Action Traj2 = drive.actionBuilder(drive.pose)
                    .lineToY(40)
                        .turn(Math.toRadians(90))
                        .lineToX(-40)
                        .turn(Math.toRadians(90))
                        .lineToY(-40)
                        .turn(Math.toRadians(90))
                        .lineToX(40)
                        .turn(Math.toRadians(90))
                 .build();

        // Wait for the game to start (driver presses START)
        waitForStart();

        Actions.runBlocking(new SequentialAction(
//                new Action() {
//                    @Override
//                    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
//                        telemetry.addLine("Running Traj1...");
//                        telemetry.update();
//                        return Traj1.run(telemetryPacket); // Ensure Traj1 is run properly
//                    }
//                }
                Traj1
        ));

        telemetry.addData("Path", "Execution complete");
        telemetry.update();
    }
}