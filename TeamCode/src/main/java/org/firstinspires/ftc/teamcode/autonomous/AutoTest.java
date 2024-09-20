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
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        Action Traj1 = drive.actionBuilder(drive.pose)
                .strafeTo(new Vector2d(30, 30))
                .turn(Math.PI / 2)
                .build();

        // Wait for the game to start (driver presses START)
        waitForStart();

        Actions.runBlocking(new SequentialAction(
                new Action() {
                    @Override
                    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                        telemetry.addLine("Running Traj1...");
                        telemetry.update();
                        return Traj1.run(telemetryPacket); // Ensure Traj1 is run properly
                    }
                }
        ));

        telemetry.addData("Path", "Execution complete");
        telemetry.update();
    }
}
