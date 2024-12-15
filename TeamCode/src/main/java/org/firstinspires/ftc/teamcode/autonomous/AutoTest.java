package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystem.Elevator;
import org.firstinspires.ftc.teamcode.subsystem.Grabber;
import org.firstinspires.ftc.teamcode.PinpointDrive;

@Autonomous(group = "Auto")
@Config
public class AutoTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        PinpointDrive drive = new PinpointDrive(hardwareMap, new Pose2d(0, 0, 0));
        Elevator elevator = new Elevator(hardwareMap);
        Grabber grabber = new Grabber(hardwareMap);

        waitForStart();

        Actions.runBlocking(new SequentialAction(grabber.roll(90), grabber.roll(0)));
    }
}