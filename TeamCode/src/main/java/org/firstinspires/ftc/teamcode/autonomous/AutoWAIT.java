package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import static org.firstinspires.ftc.teamcode.test.TestMap.*;

@Autonomous(group = "AutoWAIT")
public class AutoWAIT extends LinearOpMode {

    @Override
    public void runOpMode() {
        initTestRobot(hardwareMap);

        waitForStart();

        rotateWait(300, 0.4);
        sleep(500);
        drive.driveRobotCentric(0, 0, 0.3);
        sleep(200);
        drive.driveRobotCentric(0, 0, -0.3);
        sleep(200);
        drive.driveRobotCentric(0, 0, 0.3);
        sleep(200);
        drive.driveRobotCentric(0, 0, -0.3);
        sleep(200);
        drive.driveRobotCentric(0, 0, 0.3);
        sleep(200);
        drive.driveRobotCentric(0, 0, -0.3);
        sleep(200);
    }
}