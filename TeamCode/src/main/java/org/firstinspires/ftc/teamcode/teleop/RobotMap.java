package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.hardware.*;

import org.firstinspires.ftc.teamcode.subsystem.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystem.Elevator;
import org.firstinspires.ftc.teamcode.subsystem.Grabber;

public class RobotMap {
    static HardwareMap hwMap;

    static public Drivetrain drive;
    static public Elevator elevator;
    static public Grabber grabber;

    public static void initRobot(HardwareMap hardwareMap) {
        hwMap = hardwareMap;

        drive = new Drivetrain(hardwareMap);
        elevator = new Elevator(hardwareMap);
        grabber = new Grabber(hardwareMap);
    }
}