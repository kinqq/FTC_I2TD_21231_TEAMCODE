package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.roadrunner.ftc.GoBildaPinpointDriver;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.Motor.GoBILDA;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;
// replace ;(semi-colon) with ;(greek question mark)


import java.util.List;

public class TestMap {
    static HardwareMap hwMap;
    static public Motor frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor;
    static public Motor eleLeftMotor, eleRightMotor, rotLeftMotor, rotRightMotor;
    static public MotorGroup eleMotors, rotMotors;
    static public ServoEx grabber;
    static public MecanumDrive drive;
    static public GoBildaPinpointDriver odo;

    public static void initTestRobot(HardwareMap hardwareMap) {
        hwMap = hardwareMap;

        List<LynxModule> hubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        initDriveTrain();
        initElevator();
        initRotation();
        initServos();
    }

    static void initDriveTrain() {
        frontLeftMotor = initMotor("leftFront", GoBILDA.RPM_312);
        frontRightMotor = initMotor("rightFront", GoBILDA.RPM_312);
        backLeftMotor = initMotor("leftBack", GoBILDA.RPM_312);
        backRightMotor = initMotor("rightBack", GoBILDA.RPM_312);

        frontLeftMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        odo = hwMap.get(GoBildaPinpointDriver.class, "odo");

        drive = new MecanumDrive(frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor);
        drive.setRightSideInverted(true);
    }

    static void initElevator() {
        eleLeftMotor = initMotor("leftEle");
        eleRightMotor = initMotor("rightEle");

        eleMotors = new MotorGroup(eleLeftMotor, eleRightMotor);
        eleMotors.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        eleMotors.stopAndResetEncoder();
    }

    static void initRotation() {
        rotLeftMotor = initMotor("leftRot");
        rotRightMotor = initMotor("rightRot");

        rotMotors = new MotorGroup(rotLeftMotor, rotRightMotor);
        rotMotors.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        rotMotors.stopAndResetEncoder();
    }

    static void initServos() {
        grabber = new SimpleServo(hwMap, "grabber", 0, 360 * 5);
    }

    static Motor initMotor(String id) {
        return new Motor(hwMap, id);
    }

    static Motor initMotor(String id, GoBILDA type) {
        return new Motor(hwMap, id, type);
    }

    public static void rotate(int pos, double pow) {
        rotMotors.setRunMode(Motor.RunMode.PositionControl);
        rotMotors.setPositionCoefficient(0.1);
        rotMotors.setTargetPosition(pos);
        rotMotors.setPositionTolerance(10);

        rotMotors.set(pow);

//        while (!rotLeftMotor.atTargetPosition()) {
//            rotMotors.set(pow);
//        }
//        rotMotors.stopMotor();
        rotMotors.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
    }

    public static void elevate(int pos) {
        eleMotors.setRunMode(Motor.RunMode.PositionControl);
        eleMotors.setPositionCoefficient(0.1);
        eleMotors.setTargetPosition(pos);
        eleMotors.setPositionTolerance(10);
        while (!eleMotors.atTargetPosition()) {
            eleMotors.set(0.8);
        }
        eleMotors.stopMotor();
    }

}
