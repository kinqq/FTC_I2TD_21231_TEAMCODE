package org.firstinspires.ftc.teamcode.teleop;

import static org.firstinspires.ftc.teamcode.util.Constants.ARM_KP;
import static org.firstinspires.ftc.teamcode.util.Constants.ELE_KP;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.*;

import org.firstinspires.ftc.teamcode.util.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.util.LED;

import java.util.List;

public class RobotMap {

    static HardwareMap hwMap;
    static public Motor frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor;
    static public Motor eleLeftMotor, eleRightMotor, rotLeftMotor, rotRightMotor;
    static public ServoEx grabber;
    static public GoBildaPinpointDriver odo;
    static public MecanumDrive drive;
    static public MotorGroup eleMotors, rotMotors;

    public static void initRobot(HardwareMap hardwareMap) {
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
        frontLeftMotor = initMotor("leftFront", Motor.GoBILDA.RPM_312);
        frontRightMotor = initMotor("rightFront", Motor.GoBILDA.RPM_312);
        backLeftMotor = initMotor("leftBack", Motor.GoBILDA.RPM_312);
        backRightMotor = initMotor("rightBack", Motor.GoBILDA.RPM_312);

        frontLeftMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        odo = hwMap.get(GoBildaPinpointDriver.class, "odo");
        odo.setOffsets(0, 0);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        odo.resetPosAndIMU();

        drive = new MecanumDrive(frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor);
        drive.setRightSideInverted(true);
    }

    static void initElevator() {
        eleLeftMotor = initMotor("leftEle");
        eleRightMotor = initMotor("rightEle");

        eleMotors = new MotorGroup(eleLeftMotor, eleRightMotor);
        eleMotors.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
//        eleMotors.stopAndResetEncoder();
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

    static Motor initMotor(String id, Motor.GoBILDA type) {
        return new Motor(hwMap, id, type);
    }

    public static void rotate(int pos, double pow) {
        rotMotors.setRunMode(Motor.RunMode.PositionControl);

        rotMotors.setPositionCoefficient(ARM_KP);
        rotMotors.setTargetPosition(pos);
        rotMotors.set(0);
        rotMotors.setPositionTolerance(10);

        while (!rotMotors.atTargetPosition()) rotMotors.set(pow);

        rotMotors.stopMotor();
        rotMotors.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
    }

    public static void elevate(int pos) {
        eleMotors.setRunMode(Motor.RunMode.PositionControl);
        eleMotors.setPositionCoefficient(ELE_KP);
        eleMotors.setTargetPosition(pos);
        eleMotors.setPositionTolerance(10);
        while (!eleMotors.atTargetPosition()) {
            eleMotors.set(0.8);
        }
        eleMotors.stopMotor();
    }
}