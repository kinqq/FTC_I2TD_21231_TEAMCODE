package org.firstinspires.ftc.teamcode.teleop;

import static org.firstinspires.ftc.teamcode.util.Constants.GRABBER_CLOSE;
import static org.firstinspires.ftc.teamcode.util.Constants.GRABBER_OPEN;

import com.acmerobotics.roadrunner.ftc.GoBildaPinpointDriver;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.*;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.util.DcMotorGroup;
import org.firstinspires.ftc.teamcode.util.LED;

import java.util.List;

public class RobotMap {

    static HardwareMap hwMap;
    static public Motor frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor;
    static public DcMotorEx eleLeftMotor, eleRightMotor, rotLeftMotor, rotRightMotor;
    static public ServoEx grabber;
    static public Servo grabberPitch, grabberRoll;
    static public GoBildaPinpointDriver odo;
    static public MecanumDrive drive;
    static public DcMotorGroup eleMotors, rotMotors;

    public static void initRobot(HardwareMap hardwareMap) {
        List<LynxModule> hubs = hardwareMap.getAll(LynxModule.class);
        hwMap = hardwareMap;

        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        initDriveTrain();
        initElevator();
        initRotation();
        initServos();
    }

    static void initDriveTrain() {
//        frontLeftMotor = initMotor("leftFront", Motor.GoBILDA.RPM_312);
//        frontRightMotor = initMotor("rightFront", Motor.GoBILDA.RPM_312);
//        backLeftMotor = initMotor("leftBack", Motor.GoBILDA.RPM_312);
//        backRightMotor = initMotor("rightBack", Motor.GoBILDA.RPM_312);
        frontLeftMotor = initMotor("leftFront");
        frontRightMotor = initMotor("rightFront");
        backLeftMotor = initMotor("leftBack");
        backRightMotor = initMotor("rightBack");

        frontLeftMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        odo = hwMap.get(GoBildaPinpointDriver.class, "odo");
        odo.setOffsets(0, 0);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED);
        odo.resetPosAndIMU();
        odo.setPosition(new Pose2D(DistanceUnit.MM, 0, 0, AngleUnit.DEGREES, -45));

        drive = new MecanumDrive(frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor);
        drive.setRightSideInverted(true);
    }

    static void initElevator() {
        eleLeftMotor = initDcMotor("leftEle");
        eleRightMotor = initDcMotor("rightEle");

        eleLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        eleRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        eleMotors = new DcMotorGroup(eleLeftMotor, eleRightMotor);
        eleMotors.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        eleMotors.setTolerance(10);
    }

    static void initRotation() {
        rotLeftMotor = initDcMotor("leftRot");
        rotRightMotor = initDcMotor("rightRot");

        rotMotors = new DcMotorGroup(rotLeftMotor, rotRightMotor);
        rotMotors.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rotMotors.setTolerance(10);
    }

    static void initServos() {
        grabber = new SimpleServo(hwMap, "grabber", 0, 360 * 5);
        grabberPitch = hwMap.get(Servo.class , "pitch");
        grabberRoll = hwMap.get(Servo.class, "roll");
    }

    static Motor initMotor(String id) {
        return new Motor(hwMap, id);
    }

    static DcMotorEx initDcMotor(String id) {
        return hwMap.get(DcMotorEx.class, id);
    }

    public static void rotate(int pos, double pow) {
        rotMotors.setTargetPosition(pos);
        rotMotors.setPower(pow);
        rotMotors.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public static void elevate(int pos, double pow) {
        eleMotors.setTargetPosition(pos);
        eleMotors.setPower(pow);
        eleMotors.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public static void pitch(double pos) {
        grabberPitch.setPosition(pos);
    }

    public static void roll (double pos) {
        grabberRoll.setPosition(pos);
    }

    public static void grab() {
        grabber.setPosition(GRABBER_CLOSE);
    }

    public static void release() {
        grabber.setPosition(GRABBER_OPEN);
    }
 }