package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.roadrunner.ftc.GoBildaPinpointDriver;
import com.acmerobotics.roadrunner.ftc.GoBildaPinpointDriverRR;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;

public class RobotMap {
    static HardwareMap hwMap;

    static public DcMotorEx frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor, eleLeftMotor, eleRightMotor;
    static public IMU imu;
    static public GoBildaPinpointDriverRR odo;
    static public NormalizedColorSensor colorSensor;

    public static void initRobot(HardwareMap hwMap) {
        frontLeftMotor = hwMap.get(DcMotorEx.class, "leftFront");
        frontRightMotor = hwMap.get(DcMotorEx.class, "rightFront");
        backLeftMotor = hwMap.get(DcMotorEx.class, "leftBack");
        backRightMotor = hwMap.get(DcMotorEx.class, "rightBack");

        eleLeftMotor = hwMap.get(DcMotorEx.class, "leftEle");
        eleRightMotor = hwMap.get(DcMotorEx.class, "rightEle");

//        colorSensor = hwMap.get(NormalizedColorSensor.class, "racistSensor");

        frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
        backRightMotor.setDirection(DcMotor.Direction.REVERSE);

        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        eleLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        eleRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        eleLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        eleRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        eleLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        eleRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        imu = hwMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);

        odo = hwMap.get(GoBildaPinpointDriverRR.class, "odo");
    }
}