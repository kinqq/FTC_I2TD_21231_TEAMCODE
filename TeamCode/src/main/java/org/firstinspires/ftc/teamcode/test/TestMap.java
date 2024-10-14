package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;

public class TestMap {
    static HardwareMap hwMap;

    static public DcMotorEx frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor;
    static public DcMotorEx eleLeftMotor, eleRightMotor, rotLeftMotor, rotRightMotor;

    public static void initTestRobot(HardwareMap hwMap) {

        frontLeftMotor = hwMap.get(DcMotorEx.class, "leftFront");
        frontRightMotor = hwMap.get(DcMotorEx.class, "rightFront");
        backLeftMotor = hwMap.get(DcMotorEx.class, "leftBack");
        backRightMotor = hwMap.get(DcMotorEx.class, "rightBack");

        eleLeftMotor = hwMap.get(DcMotorEx.class, "leftEle");
        eleRightMotor = hwMap.get(DcMotorEx.class, "rightEle");

        rotLeftMotor = hwMap.get(DcMotorEx.class, "leftRot");
        rotRightMotor = hwMap.get(DcMotorEx.class, "rightRot");

        frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
        backRightMotor.setDirection(DcMotor.Direction.REVERSE);

        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        eleLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        eleRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rotLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rotRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        eleLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        eleRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
}
