package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.*;

import org.firstinspires.ftc.teamcode.util.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.util.LED;

public class RobotMap {
    static public DcMotorEx frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor;
    static public DcMotorEx eleLeftMotor, eleRightMotor, rotLeftMotor, rotRightMotor;
    static public Servo grabber;
    static public GoBildaPinpointDriver odo;
    static public LED led;
    static public NormalizedColorSensor colorSensor;

    public static void initRobot(HardwareMap hwMap) {
        grabber = hwMap.get(Servo.class, "grabber");
        colorSensor = hwMap.get(NormalizedColorSensor.class, "racistSensor");
        odo = hwMap.get(GoBildaPinpointDriver.class, "odo");
        led = new LED(hwMap.get(RevBlinkinLedDriver.class, "led"));

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

        rotLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rotRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rotLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rotRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rotLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        rotRightMotor.setDirection(DcMotor.Direction.REVERSE);

        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

//      Not resetting encoder in case the elevator malfunctioned and didn't stopped at rest position.
//      eleLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//      eleRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

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
    }
}