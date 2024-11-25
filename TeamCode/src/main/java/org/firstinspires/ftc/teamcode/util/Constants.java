package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.config.Config;

@Config
public class Constants {
    public enum EleControlMode {
        CHAMBER,
        BASKET
    }

    // Grabber servo position
    public static double GRABBER_CLOSE = 0.21;
    public static double GRABBER_OPEN = 0;

    public static double PITCH_FORWARD = 0.498;
    public static double PITCH_UP = 0.555;
    public static double PITCH_BACKWARD = 0.612; // 0.376 for the other way
    public static double PITCH_GRAB = 0.54;

    public static double ROLL_NEG_45 = 0.5;
    public static double ROLL_45 = 0.1436;
    public static double ROLL_0 = 0.3026;
    public static double ROLL_90 = 0.0;
    public static double ROLL_180 = 0.9542;

    // Elevator position
    public static int ELE_BOT = 0;

    public static int ELE_CHAMBER_LOW = 600;
    public static int ELE_CHAMBER_LOW_DROP = 200;
    public static int ELE_CHAMBER_HIGH = 1200;
    public static int ELE_CHAMBER_HIGH_DROP = 300;

    public static int ELE_BASKET_LOW = 1500;
    public static int ELE_BASKET_HIGH = 3500;

    public static int ROT_UP = 0;
    public static int ROT_DOWN = 655;
    public static int ROT_GRAB = 490;

    // Drivetrain motor speed
    public static double SAFE_MODE = 0.7;
    public static double PRECISION_MODE = 0.3;
    public static double NORMAL_MODE = 1.0;

    // Autonomous configs...
    public static double angleCoeff = 0.5; //angleCorrectionCoefficient
    public static double powerCoeff = 1;
}
