package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.config.Config;

@Config
public class Constants {
    public enum EleControlMode {
        CHAMBER,
        BASKET
    }

    // Grabber servo position
    public static double GRABBER_CLOSE = 0.5;
    public static double GRABBER_OPEN = 0.8;

    public static double PITCH_FORWARD = 0.98;
    public static double PITCH_PARALLEL = 0.75;
    public static double PITCH_UP = 0.6;
    public static double PITCH_BACKWARD = 0.38;
    public static double PITCH_GRAB = 0.75;
    public static double PITCH_CLIP = 0.84;
    public static double PITCH_SPECIMEN = 0.35;
    public static double PITCH_SUBMERSIBLE = 0.19;
    public static double PITCH_DOWN = 0.28;
    public static double PITCH_BASKET_READY = 0.47;
    public static double PITCH_BASKET = 0.67;

    public static double ROLL_TICK_ON_ZERO = 0.5;
    public static double ROLL_TICK_PER_DEG = 0.1 / 180;

    public static double PIVOT_SPECIMEN = 0.05;
    public static double PIVOT_SUBMERSIBLE = 0.4;
    public static double PIVOT_DOWN = 0.5;
    public static double PIVOT_BASKET = 0.41;
    public static double PIVOT_BASKET_READY = 0.47;
    public static double PIVOT_CLIP = 0.75;

    // Elevator position
    public static int ELE_BOT = 0;

    public static int ELE_CHAMBER_LOW = 400;
    public static int ELE_CHAMBER_HIGH = 680;
    public static int ELE_CHAMBER_HIGH_DROP = 180;

    public static int ELE_BASKET_LOW = 1500;
    public static int ELE_BASKET_HIGH = 2400;

    public static int ELE_HANG = 900;
    public static int ELE_CLIP = 850;

    public static int ROT_UP = -10; // Giving negative to maintain position (giving a constant negative power)
    public static int ROT_DOWN = 640;
    public static int ROT_GRAB = 440;
    public static int ROT_HANG_DOWN = 170;
    public static int ROT_CLIP = 240;
    public static int ROT_SPECIMEN = 200;

    // Drivetrain motor speed
    public static double SAFE_MODE = 0.7;
    public static double PRECISION_MODE = 0.3;
    public static double NORMAL_MODE = 1.0;

    // Autonomous configs...
    public static double angleCoeff = 0.5; //angleCorrectionCoefficient
    public static double powerCoeff = 1;
}
