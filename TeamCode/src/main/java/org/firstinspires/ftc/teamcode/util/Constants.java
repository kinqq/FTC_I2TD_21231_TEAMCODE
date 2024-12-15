package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.config.Config;

@Config
public class Constants {
    public enum EleControlMode {
        CHAMBER,
        BASKET
    }

    // Grabber servo position
    public static double GRABBER_CLOSE = 0.415;
    public static double GRABBER_OPEN = 0.195;

    public static double PITCH_FORWARD = 0.98;
    public static double PITCH_UP = 0.65;
    public static double PITCH_BACKWARD = 0.38;
    public static double PITCH_GRAB = 0.78;
    public static double PITCH_CLIP = 0.57;

    public static double ROLL_TICK_ON_ZERO = 0.38;
    public static double ROLL_TICK_PER_DEG = 1.0 / 300;

    // Elevator position
    public static int ELE_BOT = 0;

    public static int ELE_CHAMBER_LOW = 400;
    public static int ELE_CHAMBER_LOW_DROP = 200;
    public static int ELE_CHAMBER_HIGH = 680;
    public static int ELE_CHAMBER_HIGH_DROP = 140;

    public static int ELE_BASKET_LOW = 1500;
    public static int ELE_BASKET_HIGH = 2200;

    public static int ELE_HANG = 870;

    public static int ROT_UP = -10; // Giving negative to maintain position (giving a constant negative power)
    public static int ROT_DOWN = 675;
    public static int ROT_GRAB = 430;
    public static int ROT_HANG_DOWN = 260;

    // Drivetrain motor speed
    public static double SAFE_MODE = 0.7;
    public static double PRECISION_MODE = 0.3;
    public static double NORMAL_MODE = 1.0;

    // Autonomous configs...
    public static double angleCoeff = 0.5; //angleCorrectionCoefficient
    public static double powerCoeff = 1;
}
