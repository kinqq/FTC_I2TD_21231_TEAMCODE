package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.config.Config;

@Config
public class Constants {
        // Grabber servo position
        public static double GRABBER_CLOSE = 0.006;
        public static double GRABBER_OPEN  = 0.325;

        // Elevator position
        public static int ELE_BOT  = 0;
        public static int ELE_LOW = 300;
        public static int ELE_HIGH_RUNG = 1150;
        public static int ELE_DROP_SPECIMEN = 500;
        public static int ELE_MID  = 1600;
        public static int ELE_HIGH = 2020;

        public static int ROT_UP = 0;
        public static int ROT_DOWN = -720;
        public static int ROT_GRAB = -1000;

        public static double ARM_KP = 0.01;
        public static double ELE_KP = 0.01;

//        public static double aS = 0;
//        public static double aCos = 0;
//        public static double aV = 0;
//        public static double aA = 0;
//
//        public static double eS = 0;
//        public static double eG = 0;
//        public static double eV = 0;
//        public static double eA = 0;




        // Drivetrain motor speed
        public static double SAFE_MODE = 0.5;
        public static double PRECISION_MODE = 0.25;
        public static double NORMAL_MODE = 1.0;
}
