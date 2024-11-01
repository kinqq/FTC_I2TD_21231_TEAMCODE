package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.config.Config;

@Config
public class Constants {
        public enum EleControlMode {
                CHAMBER,
                BASKET
        }

        // Grabber servo position
        public static double GRABBER_CLOSE = 0.006;
        public static double GRABBER_OPEN  = 0.325;

        // Elevator position
        //TODO: ELE_CHAMBER_POS and ELE_BASKET_POS
        public static int ELE_BOT  = 0;
        public static int ELE_LOW = 300;
        public static int ELE_HIGH_CHAMBER = 1150;
        public static int ELE_DROP_SPECIMEN = 500;
        public static int ELE_MID  = 1600;
        public static int ELE_HIGH = 2020;

        public static int ROT_UP = 0;
        public static int ROT_DOWN = 680;
        public static int ROT_GRAB = 840;

        public static double ARM_KP = 0.01;
        public static double ELE_KP = 0.01;

        // Drivetrain motor speed
        public static double SAFE_MODE = 0.5;
        public static double PRECISION_MODE = 0.25;
        public static double NORMAL_MODE = 1.0;
}
