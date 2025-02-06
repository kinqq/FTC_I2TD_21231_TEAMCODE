package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    enum Alliance {
        RED,
        BLUE
    }

    enum Position {
        LEFT,
        RIGHT
    }

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15.904)
                .setDimensions(15.4, 14.3)
                .build();

        Alliance alliance = Alliance.RED;
        Position position = Position.RIGHT;

        if (position == Position.LEFT && alliance == Alliance.RED) {
            myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(39, 62, Math.toRadians(-180)))
                    .strafeToLinearHeading(new Vector2d(59, 59), Math.toRadians(-135))
                    .waitSeconds(1)
                    .strafeToLinearHeading(new Vector2d(49, 39.5), Math.toRadians(270))
                    .waitSeconds(1)
                    .strafeToLinearHeading(new Vector2d(59, 59), Math.toRadians(-135))
                    .waitSeconds(1)
                    .strafeToLinearHeading(new Vector2d(58.6, 39.5), Math.toRadians(270))
                    .waitSeconds(1)
                    .strafeToLinearHeading(new Vector2d(59, 59), Math.toRadians(-135))
                    .waitSeconds(1)
                    .strafeToLinearHeading(new Vector2d(57, 35), Math.toRadians(-45))
                    .waitSeconds(1)
                    .strafeToLinearHeading(new Vector2d(59, 59), Math.toRadians(-135))
                    .waitSeconds(1)
                    .splineTo(new Vector2d(22, 10), Math.toRadians(180)) // 16.92
                    .waitSeconds(1)
                    .setTangent(Math.toRadians(0))
                    .splineTo(new Vector2d(59, 59), Math.toRadians(45))

                    .build());
        }
        if (position == Position.RIGHT && alliance == Alliance.RED) {
            myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-8, 63, Math.toRadians(-90)))
                    .strafeTo(new Vector2d(-0, 27.5))
                    .waitSeconds(0.5)
                    .setTangent(Math.toRadians(90))
                    .splineToConstantHeading(new Vector2d(-33, 36), Math.toRadians(-90))
                    .splineToConstantHeading(new Vector2d(-33, 12), Math.toRadians(-90))
                    .splineToConstantHeading(new Vector2d(-40, 12), Math.toRadians(90))
                    .waitSeconds(0)
                    .strafeTo(new Vector2d(-40, 56))
                    .waitSeconds(0)
                    .splineToConstantHeading(new Vector2d(-40, 25), Math.toRadians(-90))
                    .splineToConstantHeading(new Vector2d(-48, 15), Math.toRadians(90))
                    .waitSeconds(0)
                    .strafeTo(new Vector2d(-48, 56))
                    .waitSeconds(0)
                    .splineToConstantHeading(new Vector2d(-48, 25), Math.toRadians(-90))
                    .splineToConstantHeading(new Vector2d(-57, 15), Math.toRadians(90))
                    .waitSeconds(0)
                    .strafeTo(new Vector2d(-57, 62))

                    .waitSeconds(0.5)
                    .strafeToConstantHeading(new Vector2d(-10, 48))
                    .splineToConstantHeading(new Vector2d(-2.5, 33), Math.toRadians(-75))
                    .waitSeconds(0.5)
                    .strafeToConstantHeading(new Vector2d(-8, 40))
                    .splineToConstantHeading(new Vector2d(-37, 62), Math.toRadians(90))
                    .waitSeconds(0.5)
                    .strafeToConstantHeading(new Vector2d(-8.5, 48))
                    .splineToConstantHeading(new Vector2d(-1, 33), Math.toRadians(-75))
                    .waitSeconds(0.5)
                    .strafeToConstantHeading(new Vector2d(-8, 40))
                    .splineToConstantHeading(new Vector2d(-37, 62), Math.toRadians(90))
                    .waitSeconds(0.5)
                    .strafeToConstantHeading(new Vector2d(-7, 48))
                    .splineToConstantHeading(new Vector2d(0.5, 33), Math.toRadians(-75))
                    .waitSeconds(0.5)
                    .strafeToConstantHeading(new Vector2d(-8, 40))
                    .splineToConstantHeading(new Vector2d(-37, 62), Math.toRadians(90))
                    .waitSeconds(0.5)
                    .strafeToConstantHeading(new Vector2d(-5.5, 48))
                    .splineToConstantHeading(new Vector2d(2, 33), Math.toRadians(-75))
                    .waitSeconds(0.5)
                    .strafeToLinearHeading(new Vector2d(-40, 60), Math.toRadians(180))
                    .build());
        }
        if (position == Position.RIGHT && alliance == Alliance.BLUE) {
            myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(25, 60, Math.toRadians(270)))
                    .strafeTo(new Vector2d(8, 37))
                    .waitSeconds(2)
                    .strafeTo(new Vector2d(48, 39))
                    .waitSeconds(1)
                    .strafeToLinearHeading(new Vector2d(52, 52), Math.toRadians(45))
                    .waitSeconds(.5)
                    .strafeToLinearHeading(new Vector2d(8, 33), Math.toRadians(270))
                    .build());

        }
        if (position == Position.LEFT && alliance == Alliance.BLUE) {
            myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-25, 60, Math.toRadians(270)))
                    .strafeTo(new Vector2d(-4, 37))
                    .waitSeconds(2)
                    .strafeTo(new Vector2d(-48, 39))
                    .waitSeconds(1)
                    .strafeToLinearHeading(new Vector2d(52, 52), Math.toRadians(45))
                    .waitSeconds(.5)
                    .strafeToLinearHeading(new Vector2d(4, -33), Math.toRadians(270))
                    .build());
        }


        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}