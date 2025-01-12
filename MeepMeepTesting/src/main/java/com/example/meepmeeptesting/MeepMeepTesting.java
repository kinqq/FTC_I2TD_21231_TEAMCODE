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
                .setDimensions(17.4, 14.3)
                .build();

        Alliance alliance = Alliance.RED;
        Position position = Position.RIGHT;

        if (position == Position.LEFT && alliance == Alliance.RED) {
            myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(15, 63, Math.toRadians(90)))
                    .strafeTo(new Vector2d(3, 27))
                    .waitSeconds(1)
                    .strafeTo(new Vector2d(3, 40))
                    .strafeToLinearHeading(new Vector2d(49, 39.5), Math.toRadians(270))
                    .waitSeconds(1)
                    .strafeToLinearHeading(new Vector2d(59, 59), Math.toRadians(-135))
                    .waitSeconds(1)
                    .strafeToLinearHeading(new Vector2d(58.6, 39.5), Math.toRadians(270))
                    .waitSeconds(1)
                    .strafeToLinearHeading(new Vector2d(59, 59), Math.toRadians(-135))
                    .waitSeconds(1)
                    .strafeToLinearHeading(new Vector2d(52.5, 24.5), Math.toRadians(0))
                    .waitSeconds(1)
                    .strafeToLinearHeading(new Vector2d(59, 59), Math.toRadians(-135))
                    .waitSeconds(1)
                    .splineToLinearHeading(new Pose2d(22, 10, Math.toRadians(180)), Math.toRadians(180))
                    .build());
        }
        if (position == Position.RIGHT && alliance == Alliance.RED) {
            myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-15, 63, Math.toRadians(-90)))
                    .strafeTo(new Vector2d(-3, 34))
                    .setTangent(Math.toRadians(90))
                    .splineToConstantHeading(new Vector2d(-35, 36), Math.toRadians(-90))
                    .splineToConstantHeading(new Vector2d(-35, 12), Math.toRadians(-90))
                    .splineToConstantHeading(new Vector2d(-42, 12), Math.toRadians(90))
                    .splineToConstantHeading(new Vector2d(-42, 56), Math.toRadians(90))
                    .splineToConstantHeading(new Vector2d(-42, 25), Math.toRadians(-90))
                    .splineToConstantHeading(new Vector2d(-51, 15), Math.toRadians(90))
                    .splineToConstantHeading(new Vector2d(-51, 56), Math.toRadians(90))
                    .splineToConstantHeading(new Vector2d(-51, 25), Math.toRadians(-90))
                    .splineToConstantHeading(new Vector2d(-61, 15), Math.toRadians(90))
                    .splineToConstantHeading(new Vector2d(-61, 56), Math.toRadians(90))

                    .strafeToLinearHeading(new Vector2d(-37, 50), Math.toRadians(90))
                    .waitSeconds(0.5)
                    .strafeTo(new Vector2d(-1, 29.4))
                    .waitSeconds(0.5)
                    .strafeTo(new Vector2d(-37, 50))
                    .waitSeconds(0.5)
                    .strafeTo(new Vector2d(1, 29.4))
                    .waitSeconds(0.5)
                    .strafeTo(new Vector2d(-37, 50))
                    .waitSeconds(0.5)
                    .strafeTo(new Vector2d(3, 29.4))
                    .waitSeconds(0.5)
                    .strafeTo(new Vector2d(-37, 50))
                    .waitSeconds(0.5)
                    .strafeTo(new Vector2d(5, 29.4))
                    .waitSeconds(0.5)
                    .strafeToLinearHeading(new Vector2d(-34, 48), Math.toRadians(140))
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