package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    static enum Alliance {
        RED,
        BLUE
    }

    static enum Position {
        LEFT,
        RIGHT
    }

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .setDimensions(16, 18)
                .build();

        Alliance alliance = Alliance.RED;
        Position position = Position.RIGHT;
        

        if (position == Position.RIGHT && alliance == Alliance.RED) {
            myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(25, -60, Math.toRadians(90)))
                    .strafeTo(new Vector2d(4, -37))
                    .waitSeconds(2)
                    .strafeTo(new Vector2d(48,-39))
                    .waitSeconds(1)
                    .strafeToLinearHeading(new Vector2d(-52,-52), Math.toRadians(225))
                    .waitSeconds(.5)
                    .strafeToLinearHeading(new Vector2d(4,-33), Math.toRadians(90))
                    .build());
        }
        if (position == Position.LEFT && alliance == Alliance.RED) {
            myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-25, -60, Math.toRadians(90)))
                    .strafeTo(new Vector2d(-8,-37))
                    .waitSeconds(2)
                    .strafeTo(new Vector2d(-48,-39))
                    .waitSeconds(1)
                    .strafeToLinearHeading(new Vector2d(-52,-52), Math.toRadians(225))
                    .waitSeconds(.5)
                    .strafeToLinearHeading(new Vector2d(-8,-33), Math.toRadians(90))
                    .build());
        }
        if (position == Position.RIGHT && alliance == Alliance.BLUE) {
            myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(25, 60, Math.toRadians(270)))
                    .strafeTo(new Vector2d(8,37))
                    .waitSeconds(2)
                    .strafeTo(new Vector2d(48,39))
                    .waitSeconds(1)
                    .strafeToLinearHeading(new Vector2d(52,52), Math.toRadians(45))
                    .waitSeconds(.5)
                    .strafeToLinearHeading(new Vector2d(8,33), Math.toRadians(270))
                    .build());

        }
        if (position == Position.LEFT && alliance == Alliance.BLUE) {
            myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-25, 60, Math.toRadians(270)))
                    .strafeTo(new Vector2d(-4, 37))
                    .waitSeconds(2)
                    .strafeTo(new Vector2d(-48,39))
                    .waitSeconds(1)
                    .strafeToLinearHeading(new Vector2d(52,52), Math.toRadians(45))
                    .waitSeconds(.5)
                    .strafeToLinearHeading(new Vector2d(4,-33), Math.toRadians(270))
                    .build());
        }


        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}