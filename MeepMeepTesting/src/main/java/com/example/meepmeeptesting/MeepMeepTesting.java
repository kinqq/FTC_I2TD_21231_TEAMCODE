package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        // weird...
        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-30, -60, Math.toRadians(90)))

                        .splineTo(new Vector2d(0, -35), Math.toRadians(90))
                        .waitSeconds(2)
                        .strafeTo(new Vector2d(0, -50))
                        .waitSeconds(0.01)
                        .splineTo(new Vector2d(30, -50), Math.toRadians(315))
                        .waitSeconds(1)
                        .splineTo(new Vector2d(0, -35), Math.toRadians(90))

                        .waitSeconds(3.5)
                        .strafeTo(new Vector2d(0, -50))
                        .waitSeconds(0.01)
                        .splineTo(new Vector2d(30, -50), Math.toRadians(315))
                        .waitSeconds(1)
                        .splineTo(new Vector2d(0, -35), Math.toRadians(90))

                        .waitSeconds(3.5)
                        .strafeTo(new Vector2d(0, -50))
                        .waitSeconds(0.01)
                        .splineTo(new Vector2d(30, -50), Math.toRadians(315))
                        .waitSeconds(1)
                        .splineTo(new Vector2d(0, -35), Math.toRadians(90))

                        .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}