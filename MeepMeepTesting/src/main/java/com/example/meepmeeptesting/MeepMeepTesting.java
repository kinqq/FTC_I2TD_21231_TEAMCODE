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
//        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(30, -60, Math.toRadians(90)))
//                        .strafeTo(new Vector2d(0, -35))
//                        .waitSeconds(1.5)
//                        .strafeTo(new Vector2d(30, -45))
//                        .strafeToSplineHeading(new Vector2d(30, 20), Math.toRadians(35))
//                        .waitSeconds(1.5)
//                        .splineTo(new Vector2d(30, -45), Math.toRadians(180))
//                        .splineTo(new Vector2d(-45, -50), Math.toRadians(225))
//                .build());

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(40, -40, Math.toRadians(90)))
                        .splineTo(new Vector2d(30, 30), Math.PI / 3)
                .turn(4 * Math.PI / 6)
                .splineTo(new Vector2d(0, 0), 0)
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}