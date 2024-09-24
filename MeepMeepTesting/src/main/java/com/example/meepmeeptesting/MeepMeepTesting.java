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
        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(30, -60, Math.toRadians(90)))
                        .strafeTo(new Vector2d(0, -45))
                        .waitSeconds(1.5)
                        .splineToSplineHeading(new Pose2d(new Vector2d(30, -35), Math.toRadians(0)), Math.toRadians(0))
                // robot stops here.. why?
                        .splineTo(new Vector2d(35, 15), Math.toRadians(35))
                        .waitSeconds(1.5)
                        .splineTo(new Vector2d(25, -50), Math.toRadians(180))
                        .splineTo(new Vector2d(-45, -50), Math.toRadians(225))
                        .waitSeconds(1.5)
                        .strafeToLinearHeading(new Vector2d(60, -60), Math.toRadians(90))
//                        .strafeTo(new Vector2d(60, -60))
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}