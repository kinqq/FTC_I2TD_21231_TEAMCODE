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
                    .strafeToLinearHeading(new Vector2d(58, 58), Math.toRadians(-135))
                    .waitSeconds(1)
                    .strafeToLinearHeading(new Vector2d(49, 39.5), Math.toRadians(270))
                    .waitSeconds(1)
                    .strafeToLinearHeading(new Vector2d(58, 58), Math.toRadians(-135))
                    .waitSeconds(1)
                    .strafeToLinearHeading(new Vector2d(58.6, 39.5), Math.toRadians(270))
                    .waitSeconds(1)
                    .strafeToLinearHeading(new Vector2d(58, 58), Math.toRadians(-135))
                    .waitSeconds(1)
                    .strafeToLinearHeading(new Vector2d(57, 35), Math.toRadians(-45))
                    .waitSeconds(1)
                    .strafeToLinearHeading(new Vector2d(57, 57), Math.toRadians(-135))
                    .waitSeconds(1)
                    .splineTo(new Vector2d(28, 10), Math.toRadians(180))
                    .splineTo(new Vector2d(22, 10), Math.toRadians(180)) // 16.92
                    .waitSeconds(1)
                    .setTangent(0)
                    .splineTo(new Vector2d(28, 10), Math.toRadians(0))
                    .splineTo(new Vector2d(57, 57), Math.toRadians(45))

                    .build());
        }
        if (position == Position.RIGHT && alliance == Alliance.RED) {
            Vector2d PRELOAD_SPECIMEN_CLIP = new Vector2d(8, 28);
            Vector2d HP_SPECIMEN_POSE = new Vector2d(-37, 62.5);
            Vector2d HP_SPECIMEN_CLIP_CONTROL = new Vector2d(-5, 45);
            Vector2d HP_SPECIMEN_CLIP = new Vector2d(5, 27);
            Vector2d GRAB_CONTROL = new Vector2d(-8, 40);
            Vector2d FIRST_SPECIMEN_CLIP_CONTROL = new Vector2d(-9, 45);
            Vector2d FIRST_SPECIMEN_CLIP = new Vector2d(2, 27);
            Vector2d SECOND_SPECIMEN_CLIP_CONTROL = new Vector2d(-10, 45);
            Vector2d SECOND_SPECIMEN_CLIP = new Vector2d(-1, 27);
            Vector2d THIRD_SPECIMEN_CLIP_CONTROL = new Vector2d(-12, 45);
            Vector2d THIRD_SPECIMEN_CLIP = new Vector2d(-4, 27);
            Vector2d PARKING_POSE = new Vector2d(-40, 60);
            double PARKING_HEADING = Math.toRadians(180);

            myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-8, 63, Math.toRadians(-90)))
                    .strafeTo(PRELOAD_SPECIMEN_CLIP)
                    .waitSeconds(0.5)
                    .setTangent(Math.toRadians(90))
                    .splineToConstantHeading(new Vector2d(-36.5, 36), Math.toRadians(-90))
                    .splineToConstantHeading(new Vector2d(-36.5, 16), Math.toRadians(-90))
                    .splineToConstantHeading(new Vector2d(-45, 12), Math.toRadians(90))
//                    .waitSeconds(0)
                    .strafeTo(new Vector2d(-45, 52))
                    .waitSeconds(0)
                    .splineToConstantHeading(new Vector2d(-45, 25), Math.toRadians(-90))
                    .splineToConstantHeading(new Vector2d(-54, 15), Math.toRadians(90))
//                    .waitSeconds(0)
                    .strafeTo(new Vector2d(-54, 52))
                    .waitSeconds(0)
                    .splineToConstantHeading(new Vector2d(-54, 25), Math.toRadians(-90))
                    .splineToConstantHeading(new Vector2d(-62.5, 15), Math.toRadians(90))
//                    .waitSeconds(0)
                    .strafeTo(new Vector2d(-62.5, 62))

                    .waitSeconds(0.5)
                    .strafeToConstantHeading(HP_SPECIMEN_CLIP_CONTROL)
                    .splineToConstantHeading(HP_SPECIMEN_CLIP, Math.toRadians(-90))
                    .waitSeconds(0.5)
                    .strafeToConstantHeading(GRAB_CONTROL)
                    .splineToConstantHeading(HP_SPECIMEN_POSE, Math.toRadians(90))
                    .waitSeconds(0.5)
                    .strafeToConstantHeading(FIRST_SPECIMEN_CLIP_CONTROL)
                    .splineToConstantHeading(FIRST_SPECIMEN_CLIP, Math.toRadians(-90))
                    .waitSeconds(0.5)
                    .strafeToConstantHeading(GRAB_CONTROL)
                    .splineToConstantHeading(HP_SPECIMEN_POSE, Math.toRadians(90))
                    .waitSeconds(0.5)
                    .strafeToConstantHeading(SECOND_SPECIMEN_CLIP_CONTROL)
                    .splineToConstantHeading(SECOND_SPECIMEN_CLIP, Math.toRadians(-90))
                    .waitSeconds(0.5)
                    .strafeToConstantHeading(GRAB_CONTROL)
                    .splineToConstantHeading(HP_SPECIMEN_POSE, Math.toRadians(90))
                    .waitSeconds(0.5)
                    .strafeToConstantHeading(THIRD_SPECIMEN_CLIP_CONTROL)
                    .splineToConstantHeading(THIRD_SPECIMEN_CLIP, Math.toRadians(-90))
                    .waitSeconds(0.5)
                    .strafeToLinearHeading(PARKING_POSE, PARKING_HEADING)
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