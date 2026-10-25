package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepBlueFlower {

    static RoadRunnerBotEntity blueFlowerBot = new DefaultBotBuilder(MeepMeepTesting.meepMeep)
            // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
            .setDimensions(15, 15)
            .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
            .build();

    public static void blueFlowerPath(){

        blueFlowerBot.runAction(blueFlowerBot.getDrive().actionBuilder(new Pose2d(-40,63, Math.toRadians(90)))
                .setReversed(true)
                .splineTo(
                        new Vector2d(-43, 31),
                        Math.toRadians(315)
                )
                .waitSeconds(1.0)
                .setReversed(false)
                .splineTo(
                        new Vector2d(-60, 47),
                        Math.toRadians(150)
                )
                .splineTo(
                        new Vector2d(-64, 61),
                        Math.toRadians(100)
                )
                .waitSeconds(1.0)
                .setReversed(true)
                .splineToLinearHeading(
                        new Pose2d(43, 31, Math.toRadians(45)),
                        Math.toRadians(-45)
                )
                .waitSeconds(1.0)
                .setReversed(false)
                .splineTo(
                        new Vector2d(63, 31),
                        Math.toRadians(-40)
                )
                .waitSeconds(1.0)
                .setReversed(true)
                .splineTo(
                        new Vector2d(43, 31),
                        Math.toRadians(225)
                )
                .waitSeconds(1.0)
                .setTangent(Math.toRadians(135))
                .splineToLinearHeading(
                        new Pose2d(-17, 61, Math.toRadians(135)),
                        Math.toRadians(135)
                )
                .waitSeconds(1.0)
                .strafeTo(new Vector2d(-43, 31))
                .waitSeconds(1.0)
                .setTangent(Math.toRadians(30))
                .splineToLinearHeading(
                        new Pose2d(23, 58, Math.toRadians(75)),
                        Math.toRadians(30)
                )
                .build());
    }

}

