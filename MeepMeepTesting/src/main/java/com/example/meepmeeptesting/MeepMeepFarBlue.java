package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepFarBlue {

    static RoadRunnerBotEntity farBlueBot = new DefaultBotBuilder(MeepMeepTesting.meepMeep)
            // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
            .setDimensions(12.5, 17.5)
            .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
            .build();

    public static void farBluePath(){

        farBlueBot.runAction(farBlueBot.getDrive().actionBuilder(new Pose2d(14.5,61, Math.toRadians(-65)))

                .splineTo(
                          new Vector2d(49, 13),
                          Math.toRadians(0)
                  )
                .waitSeconds(2.0)
                .splineTo(
                        new Vector2d(60, 17),
                        Math.toRadians(30)
                )
                .waitSeconds(2.0)
                .setReversed(true)
                .splineTo(
                        new Vector2d(-60, 13),
                        Math.toRadians(180)
                )
                .waitSeconds(2.0)
                .setReversed(false)
                .splineTo(
                        new Vector2d(-65, 62),
                        Math.toRadians(115)
                )
                .waitSeconds(2.0)
                .setReversed(true)
                .splineTo(
                        new Vector2d(-60, 13),
                        Math.toRadians(180)
                )
                .waitSeconds(2.0)
                .setReversed(false)
                .splineTo(
                        new Vector2d(23, 61.5),
                        Math.toRadians(30)
                )
                .build());

    }

}

