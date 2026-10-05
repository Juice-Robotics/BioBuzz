package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepBlueGarden {

    static RoadRunnerBotEntity blueGardenBot = new DefaultBotBuilder(MeepMeepTesting.meepMeep)
            // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
            .setDimensions(15, 15)
            .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
            .build();

    public static void blueGardenPath(){

        blueGardenBot.runAction(blueGardenBot.getDrive().actionBuilder(new Pose2d(-59.5,30, Math.toRadians(150)))
                // preload, tip 1
                .waitSeconds(2)

                // garden spikes
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(-62, 60, Math.toRadians(90)), Math.toRadians(90))

                // swap sides, go to flower
                .setTangent(Math.toRadians(-30))
                .splineToLinearHeading(new Pose2d(0, 48, Math.toRadians(0)), Math.toRadians(0))
                .splineTo(new Vector2d(60, 20), Math.toRadians(20))

                // shoot garden spike + flower, tip 2
                .waitSeconds(1)

                // collect teammate preloads
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(36, 62, Math.toRadians(90)), Math.toRadians(90))

                // run to left shoot spot
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(-36, 38, Math.toRadians(130)), Math.toRadians(180))

                // shoot teammate preloads
                .waitSeconds(2)

                // collect flower
                .setTangent(Math.toRadians(30))
                .splineToLinearHeading(new Pose2d(-24, 58, Math.toRadians(90)), Math.toRadians(90))
                .waitSeconds(1)

                // shoot flower, tip 3
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(-36, 38, Math.toRadians(130)), Math.toRadians(-150))
                .waitSeconds(1)

                // collect pollen on wall
                .splineTo(new Vector2d(-62, 12), Math.toRadians(-90))
                .setTangent(Math.toRadians(0))

                // swap side around hive, shoot wall pollen
                .setTangent(Math.toRadians(45))
                .splineToSplineHeading(new Pose2d(-24, 36, Math.toRadians(0)), Math.toRadians(0))
                .splineTo(new Vector2d(24, 36), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(48, 12, Math.toRadians(0)), Math.toRadians(-45))
                .waitSeconds(1)

                // collect wall pollen again, shoot wall pollen, tip 4
                //.lineToX(62)
                //.waitSeconds(1)
                //.lineToX(61)

                // park
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(48, 55, Math.toRadians(-80)), Math.toRadians(90))


                // total: hopefully 4 tip,

                .build());

    }

}

