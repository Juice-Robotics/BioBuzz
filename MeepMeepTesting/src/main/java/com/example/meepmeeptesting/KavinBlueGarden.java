
package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class KavinBlueGarden {

    static RoadRunnerBotEntity kavinBlueGardenBot =
            new DefaultBotBuilder(MeepMeepTesting.meepMeep)
                    .setDimensions(15, 15)
                    .setConstraints(
                            60, 60,
                            Math.toRadians(180),
                            Math.toRadians(180),
                            15
                    )
                    .build();

    public static void kavinBlueGardenPath() {

        kavinBlueGardenBot.runAction(
                kavinBlueGardenBot.getDrive()
                        .actionBuilder(new Pose2d(
                                -59.5, 30, Math.toRadians(150)))

                        .waitSeconds(2)

                        .strafeToLinearHeading(
                                new Vector2d(-62, 60),
                                Math.toRadians(90))

                        .strafeToLinearHeading(
                                new Vector2d(0, 48),
                                Math.toRadians(0))
                        .strafeTo(new Vector2d(60, 20))

                        .waitSeconds(1)

                        .strafeToLinearHeading(
                                new Vector2d(36, 62),
                                Math.toRadians(90))

                        .strafeToLinearHeading(
                                new Vector2d(-36, 38),
                                Math.toRadians(130))

                        .waitSeconds(2)

                        .strafeToLinearHeading(
                                new Vector2d(-24, 58),
                                Math.toRadians(90))
                        .waitSeconds(1)

                        .strafeToLinearHeading(
                                new Vector2d(-36, 38),
                                Math.toRadians(130))
                        .waitSeconds(1)

                        .strafeTo(new Vector2d(-62, 12))

                        .strafeToLinearHeading(
                                new Vector2d(-24, 36),
                                Math.toRadians(0))
                        .strafeTo(new Vector2d(24, 36))
                        .strafeToLinearHeading(
                                new Vector2d(48, 12),
                                Math.toRadians(0))
                        .waitSeconds(1)


                        .strafeToLinearHeading(
                                new Vector2d(48, 55),
                                Math.toRadians(-80))

                        .build()
        );
    }
}
