
package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepBlueGardenKavin {

    static RoadRunnerBotEntity meepmeepblueGardenBot =
            new DefaultBotBuilder(MeepMeepTesting.meepMeep)
                    .setDimensions(15, 15)
                    .setConstraints(
                            60,
                            60,
                            Math.toRadians(180),
                            Math.toRadians(180),
                            15
                    )
                    .build();

    private static double rad(double degrees) {
        return Math.toRadians(degrees);
    }

    public static void kavinBlueGardenPath() {

        meepmeepblueGardenBot.runAction(
                meepmeepblueGardenBot.getDrive()
                        .actionBuilder(
                                new Pose2d(-59.5, 30, rad(150))
                        )
                        .waitSeconds(2)


                        .setTangent(rad(90))
                        .splineToLinearHeading(
                                new Pose2d(-62, 60, rad(90)),
                                rad(90)
                        )

                        .setTangent(rad(0))
                        .splineToLinearHeading(
                                new Pose2d(45, 52, rad(0)),
                                rad(0)
                        )


                        .setTangent(rad(-45))
                        .splineToLinearHeading(
                                new Pose2d(42, 20, rad(0)),
                                rad(0)
                        )

                        .lineToX(60)



                        .setTangent(rad(145))
                        .splineToLinearHeading(
                                new Pose2d(36, 50, rad(90)),
                                rad(90)
                        )
                        .lineToY(62)

                        .setTangent(rad(-140))
                        .splineToLinearHeading(
                                new Pose2d(-36, 38, rad(130)),
                                rad(-150)
                        )


                        .setTangent(rad(60))
                        .splineToLinearHeading(
                                new Pose2d(-24, 45, rad(90)),
                                rad(90)
                        )
                        .lineToY(58)

                        .setTangent(rad(-120))
                        .splineToLinearHeading(
                                new Pose2d(-36, 38, rad(130)),
                                rad(-150)
                        )


                        .setTangent(rad(-135))
                        .splineToLinearHeading(
                                new Pose2d(-62, 24, rad(-90)),
                                rad(-90)
                        )
                        .lineToY(12)

                        .setTangent(rad(90))
                        .splineToLinearHeading(
                                new Pose2d(-48, 48, rad(0)),
                                rad(0)
                        )

                        .splineToLinearHeading(
                                new Pose2d(36, 48, rad(0)),
                                rad(0)
                        )
                        .setTangent(rad(-90))
                        .splineToLinearHeading(
                                new Pose2d(56, 20, rad(0)),
                                rad(-90)
                        )


                        .setTangent(rad(90))
                        .splineToLinearHeading(
                                new Pose2d(48, 55, rad(-80)),
                                rad(90)
                        )

                        .build()
        );
    }
}
