package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepCloseBlue {

    static RoadRunnerBotEntity closeBlueBot = new DefaultBotBuilder(MeepMeepTesting.meepMeep)
            // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
            .setDimensions(12.5, 17.5)
            .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
            .build();

    public static void closeBluePath(){

        closeBlueBot.runAction(closeBlueBot.getDrive().actionBuilder(new Pose2d(-61.75,12.7, Math.toRadians(0)))

                .setTangent(Math.toRadians(180))
                        .waitSeconds(3.0)
                .strafeTo(new Vector2d(40, 12.7))

                        .turn(Math.toRadians(180))
                        .waitSeconds(3.0)
                        .turnTo(Math.toRadians(85.8517453477))
                .strafeTo(new Vector2d(43.2, 58.2))


                .build());

    }

}

