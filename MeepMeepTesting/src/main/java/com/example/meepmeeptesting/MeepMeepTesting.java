package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static MeepMeep meepMeep = new MeepMeep(600);

    static RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
            // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
            .setDimensions(13.5, 17.5)
            .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
            .build();
    public static void main(String[] args) {
        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-53.15,46.85, Math.toRadians(-53)))
                .splineTo(new Vector2d(13, -30), Math.toRadians(-90))
                .waitSeconds(.1)
                .splineTo(new Vector2d(13, -49), Math.toRadians(-90))
                .build());

//        MeepMeepFarRed.farRedPath();
//        MeepMeepFarBlue.farBluePath();
//        MeepMeepCloseBlue.closeBluePath();
//        MeepMeepBlueSoham.blueSohamPath();
//        MeepMeepBlueGate.blueGatePath();

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}