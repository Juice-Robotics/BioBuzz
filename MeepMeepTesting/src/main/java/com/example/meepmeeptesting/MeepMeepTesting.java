package com.example.meepmeeptesting;

import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import javax.imageio.ImageIO;
import java.awt.Image;
import java.io.IOException;
import java.util.Objects;

public class MeepMeepTesting {

    public static MeepMeep meepMeep = new MeepMeep(600);

    static RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
            .setDimensions(13.5, 17.5)
            .setConstraints(
                    60,
                    60,
                    Math.toRadians(180),
                    Math.toRadians(180),
                    15
            )
            .build();

    public static void main(String[] args) throws IOException {

        MeepMeepCloseBlue.closeBluePath();
        MeepMeepBlueGarden.blueGardenPath();
        MeepMeepFarBlue.farBluePath();

        Image bioBuzzField = ImageIO.read(
                Objects.requireNonNull(
                        MeepMeepTesting.class.getResource(
                                "/Juice-BIOBUZZ-Black.png"
                        )
                )
        );

        meepMeep.setBackground(bioBuzzField)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
//                .addEntity(MeepMeepCloseBlue.closeBlueBot)
//                .addEntity(MeepMeepFarBlue.farBlueBot)
                .addEntity(MeepMeepBlueGarden.blueGardenBot)
                .start();
    }
    }
