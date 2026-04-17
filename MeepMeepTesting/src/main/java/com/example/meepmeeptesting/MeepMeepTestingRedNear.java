package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.awt.Image;
import java.io.File;
import java.io.IOException;

import javax.imageio.ImageIO;

public class MeepMeepTestingRedNear {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();


        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-50, 50, Math.toRadians(38)))
                .strafeToConstantHeading(new Vector2d(-9.3, 9.3))
                .waitSeconds(1.2)
                .turn(Math.toRadians(-5))
                .waitSeconds(1.2)
                .turn(Math.toRadians(10))
                .waitSeconds(1.2)
                .strafeToLinearHeading(new Vector2d(-11, 11), Math.toRadians(90))  // Y and heading flipped
                .strafeToConstantHeading(new Vector2d(-11, 50))
                .strafeToLinearHeading(new Vector2d(-9.3, 9.3), Math.toRadians(38))
                .waitSeconds(1.2)
                .turn(Math.toRadians(-5))
                .waitSeconds(1.2)
                .turn(Math.toRadians(10))
                .waitSeconds(1.2)
                .strafeToLinearHeading(new Vector2d(11, 11), Math.toRadians(90))
                .strafeToConstantHeading(new Vector2d(11, 50))
                .strafeToLinearHeading(new Vector2d(-9.3, 9.3), Math.toRadians(38))
                .waitSeconds(1.2)
                .turn(Math.toRadians(-5))
                .waitSeconds(1.2)
                .turn(Math.toRadians(10))
                .waitSeconds(1.2)
                .build());

        Image img = null;
        try {
            img = ImageIO.read(new File("C:\\Users\\alben\\Downloads\\decode-custom-field-images-meepmeep-compatible-printer-v0-xsjhmvxpoonf1.png"));
        } catch(IOException e) {
            System.out.println(e.toString());
        }

        meepMeep.setBackground(img)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}