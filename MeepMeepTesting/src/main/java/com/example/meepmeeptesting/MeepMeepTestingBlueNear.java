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

public class MeepMeepTestingBlueNear {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(100, 100, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-50, -50, Math.toRadians(-38-90)))

                // Drive to initial shooting position
                .strafeToConstantHeading(new Vector2d(-6, -6))
                .turn(Math.toRadians(40))
                .waitSeconds(2)
                .turn(Math.toRadians(47))

                // SHOOT 3 BALLS (pre-loaded) - shooting sequence with turns
                .waitSeconds(0.4)  // Shooter spin-up
                // Shot 1: Mid (already at -38°, shoot mid with 0° offset)
                .waitSeconds(0.9)
                // Shot 2: Front (turn -5° for front chamber)
                .turn(Math.toRadians(-5))
                .waitSeconds(0.9)
                // Shot 3: Back (turn +7° to get to +2° offset)
                .turn(Math.toRadians(7))
                .waitSeconds(0.9)

                // CYCLE 1: Drive to first stack
                .strafeToLinearHeading(new Vector2d(2, -11), Math.toRadians(-90))
                .strafeToConstantHeading(new Vector2d(2, -55))
                .waitSeconds(0.75)  // Intake
                .strafeToLinearHeading(new Vector2d(-6, -6), Math.toRadians(-38))

                // SHOOT 3 BALLS (cycle 1)
                .waitSeconds(0.4)  // Shooter spin-up
                .waitSeconds(0.9)  // Shot 1: Mid
                .turn(Math.toRadians(-5))
                .waitSeconds(0.9)  // Shot 2: Front
                .turn(Math.toRadians(7))
                .waitSeconds(0.9)  // Shot 3: Back

                // CYCLE 2: Drive to second stack
                .strafeToLinearHeading(new Vector2d(31, -11), Math.toRadians(-90))
                .strafeToConstantHeading(new Vector2d(34, -65))
                .waitSeconds(0.75)  // Intake
                .strafeToLinearHeading(new Vector2d(-6, -6), Math.toRadians(-38))

                // SHOOT 3 BALLS (cycle 2)
                .waitSeconds(0.4)
                .waitSeconds(0.9)  // Shot 1: Mid
                .turn(Math.toRadians(-5))
                .waitSeconds(0.9)  // Shot 2: Front
                .turn(Math.toRadians(7))
                .waitSeconds(0.9)  // Shot 3: Back

                .build());


        Image img = null;
        try {
            img = ImageIO.read(new File("C:\\Users\\alben\\Downloads\\decode-custom-field-images-meepmeep-compatible-printer-v0-xsjhmvxpoonf1.png"));
        }
        catch(IOException e) {
            System.out.println(e.toString());
        }

        meepMeep.setBackground(img)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}