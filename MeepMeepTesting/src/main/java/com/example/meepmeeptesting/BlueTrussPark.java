package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.awt.Image;
import java.io.File;
import java.io.IOException;

import javax.imageio.ImageIO;

public class BlueTrussPark {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(650);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-62, -34, 0))
                                .forward(26)
                                .strafeLeft(40)
                                .lineToLinearHeading(new Pose2d(-34, 48, Math.toRadians(90)))
                                .back(7)
                                .splineToConstantHeading(new Vector2d(-58, 58), Math.toRadians(90))
                                .build()
                );
        Image img = null;
        try { img = ImageIO.read(new File("C:\\Users\\danie\\Desktop\\18228-Code-CentreStage-20232024\\MeepMeepTesting\\src\\main\\java\\com\\example\\meepmeeptesting\\resources\\bg\\field-2023-juice-dark.png")); }
        catch (IOException e) {}

        meepMeep.setBackground(img)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
