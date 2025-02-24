package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(50, 50, Math.toRadians(180), Math.toRadians(180), 15)
                .setDimensions(18,18)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(24, 62, Math.toRadians(270)))
                .strafeToLinearHeading(new Vector2d(56,56),Math.toRadians(45))
                .waitSeconds(0.5)
                .strafeToLinearHeading(new Vector2d(48,38),Math.toRadians(90))
                .waitSeconds(0.5)
                .strafeToLinearHeading(new Vector2d(56,56),Math.toRadians(45))
                .waitSeconds(0.5)
                .strafeToLinearHeading(new Vector2d(58,38),Math.toRadians(90))
                .waitSeconds(0.5)
                .strafeToLinearHeading(new Vector2d(56,56),Math.toRadians(45))
                .waitSeconds(0.5)
                .strafeToLinearHeading(new Vector2d(56,25),Math.toRadians(180))
                .waitSeconds(0.5)
                .strafeToLinearHeading(new Vector2d(56,56),Math.toRadians(45))
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}