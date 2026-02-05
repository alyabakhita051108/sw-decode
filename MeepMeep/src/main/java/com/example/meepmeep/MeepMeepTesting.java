package com.example.meepmeep;

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
                .setDimensions(16.929,17.717)
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(61.06, 12.27, Math.toRadians(150.16)))

                .waitSeconds(3)
                .splineTo(new Vector2d(34.3, 27.28), Math.toRadians(92.41))
                .splineTo(new Vector2d(34.4, 58.08), Math.toRadians(91.61))
                .waitSeconds(0.5)
                //ballintked
                .lineToYConstantHeading((48))
                .splineTo(new Vector2d(60.74, 11.02), Math.toRadians(-63.43))

                //shoot2
                .waitSeconds(3)
                .splineTo(new Vector2d(19.93, 24.63), Math.toRadians(132.74))
                .splineTo(new Vector2d(11.5, 58.24), Math.toRadians(88.81))
                .waitSeconds(0.5)

//                .splineTo(new Vector2d(54.18, 6.33), Math.toRadians(173.46))
//                .splineTo(new Vector2d(25.25, 12.27), Math.toRadians(181.24))
//                .splineTo(new Vector2d(11.3, 56.90), Math.toRadians(94.64))
                //ball 3 intaked
                .lineToYConstantHeading(48)
                .splineTo(new Vector2d(59.34, 9.77), Math.toRadians(-42.51))


                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
//                .addEntity(myBot)
                .start();
    }
}