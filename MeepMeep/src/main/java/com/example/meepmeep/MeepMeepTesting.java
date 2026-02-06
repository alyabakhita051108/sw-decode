package com.example.meepmeep;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);



        RoadRunnerBotEntity strategyOne = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setDimensions(16.929,17.717)
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        strategyOne.runAction(strategyOne.getDrive().actionBuilder(new Pose2d(-57, 45,Math.toRadians(127.16)))
                .lineToYConstantHeading((44))
                .splineTo(new Vector2d(-10.35, 10.35), Math.toRadians((-45.00)))
                //shoot1
                .waitSeconds(2)

                //shoot 1 selesai terus jalan ke dua mulai
                .lineToYConstantHeading((7))
                .waitSeconds(1.0)
//                .afterTime(0.0, () -> turret.setPower(-1))
                .splineTo(new Vector2d(-10.24, 21.50), Math.toRadians((94.56)))
                .splineTo(new Vector2d(-11.80, 59.96), Math.toRadians((90.00)))
                .waitSeconds(0.5)
                .lineToYConstantHeading((54))
//                .afterTime(0.0, () -> turret.setPower(0))
                .splineTo(new Vector2d(-13.06, 13.99), Math.toRadians((270)))

                .waitSeconds(2)
                //shoot 2 selesai
                .splineTo(new Vector2d(12.12, 23.69), Math.toRadians((89.78)))
                .splineTo(new Vector2d(11.6, 57.3), Math.toRadians((89.78)))
                //.splineTo(new Vector2d(11.34, 55.27), Math.toRadians(89.27))
                .waitSeconds(0.5)
                .lineToYConstantHeading((48))
                .splineTo(new Vector2d(-12.74, 13.84), Math.toRadians((239.92)))
                .waitSeconds(0.5)
                .build());

        RoadRunnerBotEntity strategyTwo_ORI = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setDimensions(16.929,17.717)
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        strategyTwo_ORI.runAction(strategyTwo_ORI.getDrive().actionBuilder(new Pose2d(61., 12.27, Math.toRadians(150.16)))

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
        RoadRunnerBotEntity strategyTwo = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setDimensions(16.929,17.717)
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        strategyTwo.runAction(strategyTwo.getDrive().actionBuilder(new Pose2d(61.77, 12.27, Math.toRadians(180)))

//                .waitSeconds(3)
                .splineTo(new Vector2d(41.72, 15.79), Math.toRadians(129.81))
//                .waitSeconds(3)

                .splineTo(new Vector2d(34.3, 27.28), Math.toRadians(92.41))
                .splineTo(new Vector2d(34.4, 58.08), Math.toRadians(91.61))
                .waitSeconds(1.0)
                //ballintked
                .lineToYConstantHeading((48))
                .splineTo(new Vector2d(34.4, 38.33), Math.toRadians(-90.00))
                .splineTo(new Vector2d(57.95, 11.53), Math.toRadians(0.00))
                //shoot2
                .waitSeconds(3)
                .splineTo(new Vector2d(34.77, 12.27), Math.toRadians(180.00))
                .splineTo(new Vector2d(12.18, 30.42), Math.toRadians(90.00))
                .splineTo(new Vector2d(12.18, 47.55), Math.toRadians(90.00))
                .waitSeconds(1.0)

//                .splineTo(new Vector2d(54.18, 6.33), Math.toRadians(173.46))
//                .splineTo(new Vector2d(25.25, 12.27), Math.toRadians(181.24))
//                .splineTo(new Vector2d(11.3, 56.90), Math.toRadians(94.64))
                //ball 3 intaked
                .lineToYConstantHeading(47)
                .splineTo(new Vector2d(11.5, 33.85), Math.toRadians(270.00))
                .splineTo(new Vector2d(30.51, 11.71), Math.toRadians(0.00))
                .splineTo(new Vector2d(60.24, 11.71), Math.toRadians(0.00))

                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
//                .addEntity(strategyOne)
//                .addEntity(strategyTwo_ORI)
                .addEntity(strategyTwo)
                .start();
    }
}