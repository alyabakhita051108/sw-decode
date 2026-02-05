package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.opencv.core.Mat;

import java.util.jar.Attributes;

@Autonomous(name = "auto2tengah")
public class auto2tengah extends LinearOpMode {
    private DcMotor turret, shooter, intake;
    private CRServo servo1;

    @Override
    public void runOpMode() throws InterruptedException {

            turret = hardwareMap.get(DcMotor.class, "turret");
            shooter = hardwareMap.get(DcMotor.class, "shooter");
            intake = hardwareMap.get(DcMotor.class, "intake");
            servo1 = hardwareMap.get(CRServo.class, "servo1");

        Pose2d beginPose = new Pose2d(
                new Vector2d(-53, 48),
                Math.toRadians(90)
        );

        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

        waitForStart();

        Action path = drive.actionBuilder(beginPose)
                //shoot 1
                .afterTime(0.0, () -> shooter.setPower(-1))
                .afterTime(0.0, () -> intake.setPower(1))
                .waitSeconds(3.0)
                .afterTime(0, () -> servo1.setPower(-1))
                .waitSeconds(0.5)
                .afterTime(0, () -> servo1.setPower(1))
                .waitSeconds(0.5)
                .afterTime(0, () -> servo1.setPower(0))
                .waitSeconds(3)
                .afterTime(0.0, () -> shooter.setPower(0))
                .afterTime(0.0, () -> intake.setPower(0))
                //maju ke 3 bola terdepan
                .splineTo(new Vector2d(41.67, 26.35), Math.toRadians(117.34))
                .afterTime(0.0, () -> intake.setPower(1))
                .splineTo(new Vector2d(34.13, 56.27), Math.toRadians(90.00))
                //ball 3 intaked
                .lineToYConstantHeading(48)
                .afterTime(0.0, () -> shooter.setPower(0))
                .splineTo(new Vector2d(59.34, 11.02), Math.toRadians(-63.66))
                .afterTime(0.0, () -> shooter.setPower(-1))
                .afterTime(0.0, () -> intake.setPower(1))
                .waitSeconds(3.0)
                .afterTime(0, () -> servo1.setPower(-1))
                .waitSeconds(1.0)
                .afterTime(0, () -> servo1.setPower(0))
                //shoot2
                .waitSeconds(3)
                .afterTime(0.0, () -> shooter.setPower(0))
                .splineTo(new Vector2d(54.18, 6.33), Math.toRadians(173.46))
                .splineTo(new Vector2d(25.25, 12.27), Math.toRadians(181.24))
                .splineTo(new Vector2d(11.3, 56.90), Math.toRadians(94.64))
                //ball 3 intaked
                .lineToYConstantHeading(48)
                .splineTo(new Vector2d(12.59, 40.26), Math.toRadians(-87.32))
                .splineTo(new Vector2d(60.27, 11.96), Math.toRadians(-11.56))
                //shoot3
                .waitSeconds(3)


                .build();

        Actions.runBlocking(new SequentialAction(path));

    }
}
