package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.controllers.RobotPoseController;
import org.firstinspires.ftc.teamcode.controllers.ShooterController;
import org.firstinspires.ftc.teamcode.controllers.ShooterRotatorController;
import org.opencv.core.Mat;

import java.util.jar.Attributes;

@Autonomous(name = "auto2")
public class auto2 extends LinearOpMode {
    private ElapsedTime opModeTime = new ElapsedTime();

    private DcMotorEx intake;
    private CRServo servo1;
    private static final double COUNTS_PER_REV = 28.0;
    private static final double SHOOTER_VELOCITY = 1100; // ticks/sec


    private RobotPoseController robotPoseController;
    private ShooterRotatorController turret;
    private ShooterController shooter;
    private MecanumDrive drive;

    private static final double P = 90;
    private static final double F = 17.6;
    // Set this to true for Red Alliance, false for Blue
    private boolean isBlue = false;

    public Pose2d reflect(double x, double y, double degrees) {
        return isBlue ? new Pose2d(x, -y, -degrees) : new Pose2d(x, y, degrees);
    }

    public Vector2d reflectV(double x, double y) {
        return isBlue ? new Vector2d(x, -y) : new Vector2d(x, y);
    }

    public double reflect(double value) {
        return isBlue ? -value : value;
    }

    private double DEGREE_OFFSET = 2;

    private boolean hasBeenReset = false;


    @Override
    public void runOpMode() throws InterruptedException {
        robotPoseController = new RobotPoseController(hardwareMap);
        turret =new ShooterRotatorController(hardwareMap, robotPoseController, "turret");
        shooter = new ShooterController(hardwareMap, "shooter");
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        servo1 = hardwareMap.get(CRServo.class, "servo1");


        while (true) {
            if (gamepad1.crossWasPressed()) {
                isBlue = !isBlue;
            }

            if (gamepad1.triangleWasPressed()) {
                robotPoseController.resetYaw();
                hasBeenReset = true;
                telemetry.addLine("ROBOT YAW JUST SET TO RESET!");
            }

            if (hasBeenReset) {
                telemetry.addLine("ROBOT YAW ALREADY BEEN RESET!");
            }

            telemetry.addData("ROBOT CURRENT HEADING",robotPoseController.getRobotYaw() );

            telemetry.addData("Current Team : ", isBlue ? "BLUE" : "RED");
            telemetry.addLine("press gamepad 1 triangle button to reset robot yaw if necessary.");


            telemetry.addData("Current Team : ", isBlue ? "BLUE" : "RED");
            telemetry.update();
            if (isStarted()) {
                break;
            }
        }

//        robotPoseController.resetYaw();

        Pose2d beginPose = reflect(
                61.06, 12.27,Math.toRadians(150.16)
        );

        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

//        Action path = drive.actionBuilder(beginPose)
//               // .afterTime(0.0, () -> shooter.setPower(-1))
//                .waitSeconds(3)
//                .splineTo(reflectV(34.3, 27.28), Math.toRadians(reflect(92.41)))
//                .splineTo(reflectV(34.4, 58.08), Math.toRadians(reflect(91.61)))
//                .waitSeconds(0.5)
//                //ballintked
//                .lineToYConstantHeading(reflect(48))
//                .splineTo(reflectV(60.74, 11.02), Math.toRadians(reflect(-63.43)))
//
//                //shoot2
//                .waitSeconds(3)
//                .splineTo(reflectV(19.93, 24.63), Math.toRadians(reflect(132.74)))
//                .splineTo(reflectV(11.5, 58.24), Math.toRadians(reflect(88.81)))
//                .waitSeconds(0.5)
//
////                .splineTo(new Vector2d(54.18, 6.33), Math.toRadians(173.46))
////                .splineTo(new Vector2d(25.25, 12.27), Math.toRadians(181.24))
////                .splineTo(new Vector2d(11.3, 56.90), Math.toRadians(94.64))gi
//                //ball 3 intaked
//                .lineToYConstantHeading(reflect(reflect(48)))
//                .splineTo(reflectV(59.34, 9.77), Math.toRadians(reflect(-42.51)))
//
//                .build();

        Action path = drive.actionBuilder(beginPose)
                // .afterTime(0.0, () -> shooter.setPower(-1))

                // BLM DI TEST
                .afterTime(0.0, () -> {
//                    turret.setTargetWorldAngle(158.9-DEGREE_OFFSET);

                    shooter.setTargetVelocity(SHOOTER_VELOCITY);
                })

                .afterTime(0.5, () -> intake.setPower(1))
//                .waitSeconds(1)
                .afterTime(1, () -> servo1.setPower(-1))
                .afterTime(1.5, () -> {
//                    turret.setTargetWorldAngle();
                    servo1.setPower(1);
                    intake.setPower(0);
                })
                .waitSeconds(2.5) // try 2

                .waitSeconds(3)
                .splineTo(reflectV(34.3, 27.28), Math.toRadians(reflect(92.41)))
                .splineTo(reflectV(34.4, 58.08), Math.toRadians(reflect(91.61)))
                .waitSeconds(0.5)
                //ballintked
                .lineToYConstantHeading(reflect(48))
                .splineTo(reflectV(34.4, 38.33), Math.toRadians(reflect(-90.00)))
                .splineTo(reflectV(57.95, 11.53), Math.toRadians(reflect(0.00)))

                //shoot2
                // BLM DI TEST
                .afterTime(0.0, () -> {
//                    turret.setTargetWorldAngle(reflect(158.9-DEGREE_OFFSET)); // di check dulu
                    intake.setPower(1);
                })
//                .waitSeconds(1)
                .afterTime(0.5, () -> servo1.setPower(-1))
                .afterTime(2, () -> {
//                    turret.setTargetWorldAngle();
                    servo1.setPower(1);
                    intake.setPower(0);
                })
                .waitSeconds(2) // try 1.5

                .waitSeconds(3)
                .splineTo(reflectV(34.77, 12.27), Math.toRadians(reflect(180.00)))
                .splineTo(reflectV(12.18, 30.42), Math.toRadians(reflect(90.00)))
                .splineTo(reflectV(12.18, 47.55), Math.toRadians(reflect(90.00)))
                .waitSeconds(0.5)

//                .splineTo(new Vector2d(54.18, 6.33), Math.toRadians(173.46))
//                .splineTo(new Vector2d(25.25, 12.27), Math.toRadians(181.24))
//                .splineTo(new Vector2d(11.3, 56.90), Math.toRadians(94.64))gi
                //ball 3 intaked
                .lineToYConstantHeading(reflect(47))
                .splineTo(reflectV(11.5, 33.85), Math.toRadians(reflect(270.00)))
                .splineTo(reflectV(30.51, 11.71), Math.toRadians(0.00))
                .splineTo(reflectV(60.24, 11.71), Math.toRadians(0.00))

                .afterTime(0.0, () -> {
//                    turret.setTargetWorldAngle(reflect(158.9-DEGREE_OFFSET)); // di check dulu
                    intake.setPower(1);
                })
//                .waitSeconds(1)
                .afterTime(0.5, () -> servo1.setPower(-1))
                .afterTime(2, () -> {
//                    turret.setTargetWorldAngle();
                    servo1.setPower(1);
                    intake.setPower(0);
                })

                .build();

        waitForStart();



//        Actions.runBlocking(new SequentialAction(path));
        if (isStopRequested()) return;

        // Build and execute the action


        while (opModeIsActive()) {
            opModeTime.reset();
            runBlocking(path);
        }
    }



    public void runBlocking(Action action) {
//        FtcDashboard dash = FtcDashboard.getInstance();
//        Canvas previewCanvas = new Canvas();
//        action.preview(previewCanvas);

        boolean running = true;
        while (running && !Thread.currentThread().isInterrupted()) {
            TelemetryPacket packet = new TelemetryPacket();
//            packet.fieldOverlay().getOperations().addAll(previewCanvas.getOperations());
            packet.put("time", opModeTime);
            shooter.update();

            robotPoseController.update();
            turret.update();
            turret.activate();

            running = action.run(packet);

            Pose2d pose = drive.localizer.getPose();

            telemetry.addData("Shooter Velocity", shooter.getVelocity());
            telemetry.update();
            packet.fieldOverlay().setStroke("#3F51B5");
            Drawing.drawRobot(packet.fieldOverlay(), pose);
            FtcDashboard.getInstance().sendTelemetryPacket(packet);


//            extendo.runAuto();
//            lifter.runAuto();
////            lifter.sendTelemetryAuto(packet);
//            dash.sendTelemetryPacket(packet);
        }
    }
}
