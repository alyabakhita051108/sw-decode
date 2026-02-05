package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.controllers.RobotPoseController;
import org.firstinspires.ftc.teamcode.controllers.ShooterController;
import org.firstinspires.ftc.teamcode.controllers.ShooterRotatorController;
import org.opencv.core.Mat;

import java.util.jar.Attributes;

@Autonomous(name = "auto1")
public class auto1 extends LinearOpMode {
    private ElapsedTime opModeTime = new ElapsedTime();

    private DcMotorEx  intake;
    private CRServo servo1;

    private static final double COUNTS_PER_REV = 28.0;
    private static final double SHOOTER_VELOCITY = 935; // ticks/sec

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

    @Override
    public void runOpMode() throws InterruptedException {
        robotPoseController = new RobotPoseController(hardwareMap);
        turret =new ShooterRotatorController(hardwareMap, robotPoseController, "turret");
        shooter = new ShooterController(hardwareMap, "shooter");
//        shooter =
        intake = hardwareMap.get(DcMotorEx.class , "intake");
        servo1 = hardwareMap.get(CRServo.class, "servo1");

        while (true) {
            if (gamepad1.crossWasPressed()) {
                isBlue = !isBlue;
            }

            telemetry.addData("Current Team : ", isBlue ? "BLUE" : "RED");
            telemetry.update();
            if (isStarted()) {
                break;
            }
        }

        Pose2d beginPose = reflect(-53.24, 48.70,Math.toRadians(127.16));

        drive = new MecanumDrive(hardwareMap, beginPose);

        TrajectoryActionBuilder trajectoryActionBuilder = drive.actionBuilder(beginPose)
                .afterTime(0.0, () -> {
                    shooter.setTargetVelocity(SHOOTER_VELOCITY);
                    turret.setTargetWorldAngle(reflect(-2));
                })
                .lineToYConstantHeading(reflect(48))
                .splineTo(reflectV(-10.35, 10.35), Math.toRadians(reflect(-45.00)))
                //shoot1
                .afterTime(0.0, () -> intake.setPower(1))
//                .waitSeconds(1)
                .afterTime(0.5, () -> servo1.setPower(-1))
                .afterTime(2, () -> {
//                    turret.setTargetWorldAngle();
                    servo1.setPower(1);
                    intake.setPower(0);
                })
                .waitSeconds(2)

                //shoot 1 selesai terus jalan ke dua mulai
                .lineToYConstantHeading(reflect(7))
                .waitSeconds(1.0)
                .afterTime(0.0, () -> intake.setPower(1))
//                .afterTime(0.0, () -> turret.setPower(-1))
                .splineTo(reflectV(-10.24, 21.50), Math.toRadians(reflect(94.56)))
                .splineTo(reflectV(-11.80, 59.96), Math.toRadians(reflect(90.00)))
                .waitSeconds(0.5)
                .lineToYConstantHeading(reflect(54))
                .afterTime(0.0, () -> intake.setPower(0))
//                .afterTime(0.0, () -> turret.setPower(0))
                .splineTo(reflectV(-13.06, 13.99), Math.toRadians(reflect(268.58)))
                //shoot2
                .afterTime(0.0, () -> {
                    turret.setTargetWorldAngle(reflect(95));

                    intake.setPower(1);
                    }
                )
//                .waitSeconds(1)
                .afterTime(0.5, () -> servo1.setPower(-1))
                .afterTime(2, () -> {
                    servo1.setPower(1);
                    intake.setPower(0);
                })
                .waitSeconds(2)
                //shoot 2 selesai
                .splineTo(reflectV(12.12, 23.69), Math.toRadians(reflect(89.78)))
                .afterTime(0.0, () -> intake.setPower(1))
                .splineTo(reflectV(11.6, 57.3), Math.toRadians(reflect(89.78)))
                //.splineTo(new Vector2d(11.34, 55.27), Math.toRadians(89.27))
                .waitSeconds(0.5)
                .lineToYConstantHeading(reflect(48))
                .afterTime(0.0, () -> intake.setPower(0))
                .splineTo(reflectV(-12.74, 13.84), Math.toRadians(reflect(239.92)))
                .waitSeconds(0.5)
                //shoot 3
                .afterTime(0.0, () -> {
                    turret.setTargetWorldAngle(reflect(95));
                    intake.setPower(1);
                })
                .afterTime(0.5, () -> servo1.setPower(-1))
                .afterTime(2, () -> {
                    servo1.setPower(1);
                    intake.setPower(0);
                });
        //.afterTime(0.0, () -> intake.setPower(1))
        //.afterTime(3.0, () -> intake.setPower(0))
        // .waitSeconds(3)
        //.afterTime(0.0, () -> shooter.setPower(0))

        waitForStart();

        Action path = trajectoryActionBuilder
                .build();


//        Actions.runBlocking(new SequentialAction(path));
        if (isStopRequested()) return;

        // Build and execute the action
        Action driveAction = trajectoryActionBuilder
                .build();

        while (opModeIsActive()) {
            opModeTime.reset();
            runBlocking(driveAction);
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

            robotPoseController.update();
            turret.update();
            turret.activate();
            shooter.update();

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
