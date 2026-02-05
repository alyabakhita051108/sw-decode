package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.controllers.PIDCoefficients;
import org.firstinspires.ftc.teamcode.controllers.PIDFController;
import org.firstinspires.ftc.teamcode.controllers.RobotPoseController;
import org.firstinspires.ftc.teamcode.controllers.ShooterController;
import org.firstinspires.ftc.teamcode.controllers.ShooterRotatorController;

@TeleOp(name = "cobacoba")
public class cobacoba extends LinearOpMode {

    private double SHOOTER_ANGLE_MINIMUM = 32;
    private double SHOOTER_ANGLE_MAXIMUM = 46;


    /* ================= DRIVE ================= */
    private DcMotor leftFront, leftBack, rightFront, rightBack;

    /* ================= MECHANISMS ================= */
    private DcMotorEx  shooter, intake;
    private CRServo servo1;
    private Servo servoLeft;


    /* ================= SENSORS ================= */
    private IMU imu;
    private VoltageSensor batteryVoltageSensor;

    /* ================= SERVO TIMING ================= */
    private ElapsedTime servoTimer = new ElapsedTime();
    private double servoCurrentDegree = 32;
    private static final double SERVO_MAX = 1.0;
    private static final double SERVO_SPEED = 1.0  ;


    public static double shooterVelocityTarget1 = 1200;
    public static double shooterVelocityTarget2 = 1600;

    private boolean lastLeftBumper = false;

    private int shooterMode = 0;

    private double SHOOTER_MOTOR_COUNTS_PER_REV = 28.0;

    private ShooterRotatorController turret;
    private RobotPoseController robotPoseController;

    private ShooterController shooterController;

    public static double F = 0.00017;
    public static PIDCoefficients coeffs = new PIDCoefficients(0, 0, 0.003, 0.0003, 0, 0.001);
    public static PIDFController pidfController = new PIDFController(coeffs, (d, v) -> {
        if (v != null) {
            return v*F;
        }
        return 0;
    });


    private double deadzone(double value, double dz) {
        return Math.abs(value) > dz ? value : 0.0;
    }


    @Override
    public void runOpMode() {
        shooterController = new ShooterController();
        robotPoseController = new RobotPoseController(hardwareMap);
        turret = new ShooterRotatorController(hardwareMap,robotPoseController, "turret");

        /* ================= HARDWARE MAP ================= */
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");

        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        intake = hardwareMap.get(DcMotorEx.class, "intake");

        servoLeft = hardwareMap.get(Servo.class, "servoLeft");
        servo1 = hardwareMap.get(CRServo.class, "servo1");

        imu = hardwareMap.get(IMU.class, "imu");
        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        /* ================= IMU INIT ================= */
        IMU.Parameters imuParams = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )
        );
        imu.initialize(imuParams);

        /* ================= MOTOR DIRECTIONS ================= */
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.FORWARD);

        shooter.setDirection(DcMotor.Direction.REVERSE);
        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

//        shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, coeffs);

        /* ================= ZERO POWER ================= */
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        pidfController.targetVelocity = 0;

        telemetry.addLine("Ready - Press START");
        telemetry.update();

        waitForStart();
        imu.resetYaw();
        servoTimer.reset();

        /* ================= MAIN LOOP ================= */
        while (opModeIsActive()) {

            /* ===== DRIVE ===== */
            double forward = deadzone(gamepad1.left_stick_y, 0.05);
            double strafe = deadzone(-gamepad1.left_stick_x, 0.05);
            double rotate = deadzone(-gamepad1.right_stick_x, 0.05);

            strafe *= 1.2;

            double lf = forward + strafe + rotate;
            double rf = forward - strafe - rotate;
            double lb = forward - strafe + rotate;
            double rb = forward + strafe - rotate;

            double max = Math.max(1.0,
                    Math.max(Math.abs(lf),
                            Math.max(Math.abs(rf),
                                    Math.max(Math.abs(lb), Math.abs(rb)))));

            leftFront.setPower(lf / max);
            rightFront.setPower(rf / max);
            leftBack.setPower(lb / max);
            rightBack.setPower(rb / max);

            /* ===== TURRET ===== */
            if (gamepad1.dpad_left) {
                turret.setPower(1.0);
            } else if (gamepad1.dpad_right) {
                turret.setPower(-1.0);
            } else {
                turret.setPower(0.0);
            }

            //shooter nambah power
            if (gamepad2.rightBumperWasPressed()) {
                shooterMode++;

                if (shooterMode > 2) {
                    shooterMode = 0;
                }
            }


            switch (shooterMode) {
                case 0: // mati
                    shooter.setPower(0);
                    pidfController.targetVelocity = 0;
                    break;

                case 1: // 930
                    pidfController.targetVelocity = shooterVelocityTarget1;
                    break;

                case 2: // 1400
                    pidfController.targetVelocity = shooterVelocityTarget2;

                    break;
            }

            double shooterPower = pidfController.update(System.nanoTime(), 0, shooter.getVelocity());
            shooter.setPower(shooterPower);


            /* ===== INTAKE ===== */
            if (gamepad1.left_bumper) {
                intake.setPower(1.0);
            } else if (gamepad1.right_bumper) {
                intake.setPower(-1.0);
            } else {
                intake.setPower(0.0);
            }

            /* ===== SERVO LEFT & RIGHT ===== */
            double dt = servoTimer.seconds();
            servoTimer.reset();

            if (gamepad1.y && servoCurrentDegree < SHOOTER_ANGLE_MAXIMUM) {
                servoLeft.setPosition(shooterController.angleToServo(servoCurrentDegree));
                servoCurrentDegree -= 0.05;
            } else if (gamepad1.a && servoCurrentDegree > SHOOTER_ANGLE_MINIMUM) {
                servoLeft.setPosition(shooterController.angleToServo(servoCurrentDegree));
                servoCurrentDegree += 0.05;
            }

            /* ==       www=== SERVO 1 (FIXED) ===== */
            if (gamepad2.y) {
                servo1.setPower(-1.0);
            } else if (gamepad2.a) {
                servo1.setPower(1.0);
            } else {
                servo1.setPower(0.0);
            }

            /* ===== IMU ===== */
            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
            double yaw = orientation.getYaw(AngleUnit.DEGREES);

            double SHOOTER_RPM = shooter.getVelocity() / SHOOTER_MOTOR_COUNTS_PER_REV * 60;

            robotPoseController.update();
            turret.update();

            /* ===== TELEMETRY ===== */
            telemetry.addData("TURRET ANGLE", turret.getTurretWorldAngle());

            telemetry.addData("SHOOTER_RPM", SHOOTER_RPM);
            telemetry.addData("SHOOTER_RPM VELOCITY", shooter.getVelocity());


            telemetry.addData("Yaw (deg)", yaw);
            telemetry.addData("Battery (V)", batteryVoltageSensor.getVoltage());
            telemetry.addData("Servo Degree", servoCurrentDegree  );
            telemetry.addData("Intake", intake.getPower());

            telemetry.addData("Shooter Mode",
                    shooterMode == 0 ? "OFF" :
                            shooterMode == 1 ? "930 RPM" : "1400 RPM");
            telemetry.update();
        }
    }
}
