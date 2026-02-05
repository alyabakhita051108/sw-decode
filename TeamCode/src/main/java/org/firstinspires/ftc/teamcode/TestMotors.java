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

@Config
@TeleOp(name = "Test Motor Drive Base")
public class TestMotors extends LinearOpMode {

    /* ================= DRIVE ================= */
    private DcMotor leftFront, leftBack, rightFront, rightBack;

    /* ================= MECHANISMS ================= */
    private DcMotorEx turret, shooter, intake;
    private CRServo servo1;
    private Servo servoLeft;


    /* ================= SENSORS ================= */
    private IMU imu;
    private VoltageSensor batteryVoltageSensor;

    /* ================= SERVO TIMING ================= */
    private ElapsedTime servoTimer = new ElapsedTime();
    private double servoPosition = 0.0;
    private static final double SERVO_MAX = 1.0;
    private static final double SERVO_SPEED = 1.0;

    public static PIDFCoefficients coeffs = new PIDFCoefficients(90,0,0,17.6);

    public static double shooterVelocityTarget = 2100;

    private double SHOOTER_MOTOR_COUNTS_PER_REV = 28.0;


    private double deadzone(double value, double dz) {
        return Math.abs(value) > dz ? value : 0.0;
    }


    @Override
    public void runOpMode() {

        /* ================= HARDWARE MAP ================= */
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");

        turret = hardwareMap.get(DcMotorEx.class, "turret");
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
        shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, coeffs);

        /* ================= ZERO POWER ================= */
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        telemetry.addLine("Ready - Press START");
        telemetry.update();

        waitForStart();
        imu.resetYaw();
        servoTimer.reset();

        /* ================= MAIN LOOP ================= */
        while (opModeIsActive()) {
            if (gamepad1.square) {
                leftFront.setPower(0.5);
            }else if (!gamepad1.square) {
                leftFront.setPower(0);
            }
            if (gamepad1.cross) {
                rightFront.setPower(1);
            }else if (!gamepad1.cross) {
                rightFront.setPower(0);
            }
            if (gamepad1.circle) {
                leftBack.setPower(1);
            }else if (!gamepad1.circle) {
                leftBack.setPower(0);
            }
            if (gamepad1.triangle) {
                rightBack.setPower(1);
            }else if (!gamepad1.triangle) {
                rightBack.setPower(0);
            }

//
            // backleft bener, rear right <->rear leftb, front right <-> rear right,front left bener
        }
    }
}
