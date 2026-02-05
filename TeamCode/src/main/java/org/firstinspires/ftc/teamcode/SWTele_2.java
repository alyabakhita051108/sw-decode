package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp(name = "SWTELE OLD")
public  class SWTele_2 extends LinearOpMode {

    private DcMotor leftFront, leftBack, rightFront, rightBack;
    private DcMotor turret, shooter, intake;
    private CRServo servoLeft, servoRight;
    private IMU imu;

    private ElapsedTime servoTimer = new ElapsedTime();
    private double servoPosition = 0.0;
    private final double SERVO_MAX = 1.3;
    private final double SERVO_SPEED = 1.0;

    @Override
    public void runOpMode() throws InterruptedException {

        // -------- Hardware Mapping --------
        leftFront  = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftBack   = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack  = hardwareMap.get(DcMotor.class, "rightBack");

        turret  = hardwareMap.get(DcMotor.class, "turret");
        shooter = hardwareMap.get(DcMotor.class, "shooter");
        intake  = hardwareMap.get(DcMotor.class, "intake");

        servoLeft  = hardwareMap.get(CRServo.class, "servoLeft");
        servoRight = hardwareMap.get(CRServo.class, "servoRight");

        imu = hardwareMap.get(IMU.class, "imu");

        // -------- IMU INITIALIZATION --------
        IMU.Parameters imuParams = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )
        );
        imu.initialize(imuParams);

        // -------- Motor Directions --------
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.FORWARD);
        shooter.setDirection(DcMotor.Direction.REVERSE);

        // -------- Brake --------
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        servoTimer.reset();

        telemetry.addLine("IMU Ready - Press Start");
        telemetry.update();

        waitForStart();

        imu.resetYaw(); // VERY IMPORTANT

        // -------- MAIN LOOP --------
        while (opModeIsActive()) {

            // ----- DRIVE INPUT -----
            double forward  = -gamepad1.left_stick_y;
            double strafe   = -gamepad1.left_stick_x;                                                                                                                                                                                                                                                                                                               //gamepad1.left_stick_x;
            double rotate   = gamepad1.right_stick_x;
            double diagonal = -gamepad1.right_stick_y;

            // ----- TURRET -----
            if (gamepad2.a) {
                turret.setPower(1.0);
            } else if (gamepad2.y) {
                turret.setPower(-1.0);
            } else {
                turret.setPower(0.0);
            }

            // ----- SHOOTER -----
            shooter.setPower(gamepad1.x ? -1.0 : 0.0);

            // ----- INTAKE -----
            if (gamepad1.a) {
                intake.setPower(1.0);
            } else if (gamepad1.y) {
                intake.setPower(-1.0);
            } else {
                intake.setPower(0.0);
            }

            // ----- SERVO LIMIT CONTROL -----
            double dt = servoTimer.seconds();
            servoTimer.reset();

            boolean up   = gamepad1.left_bumper;
            boolean down = gamepad1.right_bumper;

            if (up && servoPosition > 0) {
                servoLeft.setPower(SERVO_SPEED);
                servoRight.setPower(SERVO_SPEED);
                servoPosition -= dt;
            } else if (down && servoPosition < SERVO_MAX) {
                servoLeft.setPower(-SERVO_SPEED);
                servoRight.setPower(-SERVO_SPEED);
                servoPosition += dt;
            } else {
                servoLeft.setPower(0);
                servoRight.setPower(0);
            }

            servoPosition = Math.max(0, Math.min(SERVO_MAX, servoPosition));

            // ----- MECANUM DRIVE -----
            double y  = forward + diagonal;
            double x  = strafe;
            double rx = rotate;

            double lf = y + x + rx;
            double rf = y - x - rx;
            double lb = y - x + rx;
            double rb = y + x - rx;

            double max = Math.max(1.0, Math.max(
                    Math.abs(lf),
                    Math.max(Math.abs(rf), Math.max(Math.abs(lb), Math.abs(rb)))
            ));

            leftFront.setPower(lf / max);
            rightFront.setPower(rf / max);
            leftBack.setPower(lb / max);
            rightBack.setPower(rb / max);

            // ----- IMU READ -----
            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
            double yaw = orientation.getYaw(AngleUnit.DEGREES);

            // ----- TELEMETRY -----
            telemetry.addData("Yaw (deg)", yaw);
            telemetry.addData("Servo Pos", servoPosition);
            telemetry.update();
        }
    }
}