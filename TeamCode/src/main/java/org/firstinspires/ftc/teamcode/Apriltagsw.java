package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@TeleOp(name = "Apriltagsw")
public class Apriltagsw extends LinearOpMode {

    // ================= DRIVE =================
    private DcMotor leftFront, leftBack, rightFront, rightBack;

    // ================= MECHANISM =================
    private DcMotor turret, shooter, intake;
    private CRServo servoLeft, servoRight;

    // ================= SERVO LIMIT =================
    private ElapsedTime servoTimer = new ElapsedTime();
    private double servoPosition = 0.0;
    private static final double SERVO_MAX = 1.5;
    private static final double SERVO_SPEED = 1.0;

    // ================= APRILTAG =================
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;

    // ================= TURRET CONTROL =================
    private double lastSeenYaw = 0;
    private boolean scanRight = true;
    private ElapsedTime lostTimer = new ElapsedTime();

    // ================= TUNING =================
    private static final double KP_TURRET = 0.015;
    private static final double MAX_TURRET_POWER = 0.45;
    private static final double SEARCH_POWER = 0.45;
    private static final double LOST_TIMEOUT = 0.6;

    // ================= TURRET LIMIT (ENCODER) =================
    private static final int TURRET_MIN = 0;
    private static final int TURRET_MAX = 450; // ≈180°

    @Override
    public void runOpMode() {

        // ================= HARDWARE MAP =================
        leftFront  = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftBack   = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack  = hardwareMap.get(DcMotor.class, "rightBack");

        turret  = hardwareMap.get(DcMotor.class, "turret");
        shooter = hardwareMap.get(DcMotor.class, "shooter");
        intake  = hardwareMap.get(DcMotor.class, "intake");

        servoLeft  = hardwareMap.get(CRServo.class, "servoLeft");
        servoRight = hardwareMap.get(CRServo.class, "servoRight");

        // ================= DRIVE SETUP =================
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // ================= TURRET SETUP =================
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // ================= APRILTAG INIT =================
        aprilTag = AprilTagProcessor.easyCreateWithDefaults();

        visionPortal = VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(WebcamName.class, "Webcam 1"),
                aprilTag
        );

        telemetry.addLine("READY - AprilTag & Turret Initialized");
        telemetry.update();

        waitForStart();

        servoTimer.reset();
        lostTimer.reset();

        // ================= MAIN LOOP =================
        while (opModeIsActive()) {

            // -------- MECANUM DRIVE --------
            double y  = -gamepad1.left_stick_y;
            double x  = gamepad1.right_stick_x;
            double rx = gamepad1.left_stick_x;

            double lf = y + x + rx;
            double rf = y - x - rx;
            double lb = y - x + rx;
            double rb = y + x - rx;

            double max = Math.max(1.0,
                    Math.max(Math.abs(lf),
                            Math.max(Math.abs(rf),
                                    Math.max(Math.abs(lb), Math.abs(rb)))));

            leftFront.setPower(lf / max);
            rightFront.setPower(rf / max);
            leftBack.setPower(lb / max);
            rightBack.setPower(rb / max);

            // -------- SHOOTER --------
            shooter.setPower(gamepad2.x ? 1.0 : 0.0);

            // -------- INTAKE --------
            if (gamepad1.a) intake.setPower(1);
            else if (gamepad1.y) intake.setPower(-1);
            else intake.setPower(0);

            // ================= APRILTAG TURRET LOGIC =================
            int turretPos = turret.getCurrentPosition();
            AprilTagDetection target = null;

            if (!aprilTag.getDetections().isEmpty()) {
                target = aprilTag.getDetections().get(0);
            }

            if (target != null) {
                double errorYaw = target.ftcPose.yaw;
                lastSeenYaw = errorYaw;
                lostTimer.reset();

                double power = errorYaw * KP_TURRET;
                power = Math.max(-MAX_TURRET_POWER, Math.min(MAX_TURRET_POWER, power));

                if ((turretPos <= TURRET_MIN && power < 0) ||
                        (turretPos >= TURRET_MAX && power > 0)) {
                    power = 0;
                }

                turret.setPower(power);
            }
            else if (lostTimer.seconds() < LOST_TIMEOUT) {
                double power = Math.signum(lastSeenYaw) * SEARCH_POWER;
                turret.setPower(power);
            }
            else {
                if (scanRight) {
                    turret.setPower(SEARCH_POWER);
                    if (turretPos >= TURRET_MAX) scanRight = false;
                } else {
                    turret.setPower(-SEARCH_POWER);
                    if (turretPos <= TURRET_MIN) scanRight = true;
                }
            }

            // ================= SERVO CONTROL =================
            double dt = servoTimer.seconds();
            servoTimer.reset();

            if (gamepad1.left_bumper && servoPosition > 0) {
                servoLeft.setPower(SERVO_SPEED);
                servoRight.setPower(SERVO_SPEED);
                servoPosition -= dt;
            }
            else if (gamepad1.right_bumper && servoPosition < SERVO_MAX) {
                servoLeft.setPower(-SERVO_SPEED);
                servoRight.setPower(-SERVO_SPEED);
                servoPosition += dt;
            }
            else {
                servoLeft.setPower(0);
                servoRight.setPower(0);
            }

            servoPosition = Math.max(0, Math.min(SERVO_MAX, servoPosition));

            // ================= TELEMETRY =================
            telemetry.addData("TurretPos", turretPos);
            telemetry.addData("ScanDir", scanRight ? "RIGHT" : "LEFT");
            telemetry.addData("AprilTag", target != null ? "FOUND" : "SEARCHING");
            telemetry.update();
        }

        // ===== CLEAN SHUTDOWN =====
        visionPortal.close();
    }
}
