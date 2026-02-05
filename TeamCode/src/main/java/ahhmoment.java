package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "ahhmoment")
public class ahhmoment extends LinearOpMode {

    DcMotor leftFront, leftBack, rightFront, rightBack;
    DcMotor shooter, intake,turret;
    CRServo servoLeft, servoRight;
    Servo servo1;




    @Override
    public void runOpMode() {

        // ===== Hardware Map =====
        leftFront  = hardwareMap.get(DcMotor.class, "leftFront");
        leftBack   = hardwareMap.get(DcMotor.class, "leftBack");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightBack  = hardwareMap.get(DcMotor.class, "rightBack");


        turret = hardwareMap.get(DcMotor.class, "turret");
        shooter = hardwareMap.get(DcMotor.class, "shooter");
        intake = hardwareMap.get(DcMotor.class, "intake");

        servoLeft = hardwareMap.get(CRServo.class, "servoLeft");
        servoRight = hardwareMap.get(CRServo.class, "servoRight");
        servo1 = hardwareMap.get(Servo.class, "servo1");

        // ===== Motor Direction =====
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addLine("Ready for AUTO");
        telemetry.update();

        waitForStart();

        /* ===== AUTO START ===== */

        shooter.setPower(1.0);

        leftFront.setPower(1);
        leftBack.setPower(1);
        rightFront.setPower(1);
        rightBack.setPower(1);
        sleep(3500);

        intake.setPower(-1);
        sleep(3000);

        servo1.setPosition(0);
        sleep(1000);

        shooter.setPower(0);



    }

    // ===== Helper Function =====
    private void setDrivePower(double power) {
        leftFront.setPower(power);
        leftBack.setPower(power);
        rightFront.setPower(power);
        rightBack.setPower(power);
    }
}
