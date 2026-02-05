package org.firstinspires.ftc.teamcode.tuning;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Servo Tester Hood", group = "Test")
public class  ServoTesterHood extends OpMode {
    private Servo servoLeft, servoRight;

    private double currentPos = 0;
    private double limit = 1;
    // max 0.37

    @Override
    public void init() {
        servoLeft = hardwareMap.get(Servo.class, "servoLeft");
        servoLeft.setDirection(Servo.Direction.FORWARD);

        servoRight = hardwareMap.get(Servo.class, "servoRight");
        servoRight.setDirection(Servo.Direction.REVERSE);
//        armPivot = hardwareMap.get(Servo.class, "armPivot");
    }

    @Override
    public void loop() {

        if (gamepad1.dpad_up && currentPos <= limit) {
            currentPos += 0.005;
        }else if (gamepad1.dpad_down && currentPos >= 0) {
            currentPos -= 0.005;
        }

        servoLeft.setPosition(currentPos);
        servoRight.setPosition(currentPos);

        telemetry.addData("shooterAd", currentPos);


        telemetry.update();
    }

}

// 12.03
// 33.5 / 37.5
// 22.5 / 28.5

// 23 degrees & 38 degress