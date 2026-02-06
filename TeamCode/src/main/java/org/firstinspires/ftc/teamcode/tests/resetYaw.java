package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name ="Reset Yaw", group = "Test")

public class resetYaw extends OpMode {
    private IMU imu;


    @Override
    public void init() {

        // IMU Init
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                )
        ));
        imu.resetYaw();
    }

    @Override
    public void loop() {


        telemetry.addLine("Press Gamepad1 Triangle Button to reset the robot's yaw");

        if (gamepad1.triangleWasPressed()) {
            imu.resetYaw();
        }

        /* ================== CALCULATE ANGLES ================== */
        // Robot Heading (World Frame)
        double robotYaw = normalizeAngle(
                imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)
        );

        telemetry.addData("Robot Yaw (IMU)", "%.2f", robotYaw);
        telemetry.update();
    }

    /* ================== HELPERS ================== */
    private double normalizeAngle(double a) {
        while (a > 180) a -= 360;
        while (a <= -180) a += 360;
        return a;
    }
}
