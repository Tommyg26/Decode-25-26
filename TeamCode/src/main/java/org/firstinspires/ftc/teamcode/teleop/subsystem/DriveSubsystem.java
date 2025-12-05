package org.firstinspires.ftc.teamcode.teleop.subsystem;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DriveSubsystem {

    private DcMotorEx FR, FL, BR, BL;

    // Global speed scale (like your drive_speed_M)
    private double speedScale = 1.0;

    public DriveSubsystem(HardwareMap hardwareMap) {

        FR = hardwareMap.get(DcMotorEx.class, "rightFront");
        FL = hardwareMap.get(DcMotorEx.class, "leftFront");
        BR = hardwareMap.get(DcMotorEx.class, "rightBack");
        BL = hardwareMap.get(DcMotorEx.class, "leftBack");

        // reverse right motors to match your TeleOp code
        FR.setDirection(DcMotorEx.Direction.REVERSE);
        BR.setDirection(DcMotorEx.Direction.REVERSE);
    }

    /**
     * Main drive method for mecanum movement.
     * Same mapping you used in TeleopDrive.
     */
    public void drive(double y, double x, double rx) {

        // Mecanum denominator logic (avoid overpowering)
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

        double frontLeftPower = speedScale * (y - x + rx) / denominator;
        double backLeftPower = speedScale * (y + x + rx) / denominator;
        double frontRightPower = speedScale * (y - x - rx) / denominator;
        double backRightPower = speedScale * (y + x - rx) / denominator;

        // Apply motor powers
        FL.setPower(frontLeftPower);
        BL.setPower(backLeftPower);
        FR.setPower(frontRightPower);
        BR.setPower(backRightPower);
    }

    /**
     * Stops all drive motors.
     */
    public void stop() {
        FL.setPower(0);
        BL.setPower(0);
        FR.setPower(0);
        BR.setPower(0);
    }

}
