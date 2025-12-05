package org.firstinspires.ftc.teamcode.teleop.subsystem;

import com.qualcomm.robotcore.hardware.DcMotorEx;

public class ShooterSubsystem {

    private DcMotorEx L;
    private DcMotorEx R;

    // Shooter constants
    private static final double MIN_SPEED = 0.75;   // At 4 ft and below
    private static final double MAX_SPEED = 1.00;   // At 11 ft
    private static final double MIN_DIST = 4.0;     // ft
    private static final double MAX_DIST = 11.0;    // ft

    public ShooterSubsystem(DcMotorEx left, DcMotorEx right) {
        this.L = left;
        this.R = right;
    }

    /**
     * Given distance in feet, compute shooter power:
     *  - Below 4 ft: always 0.75
     *  - Between 4 and 11 ft: linear scale to 1.0
     *  - Above 11 ft: clamp to 1.0
     */
    public double computeShooterSpeed(double distanceFt) {
        if (distanceFt <= MIN_DIST) {
            return MIN_SPEED;
        }
        if (distanceFt >= MAX_DIST) {
            return MAX_SPEED;
        }

        // Linear interpolation
        double slope = (MAX_SPEED - MIN_SPEED) / (MAX_DIST - MIN_DIST);
        return MIN_SPEED + slope * (distanceFt - MIN_DIST);
    }

    /**
     * Sets BOTH shooter motors to the correct speed based on distance.
     */
    public void setShooterSpeedFromDistance(double distanceFt) {
        double speed = computeShooterSpeed(distanceFt);

        // YOUR robot: left motor forward, right reversed
        L.setPower(speed);
        R.setPower(-speed);
    }

    /**
     * Stop both shooter motors.
     */
    public void stopShooter() {
        L.setPower(0);
        R.setPower(0);
    }
}
