package org.firstinspires.ftc.teamcode.teleop.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class ShooterSubsystem {

    private final DcMotorEx spin1, spin2;
    private final PIDController controller;

    // PID constants (tune these)
    public static double P = 0.0005;
    public static double I = 0.0001;
    public static double D = 0;

    // Target velocity in ticks per second
    public static double targetVelocity = 2000;

    // Store PID outputs for telemetry
    public double pid1;
    public double pid2;

    public ShooterSubsystem(HardwareMap hardwareMap) {
        spin1 = hardwareMap.get(DcMotorEx.class, "spin1");
        spin2 = hardwareMap.get(DcMotorEx.class, "spin2");

        spin1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        spin2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        controller = new PIDController(P, I, D);
    }

    public void setTargetVelocity(double velocity) {
        targetVelocity = velocity;
    }

    public double getSpin1Velocity() {
        return spin1.getVelocity();
    }

    public double getSpin2Velocity() {
        return spin2.getVelocity();
    }

    public double getPid1() {
        return pid1;
    }

    public double getPid2() {
        return pid2;
    }

    public void update() {
        // PID calculation based on current velocity
        pid1 = controller.calculate(spin1.getVelocity(), targetVelocity);
        pid2 = controller.calculate(spin2.getVelocity(), targetVelocity);

        // Clip power to [-1, 1]
        pid1 = Math.max(-1, Math.min(1, pid1));
        pid2 = Math.max(-1, Math.min(1, pid2));

        // Apply power to motors
        spin1.setPower(pid1);
        spin2.setPower(pid2);
    }
}
