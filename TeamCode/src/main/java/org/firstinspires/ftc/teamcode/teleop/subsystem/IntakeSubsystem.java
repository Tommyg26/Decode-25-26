package org.firstinspires.ftc.teamcode.teleop.subsystem;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class IntakeSubsystem {

    private DcMotorEx intakeMotor;

    private static final double INTAKE_POWER = 0.9;

    public IntakeSubsystem(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intake"); 
    }

    /** Run intake forward at 0.9 power */
    public void intakeForward() {
        intakeMotor.setPower(INTAKE_POWER);
    }

    /** Run intake backward at 0.9 power */
    public void intakeBackward() {
        intakeMotor.setPower(-INTAKE_POWER);
    }

    /** Stop intake motor */
    public void stop() {
        intakeMotor.setPower(0);
    }

    /** 
     * Brief reverse pulse to clear jams.
     * Example usage: smallBack(40);
     */
    public void smallBack(int ms) {
        intakeMotor.setPower(-INTAKE_POWER);
        try { Thread.sleep(ms); } catch (InterruptedException ignored) {}
        intakeMotor.setPower(0);
    }

    /** 
     * Convenience overload: default small back is 30ms
     */
    public void smallBack() {
        smallBack(30);
    }

    /** Shoot = same as intake forward */
    public void shoot() {
        intakeForward();
    }
}
