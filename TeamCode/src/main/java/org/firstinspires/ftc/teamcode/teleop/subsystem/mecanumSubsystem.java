package org.firstinspires.ftc.teamcode.Teleop.subsystem;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Config
public class mecanumSubsystem {
    private final DcMotorEx FR;
    private final DcMotorEx FL;
    private final DcMotorEx BR;
    private final DcMotorEx BL;
    public static double drive_speed_M = 1;

    public mecanumSubsystem(HardwareMap map){
        FR = map.get(DcMotorEx.class, "rightFront");
        FL = map.get(DcMotorEx.class, "leftFront");
        BR = map.get(DcMotorEx.class, "rightBack");
        BL = map.get(DcMotorEx.class, "leftBack");
        controller = newPIDController(P, I, D);
    }

    public void init(){

    }

    public void drive(double x, double y, double rx){
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = 0.95 * (y - x + rx) / denominator;
        double backLeftPower = 0.95 * (y + x + rx) / denominator;
        double frontRightPower = 0.95 * (y - x - rx) / denominator;
        double backRightPower = 0.95 * (y + x - rx) / denominator;

        FL.setPower(frontLeftPower);
        BL.setPower(backLeftPower);
        FR.setPower(-frontRightPower);
        BR.setPower(-backRightPower);        
    }
}