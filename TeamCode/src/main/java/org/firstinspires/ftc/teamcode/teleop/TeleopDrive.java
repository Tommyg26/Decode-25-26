package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;



import java.util.List;


@TeleOp(name="TeleopDrive")
public class TeleopDrive extends LinearOpMode {

    DcMotorEx FR;
    DcMotorEx FL;
    DcMotorEx BR;
    DcMotorEx BL;
    public static double drive_speed_M = 1;


    @Override
    public void runOpMode() throws InterruptedException {


        FR = hardwareMap.get(DcMotorEx.class, "rightFront");
        FL = hardwareMap.get(DcMotorEx.class, "leftFront");
        BR = hardwareMap.get(DcMotorEx.class, "rightBack");
        BL = hardwareMap.get(DcMotorEx.class, "leftBack");

        waitForStart();
        while (opModeIsActive()) {

            //////////////////////////
            /// Gamepad 1 controls ///
            //////////////////////////
            // --- Drivetrain Control ---
            double y = gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x * 1.1;
            double rx = gamepad1.right_trigger - gamepad1.left_trigger;

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
}