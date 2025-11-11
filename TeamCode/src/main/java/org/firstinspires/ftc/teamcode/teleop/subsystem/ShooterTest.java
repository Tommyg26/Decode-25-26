package org.firstinspires.ftc.teamcode.teleop.subsystem;

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


@TeleOp(name="ShooterTest")
public class ShooterTest extends LinearOpMode {

    DcMotorEx L;
    DcMotorEx R;
 
    public static double drive_speed_M = 1;


    @Override
    public void runOpMode() throws InterruptedException {


        R = hardwareMap.get(DcMotorEx.class, "r");
        L = hardwareMap.get(DcMotorEx.class, "lt");
       

        waitForStart();
        while (opModeIsActive()) {

            //////////////////////////
            /// Gamepad 1 controls ///
            //////////////////////////
            // --- Drivetrain Control ---
            if (gamepad1.x)
		
		L.setPower(.5);
		R.setPower(.5);


        }
    }
}