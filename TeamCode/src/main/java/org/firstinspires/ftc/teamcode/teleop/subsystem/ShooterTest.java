package org.firstinspires.ftc.teamcode.teleop.subsystem;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name="ShooterTest")
public class ShooterTest extends LinearOpMode {

    private ShooterSubsystem shooter;

    @Override
    public void runOpMode() throws InterruptedException {

        DcMotorEx L = hardwareMap.get(DcMotorEx.class, "L");
        DcMotorEx R = hardwareMap.get(DcMotorEx.class, "R");

        shooter = new ShooterSubsystem(L, R);

        telemetry.addLine("ShooterTest READY");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // PRESS X → test at 4 ft (0.75)
            if (gamepad1.x) {
                shooter.setShooterSpeedFromDistance(4.0);
            }

            // PRESS Y → test at 8 ft (mid value)
            if (gamepad1.y) {
                shooter.setShooterSpeedFromDistance(8.0);
            }

            // PRESS B → test at 11 ft (1.0)
            if (gamepad1.b) {
                shooter.setShooterSpeedFromDistance(11.0);
            }

            // PRESS A → stop shooter
            if (gamepad1.a) {
                shooter.stopShooter();
            }

            telemetry.addLine("Press X/Y/B to test shooter power.");
            telemetry.update();
        }
    }
}
