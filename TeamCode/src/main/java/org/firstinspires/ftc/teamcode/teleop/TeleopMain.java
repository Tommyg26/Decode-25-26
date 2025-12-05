package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import org.firstinspires.ftc.teamcode.teleop.subsystem.DriveSubsystem;
import org.firstinspires.ftc.teamcode.teleop.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.teleop.subsystem.ShooterSubsystem;

@TeleOp(name="MainTeleOp")
public class TeleopMain extends LinearOpMode {

    private DriveSubsystem drive;
    private IntakeSubsystem intake;
    private ShooterSubsystem shooter;

    private Limelight3A limelight;

    @Override
    public void runOpMode() throws InterruptedException {

        // Initialize subsystems
        drive = new DriveSubsystem(hardwareMap);
        intake = new IntakeSubsystem(hardwareMap);
        shooter = new ShooterSubsystem(
                hardwareMap.get(DcMotorEx.class, "L"),
                hardwareMap.get(DcMotorEx.class, "R")
        );

        // Limelight
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(0); // AprilTag pipeline
        limelight.start();

        telemetry.addLine("TeleOp READY");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // -----------------------------
            //          DRIVE
            // -----------------------------
            double y  = -gamepad1.right_stick_y;
            double x  = gamepad1.right_stick_x * 1.1;
            double rx = gamepad1.right_trigger - gamepad1.left_trigger;

            drive.drive(y, x, rx);

            // -----------------------------
            //          INTAKE
            // -----------------------------
            if (gamepad1.dpad_down) {
                intake.intakeBackward();
            }
            else if (gamepad1.b) {
                intake.intakeForward();
            }
            else if (gamepad1.x) {
                intake.shoot();   // feed to shooter
            }

            // If nothing pressed → stop intake
            else {
                intake.stop();
            }

            // -----------------------------
            //        SHOOTER CONTROL
            // -----------------------------
            if (gamepad1.y) {

                // Read distance in FEET using Limelight pose
                double distanceFt = getDistanceFt();

                shooter.setShooterSpeedFromDistance(distanceFt);

                telemetry.addData("Shooter Mode", "Auto Power");
                telemetry.addData("Distance (ft)", distanceFt);
                telemetry.addData("Power", shooter.computeShooterSpeed(distanceFt));
            }

            if (gamepad1.a) {
                shooter.stopShooter();
                telemetry.addData("Shooter", "STOPPED");
            }

            telemetry.update();
        }

        shooter.stopShooter();
        intake.stop();
        drive.stop();
        limelight.stop();
    }

    // -----------------------------------------------------
    // GET DISTANCE IN FEET FROM LIMELIGHT
    // -----------------------------------------------------
    private double getDistanceFt() {

        LLResult result = limelight.getLatestResult();

        if (result == null || !result.isValid()) {
            return 4.0; // safe fallback
        }

        LLResultTypes.FiducialResult best = null;

        for (LLResultTypes.FiducialResult f : result.getFiducialResults()) {
            best = f;  // just take the first detected tag
            break;
        }

        if (best == null) return 4.0;

        Pose3D pose = best.getRobotPoseTargetSpace();
        if (pose == null) return 4.0;

        double meters = pose.getPosition().z;
        return meters * 3.28084;
    }
}
