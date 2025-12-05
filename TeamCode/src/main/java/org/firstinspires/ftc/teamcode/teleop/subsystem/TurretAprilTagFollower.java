package org.firstinspires.ftc.teamcode.teleop.subsystem;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.Range;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;

import java.util.List;

@TeleOp(name="Turret AprilTag Follower", group="TeleOp")
public class TurretAprilTagFollower extends LinearOpMode {

    private CRServo leftTurret, rightTurret;
    private Limelight3A limelight;

    private static final int TARGET_TAG_ID = 24;

    private static final double TURN_GAIN = 0.025;
    private static final double MAX_TURN_SPEED = 0.35;
    private static final double MIN_TURN_POWER = 0.10;

    // new safety constants:
    private static final double MAX_ANGLE = 180;
    private static final double MIN_ANGLE = -180;
    private static final double ANGLE_GAIN = 8.0;  // deg per cycle per power
    private double turretAngle = 0;                // virtual turret angle

    private double lastTurnPower = 0.0;
    private long lastUpdateNanos = 0L;
    private static final double MAX_TURN_SLEW_PER_SEC = 2.0;

    @Override
    public void runOpMode() {

        leftTurret  = hardwareMap.get(CRServo.class, "leftTurret");
        rightTurret = hardwareMap.get(CRServo.class, "rightTurret");

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(0);
        limelight.start();

        telemetry.addLine("Turret AprilTag Follower READY");
        telemetry.update();

        waitForStart();

        // Start in scanning mode
        boolean scanningLeft = true;

        while (opModeIsActive()) {

            LLResult result = limelight.getLatestResult();
            boolean hasTag = false;
            double tx = 0;

            // ------------------------------
            // READ LIMELIGHT APRILTAG
            // ------------------------------
            if (result != null && result.isValid()) {

                List<LLResultTypes.FiducialResult> fid = result.getFiducialResults();
                if (fid != null) {
                    for (LLResultTypes.FiducialResult f : fid) {
                        if (f.getFiducialId() == TARGET_TAG_ID) {
                            hasTag = true;
                            tx = f.getTargetXDegrees();
                            break;
                        }
                    }
                }
            }

            double turretCmd;

            if (hasTag) {
                // ------------------------------
                // LOCK-ON MODE
                // ------------------------------

                double desired = (-tx) * TURN_GAIN;
                desired = Range.clip(desired, -MAX_TURN_SPEED, MAX_TURN_SPEED);

                if (Math.abs(desired) < MIN_TURN_POWER && Math.abs(desired) > 0) {
                    desired = Math.signum(desired) * MIN_TURN_POWER;
                }

                turretCmd = smooth(desired);

            } else {
                // ------------------------------
                // TAG LOST → HANDLE RECOVERY
                // ------------------------------

                // Step 1: return to origin
                if (Math.abs(turretAngle) > 3) {
                    double dir = turretAngle > 0 ? -0.25 : 0.25;
                    turretCmd = dir;
                    updateAngle(turretCmd);

                } else {
                    // Step 2: scanning mode left-right-left-right etc.

                    if (scanningLeft) {
                        if (turretAngle > MIN_ANGLE)
                            turretCmd = -0.25;
                        else {
                            scanningLeft = false;
                            turretCmd = 0.25;
                        }
                    } else {
                        if (turretAngle < MAX_ANGLE)
                            turretCmd = 0.25;
                        else {
                            scanningLeft = true;
                            turretCmd = -0.25;
                        }
                    }

                    updateAngle(turretCmd);
                }

                // Apply minimum power deadband
                if (Math.abs(turretCmd) < MIN_TURN_POWER) {
                    turretCmd = Math.signum(turretCmd) * MIN_TURN_POWER;
                }
            }

            // ------------------------------
            // APPLY POWER (OPPOSITE SERVOS)
            // ------------------------------
            leftTurret.setPower(turretCmd);
            rightTurret.setPower(-turretCmd);

            telemetry.addData("HasTag", hasTag);
            telemetry.addData("TX", tx);
            telemetry.addData("TurretCmd", turretCmd);
            telemetry.addData("Angle", turretAngle);
            telemetry.update();

            sleep(20);
        }

        leftTurret.setPower(0);
        rightTurret.setPower(0);
        limelight.stop();
    }

    // ------------------------------
    // HELPER: smoothing
    // ------------------------------
    private double smooth(double desired) {
        long now = System.nanoTime();
        if (lastUpdateNanos == 0) lastUpdateNanos = now;

        double dt = (now - lastUpdateNanos) / 1e9;
        double maxStep = MAX_TURN_SLEW_PER_SEC * dt;

        double delta = desired - lastTurnPower;
        delta = Range.clip(delta, -maxStep, maxStep);

        double cmd = lastTurnPower + delta;
        lastTurnPower = cmd;
        lastUpdateNanos = now;

        updateAngle(cmd);
        return cmd;
    }

    // ------------------------------
    // HELPER: track turret angle
    // ------------------------------
    private void updateAngle(double power) {
        double next = turretAngle + (power * ANGLE_GAIN);

        if (next > MAX_ANGLE) {
            turretAngle = MAX_ANGLE;
            return;
        }
        if (next < MIN_ANGLE) {
            turretAngle = MIN_ANGLE;
            return;
        }
        turretAngle = next;
    }
}
