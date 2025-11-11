package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public class AprilTagLimelight extends OpMode {
    private Limelight3A limelight;

    @Override
    public void init(){
        limelight = hardwareMap.get(Limelight3A.class, "Limelight 3A");
        limelight.pipelineSwitch(1); //april tag id
    }
    @Override
    public void start(){
        super.start();
    }
    @Override
    public void loop(){

    }
}
