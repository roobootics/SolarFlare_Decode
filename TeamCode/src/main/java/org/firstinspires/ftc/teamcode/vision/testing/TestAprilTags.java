package org.firstinspires.ftc.teamcode.vision.testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.vision.VisionManager;

@Autonomous
public class TestAprilTags extends OpMode {
    VisionManager visionManager = new VisionManager(hardwareMap, telemetry);
    @Override
    public void init(){

    }

    @Override
    public void loop(){
        telemetry.addData("obelisk id", visionManager.getObeliskID());
        telemetry.addData("botpose", visionManager.getBotPoseAprilTags());
        telemetry.addData("pipeline type", visionManager.getCurrentPipelineType());
        telemetry.update();
    }
}
