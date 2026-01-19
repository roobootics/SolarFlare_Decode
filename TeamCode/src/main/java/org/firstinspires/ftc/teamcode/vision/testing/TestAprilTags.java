package org.firstinspires.ftc.teamcode.vision.testing;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.vision.VisionControl;

@TeleOp
public class TestAprilTags extends OpMode {
    VisionControl visionControl;
    @Override
    public void init(){
        visionControl = new VisionControl(hardwareMap, telemetry);
    }

    @Override
    public void loop(){
        Integer id = visionControl.getObeliskID();
        Pose botPose = visionControl.getBotPoseMT1();
        String pipeLineName = visionControl.getCurrentPipelineType();

        if (id != null || botPose != null || pipeLineName != null){
            telemetry.addData("obelisk id", visionControl.getObeliskID());
            telemetry.addData("botPose", visionControl.getBotPoseMT1());
            telemetry.addData("pipeline type", visionControl.getCurrentPipelineType());
            telemetry.update();
        }
    }
}
