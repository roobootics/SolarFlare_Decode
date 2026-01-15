package org.firstinspires.ftc.teamcode.vision.testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.vision.VisionControl;

@TeleOp
public class TestAprilTags extends OpMode {
    VisionControl visionControl = new VisionControl(hardwareMap, telemetry);
    @Override
    public void init(){

    }

    @Override
    public void loop(){
        telemetry.addData("obelisk id", visionControl.getObeliskID());
        telemetry.addData("botpose", visionControl.getBotPoseMT1());
        telemetry.addData("pipeline type", visionControl.getCurrentPipelineType());
        telemetry.update();
    }
}
