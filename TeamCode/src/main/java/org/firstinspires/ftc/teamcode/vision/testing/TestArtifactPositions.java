package org.firstinspires.ftc.teamcode.vision.testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.vision.VisionManager;
import org.firstinspires.ftc.teamcode.vision.descriptors.DetectionDescriptor;

import java.util.List;


@Autonomous
public class TestArtifactPositions extends OpMode {
    VisionManager visionManager;

    @Override
    public void init(){
        visionManager = new VisionManager(hardwareMap, telemetry);
    }

    @Override
    public void loop(){
        List<DetectionDescriptor> detections = visionManager.getDetectionDescriptors();
        if (detections.isEmpty()){
            telemetry.addLine("No detections");
        }
        for (DetectionDescriptor detection :  detections) {
            telemetry.addData("class", detection.getClassName());
            telemetry.addData("leftRightOffset", detection.getLeftRightOffset());
            telemetry.addData("y", detection.getForwardOffset());
            telemetry.addData("tx", detection.getTx());
            telemetry.addData("ty", detection.getTy());
            telemetry.addData("target pixels leftRightOffset", detection.getTargetPixels()[0]);
            telemetry.addData("target pixels y", detection.getTargetPixels()[1]);
        }
        telemetry.update();
    }
}
