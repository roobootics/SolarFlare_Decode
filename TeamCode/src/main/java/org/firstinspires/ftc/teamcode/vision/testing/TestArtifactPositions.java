package org.firstinspires.ftc.teamcode.vision.testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.vision.VisionControl;
import org.firstinspires.ftc.teamcode.vision.descriptors.DetectionDescriptor;

import java.util.List;

@TeleOp
public class TestArtifactPositions extends OpMode {
    VisionControl visionControl;

    @Override
    public void init(){
        visionControl = new VisionControl(hardwareMap, telemetry);
    }

    @Override
    public void loop(){
        List<DetectionDescriptor> detections = visionControl.getDetectionDescriptors();
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
