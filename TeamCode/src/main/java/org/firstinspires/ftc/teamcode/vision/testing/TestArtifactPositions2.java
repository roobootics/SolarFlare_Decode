package org.firstinspires.ftc.teamcode.vision.testing;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.vision.descriptors.DetectionDescriptor;
import org.firstinspires.ftc.teamcode.vision.pipelines.FindArtifactRelativePositions;

import java.util.ArrayList;
import java.util.List;


@Autonomous
public class TestArtifactPositions2 extends OpMode {
    Limelight3A limelight;
    FindArtifactRelativePositions detector;
    final double CAMERA_VERTICAL_HEIGHT_INCHES = 5; // Increases up from reference point
    final double CAMERA_OFFSET_X_INCHES = 0; // Increases to the right from reference point
    final double CAMERA_OFFSET_Y_INCHES = 0; // Increases forward from the reference point
    final double CAMERA_DOWNWARD_PITCH_DEGREES = 65; // 90 degrees is facing straight forward, decreases looking down
    double fx = 1218.145;
    double fy = 1219.481;
    double cx = 621.829;
    double cy = 500.362;
    final double[][] K = {
            {fx, 0.0, cx},
            {0.0, fy, cy},
            {0.0, 0.0, 1.0}
    };

    @Override
    public void init(){
        List<String> queryClasses = new ArrayList<>();
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(11);
        limelight.pipelineSwitch(0);
        queryClasses.add("purple");
        queryClasses.add("green");
        detector = new FindArtifactRelativePositions(
                queryClasses, limelight,
                CAMERA_VERTICAL_HEIGHT_INCHES,
                CAMERA_OFFSET_X_INCHES,
                CAMERA_OFFSET_Y_INCHES,
                CAMERA_DOWNWARD_PITCH_DEGREES,
                K);
    }

    @Override
    public void loop(){
        if (!limelight.isRunning()){
            limelight.start();
        }
        List<DetectionDescriptor> detections = detector.getDetectionDescriptors();
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
