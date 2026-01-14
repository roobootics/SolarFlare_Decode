package org.firstinspires.ftc.teamcode.vision.testing;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous
public class TestResult extends OpMode {
    Limelight3A limelight;
    @Override
    public void init(){
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(11);
        limelight.pipelineSwitch(0);
    }

    @Override
    public void start(){
        limelight.start();
    }
    @Override
    public void loop(){
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()){
            for (LLResultTypes.DetectorResult detection : result.getDetectorResults()){
                telemetry.addData("tx", detection.getTargetXDegrees());
                telemetry.addData("ty", detection.getTargetYDegrees());
                telemetry.addData("pipeline index", result.getPipelineIndex());
                telemetry.addData("class name", detection.getClassName());
            }
        }
        else {
            telemetry.addLine("Nothing :(");
            telemetry.addData("null", result == null);
            telemetry.addData("is running", limelight.isRunning());
        }
        telemetry.update();
    }

}
