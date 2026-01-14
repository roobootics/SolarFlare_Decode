package org.firstinspires.ftc.teamcode.vision.pipelines;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.teamcode.vision.descriptors.AprilTagDescriptor;

import java.util.ArrayList;
import java.util.List;

public class AprilTagDetector {
    Limelight3A limelight;
    public AprilTagDetector(Limelight3A limelight){
        this.limelight = limelight;
    }
    public List<AprilTagDescriptor> getAprilTagDescriptors(){
        List<AprilTagDescriptor> aprilTagDescriptors = new ArrayList<>();
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()){
            for (LLResultTypes.FiducialResult fiducialResult : result.getFiducialResults()){
                AprilTagDescriptor aprilTagDescriptor = new AprilTagDescriptor(fiducialResult.getTargetXDegrees(), fiducialResult.getTargetYDegrees(), fiducialResult.getFiducialId());
                aprilTagDescriptors.add(aprilTagDescriptor);
            }
            return aprilTagDescriptors;
        }
        return null;
    }
}
