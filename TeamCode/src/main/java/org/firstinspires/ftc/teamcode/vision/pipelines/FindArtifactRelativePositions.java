package org.firstinspires.ftc.teamcode.vision.pipelines;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.teamcode.vision.descriptors.DetectionDescriptor;

import java.util.ArrayList;
import java.util.List;

public class FindArtifactRelativePositions {
    List<String> queryClassNames;
    Limelight3A limelight;
    double[][] K;
    final double CAMERA_VERTICAL_HEIGHT_INCHES;
    final double CAMERA_OFFSET_X_INCHES;
    final double CAMERA_OFFSET_Y_INCHES;
    final double CAMERA_DOWNWARD_ANGLE_DEGREES;
    public FindArtifactRelativePositions(List<String> queryClassNames,
                                         Limelight3A limelight,
                                         final double CAMERA_VERTICAL_HEIGHT_INCHES,
                                         final double CAMERA_OFFSET_X_INCHES,
                                         final double CAMERA_OFFSET_Y_INCHES,
                                         final double CAMERA_DOWNWARD_ANGLE_DEGREES,
                                         final double[][] K)
    {
        this.queryClassNames = queryClassNames;

        this.limelight = limelight;

        this.CAMERA_VERTICAL_HEIGHT_INCHES = CAMERA_VERTICAL_HEIGHT_INCHES;
        this.CAMERA_OFFSET_X_INCHES = CAMERA_OFFSET_X_INCHES;
        this.CAMERA_OFFSET_Y_INCHES = CAMERA_OFFSET_Y_INCHES;
        this.CAMERA_DOWNWARD_ANGLE_DEGREES = CAMERA_DOWNWARD_ANGLE_DEGREES;

        this.K = K;
    }

    public List<DetectionDescriptor> getDetectionDescriptors(){
        List<DetectionDescriptor> detectionDescriptors = new ArrayList<>(); // Output array
        LLResult result = limelight.getLatestResult();

        if (result == null || !result.isValid()){
            return detectionDescriptors;
        }

        for (LLResultTypes.DetectorResult detectorResult : result.getDetectorResults()){
            String className = detectorResult.getClassName();

            if (queryClassNames.contains(className)) {
                List<List<Double>> corners = detectorResult.getTargetCorners();
                double[] targetLocationPixels = new double[2];
                targetLocationPixels[0] = (corners.get(2).get(0) + corners.get(3).get(0)) / 2; // We want the target to be the bottom center of the bounding box
                targetLocationPixels[1] = (corners.get(3).get(1));

                double tx = Math.toDegrees(Math.atan2(targetLocationPixels[0] - K[0][2], K[0][0]));
                double ty = Math.toDegrees(Math.atan2(K[2][2] - targetLocationPixels[1], K[1][1])); // Y dimension of camera increases downward
                double verticalAngle = CAMERA_DOWNWARD_ANGLE_DEGREES + ty;

                double depth = CAMERA_VERTICAL_HEIGHT_INCHES * Math.tan(Math.toRadians(verticalAngle)) + CAMERA_OFFSET_Y_INCHES + 2.5; // 2.5 is to find the location of the center of the artifact

                double horizontalOffset = depth * Math.tan(Math.toRadians(tx)) + CAMERA_OFFSET_X_INCHES;

                double[] data = new double[10];

                DetectionDescriptor detection = new DetectionDescriptor(tx, ty, className, horizontalOffset, depth, corners, targetLocationPixels, data);
                detectionDescriptors.add(detection);
            }
        }
        return detectionDescriptors;
    }
}
