package org.firstinspires.ftc.teamcode.vision;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.vision.descriptors.AprilTagDescriptor;
import org.firstinspires.ftc.teamcode.vision.descriptors.ArtifactDescriptor;

import java.util.ArrayList;
import java.util.List;

public class Vision {
    final double METERS_TO_INCHES = 39.3700787402;
    // Translation from reference point, usually the lowest center middle of the robot to center of Limelight lens
    final double CAMERA_VERTICAL_HEIGHT_INCHES = 0.2225 * METERS_TO_INCHES ; // Increases up from reference point TODO: find translations
    final double CAMERA_OFFSET_X_INCHES = 0; // Increases to the right from reference point
    final double CAMERA_OFFSET_Y_INCHES = 0.182 * METERS_TO_INCHES; // Increases forward from the reference point
    final double CAMERA_DOWNWARD_PITCH_DEGREES = 0; // 90 degrees is facing straight forward, decreases looking down
    final int NN_PIPELINE_INDEX = 0;
    final int APRIL_TAGS_PIPELINE_INDEX = 1;
    double fx = 1218.145;
    double fy = 1219.481;
    double cx = 621.829;
    double cy = 500.362;
    Limelight3A limelight;
    List<String> queryClasses = new ArrayList<>();
    Telemetry telemetry;
    public Vision(HardwareMap hardwareMap, Telemetry telemetry){
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        this.telemetry = telemetry;
        limelight.setPollRateHz(11);
        queryClasses.add("purple");
        queryClasses.add("green");
    }

    public List<ArtifactDescriptor> getArtifactDescriptors(Pose2D botPose){
        if (!limelight.isRunning()){
            limelight.start();
        }
        limelight.pipelineSwitch(NN_PIPELINE_INDEX);

        List<ArtifactDescriptor> artifactDescriptors = new ArrayList<>(); // Output array
        LLResult result = limelight.getLatestResult();

        if (result == null || !result.isValid()) {
            return artifactDescriptors;
        }

        for (LLResultTypes.DetectorResult detectorResult : result.getDetectorResults()) {
            String className = detectorResult.getClassName();

            if (queryClasses.contains(className)) {
                List<List<Double>> corners = detectorResult.getTargetCorners();
                double[] targetLocationPixels = new double[2];
                targetLocationPixels[0] = (corners.get(2).get(0) + corners.get(3).get(0)) / 2; // We want the target to be the bottom center of the bounding box
                targetLocationPixels[1] = (corners.get(3).get(1));

                double tx = Math.toDegrees(Math.atan2(targetLocationPixels[0] - cx, fx));
                double ty = Math.toDegrees(Math.atan2(cy - targetLocationPixels[1], fy)); // Y dimension of camera increases downward
                double verticalAngle = CAMERA_DOWNWARD_PITCH_DEGREES + ty;

                telemetry.addData("tx", tx);
                telemetry.addData("ty", ty);

                double depth = CAMERA_VERTICAL_HEIGHT_INCHES * Math.tan(Math.toRadians(verticalAngle)) + CAMERA_OFFSET_Y_INCHES + 2.5; // 2.5 is to find the location of the center of the artifact TODO: Tune this

                double horizontalOffset = depth * Math.tan(Math.toRadians(tx)) + CAMERA_OFFSET_X_INCHES;

                double x = botPose.getX(DistanceUnit.INCH);
                double y = botPose.getY(DistanceUnit.INCH);
                double headingRadians = botPose.getHeading(AngleUnit.RADIANS);

                double artifactX = x + horizontalOffset * Math.cos(headingRadians) - depth * Math.sin(headingRadians);
                double artifactY = y + horizontalOffset * Math.sin(headingRadians) + depth * Math.cos(headingRadians);

                artifactDescriptors.add(new ArtifactDescriptor(artifactX, artifactY, className));
            }
        }
        return artifactDescriptors;
    }

    public Pose getBotPoseMT1(){
        if (!limelight.isRunning()){
            limelight.start();
        }
        limelight.pipelineSwitch(APRIL_TAGS_PIPELINE_INDEX);

        Pose3D llPose;

        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()){
            llPose = result.getBotpose();
        }
        else return null;

        return limelightToPedroPose(llPose);
    }

    public Pose getBotPoseMT2(double yawDegreesStandard){
        if (!limelight.isRunning()){
            limelight.start();
        }
        limelight.pipelineSwitch(APRIL_TAGS_PIPELINE_INDEX);

        limelight.updateRobotOrientation(standardToLimelightYaw(yawDegreesStandard));

        Pose3D llPose;

        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()){
            llPose = result.getBotpose_MT2();
        }
        else return null;

        return limelightToPedroPose(llPose);
    }
    public Pose getBotPoseMT2WithMT1() {
        if (!limelight.isRunning()) {
            limelight.start();
        }
        limelight.pipelineSwitch(APRIL_TAGS_PIPELINE_INDEX);

        Pose botPoseMT1 = getBotPoseMT1();

        if (botPoseMT1 != null) {
            double yaw = Math.toDegrees(botPoseMT1.getHeading());

            double llYaw = standardToLimelightYaw(yaw);

            limelight.updateRobotOrientation(llYaw);

            Pose3D llPose;

            LLResult result = limelight.getLatestResult();

            if (result != null && result.isValid()) {
                llPose = result.getBotpose_MT2();
            } else return null;

            return limelightToPedroPose(llPose);
        }
        return null;
    }

    public Pose limelightToPedroPose(Pose3D llPose){
        double llX = llPose.getPosition().toUnit(DistanceUnit.INCH).x;
        double llY = llPose.getPosition().toUnit(DistanceUnit.INCH).y;
        double heading = llPose.getOrientation().getYaw(AngleUnit.DEGREES);

        double xTransformed = llY + 72;
        double yTransformed = -llX + 72;

        double yawTransformed = (heading + 270) % 360;

        if (yawTransformed < 0) {
            yawTransformed += 360;
        }

        yawTransformed = Math.toRadians(yawTransformed);

        return new Pose(xTransformed, yTransformed, yawTransformed);
    }

    public double standardToLimelightYaw(double standardYaw){
        double yawTransformed = standardYaw + 90;

        if (yawTransformed >= 180.0) {
            yawTransformed -= 360.0;
        }
        return yawTransformed;
    }

    public double limelightToStandardYaw(double llYaw){
        double yawTransformed = (llYaw + 270) % 360;

        if (yawTransformed < 0) {
            yawTransformed += 360;
        }
        return yawTransformed;
    }

    public void stopLimelight(){
        limelight.stop();
    }

    public String getCurrentPipelineType() {
        if (!limelight.isRunning()) {
            limelight.start();
        }
        String pipelineName = limelight.getStatus().getPipelineType();

        if (pipelineName.isEmpty()) return null;
        else return pipelineName;
    }

    public Integer getObeliskID(){
        if (!limelight.isRunning()) {
            limelight.start();
        }
        limelight.pipelineSwitch(APRIL_TAGS_PIPELINE_INDEX);

        List<AprilTagDescriptor> tags = getAprilTagDescriptors();

        if (tags != null){
            for (AprilTagDescriptor tag : tags){
                int id = tag.getId();
                if (id == 21 || id == 22 || id == 33){
                    return id;
                }
            }
        }
        telemetry.addLine("ID not in sight");
        return null;
    }
    public List<AprilTagDescriptor> getAprilTagDescriptors(){
        if (!limelight.isRunning()){
            limelight.start();
        }
        limelight.pipelineSwitch(APRIL_TAGS_PIPELINE_INDEX);

        List<AprilTagDescriptor> aprilTagDescriptors = new ArrayList<>();
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()){
            for (LLResultTypes.FiducialResult fiducialResult : result.getFiducialResults()){
                AprilTagDescriptor aprilTagDescriptor = new AprilTagDescriptor(fiducialResult.getTargetXDegrees(), fiducialResult.getTargetYDegrees(), fiducialResult.getFiducialId());
                aprilTagDescriptors.add(aprilTagDescriptor);
            }
        }
        return aprilTagDescriptors;
    }
}
