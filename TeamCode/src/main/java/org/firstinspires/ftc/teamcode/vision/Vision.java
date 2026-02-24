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
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.vision.descriptors.AprilTagDescriptor;
import org.firstinspires.ftc.teamcode.vision.descriptors.ArtifactDescriptor;

import java.util.ArrayList;
import java.util.List;

public class Vision {
    CAMERA_ORIENTATION cameraOrientation = CAMERA_ORIENTATION.NORMAL;
    public double intakingWidthInches = 13;
    public double fx = 1218.145;
    public double fy = 1219.418;
    public double cx = 621.829;
    public double cy = 500.362;
    public double fxNN = fx / 2;
    public double fyNN = fy / 2;
    public double cxNN = cx / 2;
    public double cyNN = cy / 2;
    final int NN_PIPELINE_INDEX = 0;
    final int APRIL_TAGS_PIPELINE_INDEX = 1;
    Pose3D cameraPoseOnRobot =  new Pose3D(new Position(DistanceUnit.METER, 0.182, 0, 0.2225, 0), new YawPitchRollAngles(AngleUnit.DEGREES, 0, 0, 0, 0));
    Limelight3A limelight;
    Telemetry telemetry;

    public enum CAMERA_ORIENTATION{
        NORMAL,
        UPSIDE_DOWN,
        CLOCKWISE_90,
        COUNTER_CLOCKWISE_90
    }

    public Vision(HardwareMap hardwareMap, Telemetry telemetry){
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        this.telemetry = telemetry;
        limelight.setPollRateHz(11);
    }

    public Vision(HardwareMap hardwareMap, Telemetry telemetry, Pose3D cameraPoseOnRobot, CAMERA_ORIENTATION cameraOrientation){
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        this.telemetry = telemetry;
        limelight.setPollRateHz(11);
        this.cameraPoseOnRobot = cameraPoseOnRobot;
        this.cameraOrientation = cameraOrientation;
    }

    public List<ArtifactDescriptor> getRelativeArtifactDescriptors(List<String> acceptedClasses){
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
            List<List<Double>> corners = detectorResult.getTargetCorners();

            telemetry.addData("corner 0", corners.get(0));
            telemetry.addData("corner 1", corners.get(1));
            telemetry.addData("corner 2", corners.get(2));
            telemetry.addData("corner 3", corners.get(3));

            if (acceptedClasses.contains(className)) {
                double targetX = getTargetXPixels(cameraOrientation, corners);
                double targetY = getTargetYPixels(cameraOrientation, corners);

                double xOffset = targetX - cxNN;
                double yOffset = cyNN - targetY;

                double tx = Math.toDegrees(Math.atan(xOffset / fxNN));
                double ty = Math.toDegrees(Math.atan(yOffset / fyNN));

                telemetry.addData("tx", tx);
                telemetry.addData("ty", ty);
                telemetry.addData("cx", cxNN);
                telemetry.addData("cy", cyNN);
                telemetry.addData("targetX", targetX);
                telemetry.addData("targetY", targetY);
                telemetry.addData("xOffset", xOffset);
                telemetry.addData("yOffset", yOffset);

                double verticalAngleDeg = 90 + ty + cameraPoseOnRobot.getOrientation().getPitch(AngleUnit.DEGREES);

                telemetry.addData("vertical angle", verticalAngleDeg);

                double depth = cameraPoseOnRobot.getPosition().toUnit(DistanceUnit.INCH).z * Math.tan(Math.toRadians(verticalAngleDeg));

                double horizontal = depth * Math.tan(Math.toRadians(tx + cameraPoseOnRobot.getOrientation().getYaw(AngleUnit.DEGREES)));

                depth += cameraPoseOnRobot.getPosition().toUnit(DistanceUnit.INCH).x;
                horizontal += cameraPoseOnRobot.getPosition().toUnit(DistanceUnit.INCH).y;

                ArtifactDescriptor artifact = new ArtifactDescriptor(horizontal, depth, className);

                artifact.setTargetXPixels(targetX);
                artifact.setTargetYPixels(targetY);

                artifactDescriptors.add(artifact);
            }
        }
        return artifactDescriptors;
    }

    public List<ArtifactDescriptor> getArtifactDescriptors(Pose botPosePedro, List<String> acceptedClasses){
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


            if (acceptedClasses.contains(className)) {

                List<List<Double>> corners = detectorResult.getTargetCorners();

                double targetX = getTargetXPixels(cameraOrientation, corners);
                double targetY = getTargetYPixels(cameraOrientation, corners);

                double xOffset = targetX - cxNN;
                double yOffset = cyNN - targetY;

                double tx = Math.toDegrees(Math.atan(xOffset / fxNN));
                double ty = Math.toDegrees(Math.atan(yOffset / fyNN));

                double verticalAngleDeg = 90 + ty + cameraPoseOnRobot.getOrientation().getPitch(AngleUnit.DEGREES);

                double depth = cameraPoseOnRobot.getPosition().toUnit(DistanceUnit.INCH).z * Math.tan(Math.toRadians(verticalAngleDeg));

                double horizontal = depth * Math.tan(Math.toRadians(tx + cameraPoseOnRobot.getOrientation().getYaw(AngleUnit.DEGREES)));

                depth += cameraPoseOnRobot.getPosition().toUnit(DistanceUnit.INCH).x;
                horizontal += cameraPoseOnRobot.getPosition().toUnit(DistanceUnit.INCH).y;

                double x = botPosePedro.getX();
                double y = botPosePedro.getY();
                double theta = botPosePedro.getHeading();

                double cos = Math.cos(theta);
                double sin = Math.sin(theta);

                double artifactX =
                        x + depth * cos - (-horizontal) * sin;

                double artifactY =
                        y + depth * sin + (-horizontal) * cos;

                ArtifactDescriptor artifact = new ArtifactDescriptor(artifactX, artifactY, className);

                artifact.setTargetXPixels(targetX);
                artifact.setTargetYPixels(targetY);

                artifactDescriptors.add(artifact);

                telemetry.addData("corner 0", corners.get(0));
                telemetry.addData("corner 1", corners.get(1));
                telemetry.addData("corner 2", corners.get(2));
                telemetry.addData("corner 3", corners.get(3));
                telemetry.addData("targetX", targetX);
                telemetry.addData("targetY", targetY);
                telemetry.addData("cx", cxNN);
                telemetry.addData("cy", cyNN);
                telemetry.addData("xOffset", xOffset);
                telemetry.addData("yOffset", yOffset);
                telemetry.addData("tx", tx);
                telemetry.addData("ty", ty);
                telemetry.addData("vertical angle", verticalAngleDeg);
                telemetry.addData("horizontal", horizontal);
                telemetry.addData("depth", depth);
            }
        }
        return artifactDescriptors;
    }

    public Pose getBotPoseMT1(){
        if (!limelight.isRunning()){
            limelight.start();
        }
        limelight.pipelineSwitch(APRIL_TAGS_PIPELINE_INDEX);

        Pose3D botPose;

        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()){
            botPose = result.getBotpose();
        }
        else return null;

        return limelightToPedroPose(botPose);
    }

    public Pose getBotPoseMT2(double yawDegreesStandard){
        if (!limelight.isRunning()){
            limelight.start();
        }
        limelight.pipelineSwitch(APRIL_TAGS_PIPELINE_INDEX);

        limelight.updateRobotOrientation(standardToLimelightYaw(yawDegreesStandard));

        Pose3D botPose;

        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()){
            botPose = result.getBotpose_MT2();
        }
        else return null;

        return limelightToPedroPose(botPose);
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

            Pose3D botPose;

            LLResult result = limelight.getLatestResult();

            if (result != null && result.isValid()) {
                botPose = result.getBotpose_MT2();
            } else return null;

            return limelightToPedroPose(botPose);
        }
        return null;
    }

    public Pose limelightToPedroPose(Pose3D llPose){
        double llX = llPose.getPosition().toUnit(DistanceUnit.INCH).x;
        double llY = llPose.getPosition().toUnit(DistanceUnit.INCH).y;
        double llYaw = llPose.getOrientation().getYaw(AngleUnit.DEGREES);

        double xTransformed = llY + 72;
        double yTransformed = 72 - llX;
        double yawTransformed = limelightToStandardYaw(llYaw);

        yawTransformed = Math.toRadians(yawTransformed);

        return new Pose(xTransformed, yTransformed, yawTransformed);
    }

    public Pose3D pedroToLimelightPose(Pose pedroPose){
        double x = pedroPose.getX();
        double y = pedroPose.getY();
        double yaw = Math.toDegrees(pedroPose.getHeading());

        double llX = 72 - y;
        double llY = x - 72;
        double llYaw = standardToLimelightYaw(yaw);

        return new Pose3D(new Position(DistanceUnit.INCH, llX, llY, 0, 0), new YawPitchRollAngles(AngleUnit.DEGREES, llYaw, 0, 0, 0));
    }

    public Pose2D pedroToStandardPose(Pose pedroPose){
        double x = pedroPose.getX();
        double y = pedroPose.getY();
        double yaw = pedroPose.getHeading();

        double xTransformed = 72 - y;
        double yTransformed = x - 72;
        double yawTransformed = Math.toDegrees(yaw);

        return new Pose2D(DistanceUnit.INCH, xTransformed, yTransformed, AngleUnit.DEGREES, yawTransformed);
    }

    public Pose standardToPedroPose(Pose2D standardPose){
        double x = standardPose.getX(DistanceUnit.INCH);
        double y = standardPose.getY(DistanceUnit.INCH);
        double yaw = standardPose.getHeading(AngleUnit.DEGREES);

        double xTransformed = y + 72;
        double yTransformed = 72 - x;
        double yawTransformed = Math.toRadians(yaw);

        return new Pose(xTransformed, yTransformed, yawTransformed);
    }

    public double standardToLimelightYaw(double standardYawDegrees){
        double yawTransformed = standardYawDegrees + 90;

        if (yawTransformed >= 180.0) {
            yawTransformed -= 360.0;
        }
        return yawTransformed;
    }

    public double limelightToStandardYaw(double llYawDegrees){
        double yawTransformed = (llYawDegrees + 270) % 360;

        if (yawTransformed < 0) {
            yawTransformed += 360;
        }
        return yawTransformed;
    }

    public List<ArtifactDescriptor> pedroToStandardPoseArtifacts(List<ArtifactDescriptor> artifacts){
        List<ArtifactDescriptor> out = new ArrayList<>();
        for (ArtifactDescriptor artifactDescriptor : artifacts){
            double x = artifactDescriptor.getX();
            double y = artifactDescriptor.getY();
            String className = artifactDescriptor.getClassName();

            double xTransformed = 72 - y;
            double yTransformed = x - 72;

            out.add(new ArtifactDescriptor(xTransformed, yTransformed, className));
        }
        return out;
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

    public Pose intakeOneBall(List<ArtifactDescriptor> artifacts, Pose botPose) {
        double smallestDist = Double.POSITIVE_INFINITY;
        int indexOfClosestArtifact = 0;

        if (artifacts == null || artifacts.isEmpty() || botPose == null) return null;

        for (int i = 0; i < artifacts.size(); i++){
            double artifactX = artifacts.get(i).getX();
            double artifactY = artifacts.get(i).getY();

            double botX = botPose.getX();
            double botY = botPose.getY();

            double dist = Math.sqrt(Math.pow(artifactX - botX, 2) + Math.pow(artifactY - botY, 2));

            if (dist < smallestDist) {
                smallestDist = dist;
                indexOfClosestArtifact = i;
            }
        }

        ArtifactDescriptor closestArtifact = artifacts.get(indexOfClosestArtifact);

        return new Pose(closestArtifact.getX(), closestArtifact.getY());
    }

    public Double intakingAngleArtifacts(List<ArtifactDescriptor> artifacts, Pose botPose){
        int bestCount = -1;
        double bestDist = Double.POSITIVE_INFINITY;
        double bestAngle = -1;

        for (ArtifactDescriptor artifact : artifacts) {

            double dx = artifact.getX() - botPose.getX();
            double dy = artifact.getY() - botPose.getY();
            double theta = Math.atan2(dy, dx);
            double cos = Math.cos(theta);
            double sin = Math.sin(theta);

            int count = 0;

            for (ArtifactDescriptor surroundingArtifact : artifacts) {

                double rx = surroundingArtifact.getX() - botPose.getX();
                double ry = surroundingArtifact.getY() - botPose.getY();

                double distance = Math.abs(rx * sin - ry * cos);

                if (distance < intakingWidthInches / 2) {
                    count++;
                }
            }

            double distanceToRobot = Math.sqrt(Math.pow(botPose.getX() - artifact.getX(), 2) + Math.pow(botPose.getY() - artifact.getY(), 2));

            if (count > bestCount || (count == bestCount && distanceToRobot < bestDist) ) {
                bestCount = count;
                bestDist = distanceToRobot;
                bestAngle = theta;
            }
        }

        double angleDegrees = Math.toDegrees(bestAngle);

        return angleDegrees;
    }

    public Double getTargetXPixels(Vision.CAMERA_ORIENTATION orientation, List<List<Double>> corners) {
        if (corners == null) return null;

        double targetXPixels = 0;

        List<Double> corner0 = corners.get(0);
        List<Double> corner1 = corners.get(1);
        List<Double> corner2 = corners.get(2);
        List<Double> corner3 = corners.get(3);

        if (orientation == Vision.CAMERA_ORIENTATION.NORMAL){
            targetXPixels = (corner2.get(0) + corner3.get(0)) / 2;
        }
        else if (orientation == Vision.CAMERA_ORIENTATION.UPSIDE_DOWN){
            targetXPixels = (corner0.get(0) + corner1.get(0)) / 2;
        }
        else if (orientation == Vision.CAMERA_ORIENTATION.CLOCKWISE_90){
            targetXPixels = (corner1.get(0) + corner2.get(0)) / 2;
        }
        else if (orientation == Vision.CAMERA_ORIENTATION.COUNTER_CLOCKWISE_90){
            targetXPixels = (corner3.get(0) + corner0.get(0)) / 2;
        }

        return targetXPixels;
    }
    public Double getTargetYPixels(Vision.CAMERA_ORIENTATION orientation, List<List<Double>> corners) {
        if (corners == null) return null;

        double targetYPixels = 0;

        List<Double> corner0 = corners.get(0);
        List<Double> corner1 = corners.get(1);
        List<Double> corner2 = corners.get(2);
        List<Double> corner3 = corners.get(3);

        if (orientation == Vision.CAMERA_ORIENTATION.NORMAL){
            targetYPixels = corner3.get(1);
        }
        else if (orientation == Vision.CAMERA_ORIENTATION.UPSIDE_DOWN){
            targetYPixels = corner1.get(1);
        }
        else if (orientation == Vision.CAMERA_ORIENTATION.CLOCKWISE_90){
            targetYPixels = corner2.get(1);
        }
        else if (orientation == Vision.CAMERA_ORIENTATION.COUNTER_CLOCKWISE_90){
            targetYPixels = corner0.get(1);
        }

        return targetYPixels;
    }
}
