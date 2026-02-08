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

// TODO: calculate bounding box offset for artifacts
public class Vision {

    // Config ---------------------
    CAMERA_ORIENTATION orientation = CAMERA_ORIENTATION.NORMAL;
    public final double CAMERA_WIDTH_PIXELS = 1280;
    public final double CAMERA_HEIGHT_PIXELS = 960;
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
    // ----------------------------

    Limelight3A limelight;
    List<String> queryClasses = new ArrayList<>();
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
        queryClasses.add("purple");
        queryClasses.add("green");
    }

    public Vision(HardwareMap hardwareMap, Telemetry telemetry, Pose3D cameraPoseOnRobot, CAMERA_ORIENTATION orientation){
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        this.telemetry = telemetry;
        limelight.setPollRateHz(11);
        queryClasses.add("purple");
        queryClasses.add("green");
        this.cameraPoseOnRobot = cameraPoseOnRobot;
        this.orientation = orientation;
    }

    public List<ArtifactDescriptor> getRelativeArtifactDescriptors(){
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
            List<Double> corner0 = corners.get(0);
            List<Double> corner1 = corners.get(1);
            List<Double> corner2 = corners.get(2);
            List<Double> corner3 = corners.get(3);

            telemetry.addData("corner 0", corner0);
            telemetry.addData("corner 1", corner1);
            telemetry.addData("corner 2", corner2);
            telemetry.addData("corner 3", corner3);


            if (queryClasses.contains(className)) {
                double targetX = 0;
                double targetY = 0;

                if (orientation == CAMERA_ORIENTATION.NORMAL){
                    targetX = (corner2.get(0) + corner3.get(0)) / 2;
                    targetY = corner3.get(1);
                }
                else if (orientation == CAMERA_ORIENTATION.UPSIDE_DOWN){
                    targetX = (corner0.get(0) + corner1.get(0)) / 2;
                    targetY = corner1.get(1);
                }
                else if (orientation == CAMERA_ORIENTATION.CLOCKWISE_90){
                    targetX = (corner1.get(0) + corner2.get(0)) / 2;
                    targetY = corner2.get(1);
                }
                else if (orientation == CAMERA_ORIENTATION.COUNTER_CLOCKWISE_90){
                    targetX = (corner3.get(0) + corner0.get(0)) / 2;
                    targetY = corner0.get(1);
                }

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

                telemetry.addData("camera orientation", orientation);

                double verticalAngleDeg = 90 + ty + cameraPoseOnRobot.getOrientation().getPitch(AngleUnit.DEGREES);

                telemetry.addData("vertical angle", verticalAngleDeg);

                double depth = cameraPoseOnRobot.getPosition().toUnit(DistanceUnit.INCH).z * Math.tan(Math.toRadians(verticalAngleDeg))
                        + cameraPoseOnRobot.getPosition().toUnit(DistanceUnit.INCH).x;
                // artifact center offset TODO: Tune this

                double horizontal = depth * Math.tan(Math.toRadians(tx + cameraPoseOnRobot.getOrientation().getYaw(AngleUnit.DEGREES))) + cameraPoseOnRobot.getPosition().toUnit(DistanceUnit.INCH).y;
                artifactDescriptors.add(new ArtifactDescriptor(horizontal, depth, className));
            }
        }
        return artifactDescriptors;
    }

    public List<ArtifactDescriptor> getArtifactDescriptors(Pose botPosePedro){
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
            List<Double> corner0 = corners.get(0);
            List<Double> corner1 = corners.get(1);
            List<Double> corner2 = corners.get(2);
            List<Double> corner3 = corners.get(3);

            telemetry.addData("corner 0", corner0);
            telemetry.addData("corner 1", corner1);
            telemetry.addData("corner 2", corner2);
            telemetry.addData("corner 3", corner3);


            if (queryClasses.contains(className)) {
                double targetX = 0;
                double targetY = 0;

                if (orientation == CAMERA_ORIENTATION.NORMAL){
                    targetX = (corner2.get(0) + corner3.get(0)) / 2;
                    targetY = corner3.get(1);
                }
                else if (orientation == CAMERA_ORIENTATION.UPSIDE_DOWN){
                    targetX = (corner0.get(0) + corner1.get(0)) / 2;
                    targetY = corner1.get(1);
                }
                else if (orientation == CAMERA_ORIENTATION.CLOCKWISE_90){
                    targetX = (corner1.get(0) + corner2.get(0)) / 2;
                    targetY = corner2.get(1);
                }
                else if (orientation == CAMERA_ORIENTATION.COUNTER_CLOCKWISE_90){
                    targetX = (corner3.get(0) + corner0.get(0)) / 2;
                    targetY = corner0.get(1);
                }

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

                telemetry.addData("camera orientation", orientation);

                double verticalAngleDeg = 90 + ty + cameraPoseOnRobot.getOrientation().getPitch(AngleUnit.DEGREES);

                telemetry.addData("vertical angle", verticalAngleDeg);

                double depth = cameraPoseOnRobot.getPosition().toUnit(DistanceUnit.INCH).z * Math.tan(Math.toRadians(verticalAngleDeg))
                        + cameraPoseOnRobot.getPosition().toUnit(DistanceUnit.INCH).x;
                // artifact center offset TODO: Tune this

                double horizontal = depth * Math.tan(Math.toRadians(tx + cameraPoseOnRobot.getOrientation().getYaw(AngleUnit.DEGREES))) + cameraPoseOnRobot.getPosition().toUnit(DistanceUnit.INCH).y;

                double x = botPosePedro.getX();
                double y = botPosePedro.getY();
                double theta = botPosePedro.getHeading();

                double cos = Math.cos(theta);
                double sin = Math.sin(theta);

                double artifactX =
                        x + depth * cos - horizontal * sin;

                double artifactY =
                        y + depth * sin + horizontal * cos;

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
}
