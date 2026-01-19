package org.firstinspires.ftc.teamcode.vision;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.vision.descriptors.AprilTagDescriptor;
import org.firstinspires.ftc.teamcode.vision.descriptors.ArtifactDescriptor;
import org.firstinspires.ftc.teamcode.vision.descriptors.DetectionDescriptor;

import java.util.ArrayList;
import java.util.List;

public class VisionControl {
    // Translation from reference point, usually the lowest center middle of the robot for pedro, to center of Limelight lens
    final double CAMERA_VERTICAL_HEIGHT_INCHES = 5; // Increases up from reference point
    final double CAMERA_OFFSET_X_INCHES = 0; // Increases to the right from reference point
    final double CAMERA_OFFSET_Y_INCHES = 0; // Increases forward from the reference point
    final double CAMERA_DOWNWARD_PITCH_DEGREES = 65; // 90 degrees is facing straight forward, decreases looking down
    final int NN_PIPELINE_INDEX = 0;
    final int APRIL_TAGS_PIPELINE_INDEX = 1;
    double fx = 1218.145;
    double fy = 1219.481;
    double cx = 621.829;
    double cy = 500.362;
    final double[][] K = {
            {fx, 0.0, cx},
            {0.0, fy, cy},
            {0.0, 0.0, 1.0}
    };
    Limelight3A limelight;
    GoBildaPinpointDriver pinpoint;
    List<String> queryClasses = new ArrayList<>();
    Telemetry telemetry;
    FtcDashboard ftcDashboard = FtcDashboard.getInstance();
    public VisionControl(HardwareMap hardwareMap, Telemetry telemetry){
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        this.telemetry = telemetry;
        limelight.setPollRateHz(11);
        //pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        //pinpoint.resetPosAndIMU();
        queryClasses.add("purple");
        queryClasses.add("green");
    }

    public List<ArtifactDescriptor> getArtifactDescriptors(Pose botPose){ // Converts relative positions to absolute
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
                double ty = Math.toDegrees(Math.atan2(1.0 - targetLocationPixels[1], fy)); // Y dimension of camera increases downward
                double verticalAngle = CAMERA_DOWNWARD_PITCH_DEGREES + ty;

                double depth = CAMERA_VERTICAL_HEIGHT_INCHES * Math.tan(Math.toRadians(verticalAngle)) + CAMERA_OFFSET_Y_INCHES + 2.5; // 2.5 is to find the location of the center of the artifact TODO: Tune this

                double horizontalOffset = depth * Math.tan(Math.toRadians(tx)) + CAMERA_OFFSET_X_INCHES;

                double x = botPose.getX();
                double y = botPose.getY();
                double heading = Math.toRadians(botPose.getHeading());

                double artifactX = x + horizontalOffset * Math.cos(heading) - depth * Math.sin(heading);
                double artifactY = y + horizontalOffset * Math.sin(heading) + depth * Math.cos(heading);

                artifactDescriptors.add(new ArtifactDescriptor(artifactX, artifactY, className));

            }
            return artifactDescriptors;
        }
        return null;
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

        TelemetryPacket packet = new TelemetryPacket();

        telemetry.addData("pipeline index", limelight.getStatus().getPipelineIndex());
        telemetry.addData("pipeline type", limelight.getStatus().getPipelineType());
        telemetry.addData("isRunning", limelight.isRunning());

        if (llPose != null) {
            double xRaw = llPose.getPosition().toUnit(DistanceUnit.INCH).x;
            double yRaw = llPose.getPosition().toUnit(DistanceUnit.INCH).y;
            double yawRaw = llPose.getOrientation().getYaw(AngleUnit.DEGREES);

            telemetry.addData("x raw", xRaw);
            telemetry.addData("y raw", yRaw);
            telemetry.addData("yaw raw", yawRaw);
            packet.put("x raw", xRaw);
            packet.put("y raw", yRaw);
            packet.put("yaw raw", yawRaw);

            double xTransformed = yRaw + 72;
            double yTransformed = -xRaw + 72;
            double yawTransformed = (yawRaw + 270) % 360;
            if (yawTransformed < 0) {
                yawTransformed += 360;
            }

            telemetry.addData("x transformed", xTransformed);
            telemetry.addData("y transformed", yTransformed);
            telemetry.addData("yaw transformed", yawTransformed);
            packet.put("x transformed", xTransformed);
            packet.put("y transformed", yTransformed);
            packet.put("yaw transformed", yawTransformed);

            ftcDashboard.sendTelemetryPacket(packet);

            return new Pose(xTransformed, yTransformed, yawTransformed);
        }
        else {
            telemetry.addLine("No apriltags in field of view");
            packet.put("No apriltags in field of view", 0);
            return null;

        }
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
        else return limelight.getStatus().getPipelineType();
    }

    public Integer getObeliskID(){
        if (!limelight.isRunning()) {
            limelight.start();
        }
        limelight.pipelineSwitch(APRIL_TAGS_PIPELINE_INDEX);

        List<AprilTagDescriptor> tags = getAprilTagDescriptors();
        for (AprilTagDescriptor tag : tags){
            int id = tag.getId();
            if (id == 21 || id == 22 || id == 33){
                telemetry.addData("id detected", id);
                return id;
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

        telemetry.addData("pipeline index", limelight.getStatus().getPipelineIndex());
        telemetry.addData("pipeline type", limelight.getStatus().getPipelineType());
        telemetry.addData("isRunning", limelight.isRunning());

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
