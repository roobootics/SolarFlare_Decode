package org.firstinspires.ftc.teamcode.vision;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.vision.descriptors.AprilTagDescriptor;
import org.firstinspires.ftc.teamcode.vision.descriptors.ArtifactDescriptor;
import org.firstinspires.ftc.teamcode.vision.descriptors.DetectionDescriptor;
import org.firstinspires.ftc.teamcode.vision.pipelines.GetBotPoseMT2;
import org.firstinspires.ftc.teamcode.vision.pipelines.FindArtifactRelativePositions;
import org.firstinspires.ftc.teamcode.vision.pipelines.AprilTagDetector;

import java.util.ArrayList;
import java.util.List;

public class VisionManager {
    // Translation from reference point, usually the lowest center middle of the robot for pedro, to center of Limelight lens
    final double CAMERA_VERTICAL_HEIGHT_INCHES = 5; // Increases up from reference point
    final double CAMERA_OFFSET_X_INCHES = 0; // Increases to the right from reference point
    final double CAMERA_OFFSET_Y_INCHES = 0; // Increases forward from the reference point
    final double CAMERA_DOWNWARD_PITCH_DEGREES = 65; // 90 degrees is facing straight forward, decreases looking down
    final int NN_PIPELINE_INDEX = 0;
    final int APRIL_TAGS_PIPELINE_INDEX = 2;
    double fx = 1218.145;
    double fy = 1219.481;
    double cx = 621.829;
    double cy = 500.362;
    final double[][] K = {
            {fx, 0.0, cx},
            {0.0, fy, cy},
            {0.0, 0.0, 1.0}
    };
    FindArtifactRelativePositions artifactDetector;
    AprilTagDetector aprilTagDetector;
    GetBotPoseMT2 llLocalizer;
    Limelight3A limelight;
    GoBildaPinpointDriver pinpoint;
    List<String> queryClasses = new ArrayList<>();
    Telemetry telemetry;
    public VisionManager(HardwareMap hardwareMap, Telemetry telemetry){
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        this.telemetry = telemetry;
        limelight.setPollRateHz(11);
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.resetPosAndIMU();
        queryClasses.add("purple");
        queryClasses.add("green");
        artifactDetector = new FindArtifactRelativePositions(
                queryClasses, limelight,
                CAMERA_VERTICAL_HEIGHT_INCHES,
                CAMERA_OFFSET_X_INCHES,
                CAMERA_OFFSET_Y_INCHES,
                CAMERA_DOWNWARD_PITCH_DEGREES,
                K);
        aprilTagDetector = new AprilTagDetector(limelight);
        llLocalizer = new GetBotPoseMT2(limelight, pinpoint, telemetry);
    }

    public List<DetectionDescriptor> getDetectionDescriptors(){ // Returns a list of all detections from a single frame
        if (!limelight.isRunning()){
            limelight.start();
        }
        limelight.pipelineSwitch(NN_PIPELINE_INDEX);
        telemetry.addData("pipeline index", limelight.getStatus().getPipelineIndex());
        telemetry.addData("pipeline type", limelight.getStatus().getPipelineType());
        telemetry.addData("isRunning", limelight.isRunning());
        return artifactDetector.getDetectionDescriptors();
    }

    public List<ArtifactDescriptor> getArtifactDescriptors(){ // Converts relative positions to absolute
        if (!limelight.isRunning()){
            limelight.start();
        }
        List<DetectionDescriptor> detectionDescriptors = artifactDetector.getDetectionDescriptors();
        for (DetectionDescriptor detection: detectionDescriptors){
            double leftRightOffset = detection.getLeftRightOffset();
            double forwardOffset = detection.getForwardOffset();

        }
        return null;
    }

    public Pose getBotPoseAprilTags(){
        if (!limelight.isRunning()){
            limelight.start();
        }
        limelight.pipelineSwitch(APRIL_TAGS_PIPELINE_INDEX);
        Pose3D llPose = llLocalizer.getBotPoseMT2();

        telemetry.addData("pipeline index", limelight.getStatus().getPipelineIndex());
        telemetry.addData("pipeline type", limelight.getStatus().getPipelineType());
        telemetry.addData("isRunning", limelight.isRunning());
        if (llPose != null) {
            return new Pose(llPose.getPosition().x + 72, llPose.getPosition().y + 72, llPose.getOrientation().getYaw());
        }
        else return null;
    }

    public List<AprilTagDescriptor> getAprilTagDescriptors(){ // Returns a list of all detections from a single frame
        if (!limelight.isRunning()){
            limelight.start();
        }
        limelight.pipelineSwitch(APRIL_TAGS_PIPELINE_INDEX);
        telemetry.addData("pipeline index", limelight.getStatus().getPipelineIndex());
        telemetry.addData("pipeline type", limelight.getStatus().getPipelineType());
        telemetry.addData("isRunning", limelight.isRunning());
        return aprilTagDetector.getAprilTagDescriptors();
    }
    public void stopLimelight(){
        limelight.stop();
    }

    public String getCurrentPipelineType() {
        if (!limelight.isRunning()) {
            limelight.start();
        }
        String pipelineName = limelight.getStatus().getPipelineType();

        if (pipelineName.isEmpty()) return "";
        else return limelight.getStatus().getPipelineType();
    }

    public int getObeliskID(){
        List<AprilTagDescriptor> tags = getAprilTagDescriptors();
        for (AprilTagDescriptor tag : tags){
            int id = tag.getId();
            if (id == 21 || id == 22 || id == 33){
                return id;
            }
        }
        return 21; // Default when the obelisk is not detected
    }
}
