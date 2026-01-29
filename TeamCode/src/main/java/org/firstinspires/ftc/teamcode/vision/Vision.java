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
    final int NN_PIPELINE_INDEX = 0;
    final int APRIL_TAGS_PIPELINE_INDEX = 1;
    Limelight3A limelight;
    List<String> queryClasses = new ArrayList<>();
    Telemetry telemetry;
    Pose3D cameraPoseOnRobot;
    public Vision(HardwareMap hardwareMap, Telemetry telemetry, Pose3D cameraPoseOnRobot){
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        this.telemetry = telemetry;
        limelight.setPollRateHz(11);
        queryClasses.add("purple");
        queryClasses.add("green");
        this.cameraPoseOnRobot = cameraPoseOnRobot;
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

            if (queryClasses.contains(className)) {

                double tx = detectorResult.getTargetXDegrees();
                double ty = detectorResult.getTargetYDegrees();

                double totalAngleDeg = cameraPoseOnRobot.getOrientation().getPitch(AngleUnit.DEGREES) + ty;
                double totalAngleRad = Math.toRadians(totalAngleDeg);

                double depth = cameraPoseOnRobot.getPosition().toUnit(DistanceUnit.INCH).z / -Math.tan(totalAngleRad)
                                + cameraPoseOnRobot.getPosition().toUnit(DistanceUnit.INCH).x;
                                 // artifact center offset TODO: Tune this

                double horizontal = depth * Math.tan(Math.toRadians(tx)) + cameraPoseOnRobot.getPosition().toUnit(DistanceUnit.INCH).y;

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

        Pose3D cameraPose;

        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()){
            cameraPose = result.getBotpose();
        }
        else return null;

        Pose3D robotPose = applyOffset(cameraPose, cameraPoseOnRobot);

        return limelightToPedroPose(robotPose);
    }

    public Pose getBotPoseMT2(double yawDegreesStandard){
        if (!limelight.isRunning()){
            limelight.start();
        }
        limelight.pipelineSwitch(APRIL_TAGS_PIPELINE_INDEX);

        limelight.updateRobotOrientation(standardToLimelightYaw(yawDegreesStandard));

        Pose3D cameraPose;

        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()){
            cameraPose = result.getBotpose_MT2();
        }
        else return null;

        Pose3D robotPose = applyOffset(cameraPose, cameraPoseOnRobot);

        return limelightToPedroPose(robotPose);
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

            Pose3D cameraPose;

            LLResult result = limelight.getLatestResult();

            if (result != null && result.isValid()) {
                cameraPose = result.getBotpose_MT2();
            } else return null;

            Pose3D robotPose = applyOffset(cameraPose, cameraPoseOnRobot);

            return limelightToPedroPose(robotPose);
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

    public Pose3D pedroToLimelightPose(Pose pedroPose){
        double x = pedroPose.getX();
        double y = pedroPose.getY();
        double yaw = Math.toDegrees(pedroPose.getHeading());

        double llX = 72 - y;
        double llY = x - 72;
        double llYaw = standardToLimelightYaw(yaw);

        return new Pose3D(new Position(DistanceUnit.INCH, llX, llY, 0, 0), new YawPitchRollAngles(AngleUnit.DEGREES, llYaw, 0, 0, 0));
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

    public static Pose3D applyOffset(Pose3D cameraPoseField,
                                     Pose3D cameraPoseOnRobot) {

        // Step 1: invert camera-in-robot pose
        double ox = cameraPoseOnRobot.getPosition().toUnit(DistanceUnit.INCH).x;
        double oy = cameraPoseOnRobot.getPosition().toUnit(DistanceUnit.INCH).y;
        double oz = cameraPoseOnRobot.getPosition().toUnit(DistanceUnit.INCH).z;

        YawPitchRollAngles or = cameraPoseOnRobot.getOrientation();

        double ir = -or.getRoll(AngleUnit.RADIANS); // inverse roll
        double ip = -or.getPitch(AngleUnit.RADIANS); // inverse pitch
        double iy = -or.getYaw(AngleUnit.RADIANS); // inverse yaw

        // Rotate the negative offset by the inverse rotation
        double cr = Math.cos(ir), sr = Math.sin(ir);
        double cp = Math.cos(ip), sp = Math.sin(ip);
        double cy = Math.cos(iy), sy = Math.sin(iy);

        double ix =
                cy * cp * (-ox)
                        + (cy * sp * sr - sy * cr) * (-oy)
                        + (cy * sp * cr + sy * sr) * (-oz);

        double iy_ =
                sy * cp * (-ox)
                        + (sy * sp * sr + cy * cr) * (-oy)
                        + (sy * sp * cr - cy * sr) * (-oz);

        double iz =
                -sp * (-ox)
                        + cp * sr * (-oy)
                        + cp * cr * (-oz);

        // Step 2: rotate offset into field frame
        YawPitchRollAngles fr = cameraPoseField.getOrientation();

        double frx = fr.getRoll(AngleUnit.RADIANS);
        double fry = fr.getPitch(AngleUnit.RADIANS);
        double frz = fr.getYaw(AngleUnit.RADIANS);

        cr = Math.cos(frx); sr = Math.sin(frx);
        cp = Math.cos(fry); sp = Math.sin(fry);
        cy = Math.cos(frz); sy = Math.sin(frz);

        double rx =
                cy * cp * ix
                        + (cy * sp * sr - sy * cr) * iy_
                        + (cy * sp * cr + sy * sr) * iz;

        double ry =
                sy * cp * ix
                        + (sy * sp * sr + cy * cr) * iy_
                        + (sy * sp * cr - cy * sr) * iz;

        double rz =
                -sp * ix
                        + cp * sr * iy_
                        + cp * cr * iz;

        // Step 3: add translations and rotations
        return new Pose3D(
                new Position(DistanceUnit.INCH,
                cameraPoseField.getPosition().toUnit(DistanceUnit.INCH).x + rx,
                cameraPoseField.getPosition().toUnit(DistanceUnit.INCH).y + ry,
                cameraPoseField.getPosition().toUnit(DistanceUnit.INCH).z + rz, 0),

                new YawPitchRollAngles(
                        AngleUnit.RADIANS,
                        frx + ir,
                        fry + ip,
                        frz + iy,
                        0
                )
        );
    }
}
