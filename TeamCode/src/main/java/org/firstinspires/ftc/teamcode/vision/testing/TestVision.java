package org.firstinspires.ftc.teamcode.vision.testing;

import static org.firstinspires.ftc.teamcode.pedroPathing.Pedro.follower;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.pedroPathing.Pedro;
import org.firstinspires.ftc.teamcode.vision.Vision;
import org.firstinspires.ftc.teamcode.vision.descriptors.ArtifactDescriptor;

import java.util.List;

@Config
@TeleOp
public class TestVision extends OpMode {
    Vision vision;
    final static double ARTIFACT_CENTER_OFFSET = 2.5;
    Pose3D cameraPose = new Pose3D(new Position(DistanceUnit.METER, 0.182, 0, 0.2225, 0), new YawPitchRollAngles(AngleUnit.DEGREES, 0, 0, 0, 0));
    FtcDashboard dashboard;
    Pose initPosePedro = new Pose(0,0,Math.toRadians(90));

    @Override
    public void init(){
        vision = new Vision(hardwareMap, telemetry, cameraPose);
        dashboard = FtcDashboard.getInstance();
        Pedro.createFollower(initPosePedro);
        follower.setStartingPose(initPosePedro);
    }

    @Override
    public void loop(){
        follower.updatePose();
        TelemetryPacket packet = new TelemetryPacket();
        Canvas fieldOverlay = packet.fieldOverlay();

        Pose botPose = follower.getPose();

        telemetry.addData("botPose x", botPose.getX());
        telemetry.addData("botPose y", botPose.getY());
        telemetry.addData("botPose Heading", Math.toDegrees(botPose.getHeading()));

        Integer id = vision.getObeliskID();
        Pose botPoseMT1 = vision.getBotPoseMT1();
        Pose botPoseMT2 = vision.getBotPoseMT2(Math.toDegrees(botPose.getHeading()));
        Pose botPoseMT2WithMT1 = vision.getBotPoseMT2WithMT1();

        List<ArtifactDescriptor> artifacts = vision.getArtifactDescriptors(botPose);

        if (artifacts != null) {
            artifacts = vision.pedroToStandardPoseArtifacts(artifacts);

            for (ArtifactDescriptor artifact : artifacts) {
                double x = artifact.getX();
                double y = artifact.getY();
                String className = artifact.getClassName();

                packet.put("x", x);
                packet.put("y", y + ARTIFACT_CENTER_OFFSET);
                packet.put("className", className);

                telemetry.addData("x", x);
                telemetry.addData("y", y);
                telemetry.addData("className", className);

                fieldOverlay.setFill(className);
                fieldOverlay.fillCircle(x, y, 2.5);

            }
        }
        else {
            telemetry.addLine("No artifacts detected");
        }

        if (id != null){
            telemetry.addData("obelisk id", id);
        }
        else {
            telemetry.addLine("id is null");
        }

        if (botPoseMT1 != null){
            telemetry.addData("botPoseMT1 x", botPoseMT1.getX());
            telemetry.addData("botPoseMT1 y", botPoseMT1.getY());
            telemetry.addData("botPoseMT1 heading", Math.toDegrees(botPoseMT1.getHeading()));
        }
        else {
            telemetry.addLine("MT1 is null");
        }

        if (botPoseMT2 != null){
            telemetry.addData("botPoseMT2 x", botPoseMT2.getX());
            telemetry.addData("botPoseMT2 y", botPoseMT2.getY());
            telemetry.addData("botPoseMT2 heading", Math.toDegrees(botPoseMT2.getHeading()));
        }
        else {
            telemetry.addLine("MT2 is null");
        }

        if (botPoseMT2WithMT1 != null){
            telemetry.addData("botPoseMT2 with MT1 x", botPoseMT2WithMT1.getX());
            telemetry.addData("botPoseMT2 with MT1 y", botPoseMT2WithMT1.getY());
            telemetry.addData("botPoseMT2 with MT1 heading", Math.toDegrees(botPoseMT2WithMT1.getHeading()));
        }
        else {
            telemetry.addLine("MT2 with MT1 is null");
        }

        dashboard.sendTelemetryPacket(packet);
        telemetry.update();
    }
}
