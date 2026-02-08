package org.firstinspires.ftc.teamcode.vision.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pedropathing.geometry.CoordinateSystem;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.vision.Vision;
import org.firstinspires.ftc.teamcode.vision.descriptors.ArtifactDescriptor;

import java.util.List;

@TeleOp
@Config
public class TestArtifactsOnlyLimelight extends OpMode {
    Pose3D testerRig = new Pose3D(new Position(DistanceUnit.INCH, -0.78125, 0, 5.05, 0), new YawPitchRollAngles(AngleUnit.DEGREES, 0, -25, 0, 0));;
    Pose3D cameraPoseOnRobot = new Pose3D(new Position(DistanceUnit.METER, 0.182, 0, 0.2225, 0), new YawPitchRollAngles(AngleUnit.DEGREES, 0, 0, 0, 0));
    Pose botPose = new Pose(72, 0, Math.toRadians(90));
    Vision vision;
    public static double ARTIFACT_OFFSET = 2.5;
    FtcDashboard dashboard = FtcDashboard.getInstance();

    @Override
    public void init(){
        vision = new Vision(hardwareMap, telemetry, testerRig, Vision.CAMERA_ORIENTATION.NORMAL);
    }
    @Override
    public void loop(){
        TelemetryPacket packet = new TelemetryPacket();
        Canvas fieldOverlay = packet.fieldOverlay();

        fieldOverlay.setTranslation(-72, 72);
        fieldOverlay.setRotation(Math.toRadians(-90));

        List<ArtifactDescriptor> artifacts = vision.getArtifactDescriptors(botPose);

        if (!artifacts.isEmpty()) {
            for (ArtifactDescriptor artifact : artifacts) {
                double x = artifact.getX();
                double y = artifact.getY() + ARTIFACT_OFFSET;
                String className = artifact.getClassName();

                packet.put("x", x);
                packet.put("y", y);
                packet.put("className", className);

                fieldOverlay.setFill(className);
                fieldOverlay.fillCircle(x, y, 2.5);

                telemetry.addData("x", x);
                telemetry.addData("y", y);
                telemetry.addData("className", className);
            }
        }
        else {
            telemetry.addLine("No artifacts detected");
        }

        dashboard.sendTelemetryPacket(packet);
        telemetry.update();
    }
}
