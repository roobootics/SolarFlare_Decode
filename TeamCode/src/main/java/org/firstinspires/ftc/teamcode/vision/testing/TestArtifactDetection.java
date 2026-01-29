package org.firstinspires.ftc.teamcode.vision.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.vision.Vision;
import org.firstinspires.ftc.teamcode.vision.descriptors.ArtifactDescriptor;

import java.util.List;

@TeleOp
@Config
public class TestArtifactDetection extends OpMode {
public static double ARTIFACT_CENTER_OFFSET = 2.5;
    Vision vision;
    GoBildaPinpointDriver pinpoint;
    double initXPedro = 72;
    double initYPedro = 144;
    double initHeadingPedroDegrees = 270;
    double robotWidth;
    double robotLength;
    Pose3D cameraPose;
    enum ROBOTS{
        INFERNO,
        STRAFER
    }
    ROBOTS currentRobot = ROBOTS.STRAFER;
    FtcDashboard dashboard;

    @Override
    public void init() {
        dashboard = FtcDashboard.getInstance();
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        pinpoint.resetPosAndIMU();

        if (currentRobot == ROBOTS.INFERNO) {
            pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
            pinpoint.setOffsets(-2.625, -3.76925, DistanceUnit.INCH);
            robotWidth = 14;
            robotLength = 17.5;
            cameraPose = new Pose3D(new Position(DistanceUnit.METER, 0.182, 0, 0.2225, 0), new YawPitchRollAngles(AngleUnit.DEGREES, 0, -15, 0, 0));
        } else if (currentRobot == ROBOTS.STRAFER) {
            pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.FORWARD); //TODO make sure these are correct
            pinpoint.setOffsets(0, 6.875, DistanceUnit.INCH);
            robotWidth = 17.625;
            robotLength = 17.125;
            cameraPose = new Pose3D(new Position(DistanceUnit.INCH, 1.625, 5.125, 4.25, 0), new YawPitchRollAngles(AngleUnit.DEGREES, 0, 0, 0, 0));
        }

        vision = new Vision(hardwareMap, telemetry, cameraPose);

        Pose initPosePedro = new Pose(initXPedro + (robotWidth / 2), initYPedro + (robotLength / 2), Math.toRadians(initHeadingPedroDegrees));
        Pose2D initPosePinpoint = vision.pedroToStandardPose(initPosePedro);

        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

        pinpoint.setPosition(initPosePinpoint);
        pinpoint.setHeading(initPosePinpoint.getHeading(AngleUnit.DEGREES), AngleUnit.DEGREES);
    }

    @Override
    public void loop() {
        pinpoint.update();

        TelemetryPacket packet = new TelemetryPacket();
        Canvas fieldOverlay = packet.fieldOverlay();

        Pose2D botPosePinpoint = pinpoint.getPosition();
        Pose botPosePedro = vision.standardToPedroPose(botPosePinpoint);

        List<ArtifactDescriptor> artifacts = vision.getArtifactDescriptors(botPosePedro);

        if (artifacts != null) {
            artifacts = vision.pedroToStandardPoseArtifacts(artifacts);

            for (ArtifactDescriptor artifact : artifacts) {
                double x = artifact.getX();
                double y = artifact.getY();
                String className = artifact.getClassName();

                packet.put("x", x);
                packet.put("y", y + ARTIFACT_CENTER_OFFSET);
                packet.put("className", className);

                fieldOverlay.setFill(className);
                fieldOverlay.fillCircle(x, y, 2.5);

            }
            dashboard.sendTelemetryPacket(packet);
        }
    }
}
