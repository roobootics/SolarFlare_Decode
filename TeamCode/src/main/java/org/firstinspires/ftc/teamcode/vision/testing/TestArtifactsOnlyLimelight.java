package org.firstinspires.ftc.teamcode.vision.testing;

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
import org.firstinspires.ftc.teamcode.vision.Vision;
import org.firstinspires.ftc.teamcode.vision.descriptors.ArtifactDescriptor;

import java.util.List;

@TeleOp
@Config
public class TestArtifactsOnlyLimelight extends OpMode {
    final static double ARTIFACT_CENTER_OFFSET = 2.5;
    Pose3D testerRig = new Pose3D(new Position(DistanceUnit.MM, -29, 0, 123, 0), new YawPitchRollAngles(AngleUnit.DEGREES, 0, 0, 0, 0));;
    Pose3D cameraPoseOnRobot = new Pose3D(new Position(DistanceUnit.METER, 0.182, 0, 0.2225, 0), new YawPitchRollAngles(AngleUnit.DEGREES, 0, 0, 0, 0));
    Vision vision;

    @Override
    public void init(){
        vision = new Vision(hardwareMap, telemetry, testerRig);
    }
    @Override
    public void loop(){

        List<ArtifactDescriptor> artifacts = vision.getRelativeArtifactDescriptors();

        if (!artifacts.isEmpty()) {
            for (ArtifactDescriptor artifact : artifacts) {
                double x = artifact.getX();
                double y = artifact.getY() + ARTIFACT_CENTER_OFFSET;
                String className = artifact.getClassName();

                telemetry.addData("x", x);
                telemetry.addData("y", y);
                telemetry.addData("className", className);
            }
        }
        else {
            telemetry.addLine("No artifacts detected");
        }

        telemetry.update();
    }
}
