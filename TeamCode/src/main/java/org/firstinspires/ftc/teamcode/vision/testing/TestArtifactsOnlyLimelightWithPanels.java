package org.firstinspires.ftc.teamcode.vision.testing;

import androidx.annotation.NonNull;

import com.bylazar.field.CanvasRotation;
import com.bylazar.field.FieldManager;
import com.bylazar.field.FieldPresetParams;
import com.bylazar.field.ImagePreset;
import com.bylazar.field.PanelsField;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
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

import java.util.ArrayList;
import java.util.List;

@TeleOp
public class TestArtifactsOnlyLimelightWithPanels extends OpMode {
    Pose3D testerRig = new Pose3D(new Position(DistanceUnit.INCH, -0.78125, 0, 5.05, 0), new YawPitchRollAngles(AngleUnit.DEGREES, 0, -25, 0, 0));
    // Pose3D cameraPoseOnRobot = new Pose3D(new Position(DistanceUnit.METER, 0.182, 0, 0.2225, 0), new YawPitchRollAngles(AngleUnit.DEGREES, 0, 0, 0, 0));
    Pose botPose = new Pose(73, 0, Math.toRadians(90));
    Vision vision;
    List<String> acceptedClasses = new ArrayList<>();
    FieldManager panelsField = PanelsField.INSTANCE.getField();
    TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

    @Override
    public void init(){
        vision = new Vision(hardwareMap, telemetry, testerRig, Vision.CAMERA_ORIENTATION.NORMAL);
        acceptedClasses.add("purple");
        acceptedClasses.add("green");


        panelsField.setOffsets(new FieldPresetParams("decode", -72, -72, CanvasRotation.DEG_0, false, false, true));
    }
    @Override
    public void loop(){

        List<ArtifactDescriptor> artifacts = vision.getArtifactDescriptors(botPose, acceptedClasses);

        if (!artifacts.isEmpty()) {

            Double intakingAngle = vision.intakingAngleArtifacts2(artifacts, botPose, 1);
            telemetry.addData("intaking angle", intakingAngle);
            panelsTelemetry.addData("intaking angle", intakingAngle);

            double lineLength = 10;

            double x2 = botPose.getX() + Math.cos(Math.toRadians(intakingAngle)) * lineLength;
            double y2 = botPose.getY() + Math.sin(Math.toRadians(intakingAngle)) * lineLength;

            panelsField.setStyle("red", "red", 1);
            panelsField.moveCursor(botPose.getX(), botPose.getY());
            panelsField.line(x2, y2);

            for (ArtifactDescriptor artifact : artifacts) {
                double x = artifact.getX();
                double y = artifact.getY();
                String className = artifact.getClassName();

                panelsTelemetry.addData("x", x);
                panelsTelemetry.addData("y", y);
                panelsTelemetry.addData("className", className);

                telemetry.addData("x", x);
                telemetry.addData("y", y);
                telemetry.addData("className", className);

                panelsField.setStyle(className, className, 0);
                panelsField.moveCursor(x, y);

                panelsField.circle(2.5);
            }
        }
        else {
            telemetry.addLine("No artifacts detected");
        }

        panelsField.update();
        panelsTelemetry.update();
        telemetry.update();
    }
}
