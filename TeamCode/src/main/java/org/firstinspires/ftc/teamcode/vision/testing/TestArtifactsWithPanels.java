package org.firstinspires.ftc.teamcode.vision.testing;

import static org.firstinspires.ftc.teamcode.base.Components.initialize;
import static org.firstinspires.ftc.teamcode.pedroPathing.Pedro.follower;

import com.bylazar.field.FieldManager;
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
import org.firstinspires.ftc.teamcode.pedroPathing.Pedro;
import org.firstinspires.ftc.teamcode.robotconfigs.Inferno;
import org.firstinspires.ftc.teamcode.vision.Vision;
import org.firstinspires.ftc.teamcode.vision.descriptors.ArtifactDescriptor;

import java.util.ArrayList;
import java.util.List;

@TeleOp
public class TestArtifactsWithPanels extends OpMode {
    Vision vision;
    Pose3D cameraPose = new Pose3D(new Position(DistanceUnit.METER, 0.182, 0, 0.2225, 0), new YawPitchRollAngles(AngleUnit.DEGREES, 0, 0, 0, 0));
    Pose initPosePedro = new Pose(72,144,Math.toRadians(270));
    FieldManager panelsField = PanelsField.INSTANCE.getField();
    TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
    List<String> acceptedClasses = new ArrayList<>();
    @Override
    public void init(){
        vision = new Vision(hardwareMap, telemetry, cameraPose, Vision.CAMERA_ORIENTATION.NORMAL);

        initialize(this, new Inferno(),false,true);
        Pedro.createFollower(initPosePedro);

        panelsField.setOffsets(PanelsField.INSTANCE.getPresets().getPEDRO_PATHING());

        acceptedClasses.add("purple");
        acceptedClasses.add("green");
    }

    @Override
    public void loop() {
        follower.updatePose();

        Pose botPose = follower.getPose();

        telemetry.addData("botPose x", botPose.getX());
        telemetry.addData("botPose y", botPose.getY());
        telemetry.addData("botPose Heading", Math.toDegrees(botPose.getHeading()));

        panelsField.setStyle("black", "black", 0);
        panelsField.moveCursor(botPose.getX(), botPose.getY());
        panelsField.rect(14, 13);

        List<ArtifactDescriptor> artifacts = vision.getArtifactDescriptors(botPose, acceptedClasses);

        if (!artifacts.isEmpty()) {

            Double intakingAngle = vision.intakingAngleArtifacts(artifacts, botPose);
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
        panelsField.update();
        panelsTelemetry.update();
        telemetry.update();
    }
}
