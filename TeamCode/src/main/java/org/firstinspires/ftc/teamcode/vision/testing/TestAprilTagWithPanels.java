package org.firstinspires.ftc.teamcode.vision.testing;

import static org.firstinspires.ftc.teamcode.base.Components.initialize;
import static org.firstinspires.ftc.teamcode.pedroPathing.Pedro.follower;

import com.bylazar.configurables.annotations.Configurable;
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

import java.util.ArrayList;
import java.util.List;

@TeleOp
@Configurable
public class TestAprilTagWithPanels extends OpMode {
    public static boolean showBotPose = true;
    public static boolean showMT1 = true;
    public static boolean showMT2 = true;
    public static boolean showMT2WithMT1 = true;
    Vision vision;
    Pose3D cameraPose = new Pose3D(new Position(DistanceUnit.METER, 0.182, 0, 0.2225, 0), new YawPitchRollAngles(AngleUnit.DEGREES, 0, 0, 0, 0));
    double robotWidth = 15;
    double robotLength = 18;
    Pose initPosePedro = new Pose(72, 144 - (robotLength / 2), Math.toRadians(270));
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
    public void loop(){
        follower.updatePose();

        Pose botPose = follower.getPose();

        Integer id = vision.getObeliskID();
        Pose botPoseMT1 = vision.getBotPoseMT1(Math.toDegrees(botPose.getHeading()));
        Pose botPoseMT2 = vision.getBotPoseMT2(Math.toDegrees(botPose.getHeading()));
        Pose botPoseMT2WithMT1 = vision.getBotPoseMT2WithMT1(Math.toDegrees(botPose.getHeading()));

        telemetry.addData("botPose x", botPose.getX());
        telemetry.addData("botPose y", botPose.getY());
        telemetry.addData("botPose Heading", Math.toDegrees(botPose.getHeading()));

        panelsTelemetry.addData("botPose x", botPose.getX());
        panelsTelemetry.addData("botPose y", botPose.getY());
        panelsTelemetry.addData("botPose Heading", Math.toDegrees(botPose.getHeading()));

        if (showBotPose) vision.drawPoseOnPanels(panelsField, botPose, "blue");
        if (showMT1) vision.drawPoseOnPanels(panelsField, botPoseMT1, "red");
        if (showMT2) vision.drawPoseOnPanels(panelsField, botPoseMT2, "purple");
        if (showMT2WithMT1) vision.drawPoseOnPanels(panelsField, botPoseMT2WithMT1, "green");

        if (botPoseMT2 != null){
            panelsTelemetry.addData("mt2 x", botPoseMT2.getX());
            panelsTelemetry.addData("mt2 y", botPoseMT2.getY());
            panelsTelemetry.addData("mt2 Heading", Math.toDegrees(botPoseMT2.getHeading()));
        }

        if (id != null){
            telemetry.addData("obelisk id", id);
            panelsTelemetry.addData("obelisk id", id);
        }
        else {
            telemetry.addLine("id is null");
            panelsTelemetry.addLine("id is null");
        }

        panelsField.update();
        panelsTelemetry.update();
        telemetry.update();
    }
}
