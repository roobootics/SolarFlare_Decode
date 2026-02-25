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

import java.util.ArrayList;
import java.util.List;

@TeleOp
public class TestAprilTagWithPanels extends OpMode {
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
        Pose botPoseMT1 = vision.getBotPoseMT1();
        Pose botPoseMT2 = vision.getBotPoseMT2(Math.toDegrees(botPose.getHeading()));
        Pose botPoseMT2WithMT1 = vision.getBotPoseMT2WithMT1();

        telemetry.addData("botPose x", botPose.getX());
        telemetry.addData("botPose y", botPose.getY());
        telemetry.addData("botPose Heading", Math.toDegrees(botPose.getHeading()));

        panelsField.setStyle("black", "black", 0);
        panelsField.moveCursor(botPose.getX(), botPose.getY());
        panelsField.circle(3);


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

            panelsField.setStyle("blue", "blue", 0);
            panelsField.moveCursor(botPoseMT1.getX(), botPoseMT1.getY());
            panelsField.circle(3);
        }
        else {
            telemetry.addLine("MT1 is null");
        }

        if (botPoseMT2 != null){
            telemetry.addData("botPoseMT2 x", botPoseMT2.getX());
            telemetry.addData("botPoseMT2 y", botPoseMT2.getY());
            telemetry.addData("botPoseMT2 heading", Math.toDegrees(botPoseMT2.getHeading()));

            panelsField.setStyle("red", "red", 0);
            panelsField.moveCursor(botPoseMT2.getX(), botPoseMT2.getY());
            panelsField.circle(3);
        }
        else {
            telemetry.addLine("MT2 is null");
        }

        if (botPoseMT2WithMT1 != null){
            telemetry.addData("botPoseMT2 with MT1 x", botPoseMT2WithMT1.getX());
            telemetry.addData("botPoseMT2 with MT1 y", botPoseMT2WithMT1.getY());
            telemetry.addData("botPoseMT2 with MT1 heading", Math.toDegrees(botPoseMT2WithMT1.getHeading()));

            panelsField.setStyle("purple", "purple", 0);
            panelsField.moveCursor(botPoseMT2WithMT1.getX(), botPoseMT2WithMT1.getY());
            panelsField.circle(3);
        }
        else {
            telemetry.addLine("MT2 with MT1 is null");
        }

        panelsField.update();
        panelsTelemetry.update();
        telemetry.update();
    }
}
