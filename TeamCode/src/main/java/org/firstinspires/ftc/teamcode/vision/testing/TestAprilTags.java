package org.firstinspires.ftc.teamcode.vision.testing;

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

@TeleOp
public class TestAprilTags extends OpMode {
    Vision vision;
    GoBildaPinpointDriver pinpoint;
    double initXPedro = 0; // These should be the bottom left corner of the robot
    double initYPedro = 0;
    double initHeadingPedroDegrees = 90;
    double robotWidth;
    double robotLength;
    Pose3D cameraPose;
    enum ROBOTS{
        INFERNO,
        STRAFER
    }
    ROBOTS currentRobot = ROBOTS.STRAFER;
    Pose2D initPosePinpoint;

    @Override
    public void init(){
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        pinpoint.recalibrateIMU();
        pinpoint.resetPosAndIMU();

        if (currentRobot == ROBOTS.INFERNO){
            pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
            pinpoint.setOffsets(-2.625, -3.76925, DistanceUnit.INCH);
            robotWidth = 14;
            robotLength = 17.5;
            cameraPose = new Pose3D(new Position(DistanceUnit.METER, 0.182, 0, 0.2225, 0), new YawPitchRollAngles(AngleUnit.DEGREES, 0, -15, 0, 0));
        }
        else if (currentRobot == ROBOTS.STRAFER){
            pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD); //TODO make sure these are correct
            pinpoint.setOffsets(0, 6.875, DistanceUnit.INCH);
            robotWidth = 17.625;
            robotLength = 17.125;
            cameraPose = new Pose3D(new Position(DistanceUnit.INCH, 1.625, 5.125, 4.25, 0), new YawPitchRollAngles(AngleUnit.DEGREES, 0, 0, 0, 0));
        }

        vision = new Vision(hardwareMap, telemetry, cameraPose);

        Pose initPosePedro = new Pose(initXPedro + (robotWidth / 2), initYPedro + (robotLength / 2), Math.toRadians(initHeadingPedroDegrees));
        initPosePinpoint = vision.pedroToStandardPose(initPosePedro);

        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

        pinpoint.setPosition(initPosePinpoint);
    }

    @Override
    public void loop(){
        pinpoint.update();

        Pose2D botPosePinpoint = pinpoint.getPosition();

        telemetry.addData("raw pinpoint x", botPosePinpoint.getX(DistanceUnit.INCH));
        telemetry.addData("raw pinpoint y", botPosePinpoint.getY(DistanceUnit.INCH));
        telemetry.addData("init pinpoint pos", initPosePinpoint);

        Pose botPosePedro = vision.standardToPedroPose(botPosePinpoint);

        telemetry.addData("bot pose pedro", botPosePedro);

        Integer id = vision.getObeliskID();
        Pose botPoseMT1 = vision.getBotPoseMT1();
        Pose botPoseMT2 = vision.getBotPoseMT2(pinpoint.getHeading(AngleUnit.DEGREES));
        Pose botPoseMT2WithMT1 = vision.getBotPoseMT2WithMT1();

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

        telemetry.update();
    }
}
