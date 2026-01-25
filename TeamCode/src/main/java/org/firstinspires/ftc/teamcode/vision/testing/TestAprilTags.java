package org.firstinspires.ftc.teamcode.vision.testing;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.vision.Vision;

@TeleOp
public class TestAprilTags extends OpMode {
    Vision vision;
    GoBildaPinpointDriver pinpoint;
    double initXPedro = 0;
    double initYPedro = 0;
    double initHeadingPedroDegrees = 90;
    double robotWidth = 14;
    double robotLength = 17.5;
    Pose initPosePedro;
    @Override
    public void init(){
        vision = new Vision(hardwareMap, telemetry);
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.recalibrateIMU();
        pinpoint.resetPosAndIMU();
        initPosePedro = new Pose(initXPedro + (robotWidth / 2), initYPedro + (robotLength / 2), Math.toRadians(initHeadingPedroDegrees));
        Pose2D pinpointPose = new Pose2D(DistanceUnit.INCH, initPosePedro.getX(), initPosePedro.getY(), AngleUnit.DEGREES, Math.toDegrees(initPosePedro.getHeading()));
        pinpoint.setPosition(pinpointPose);
        pinpoint.setHeading(pinpointPose.getHeading(AngleUnit.DEGREES), AngleUnit.DEGREES);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        pinpoint.setOffsets(-2.625, -3.76925, DistanceUnit.INCH);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
    }

    @Override
    public void loop(){
        pinpoint.update();

        Pose2D botPosePinpoint = pinpoint.getPosition();
        Integer id = vision.getObeliskID();
        Pose botPoseMT1 = vision.getBotPoseMT1();
        Pose botPoseMT2 = vision.getBotPoseMT2(pinpoint.getHeading(AngleUnit.DEGREES));
        Pose botPoseMT2WithMT1 = vision.getBotPoseMT2WithMT1();


        telemetry.addData("pinpoint x", botPosePinpoint.getX(DistanceUnit.INCH));
        telemetry.addData("pinpoint y", botPosePinpoint.getY(DistanceUnit.INCH));
        telemetry.addData("pinpoint heading", botPosePinpoint.getHeading(AngleUnit.DEGREES));

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
