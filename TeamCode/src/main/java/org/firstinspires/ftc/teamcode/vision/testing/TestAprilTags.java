package org.firstinspires.ftc.teamcode.vision.testing;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.vision.VisionControl;

@TeleOp
public class TestAprilTags extends OpMode {
    VisionControl vision;
    GoBildaPinpointDriver pinpoint;
    double initX = 0;
    double initY = 0;
    double robotWidth = 14; // TODO: check these values
    double robotLength = 18;
    double initHeading = 90;
    @Override
    public void init(){
        vision = new VisionControl(hardwareMap, telemetry);
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.resetPosAndIMU();
        pinpoint.setHeading(initHeading, AngleUnit.DEGREES);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.REVERSED);
    }

    @Override
    public void loop(){
        pinpoint.update();
        Integer id = vision.getObeliskID();
        Pose botPoseMT1 = vision.getBotPoseMT1();
        Pose botPoseMT2 = vision.getBotPoseMT2(pinpoint);
        Pose botPoseMT2WithMT1 = vision.getBotPoseMT2WithMT1();

        if (id != null & botPoseMT1 != null & botPoseMT2 != null & botPoseMT2WithMT1 != null){
            telemetry.addData("pinpoint x", pinpoint.getPosition().getX(DistanceUnit.INCH) + initX + (robotWidth / 2));
            telemetry.addData("pinpoint y", pinpoint.getPosition().getY(DistanceUnit.INCH) + initY + (robotLength / 2));
            telemetry.addData("pinpoint heading", pinpoint.getPosition().getHeading(AngleUnit.DEGREES));

            telemetry.addData("obelisk id", id);

            telemetry.addData("botPoseMT1 x", botPoseMT1.getX());
            telemetry.addData("botPoseMT1 y", botPoseMT1.getY());
            telemetry.addData("botPoseMT1 heading", botPoseMT1.getHeading());

            telemetry.addData("botPoseMT2 x", botPoseMT2.getX());
            telemetry.addData("botPoseMT2 y", botPoseMT2.getY());
            telemetry.addData("botPoseMT2 heading", botPoseMT2.getHeading());

            telemetry.addData("botPoseMT2 with MT1 x", botPoseMT2WithMT1.getX());
            telemetry.addData("botPoseMT2 with MT1 y", botPoseMT2WithMT1.getY());
            telemetry.addData("botPoseMT2 with MT1 heading", botPoseMT2WithMT1.getHeading());
            telemetry.update();
        }
    }
}
