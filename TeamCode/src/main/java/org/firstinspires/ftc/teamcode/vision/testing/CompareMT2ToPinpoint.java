package org.firstinspires.ftc.teamcode.vision.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.vision.VisionControl;

@TeleOp
public class CompareMT2ToPinpoint extends OpMode {
    GoBildaPinpointDriver pinpoint;
    VisionControl vision;
    TelemetryPacket packet;
    FtcDashboard ftcDashboard;
    double robotWidth = 14; // TODO: check these values
    double robotLength = 18;
    double initX = 0;
    double initY = 0;
    double initHeading = 90;

    @Override
    public void init(){
        packet = new TelemetryPacket();
        ftcDashboard = FtcDashboard.getInstance();
        vision = new VisionControl(hardwareMap, telemetry);
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.resetPosAndIMU();
        pinpoint.setHeading(initHeading, AngleUnit.DEGREES);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.REVERSED);
    }
    @Override
    public void loop(){
        pinpoint.update();

        Pose botPoseMT2 = vision.getBotPoseMT2(pinpoint);

        double pinpointX = pinpoint.getPosition().getX(DistanceUnit.INCH) + (robotWidth / 2) + initX;
        double pinpointY = pinpoint.getPosition().getY(DistanceUnit.INCH) + (robotLength / 2) + initY;
        double pinpointHeading = pinpoint.getHeading(AngleUnit.DEGREES);

        if (botPoseMT2 != null){

            double llX = botPoseMT2.getX();
            double llY = botPoseMT2.getY();
            double llHeading = botPoseMT2.getHeading();

            double xError = llX - pinpointX;
            double yError = llY - pinpointY;
            double headingError = llHeading - pinpointHeading;

            telemetry.addData("pinpoint x", pinpointX);
            telemetry.addData("pinpoint y", pinpointY);
            telemetry.addData("pinpoint heading", pinpointHeading);

            packet.put("ll x", llX);
            packet.put("ll y", llY);
            packet.put("ll heading", llHeading);
            packet.put("pinpoint x", pinpointX);
            packet.put("pinpoint y", pinpointY);
            packet.put("pinpoint heading", pinpointHeading);
            packet.put("x error", xError);
            packet.put("y error", yError);
            packet.put("heading error", headingError);

            ftcDashboard.sendTelemetryPacket(packet);
            telemetry.update();
        }
    }
}
