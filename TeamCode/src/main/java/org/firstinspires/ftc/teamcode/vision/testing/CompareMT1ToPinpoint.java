package org.firstinspires.ftc.teamcode.vision.testing;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.vision.VisionControl;

@TeleOp
public class CompareMT1ToPinpoint extends OpMode {
    GoBildaPinpointDriver pinpoint;
    VisionControl visionControl;
    @Override
    public void init(){
        visionControl = new VisionControl(hardwareMap, telemetry);
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
    }
    @Override
    public void loop(){
        pinpoint.resetPosAndIMU();
        pinpoint.update();
        Pose botPoseMT1 = visionControl.getBotPoseMT1();
        if (botPoseMT1 != null){
            telemetry.addData("ll x", botPoseMT1.getX());
            telemetry.addData("ll y", botPoseMT1.getY());
            telemetry.addData("ll heading", botPoseMT1.getHeading());
            telemetry.addData("pinpoint x", pinpoint.getPosition().getX(DistanceUnit.INCH));
            telemetry.addData("pinpoint y", pinpoint.getPosition().getY(DistanceUnit.INCH));
            telemetry.addData("pinpoint heading", pinpoint.getPosition().getHeading(AngleUnit.DEGREES));
            telemetry.update();
        }
    }
}
