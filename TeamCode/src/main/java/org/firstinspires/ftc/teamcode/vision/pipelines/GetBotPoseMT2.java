package org.firstinspires.ftc.teamcode.vision.pipelines;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

public class GetBotPoseMT2 {
    Limelight3A limelight;
    GoBildaPinpointDriver pinpoint;
    Telemetry telemetry;
    public GetBotPoseMT2(Limelight3A limelight, GoBildaPinpointDriver pinpoint, Telemetry telemetry){
        this.limelight = limelight;
        this.pinpoint = pinpoint;
        this.telemetry = telemetry;
    }
    public Pose3D getBotPoseMT2(){
        pinpoint.update();
        double robotYawRadians = pinpoint.getHeading(AngleUnit.RADIANS);
        double robotYawDegrees = Math.toDegrees(robotYawRadians);
        telemetry.addData("pinpoint radians", robotYawRadians);
        telemetry.addData("pinpoint degrees", robotYawDegrees);
        limelight.updateRobotOrientation(robotYawDegrees);
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()){
            return result.getBotpose();
        }
        return null;
    }
}
