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
        double robotYaw = pinpoint.getHeading(AngleUnit.RADIANS);
        telemetry.addData("pinpoint radians", robotYaw);
        telemetry.addData("pinpoint degrees", Math.toDegrees(robotYaw));
        limelight.updateRobotOrientation(robotYaw);
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()){
            return result.getBotpose_MT2();
        }
        return null;
    }
}
