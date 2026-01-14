package org.firstinspires.ftc.teamcode.vision.testing;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.vision.VisionManager;

@Autonomous
public class TestMT2 extends OpMode {
    VisionManager visionManager;
    @Override
    public void init() {
        visionManager = new VisionManager(hardwareMap, telemetry);
    }

    @Override
    public void loop() {
        Pose botPose = visionManager.getBotPoseAprilTags();
        if (botPose != null){
            telemetry.addData("yaw limelight radians", botPose.getHeading());
            telemetry.addData("yaw limelight degrees", Math.toDegrees(botPose.getHeading()));
            telemetry.addData("x", botPose.getX());
            telemetry.addData("y", botPose.getY());
            telemetry.update();
        }

    }
}
