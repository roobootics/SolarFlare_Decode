package org.firstinspires.ftc.teamcode.vision.testing;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.vision.VisionControl;

@TeleOp
public class TestMT1 extends OpMode {
    VisionControl visionControl;
    @Override
    public void init() {
        visionControl = new VisionControl(hardwareMap, telemetry);
    }

    @Override
    public void loop() {
        Pose botPose = visionControl.getBotPoseMT1();
        telemetry.update();
    }
}
