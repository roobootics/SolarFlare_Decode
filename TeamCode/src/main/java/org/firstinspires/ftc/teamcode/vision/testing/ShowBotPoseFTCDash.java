package org.firstinspires.ftc.teamcode.vision.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.vision.VisionControl;

@TeleOp
public class ShowBotPoseFTCDash extends OpMode {
    FtcDashboard ftcDashboard;
    VisionControl visionControl;
    @Override
    public void init(){
        visionControl = new VisionControl(hardwareMap, telemetry);
        ftcDashboard = FtcDashboard.getInstance();
    }

    @Override
    public void loop(){
        TelemetryPacket packet = new TelemetryPacket();
        Canvas fieldOverlay = packet.fieldOverlay();

        Pose botPose = visionControl.getBotPoseMT1();

        if (botPose != null){
            double x = botPose.getX();
            double y = botPose.getY();

            fieldOverlay.setFill("black");
            fieldOverlay.fillRect(x, y, 13, 14);

            ftcDashboard.sendTelemetryPacket(packet);
        }
    }
}
