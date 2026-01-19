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
            double x_p = botPose.getX();
            double y_p = botPose.getY();

            double x_f = -(y_p - 72);
            double y_f = x_p - 72;

            fieldOverlay.setFill("black");
            fieldOverlay.fillRect(x_f, y_f, 13, 14);

            ftcDashboard.sendTelemetryPacket(packet);
        }
    }
}
