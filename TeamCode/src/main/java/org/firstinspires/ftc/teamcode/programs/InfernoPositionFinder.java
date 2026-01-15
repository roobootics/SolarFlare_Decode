package org.firstinspires.ftc.teamcode.programs;

import static org.firstinspires.ftc.teamcode.base.Components.initialize;
import static org.firstinspires.ftc.teamcode.base.Components.initializeConfig;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.presets.GenericPositionFinder;
import org.firstinspires.ftc.teamcode.robotconfigs.Inferno;
@TeleOp
public class InfernoPositionFinder extends GenericPositionFinder {
    public void runOpMode(){
        initialize(hardwareMap,telemetry);
        initializeConfig(new Inferno(new Pose(102.5, 7, 0)),true);
        super.runOpMode();
    }
}
