package org.firstinspires.ftc.teamcode.programs;

import static org.firstinspires.ftc.teamcode.base.Components.initializeConfig;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.presets.GenericPositionFinder;
import org.firstinspires.ftc.teamcode.robotconfigs.Inferno;
@TeleOp
public class InfernoPositionFinder extends GenericPositionFinder {
    public void runOpMode(){
        initializeConfig(new Inferno(),true);
        super.runOpMode();
    }
}
