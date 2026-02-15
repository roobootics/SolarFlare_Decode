package org.firstinspires.ftc.teamcode.programs;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robotconfigs.Inferno;

@Autonomous
public class PrimeSigmaREDAuto extends LinearOpMode {
    @Override
    public void runOpMode() {
        PrimeSigmaConstants.runOpMode(Inferno.Alliance.RED,this);
    }
}
