package org.firstinspires.ftc.teamcode.programs;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robotconfigs.Inferno;

@Autonomous
public class FarPrimeSigmaREDAuto extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        FarPrimeSigmaConstants.runOpMode(Inferno.Alliance.RED,this);
    }
}
