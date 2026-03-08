package org.firstinspires.ftc.teamcode.programs;

import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.Alliance.RED;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
public class FarCompatPrimeSigmaREDAuto extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        FarCompatPrimeSigmaConstants.runOpMode(RED,this);
    }
}
