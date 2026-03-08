package org.firstinspires.ftc.teamcode.programs;

import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.Alliance.BLUE;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
@Autonomous
public class FarCompatPrimeSigmaBLUEAuto extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        FarCompatPrimeSigmaConstants.runOpMode(BLUE,this);
    }
}
