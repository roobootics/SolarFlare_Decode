package org.firstinspires.ftc.teamcode.programs;

import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.backIntake;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.backIntakeGate;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.frontIntake;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.frontIntakeGate;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.setState;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.transferGate;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.base.Commands;
import org.firstinspires.ftc.teamcode.robotconfigs.Inferno;

import java.util.HashMap;
import java.util.Objects;

public class PrimeSigmaConstants {
    public static final double INITIAL_WAIT = 0.75;
    public static final double SHOT_TIME = 0.8;
    public static final double slowDownT = 0.78;
    public static final double speedUpT = 0.22;
    public static final double stopIntakeT = 0.37;
    public static final double slowDownAmount = 0.57;
    public static final double gateWait = 0.8;
    public static final double gateIntakeTimeout = 1;
    public static Commands.Command backExpelShoot;
    public static Commands.Command frontExpelShoot;
    public static Pose mirrorPose(Pose input){
        return new Pose(144-input.getX(),input.getY(),mirrorHeading(input.getHeading()));
    }
    public static double mirrorHeading(double input){
        return 2*Math.PI - input;
    }
    public static void initExpelActions(){
        backExpelShoot = new Commands.SequentialCommand(
                new Commands.ParallelCommand(setState(Inferno.RobotState.SHOOTING), new Commands.SleepCommand(SHOT_TIME)),
                new Commands.ParallelCommand(
                        frontIntake.setPowerCommand(-1.0),
                        backIntake.setPowerCommand(1.0),
                        frontIntakeGate.instantSetTargetCommand("open"),
                        backIntakeGate.instantSetTargetCommand("closed"),
                        transferGate.instantSetTargetCommand("open")
                ),
                new Commands.SleepCommand(1)
        );
        frontExpelShoot = new Commands.SequentialCommand(
                new Commands.ParallelCommand(setState(Inferno.RobotState.SHOOTING), new Commands.SleepCommand(SHOT_TIME)),
                new Commands.ParallelCommand(
                        frontIntake.setPowerCommand(1.0),
                        backIntake.setPowerCommand(-1.0),
                        frontIntakeGate.instantSetTargetCommand("closed"),
                        backIntakeGate.instantSetTargetCommand("open"),
                        transferGate.instantSetTargetCommand("open")
                ),
                new Commands.SleepCommand(1)
        );
    }
}
