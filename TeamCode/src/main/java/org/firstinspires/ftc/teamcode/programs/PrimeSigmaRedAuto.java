package org.firstinspires.ftc.teamcode.programs;

import static org.firstinspires.ftc.teamcode.base.Commands.executor;
import static org.firstinspires.ftc.teamcode.base.Components.initialize;
import static org.firstinspires.ftc.teamcode.base.Components.initializeConfig;
import static org.firstinspires.ftc.teamcode.pedroPathing.Pedro.follower;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.SHOT_TIME;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.alliance;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.loopFSM;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.setState;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.base.Commands.Command;
import org.firstinspires.ftc.teamcode.base.Commands.InstantCommand;
import org.firstinspires.ftc.teamcode.base.Commands.ParallelCommand;
import org.firstinspires.ftc.teamcode.base.Commands.SequentialCommand;
import org.firstinspires.ftc.teamcode.base.Commands.SleepCommand;
import org.firstinspires.ftc.teamcode.base.Commands.SleepUntilTrue;
import org.firstinspires.ftc.teamcode.pedroPathing.Pedro;
import org.firstinspires.ftc.teamcode.pedroPathing.Pedro.PedroCommand;
import org.firstinspires.ftc.teamcode.pedroPathing.Pedro.PedroInstantCommand;
import org.firstinspires.ftc.teamcode.pedroPathing.Pedro.PedroInstantLinearChainCommand;
import org.firstinspires.ftc.teamcode.pedroPathing.Pedro.PedroInstantLinearCommand;
import org.firstinspires.ftc.teamcode.pedroPathing.Pedro.PedroLinearCommand;
import org.firstinspires.ftc.teamcode.robotconfigs.Inferno;
import org.firstinspires.ftc.teamcode.robotconfigs.Inferno.Alliance;
import org.firstinspires.ftc.teamcode.robotconfigs.Inferno.CheckFull;
import org.firstinspires.ftc.teamcode.robotconfigs.Inferno.RobotState;

@Autonomous
public class PrimeSigmaRedAuto extends LinearOpMode {
    private final Command shoot = new SequentialCommand(
            new SleepUntilTrue(()->follower.getCurrentTValue()>0.93),
            new ParallelCommand(setState(RobotState.SHOOTING), new SleepCommand(SHOT_TIME)),
            setState(RobotState.INTAKE_FRONT)
    );
    @Override
    public void runOpMode() {
        alliance = Alliance.BLUE;
        initialize(hardwareMap,telemetry);
        initializeConfig(new Inferno(new Pose(124, 122.62, Math.toRadians(216.5))), true);
        executor.setCommands(new SequentialCommand(
                new PedroInstantLinearCommand(90.7,90.4,Math.toRadians(216.5),true), shoot,
                new PedroInstantCommand((PathBuilder b)->b.addPath(
                        new BezierCurve(
                                new Pose(90.688, 90.398),
                                new Pose(63.954, 59.413),
                                new Pose(133.871, 59.805)
                        )).setTangentHeadingInterpolation()
                        .setReversed().addPath(
                                new BezierCurve(
                                        new Pose(133.871, 59.805),
                                        new Pose(89.718, 64.593),
                                        new Pose(91.924, 84.680)
                                )
                        ).setConstantHeadingInterpolation(Math.toRadians(0)), true
                ), shoot,
                new PedroCommand(
                        (PathBuilder b)->b.addPath(
                                new BezierCurve(
                                        new Pose(91.924, 84.680),
                                        new Pose(104.596, 37.087),
                                        new Pose(131.831, 59.686)
                                )
                        ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(35)).addPath(
                                new BezierLine(
                                        new Pose(131.831, 59.686),
                                        new Pose(131.099, 54.334)
                                )
                        ).setLinearHeadingInterpolation(Math.toRadians(35), Math.toRadians(50)),
                        true
                ),
                new CheckFull(3),
                new InstantCommand(Inferno::toggleShotType),
                new PedroInstantCommand(
                        (PathBuilder b)->b.addPath(
                                new BezierCurve(
                                        new Pose(131.099, 54.334),
                                        new Pose(99.831, 52.575),
                                        new Pose(91.63, 83.903)
                                )
                        ).setLinearHeadingInterpolation(Math.toRadians(50), Math.toRadians(10)),
                        true
                ), shoot,
                new PedroInstantLinearChainCommand(
                        true,new Pose(127.227, 83.829,Math.toRadians(0)),
                        new Pose(91.549, 84.203,Math.toRadians(0))
                ), shoot,
                new PedroInstantCommand(
                        (PathBuilder b)->b.addPath(
                                new BezierCurve(
                                        new Pose(91.459, 84.203),
                                        new Pose(63.933, 27.483),
                                        new Pose(131.696, 36.131)
                                )
                        ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0)).addPath(
                                new BezierLine(
                                        new Pose(131.696, 36.131),

                                        new Pose(91.823, 83.918)
                                )
                        ).setConstantHeadingInterpolation(Math.toRadians(0)),true
                ), shoot,
                new ParallelCommand(
                        setState(RobotState.STOPPED),
                        new PedroLinearCommand(108,36,Math.toRadians(0),true)
                )
        ),
        loopFSM,
        Pedro.updateCommand());
        waitForStart();
        executor.runLoop(this::opModeIsActive);
    }
}
