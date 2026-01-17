package org.firstinspires.ftc.teamcode.programs;

import static org.firstinspires.ftc.teamcode.base.Commands.executor;
import static org.firstinspires.ftc.teamcode.base.Components.initialize;
import static org.firstinspires.ftc.teamcode.base.Components.initializeConfig;
import static org.firstinspires.ftc.teamcode.base.Components.telemetryAddData;
import static org.firstinspires.ftc.teamcode.base.Components.telemetryAddLine;
import static org.firstinspires.ftc.teamcode.pedroPathing.Pedro.follower;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.SHOT_TIME;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.alliance;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.ballStorage;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.classifierBallCount;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.currentBallPath;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.flywheel;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.gamePhase;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.getTargetPoint;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.loopFSM;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.motifShootAll;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.robotState;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.setState;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.shotType;

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
import org.firstinspires.ftc.teamcode.base.Components;
import org.firstinspires.ftc.teamcode.pedroPathing.Pedro;
import org.firstinspires.ftc.teamcode.pedroPathing.Pedro.PedroCommand;
import org.firstinspires.ftc.teamcode.pedroPathing.Pedro.PedroInstantCommand;
import org.firstinspires.ftc.teamcode.pedroPathing.Pedro.PedroInstantLinearChainCommand;
import org.firstinspires.ftc.teamcode.pedroPathing.Pedro.PedroInstantLinearCommand;
import org.firstinspires.ftc.teamcode.pedroPathing.Pedro.PedroLinearChainCommand;
import org.firstinspires.ftc.teamcode.pedroPathing.Pedro.PedroLinearCommand;
import org.firstinspires.ftc.teamcode.robotconfigs.Inferno;
import org.firstinspires.ftc.teamcode.robotconfigs.Inferno.Alliance;
import org.firstinspires.ftc.teamcode.robotconfigs.Inferno.CheckFull;
import org.firstinspires.ftc.teamcode.robotconfigs.Inferno.RobotState;

import java.util.Arrays;

@Autonomous
public class PrimeSigmaRedAuto extends LinearOpMode {
    private final Command shoot = new SequentialCommand(
            new ParallelCommand(setState(RobotState.SHOOTING), new SleepCommand(SHOT_TIME)),
            setState(RobotState.FRONT_EXPEL)
    );
    @Override
    public void runOpMode() {
        alliance = Alliance.RED;
        gamePhase = Inferno.GamePhase.AUTO;
        initialize(hardwareMap,telemetry);
        initializeConfig(new Inferno(), true);
        Pedro.createFollower(new Pose(124, 122.62, Math.toRadians(216.5)));
        executor.setWriteToTelemetry(()->{
            telemetryAddData("is busy",follower.isBusy());
            telemetryAddData("Ball Storage:", Arrays.asList(ballStorage));
            telemetryAddLine("");
            telemetryAddData("Robot State:",robotState);
            telemetryAddLine("");
            telemetryAddData("Shot Type:",shotType);
            telemetryAddLine("");
            telemetryAddData("Classifier Count:",classifierBallCount);
            telemetryAddData("Current Shot Height:",currentBallPath);
            telemetryAddData("Shoot All Motif:",motifShootAll);
            telemetryAddLine("");
            telemetryAddData("Distance", Math.sqrt((follower.getPose().getX() - getTargetPoint()[0])*(follower.getPose().getX() - getTargetPoint()[0]) + (follower.getPose().getY() - getTargetPoint()[1])*(follower.getPose().getY() - getTargetPoint()[1])));
            telemetryAddData("Flywheel Velocity",flywheel.getActuators().get("flywheelLeft").getVelocity());
            telemetryAddData("PoseX",follower.getPose().getX());
            telemetryAddData("PoseY",follower.getPose().getY());
            telemetryAddData("PoseHeading",Math.toDegrees(follower.getHeading()));
            telemetryAddData("Flywheel Left Power",flywheel.getActuators().get("flywheelLeft").getPower());
            telemetryAddData("Flywheel Right Power",flywheel.getActuators().get("flywheelRight").getPower());
        });
        waitForStart();
        Components.activateActuatorControl();
        executor.setCommands(new SequentialCommand(
                        new SleepCommand(0.75),
                        new PedroLinearCommand(90.7,90.4,Math.toRadians(216.5),true), shoot,
                        new PedroCommand((PathBuilder b)->b.addPath(
                                        new BezierCurve(
                                                new Pose(90.688, 90.398),
                                                new Pose(63.954, 59.413),
                                                new Pose(128.871, 56.805)
                                        )).setTangentHeadingInterpolation()
                                .addPath(
                                        new BezierCurve(
                                                new Pose(128.871, 56.805),
                                                new Pose(89.718, 64.593),
                                                new Pose(91.924, 84.680)
                                        )
                                ).setConstantHeadingInterpolation(Math.toRadians(360)), true
                        ), shoot,
                        new PedroCommand(
                                (PathBuilder b)->b.addPath(
                                        new BezierCurve(
                                                new Pose(91.924, 84.680),
                                                new Pose(104.596, 37.087),
                                                new Pose(128.831, 59.686)
                                        )
                                ).setLinearHeadingInterpolation(Math.toRadians(360), Math.toRadians(35)).addPath(
                                        new BezierLine(
                                                new Pose(128.831, 59.686),
                                                new Pose(126.099, 54.334)
                                        )
                                ).setLinearHeadingInterpolation(Math.toRadians(35), Math.toRadians(50)),
                                true
                        ),
                        new CheckFull(3),
                        new PedroCommand(
                                (PathBuilder b)->b.addPath(
                                        new BezierCurve(
                                                new Pose(126.099, 54.334),
                                                new Pose(99.831, 52.575),
                                                new Pose(91.63, 83.903)
                                        )
                                ).setLinearHeadingInterpolation(Math.toRadians(50), Math.toRadians(10)),
                                true
                        ), shoot,
                        new PedroLinearChainCommand(
                                true,new Pose(120.227, 83.829,Math.toRadians(0)),
                                new Pose(91.549, 84.203,Math.toRadians(0))
                        ), shoot,
                        new PedroCommand(
                                (PathBuilder b)->b.addPath(
                                        new BezierCurve(
                                                new Pose(91.459, 84.203),
                                                new Pose(63.933, 27.483),
                                                new Pose(128.696, 36.131)
                                        )
                                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0)).addPath(
                                        new BezierLine(
                                                new Pose(128.696, 36.131),

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
        executor.runLoop(this::opModeIsActive);
    }
}
