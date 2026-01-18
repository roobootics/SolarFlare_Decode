package org.firstinspires.ftc.teamcode.programs;

import static org.firstinspires.ftc.teamcode.base.Commands.executor;
import static org.firstinspires.ftc.teamcode.base.Components.initialize;
import static org.firstinspires.ftc.teamcode.base.Components.initializeConfig;
import static org.firstinspires.ftc.teamcode.base.Components.telemetryAddData;
import static org.firstinspires.ftc.teamcode.base.Components.telemetryAddLine;
import static org.firstinspires.ftc.teamcode.pedroPathing.Pedro.follower;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.SHOT_TIME;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.afterExpelSet;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.alliance;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.ballStorage;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.classifierBallCount;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.clearIntegralAtPeak;
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
import org.firstinspires.ftc.teamcode.base.Commands.ParallelCommand;
import org.firstinspires.ftc.teamcode.base.Commands.SequentialCommand;
import org.firstinspires.ftc.teamcode.base.Commands.SleepCommand;
import org.firstinspires.ftc.teamcode.base.Components;
import org.firstinspires.ftc.teamcode.pedroPathing.Pedro;
import org.firstinspires.ftc.teamcode.pedroPathing.Pedro.PedroCommand;
import org.firstinspires.ftc.teamcode.pedroPathing.Pedro.PedroLinearCommand;
import org.firstinspires.ftc.teamcode.robotconfigs.Inferno;
import org.firstinspires.ftc.teamcode.robotconfigs.Inferno.Alliance;
import org.firstinspires.ftc.teamcode.robotconfigs.Inferno.CheckFull;
import org.firstinspires.ftc.teamcode.robotconfigs.Inferno.GamePhase;
import org.firstinspires.ftc.teamcode.robotconfigs.Inferno.RobotState;

import java.util.Arrays;

@Autonomous
public class PrimeSigmaRedAuto extends LinearOpMode {
    private final Command backExpelShoot = new SequentialCommand(
            new ParallelCommand(setState(RobotState.SHOOTING), new SleepCommand(SHOT_TIME)),
            setState(RobotState.BACK_EXPEL)
    );
    private final Command frontExpelShoot = new SequentialCommand(
            new ParallelCommand(setState(RobotState.SHOOTING), new SleepCommand(SHOT_TIME)),
            setState(RobotState.FRONT_EXPEL)
    );
    @Override
    public void runOpMode() {
        alliance = Alliance.RED;
        gamePhase = GamePhase.AUTO;
        afterExpelSet = RobotState.INTAKE_FRONT;
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
                        new SleepCommand(0.85),
                        new PedroLinearCommand(90.688,90.398,Math.toRadians(216.5),true), frontExpelShoot,
                        new PedroCommand((PathBuilder b)->b.addPath(
                                        new BezierCurve(
                                                follower::getPose,
                                                new Pose(63.954, 59.413),
                                                new Pose(125.871, 58.805)
                                        )).setTangentHeadingInterpolation()
                                .addParametricCallback(0.78,()->follower.setMaxPower(0.57))
                                .addPath(
                                        new BezierCurve(
                                                follower::getPose,
                                                new Pose(89.718, 64.593),
                                                new Pose(90.924, 87.680)
                                        )
                                ).setConstantHeadingInterpolation(Math.toRadians(0))
                                .addParametricCallback(0.22,()->follower.setMaxPower(1.0))
                                .addParametricCallback(0.37,()->setState(RobotState.STOPPED).run()), true
                        ), backExpelShoot,
                        new PedroCommand(
                                (PathBuilder b)->b.addPath(
                                        new BezierCurve(
                                                follower::getPose,
                                                new Pose(104.596, 37.087),
                                                new Pose(129.831, 60.686)
                                        )
                                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(35)),
                                true
                        ),
                        new SleepCommand(0.8),
                        new PedroCommand(
                                (PathBuilder b)->b
                                .addPath(
                                        new BezierLine(
                                                follower::getPose,
                                                new Pose(128.099, 52.334)
                                        )
                                )
                                .addParametricCallback(0.6,()->follower.setMaxPower(1.0))
                                .setLinearHeadingInterpolation(Math.toRadians(35), Math.toRadians(50)),
                                true
                        ),
                        new CheckFull(1),
                        new PedroCommand(
                                (PathBuilder b)->b.addPath(
                                        new BezierCurve(
                                                follower::getPose,
                                                new Pose(99.831, 52.575),
                                                new Pose(90.63, 87.903)
                                        )
                                ).setLinearHeadingInterpolation(Math.toRadians(50), Math.toRadians(10))
                                .addParametricCallback(0.4,()->setState(RobotState.STOPPED).run()),
                                true
                        ), backExpelShoot,
                        new PedroCommand((PathBuilder b)->b
                                .addPath(
                                        new BezierLine(
                                                follower::getPose,
                                                new Pose(121.227, 79.829)
                                        )
                                ).setLinearHeadingInterpolation(Math.toRadians(10),Math.toRadians(0))
                                .addParametricCallback(0.78,()->follower.setMaxPower(0.57))
                                .addPath(
                                        new BezierLine(
                                                follower::getPose,
                                                new Pose(90.549, 87.203)
                                        )
                                ).setConstantHeadingInterpolation(Math.toRadians(0))
                                .addParametricCallback(0.22,()->follower.setMaxPower(1.0))
                                .addParametricCallback(0.37,()->setState(RobotState.STOPPED).run()),true
                        ), backExpelShoot,
                        new PedroCommand(
                                (PathBuilder b)->b.addPath(
                                        new BezierCurve(
                                                follower::getPose,
                                                new Pose(63.933, 27.483),
                                                new Pose(128.696, 36.131)
                                        )
                                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                                .addParametricCallback(0.78,()->follower.setMaxPower(0.57))
                                .addPath(
                                        new BezierLine(
                                                follower::getPose,
                                                new Pose(90.823, 87.918)
                                        )
                                ).setConstantHeadingInterpolation(Math.toRadians(360))
                                .addParametricCallback(0.22,()->follower.setMaxPower(1.0))
                                .addParametricCallback(0.37,()->setState(RobotState.STOPPED).run()),true
                        ), backExpelShoot,
                        new ParallelCommand(
                                setState(RobotState.STOPPED),
                                new PedroLinearCommand(99,79,Math.toRadians(360),true)
                        )
                ),
                clearIntegralAtPeak,
                loopFSM,
                Pedro.updateCommand());
        executor.runLoop(this::opModeIsActive);
    }
}
