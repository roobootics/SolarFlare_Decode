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

import org.firstinspires.ftc.teamcode.pedroPathing.Pedro;
import org.firstinspires.ftc.teamcode.robotconfigs.Inferno;
import org.firstinspires.ftc.teamcode.robotconfigs.Inferno.*;
import org.firstinspires.ftc.teamcode.base.Commands.*;
import org.firstinspires.ftc.teamcode.pedroPathing.Pedro.*;

import java.util.Arrays;

@Autonomous
public class PrimeSigmaBlueAuto extends LinearOpMode {
    public int counter = 0;
    private final Command shoot = new SequentialCommand(
            new InstantCommand(()->counter+=1),
            new ParallelCommand(setState(RobotState.SHOOTING), new SleepCommand(SHOT_TIME)),
            setState(RobotState.INTAKE_BACK)
    );
    @Override
    public void runOpMode() {
        alliance = Inferno.Alliance.BLUE;
        gamePhase = GamePhase.AUTO;
        initialize(hardwareMap,telemetry);
        initializeConfig(new Inferno(new Pose(20, 122.62, Math.toRadians(143.5))), true);
        executor.setWriteToTelemetry(()->{
            telemetryAddData("counter",counter);
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
        executor.setCommands(new SequentialCommand(
                        new SleepCommand(1),
                        new PedroLinearCommand(53.312,90.398,Math.toRadians(143.5),true), shoot,
                        new PedroCommand((PathBuilder b)->b.addPath(
                                        new BezierCurve(
                                                new Pose(53.312, 90.398),
                                                new Pose(88.046, 59.413),
                                                new Pose(18.129, 59.805)
                                        )).setTangentHeadingInterpolation()
                                .setReversed().addPath(
                                        new BezierCurve(
                                                new Pose(18.129, 59.805),
                                                new Pose(54.282, 64.593),
                                                new Pose(52.076, 84.680)
                                        )
                                ).setConstantHeadingInterpolation(Math.toRadians(0)), true
                        ), shoot,
                        new PedroCommand(
                                (PathBuilder b)->b.addPath(
                                        new BezierCurve(
                                                new Pose(52.076, 84.680),
                                                new Pose(39.404, 37.087),
                                                new Pose(19.169, 59.686)
                                        )
                                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-35)).addPath(
                                        new BezierLine(
                                                new Pose(19.169, 59.686),
                                                new Pose(19.901, 54.334)
                                        )
                                ).setLinearHeadingInterpolation(Math.toRadians(-35), Math.toRadians(-50)),
                                true
                        ),
                        new CheckFull(3),
                        new InstantCommand(Inferno::toggleShotType),
                        new PedroCommand(
                                (PathBuilder b)->b.addPath(
                                        new BezierCurve(
                                                new Pose(19.901, 54.334),
                                                new Pose(44.169, 52.575),
                                                new Pose(52.370, 83.903)
                                        )
                                ).setLinearHeadingInterpolation(Math.toRadians(-50), Math.toRadians(-10)),
                                true
                        ), shoot,
                        new PedroLinearChainCommand(
                                true,new Pose(24.773, 83.829,Math.toRadians(0)),
                                new Pose(52.451, 84.203,Math.toRadians(0))
                        ), shoot,
                        new PedroCommand(
                                (PathBuilder b)->b.addPath(
                                        new BezierCurve(
                                                new Pose(52.451, 84.203),
                                                new Pose(80.067, 27.483),
                                                new Pose(20.304, 36.131)
                                        )
                                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0)).addPath(
                                        new BezierLine(
                                                new Pose(20.304, 36.131),

                                                new Pose(52.177, 83.918)
                                        )
                                ).setConstantHeadingInterpolation(Math.toRadians(360)),true
                        ), shoot,
                        new ParallelCommand(
                                setState(RobotState.STOPPED),
                                new PedroLinearCommand(36,36,Math.toRadians(360),true)
                        )
                ),
                clearIntegralAtPeak,
                loopFSM,
                Pedro.updateCommand());
        executor.runLoop(this::opModeIsActive);
    }
}
