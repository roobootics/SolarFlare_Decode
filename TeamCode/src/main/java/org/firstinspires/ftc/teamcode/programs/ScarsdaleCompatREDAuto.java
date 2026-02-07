package org.firstinspires.ftc.teamcode.programs;

import static org.firstinspires.ftc.teamcode.base.Commands.executor;
import static org.firstinspires.ftc.teamcode.base.Components.initialize;
import static org.firstinspires.ftc.teamcode.base.Components.initializeConfig;
import static org.firstinspires.ftc.teamcode.base.Components.telemetryAddData;
import static org.firstinspires.ftc.teamcode.base.Components.telemetryAddLine;
import static org.firstinspires.ftc.teamcode.base.Components.timer;
import static org.firstinspires.ftc.teamcode.pedroPathing.Pedro.follower;
import static org.firstinspires.ftc.teamcode.programs.PrimeSigmaConstants.INITIAL_WAIT;
import static org.firstinspires.ftc.teamcode.programs.PrimeSigmaConstants.backExpelShoot;
import static org.firstinspires.ftc.teamcode.programs.PrimeSigmaConstants.fourthShootSlowAmount;
import static org.firstinspires.ftc.teamcode.programs.PrimeSigmaConstants.fourthShootSlowT;
import static org.firstinspires.ftc.teamcode.programs.PrimeSigmaConstants.frontExpelShoot;
import static org.firstinspires.ftc.teamcode.programs.PrimeSigmaConstants.gateIntakeTimeout;
import static org.firstinspires.ftc.teamcode.programs.PrimeSigmaConstants.gateWait;
import static org.firstinspires.ftc.teamcode.programs.PrimeSigmaConstants.getMirroredHeading;
import static org.firstinspires.ftc.teamcode.programs.PrimeSigmaConstants.getMirroredPose;
import static org.firstinspires.ftc.teamcode.programs.PrimeSigmaConstants.initExpelActions;
import static org.firstinspires.ftc.teamcode.programs.PrimeSigmaConstants.slowDownAmount;
import static org.firstinspires.ftc.teamcode.programs.PrimeSigmaConstants.slowDownT;
import static org.firstinspires.ftc.teamcode.programs.PrimeSigmaConstants.speedUpT;
import static org.firstinspires.ftc.teamcode.programs.PrimeSigmaConstants.stopIntakeT;
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
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.setShooter;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.setState;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.shotType;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.turretOffsetFromAuto;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.turretYaw;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.paths.PathBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.base.Commands;
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
public class ScarsdaleCompatREDAuto extends LinearOpMode {
    private double lastTime;
    @Override
    public void runOpMode() {
        initialize(hardwareMap,telemetry);
        initializeConfig(new Inferno(), true);
        turretOffsetFromAuto = 0;
        alliance = Alliance.RED;
        gamePhase = GamePhase.AUTO;
        initExpelActions();
        Pedro.createFollower(getMirroredPose("start"));
        follower.updatePose();
        turretYaw.call((Components.BotServo servo)->servo.switchControl("setPos"));
        executor.setCommands(
                new Commands.InstantCommand(setShooter::run),
                turretYaw.command((Components.BotServo servo)->servo.triggeredDynamicOffsetCommand(()->gamepad1.right_trigger>0.4,()->gamepad1.left_trigger>0.4,0.05))
        );
        executor.setWriteToTelemetry(()->{
            telemetryAddData("offset",turretYaw.get("turretYawFront").getOffset());
            telemetryAddLine("");
            telemetryAddData("target without offset",turretYaw.get("turretYawFront").getTargetMinusOffset());
            telemetryAddData("target with offset",turretYaw.get("turretYawFront").getTarget());
            telemetryAddData("raw servo pos",turretYaw.get("turretYawFront").getDevice().getPosition()*355);
            telemetryAddData("actual angle target",-(turretYaw.get("turretYawFront").getTarget()-180)+Math.toDegrees(follower.getHeading()));
        });
        executor.runLoop(this::opModeInInit);
        turretOffsetFromAuto = turretYaw.get("turretYawFront").getOffset();
        Components.activateActuatorControl();
        executor.setCommands(new SequentialCommand(
                        new SleepCommand(INITIAL_WAIT),
                        new PedroLinearCommand(getMirroredPose("firstShoot"),true), frontExpelShoot, setState(RobotState.INTAKE_FRONT),
                        new PedroCommand((PathBuilder b)->b.addPath(
                                        new BezierCurve(
                                                follower::getPose,
                                                getMirroredPose("secondSpikeCtrl"),
                                                getMirroredPose("secondSpike")
                                        )).setTangentHeadingInterpolation()
                                .addParametricCallback(slowDownT,()->follower.setMaxPower(slowDownAmount))
                                .addPath(
                                        new BezierCurve(
                                                follower::getPose,
                                                getMirroredPose("secondShootCtrl"),
                                                getMirroredPose("secondShoot")
                                        )
                                ).setConstantHeadingInterpolation(getMirroredHeading("secondShoot"))
                                .addParametricCallback(speedUpT,()->follower.setMaxPower(1.0))
                                .addParametricCallback(stopIntakeT,()->setState(RobotState.STOPPED).run()), true
                        ), backExpelShoot, setState(RobotState.INTAKE_FRONT),
                        new PedroCommand(
                                (PathBuilder b)->b.addPath(
                                        new BezierCurve(
                                                follower::getPose,
                                                getMirroredPose("gateOpenCtrl"),
                                                getMirroredPose("gateOpen")
                                        )
                                ).setLinearHeadingInterpolation(getMirroredHeading("secondShoot"), getMirroredHeading("gateOpen")),
                                true
                        ),
                        new SleepCommand(gateWait),
                        new PedroCommand(
                                (PathBuilder b)->b
                                        .addPath(
                                                new BezierLine(
                                                        follower::getPose,
                                                        getMirroredPose("gateIntake")
                                                )
                                        )
                                        .setLinearHeadingInterpolation(getMirroredHeading("gateOpen"), getMirroredHeading("gateIntake")),
                                true
                        ),
                        new CheckFull(gateIntakeTimeout),
                        new PedroCommand(
                                (PathBuilder b)->b.addPath(
                                                new BezierCurve(
                                                        follower::getPose,
                                                        getMirroredPose("thirdShootCtrl"),
                                                        getMirroredPose("thirdShoot")
                                                )
                                        ).setLinearHeadingInterpolation(getMirroredHeading("gateIntake"), getMirroredHeading("thirdShoot"))
                                        .addParametricCallback(stopIntakeT,()->setState(RobotState.STOPPED).run()),
                                true
                        ), backExpelShoot, setState(RobotState.INTAKE_FRONT),
                        new PedroCommand((PathBuilder b)->b
                                .addPath(
                                        new BezierLine(
                                                follower::getPose,
                                                getMirroredPose("firstSpike")
                                        )
                                ).setLinearHeadingInterpolation(getMirroredHeading("thirdShoot"),getMirroredHeading("firstSpike"))
                                .addParametricCallback(slowDownT,()->follower.setMaxPower(slowDownAmount))
                                .addPath(
                                        new BezierLine(
                                                follower::getPose,
                                                getMirroredPose("fourthShoot")
                                        )
                                ).setConstantHeadingInterpolation(getMirroredHeading("fourthShoot"))
                                .addParametricCallback(speedUpT,()->follower.setMaxPower(1.0))
                                .addParametricCallback(stopIntakeT,()->setState(RobotState.STOPPED).run())
                                .addParametricCallback(fourthShootSlowT,()->follower.setMaxPower(fourthShootSlowAmount)),true
                        ), backExpelShoot, setState(RobotState.INTAKE_FRONT),
                        new PedroCommand(
                                (PathBuilder b)->b.addPath(
                                        new BezierCurve(
                                                follower::getPose,
                                                getMirroredPose("gateOpenCtrl"),
                                                getMirroredPose("gateOpen")
                                        )
                                ).setLinearHeadingInterpolation(getMirroredHeading("secondShoot"), getMirroredHeading("gateOpen"))
                                .addParametricCallback(0,()->follower.setMaxPower(1.0)),
                                true
                        ),
                        new SleepCommand(gateWait),
                        new PedroCommand(
                                (PathBuilder b)->b
                                        .addPath(
                                                new BezierLine(
                                                        follower::getPose,
                                                        getMirroredPose("gateIntake")
                                                )
                                        )
                                        .setLinearHeadingInterpolation(getMirroredHeading("gateOpen"), getMirroredHeading("gateIntake")),
                                true
                        ),
                        new CheckFull(gateIntakeTimeout),
                        new PedroCommand(
                                (PathBuilder b)->b.addPath(
                                                new BezierCurve(
                                                        follower::getPose,
                                                        getMirroredPose("thirdShootCtrl"),
                                                        getMirroredPose("thirdShoot")
                                                )
                                        ).setLinearHeadingInterpolation(getMirroredHeading("gateIntake"), getMirroredHeading("thirdShoot"))
                                        .addParametricCallback(stopIntakeT,()->setState(RobotState.STOPPED).run()),
                                true
                        ), backExpelShoot, setState(RobotState.STOPPED),
                        new PedroLinearCommand(getMirroredPose("park"),true)
                ),
                clearIntegralAtPeak,
                Pedro.updateCommand(),
                loopFSM);
        executor.setWriteToTelemetry(()->{
            telemetryAddData("is busy",follower.isBusy());
            telemetryAddData("Ball Storage:", Arrays.asList(ballStorage));
            telemetryAddLine("");
            telemetryAddData("Robot State:",robotState);
            telemetryAddData("follower power",follower.getMaxPowerScaling());
            telemetryAddData("looptime",timer.time()-lastTime);
            lastTime = timer.time();
            telemetryAddLine("");
            telemetryAddData("Shot Type:",shotType);
            telemetryAddLine("");
            telemetryAddData("Classifier Count:",classifierBallCount);
            telemetryAddData("Current Shot Height:",currentBallPath);
            telemetryAddData("Shoot All Motif:",motifShootAll);
            telemetryAddLine("");
            telemetryAddData("Distance", Math.sqrt((follower.getPose().getX() - getTargetPoint()[0])*(follower.getPose().getX() - getTargetPoint()[0]) + (follower.getPose().getY() - getTargetPoint()[1])*(follower.getPose().getY() - getTargetPoint()[1])));
            telemetryAddData("Flywheel Velocity",flywheel.get("flywheelLeft").getVelocity());
            telemetryAddData("PoseX",follower.getPose().getX());
            telemetryAddData("PoseY",follower.getPose().getY());
            telemetryAddData("PoseHeading",Math.toDegrees(follower.getHeading()));
            telemetryAddData("Flywheel Left Power",flywheel.get("flywheelLeft").getPower());
            telemetryAddData("Flywheel Right Power",flywheel.get("flywheelRight").getPower());
            telemetryAddLine("");
            telemetryAddData("Yaw Target",turretYaw.get("turretYawFront").getTarget());
            telemetryAddData("Yaw Desired",-(turretYaw.get("turretYawFront").getTarget()-180)+Math.toDegrees(follower.getHeading()));
            telemetryAddData("Yaw Raw Pos",turretYaw.get("turretYawFront").getDevice().getPosition()*355);
        });
        executor.runLoop(this::opModeIsActive);
    }
}
