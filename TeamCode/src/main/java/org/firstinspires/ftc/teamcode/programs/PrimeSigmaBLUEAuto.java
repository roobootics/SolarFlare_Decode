package org.firstinspires.ftc.teamcode.programs;

import static org.firstinspires.ftc.teamcode.base.Commands.executor;
import static org.firstinspires.ftc.teamcode.base.Components.initialize;
import static org.firstinspires.ftc.teamcode.base.Components.initializeConfig;
import static org.firstinspires.ftc.teamcode.base.Components.telemetryAddData;
import static org.firstinspires.ftc.teamcode.base.Components.telemetryAddLine;
import static org.firstinspires.ftc.teamcode.base.Components.timer;
import static org.firstinspires.ftc.teamcode.pedroPathing.Pedro.follower;
import static org.firstinspires.ftc.teamcode.programs.PrimeSigmaConstants.fourthShootSlowAmount;
import static org.firstinspires.ftc.teamcode.programs.PrimeSigmaConstants.fourthShootSlowT;
import static org.firstinspires.ftc.teamcode.programs.PrimeSigmaConstants.frontExpelShoot;
import static org.firstinspires.ftc.teamcode.programs.PrimeSigmaConstants.backExpelShoot;
import static org.firstinspires.ftc.teamcode.programs.PrimeSigmaConstants.INITIAL_WAIT;
import static org.firstinspires.ftc.teamcode.programs.PrimeSigmaConstants.gateIntakeTimeout;
import static org.firstinspires.ftc.teamcode.programs.PrimeSigmaConstants.gateWait;
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
import static org.firstinspires.ftc.teamcode.programs.PrimeSigmaConstants.getHeading;
import static org.firstinspires.ftc.teamcode.programs.PrimeSigmaConstants.getPose;
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

import org.firstinspires.ftc.teamcode.base.Components;
import org.firstinspires.ftc.teamcode.pedroPathing.Pedro;
import org.firstinspires.ftc.teamcode.robotconfigs.Inferno;
import org.firstinspires.ftc.teamcode.robotconfigs.Inferno.*;
import org.firstinspires.ftc.teamcode.base.Commands.*;
import org.firstinspires.ftc.teamcode.pedroPathing.Pedro.*;

import java.util.Arrays;

@Autonomous
public class PrimeSigmaBLUEAuto extends LinearOpMode {
    private double lastTime;
    @Override
    public void runOpMode(){
        initialize(hardwareMap,telemetry);
        initializeConfig(new Inferno(), true);
        turretOffsetFromAuto = 0;
        alliance = Inferno.Alliance.BLUE;
        gamePhase = GamePhase.AUTO;
        initExpelActions();
        Pedro.createFollower(getPose("start"));
        follower.updatePose();
        turretYaw.call((Components.BotServo servo)->servo.switchControl("setPos"));
        executor.setCommands(
                new InstantCommand(setShooter::run),
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
        executor.setCommands(new SequentialCommand(
                        new SleepCommand(INITIAL_WAIT),
                        new PedroLinearCommand(getPose("firstShoot"),true), backExpelShoot, setState(RobotState.INTAKE_BACK),
                        new PedroCommand((PathBuilder b)->b.addPath(
                                        new BezierCurve(
                                                follower::getPose,
                                                getPose("secondSpikeCtrl"),
                                                getPose("secondSpike")
                                        )).setTangentHeadingInterpolation().setReversed()
                                .addParametricCallback(slowDownT,()->follower.setMaxPower(slowDownAmount))
                                .addPath(
                                        new BezierCurve(
                                                follower::getPose,
                                                getPose("secondShootCtrl"),
                                                getPose("secondShoot")
                                        )
                                ).setConstantHeadingInterpolation(getHeading("secondShoot"))
                                .addParametricCallback(speedUpT,()->follower.setMaxPower(1.0))
                                .addParametricCallback(stopIntakeT,()->setState(RobotState.STOPPED).run()), true
                        ), frontExpelShoot, setState(RobotState.INTAKE_BACK),
                        new PedroCommand(
                                (PathBuilder b)->b.addPath(
                                        new BezierCurve(
                                                follower::getPose,
                                                getPose("gateOpenCtrl"),
                                                getPose("gateOpen")
                                        )
                                ).setLinearHeadingInterpolation(getHeading("secondShoot"), getHeading("gateOpen")),
                                true
                        ),
                        new SleepCommand(gateWait),
                        new PedroCommand(
                                (PathBuilder b)->b
                                        .addPath(
                                                new BezierLine(
                                                        follower::getPose,
                                                        getPose("gateIntake")
                                                )
                                        )
                                        .setLinearHeadingInterpolation(getHeading("gateOpen"), getHeading("gateIntake")),
                                true
                        ),
                        new CheckFull(gateIntakeTimeout),
                        new PedroCommand(
                                (PathBuilder b)->b.addPath(
                                                new BezierCurve(
                                                        follower::getPose,
                                                        getPose("thirdShootCtrl"),
                                                        getPose("thirdShoot")
                                                )
                                        ).setLinearHeadingInterpolation(getHeading("gateIntake"), getHeading("thirdShoot"))
                                        .addParametricCallback(stopIntakeT,()->setState(RobotState.STOPPED).run()),
                                true
                        ), frontExpelShoot, setState(RobotState.INTAKE_BACK),
                        new PedroCommand((PathBuilder b)->b
                                .addPath(
                                        new BezierLine(
                                                follower::getPose,
                                                getPose("firstSpike")
                                        )
                                ).setLinearHeadingInterpolation(getHeading("thirdShoot"),getHeading("firstSpike"))
                                .addParametricCallback(slowDownT,()->follower.setMaxPower(slowDownAmount))
                                .addPath(
                                        new BezierLine(
                                                follower::getPose,
                                                getPose("fourthShoot")
                                        )
                                ).setConstantHeadingInterpolation(getHeading("fourthShoot"))
                                .addParametricCallback(speedUpT,()->follower.setMaxPower(1.0))
                                .addParametricCallback(stopIntakeT,()->setState(RobotState.STOPPED).run())
                                .addParametricCallback(fourthShootSlowT,()->follower.setMaxPower(fourthShootSlowAmount)),true
                        ), frontExpelShoot, setState(RobotState.INTAKE_BACK),
                        new PedroCommand(
                                (PathBuilder b)->b.addPath(
                                                new BezierCurve(
                                                        follower::getPose,
                                                        getPose("thirdSpikeCtrl"),
                                                        getPose("thirdSpike")
                                                )
                                        ).setConstantHeadingInterpolation(getHeading("thirdSpike"))
                                        .addParametricCallback(0,()->follower.setMaxPower(1.0))
                                        .addParametricCallback(slowDownT,()->follower.setMaxPower(slowDownAmount))
                                        .addPath(
                                                new BezierLine(
                                                        follower::getPose,
                                                        getPose("fifthShoot")
                                                )
                                        ).setConstantHeadingInterpolation(getHeading("fifthShoot"))
                                        .addParametricCallback(speedUpT,()->follower.setMaxPower(1.0))
                                        .addParametricCallback(stopIntakeT,()->setState(RobotState.STOPPED).run()),true
                        ), frontExpelShoot, setState(RobotState.STOPPED),
                        new PedroLinearCommand(getPose("park"),true)
                ),
                clearIntegralAtPeak,
                Pedro.updateCommand(),
                loopFSM);
        executor.runLoop(this::opModeIsActive);
    }
}
