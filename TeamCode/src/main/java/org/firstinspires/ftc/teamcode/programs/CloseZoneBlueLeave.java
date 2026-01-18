package org.firstinspires.ftc.teamcode.programs;

import static org.firstinspires.ftc.teamcode.base.Commands.executor;
import static org.firstinspires.ftc.teamcode.base.Components.activateActuatorControl;
import static org.firstinspires.ftc.teamcode.base.Components.initialize;
import static org.firstinspires.ftc.teamcode.base.Components.initializeConfig;
import static org.firstinspires.ftc.teamcode.base.Components.telemetryAddData;
import static org.firstinspires.ftc.teamcode.base.Components.telemetryAddLine;
import static org.firstinspires.ftc.teamcode.pedroPathing.Pedro.follower;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.afterExpelSet;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.alliance;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.ballStorage;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.classifierBallCount;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.currentBallPath;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.flywheel;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.gamePhase;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.getTargetPoint;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.motifShootAll;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.robotState;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.shotType;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.base.Components.BotMotor;
import org.firstinspires.ftc.teamcode.pedroPathing.Pedro;
import org.firstinspires.ftc.teamcode.pedroPathing.Pedro.PedroLinearCommand;
import org.firstinspires.ftc.teamcode.robotconfigs.Inferno;
import org.firstinspires.ftc.teamcode.robotconfigs.Inferno.Alliance;
import org.firstinspires.ftc.teamcode.robotconfigs.Inferno.GamePhase;
import org.firstinspires.ftc.teamcode.robotconfigs.Inferno.RobotState;

import java.util.Arrays;

@Autonomous
public class CloseZoneBlueLeave extends LinearOpMode{

    @Override
    public void runOpMode() throws InterruptedException {
        alliance = Alliance.BLUE;
        gamePhase = GamePhase.AUTO;
        afterExpelSet = RobotState.INTAKE_BACK;
        initialize(hardwareMap,telemetry);
        initializeConfig(new Inferno(), true);
        Pedro.createFollower(new Pose(20, 122.62, Math.toRadians(143.5)));
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
        activateActuatorControl();
        flywheel.call((BotMotor motor)->motor.switchControl("controlOff"));
        executor.setCommands(
                new PedroLinearCommand(24,96,Math.toRadians(143.5),true),
                Pedro.updateCommand()
        );
        executor.runLoop(this::opModeIsActive);
    }
}
