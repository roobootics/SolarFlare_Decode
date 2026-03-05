package org.firstinspires.ftc.teamcode.programs;

import static org.apache.commons.math3.util.FastMath.atan2;
import static org.firstinspires.ftc.teamcode.base.Commands.executor;
import static org.firstinspires.ftc.teamcode.base.Components.gamepad1;
import static org.firstinspires.ftc.teamcode.base.Components.initialize;
import static org.firstinspires.ftc.teamcode.base.Components.telemetry;
import static org.firstinspires.ftc.teamcode.pedroPathing.Pedro.follower;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.TURRET_PITCH_OFFSET;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.TURRET_PITCH_RATIO;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.alliance;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.ballStorage;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.classifierBallCount;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.clearIntegralAtPeak;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.currentBallPath;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.findMotif;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.flywheel;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.gamePhase;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.hoodDesired;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.leftFront;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.loopFSM;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.motif;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.robotState;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.setShooter;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.setState;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.shotType;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.targetFlywheelVelocity;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.targetPoint;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.turretOffsetFromAuto;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.turretPitch;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.turretYaw;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.vision;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.base.Commands.*;
import org.firstinspires.ftc.teamcode.base.Components;
import org.firstinspires.ftc.teamcode.pedroPathing.Pedro;
import org.firstinspires.ftc.teamcode.pedroPathing.Pedro.*;
import org.firstinspires.ftc.teamcode.robotconfigs.Inferno;

import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Objects;

import static org.apache.commons.math3.util.FastMath.tan;

public class FarPrimeSigmaConstants {
    public static final double INITIAL_WAIT = 1;
    public static final double SHOT_TIME = 1.2;
    public static final double PRE_SHOT_TIME = 0.4;
    public static final double slowDownT = 0.73;
    public static final double speedUpT = 0.13;
    public static final double stopIntakeT = 0.1;
    public static final double slowDownAmount = 1.0;
    public static Command shoot = new SequentialCommand(new SleepCommand(PRE_SHOT_TIME), setState(Inferno.RobotState.SHOOTING), new SleepCommand(SHOT_TIME), setState(Inferno.RobotState.INTAKE_FRONT));
    public static final HashMap<String, Pose> poses = new HashMap<>();
    public static Pose mirrorPose(Pose input){
        return new Pose(144-input.getX(),input.getY(),mirrorHeading(input.getHeading()));
    }
    public static double mirrorHeading(double input){return Math.PI - input;}
    static {
        poses.put("start",new Pose(60, 9.1, Math.toRadians(90)));
        poses.put("firstSpikeCtrl",new Pose(60.6,36.4,Math.toRadians(136)));
        poses.put("firstSpike",new Pose(10.7,35.1,Math.toRadians(180)));
        poses.put("firstShoot",new Pose(56.2, 21.4));
        poses.put("loadingZoneCtrl",new Pose(10.5, 35.8));
        poses.put("loadingZone",new Pose(9.4, 11.9,Math.toRadians(-100)));
        poses.put("secondShootCtrl",new Pose(31.2, 20.3));
        poses.put("shoot",new Pose(54.7, 17.6,Math.toRadians(180)));
    }
    public static Pose getPose(String input){if (alliance==Inferno.Alliance.BLUE) return poses.get(input); else return mirrorPose(Objects.requireNonNull(poses.get(input)));}
    public static double getHeading(String input){if (alliance==Inferno.Alliance.BLUE) return Objects.requireNonNull(poses.get(input)).getHeading(); else return mirrorHeading(Objects.requireNonNull(poses.get(input)).getHeading());}
    public static List<String> classNames = Arrays.asList("green","purple");
    public static double wallX = 9;
    public static double wallXOffset = 1.5;
    public static Command preloadShoot = new SequentialCommand(
            new SleepCommand(INITIAL_WAIT), shoot
    );
    public static Command firstSpike = new SequentialCommand(
            new PedroCommand(
                    b->b.addPath(new BezierCurve(getPose("start"), getPose("firstSpikeCtrl"), getPose("firstSpike")))
                            .setTangentHeadingInterpolation()
                            .addParametricCallback(slowDownT,()->follower.setMaxPower(slowDownAmount))
                        .addPath(new BezierLine(getPose("firstSpike"), getPose("firstShoot")))
                            .setTangentHeadingInterpolation().setReversed()
                            .addParametricCallback(speedUpT,()->follower.setMaxPower(1.0))
                            .addParametricCallback(stopIntakeT,()->setState(Inferno.RobotState.STOPPED).run()),
                    true
            ), shoot
    );
    public static Command loadingZone = new SequentialCommand(
            new PedroCommand(
                    b->b.addPath(new BezierCurve(getPose("firstShoot"), getPose("loadingZoneCtrl"), getPose("loadingZone")))
                            .setTangentHeadingInterpolation()
                            .addParametricCallback(slowDownT,()->follower.setMaxPower(slowDownAmount))
                        .addPath(new BezierCurve(getPose("loadingZone"), getPose("secondShootCtrl"), getPose("shoot")))
                            .setLinearHeadingInterpolation(getHeading("loadingZone"),getHeading("shoot"))
                            .addParametricCallback(speedUpT,()->follower.setMaxPower(1.0))
                            .addParametricCallback(stopIntakeT,()->setState(Inferno.RobotState.STOPPED).run()),
                    true
            ), shoot
    );
    public static Command visionIntake = new SequentialCommand(
            new PedroCommand(
                    b->{
                        Pose pos = follower.getPose();
                        Double angle = vision.intakingAngleArtifacts2(vision.getArtifactDescriptors(pos,classNames),pos,1);
                        if (Objects.isNull(angle)) angle = Math.toRadians(185);
                        else angle = Math.toRadians(angle);
                        if (angle>Math.toRadians(200)) angle = Math.toRadians(200);
                        double wallY = tan(angle)*(wallX-pos.getX()) + pos.getY();
                        return b.addPath(new BezierLine(getPose("shoot"),new Pose(wallX+wallXOffset,wallY)))
                            .setConstantHeadingInterpolation(Math.toRadians(180))
                            .addParametricCallback(slowDownT,()->follower.setMaxPower(slowDownAmount))
                        .addPath(new BezierLine(new Pose(wallX+wallXOffset,wallY), getPose("shoot")))
                            .setConstantHeadingInterpolation(Math.toRadians(180))
                            .addParametricCallback(speedUpT,()->follower.setMaxPower(1.0))
                            .addParametricCallback(stopIntakeT,()->setState(Inferno.RobotState.STOPPED).run());},
                    true
            ),
            shoot
    );
    public static void runOpMode(Inferno.Alliance alliance, LinearOpMode opMode){
        initialize(opMode,new Inferno(),true,true);
        loadingZone.reset();
        leftFront.resetEncoder();
        Inferno.alliance = alliance;
        if (alliance == Inferno.Alliance.BLUE) {wallX = 9; wallXOffset = 2;} else {wallX = 135; wallXOffset = -2;}
        turretOffsetFromAuto = 0;
        turretYaw.call(servo->servo.setOffset(0));
        gamePhase = Inferno.GamePhase.AUTO;
        Pedro.createFollower(getPose("start"));
        turretYaw.call((Components.CRBotServo servo)->servo.switchControl("PID"));
        executor.setWriteToTelemetry(()->{
            telemetry.addData("offset",turretYaw.get("turretYawTop").getOffset());
            telemetry.addLine("");
            telemetry.addLine("Please, Speed, we need this.");
        });
        executor.setCommands(
                new InstantCommand(setShooter::run),
                turretYaw.command((Components.CRBotServo servo)->servo.triggeredDynamicOffsetCommand(()->gamepad1.left_trigger>0.4,()->gamepad1.right_trigger>0.4,0.05))
        );
        executor.runLoop(opMode::opModeInInit);
        turretOffsetFromAuto = turretYaw.get("turretYawTop").getOffset();
        Components.activateActuatorControl();
        executor.setWriteToTelemetry(()->{
            telemetry.addData("Motif",Arrays.asList(motif));
            telemetry.addLine("");
            telemetry.addData("Target Flywheel Velocity", targetFlywheelVelocity);
            telemetry.addData("Flywheel Velocity", flywheel.get("flywheelLeft").getVelocity());
            telemetry.addLine("");
            telemetry.addData("Hood Angle",(turretPitch.get("turretPitchLeft").getTarget()-TURRET_PITCH_OFFSET)/TURRET_PITCH_RATIO);
            telemetry.addData("Hood Desired",hoodDesired);
            telemetry.addLine("");
            telemetry.addData("Ball Storage", Arrays.asList(ballStorage));
            telemetry.addLine("");
            telemetry.addData("Robot State",robotState);
            telemetry.addLine("");
            telemetry.addData("Shot Type",shotType);
            telemetry.addLine("");
            telemetry.addData("Classifier Count",classifierBallCount);
            telemetry.addData("Current Shot Height",currentBallPath);
            telemetry.addLine("");
            telemetry.addData("PoseX",follower.getPose().getX());
            telemetry.addData("PoseY",follower.getPose().getY());
            telemetry.addData("PoseHeading",Math.toDegrees(follower.getHeading()));
            telemetry.addLine("");
            telemetry.addData("Flywheel Left Power",flywheel.get("flywheelLeft").getPower());
            telemetry.addData("Flywheel Right Power",flywheel.get("flywheelRight").getPower());
        });
        executor.setCommands(
                findMotif,
                new SequentialCommand(
                        preloadShoot,
                        firstSpike,
                        loadingZone,
                        visionIntake,
                        visionIntake,
                        visionIntake,
                        visionIntake
                ),
                clearIntegralAtPeak,
                Pedro.updateCommand(),
                loopFSM,
                new RunResettingLoop(
                new InstantCommand(
                        ()->{
                            if (loadingZone.isBusy()) turretYaw.call(servo->servo.setTarget(Math.toDegrees(atan2(targetPoint[1]-getPose("shoot").getY(),targetPoint[0]-getPose("shoot").getX())-getHeading("shoot"))));
                        }
                ))
        );
        executor.runLoop(opMode::opModeIsActive);
    }
}
