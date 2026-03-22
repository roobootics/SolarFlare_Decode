package org.firstinspires.ftc.teamcode.programs;

import static org.apache.commons.math3.util.FastMath.atan2;
import static org.firstinspires.ftc.teamcode.base.Commands.executor;
import static org.firstinspires.ftc.teamcode.base.Components.gamepad1;
import static org.firstinspires.ftc.teamcode.base.Components.initialize;
import static org.firstinspires.ftc.teamcode.base.Components.telemetry;
import static org.firstinspires.ftc.teamcode.base.Components.timer;
import static org.firstinspires.ftc.teamcode.pedroPathing.Pedro.follower;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.TURRET_PITCH_OFFSET;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.TURRET_PITCH_RATIO;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.alliance;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.ballStorage;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.classifierBallCount;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.currentBallPath;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.findMotif;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.flywheel;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.gamePhase;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.hoodDesired;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.loopFSM;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.motif;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.readBallStorage;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.robotState;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.setState;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.setTargetPoint;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.shotType;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.targetFlywheelVelocity;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.targetPoint;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.transfer;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.turretOffsetFromAuto;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.turretPitch;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.turretYaw;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.vision;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
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
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.yawDesired;

public class FarPrimeSigmaConstants {
    public static final double INITIAL_WAIT = 0.7;
    public static final double SHOT_TIME = 1;
    public static final double PRE_SHOT_TIME = 0.35;
    public static final double slowDownT = 0.73;
    public static final double speedUpT = 0.13;
    public static double shootSlowT = 0.77;
    public static double shootSlowAmount = 0.75;
    public static final double stopIntakeT = 0.17;
    public static final double slowDownAmount = 1.0;
    public static Double angle;
    public static Command shoot = new SequentialCommand(new SleepCommand(PRE_SHOT_TIME),
            setState(Inferno.RobotState.SHOOTING),
            new SleepCommand(SHOT_TIME),
            new ConditionalCommand(
                    new IfThen(
                        ()->{
                            readBallStorage();
                            int ballsPresent = 0;

                            for (Inferno.Color color : ballStorage){
                                if (!Objects.isNull(color)) ballsPresent+=1;
                            }
                            return ballsPresent!=0;
                        },
                        new SequentialCommand(
                                new InstantCommand(transfer::reset),
                                new SleepCommand(SHOT_TIME)
                        )
                    )
            ),
            setState(Inferno.RobotState.INTAKE_FRONT));
    public static final HashMap<String, Pose> poses = new HashMap<>();
    public static double visionHeading;
    public static Pose mirrorPose(Pose input){
        return new Pose(144-input.getX(),input.getY(),mirrorHeading(input.getHeading()));
    }
    public static double mirrorHeading(double input){return Math.PI - input;}
    static {
        poses.put("start",new Pose(60, 9, Math.toRadians(90)));
        poses.put("firstSpikeCtrl",new Pose(60.6,36.4,Math.toRadians(136)));
        poses.put("firstSpike",new Pose(10.7,35.1,Math.toRadians(180)));
        poses.put("firstShoot",new Pose(56.2, 21.4,Math.toRadians(163.2430096)));
        poses.put("loadingZoneCtrl",new Pose(10.5, 35.8));
        poses.put("loadingZone",new Pose(9.4, 11.9,Math.toRadians(-100)));
        poses.put("secondShootCtrl",new Pose(31.2, 20.3));
        poses.put("shoot",new Pose(54.7, 17.6,Math.toRadians(180)));
        poses.put("park",new Pose(40.7,17.6,Math.toRadians(180)));
    }
    public static Pose getPose(String input){if (alliance==Inferno.Alliance.BLUE) return poses.get(input); else return mirrorPose(Objects.requireNonNull(poses.get(input)));}
    public static double getHeading(String input){if (alliance==Inferno.Alliance.BLUE) return Objects.requireNonNull(poses.get(input)).getHeading(); else return mirrorHeading(Objects.requireNonNull(poses.get(input)).getHeading());}
    public static List<String> classNames = Arrays.asList("green","purple");
    public static double wallX = 9;
    public static double middleX = (getPose("shoot").getX()+wallX)/2;
    public static double wallXOffset = 1;
    public static int flag = 0;
    public static boolean loadingZoneOverride = false;
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
                            .addParametricCallback(stopIntakeT,()->setState(Inferno.RobotState.STOPPED).run())
                            .addParametricCallback(shootSlowT,()->follower.setMaxPower(shootSlowAmount))
                            .addParametricCallback(0.94,()->follower.setMaxPower(1.0)),
                    true
            ), shoot
    );
    public static Command loadingZone = new SequentialCommand(
            new PedroCommand(
                    b->b.addPath(new BezierCurve(getPose("firstShoot"), getPose("loadingZoneCtrl"), getPose("loadingZone")))
                            .setTangentHeadingInterpolation()
                            .addParametricCallback(slowDownT,()->follower.setMaxPower(slowDownAmount))
                        .addPath(new BezierCurve(getPose("loadingZone"), getPose("secondShootCtrl"), getPose("shoot")))
                            .setHeadingInterpolation(
                                    HeadingInterpolator.linear(getHeading("loadingZone"),getHeading("shoot"),0.7)
                            )
                            .addParametricCallback(speedUpT,()->follower.setMaxPower(1.0))
                            .addParametricCallback(stopIntakeT,()->setState(Inferno.RobotState.STOPPED).run())
                            .addParametricCallback(0.83,()->loadingZoneOverride=true)
                            .addParametricCallback(shootSlowT,()->follower.setMaxPower(shootSlowAmount))
                            .addParametricCallback(0.94,()->follower.setMaxPower(1.0)),
                    true
            ), shoot
    );
    public static Pose getClusterPose(double x, double xOffset){
        Pose pos = follower.getPose();
        Double angle = vision.intakingAngleArtifacts2(vision.getArtifactDescriptors(pos,classNames),pos,1);
        FarPrimeSigmaConstants.angle = angle;
        if (Objects.isNull(angle)) angle = visionHeading;
        else angle = Math.toRadians(angle);
        double y = Math.max(7.5,tan(angle)*(x-pos.getX()) + pos.getY());
        return new Pose(x+xOffset,y);
    }
    public static class VisionIntake extends CompoundCommand{
        public static double firstPathStartTime;
        public static boolean failsafe = false;
        public VisionIntake(){
            setGroup(
                    new SequentialCommand(
                            new InstantCommand(()->{firstPathStartTime=timer.time(); failsafe = false;}),
                            new ConditionalCommand(
                                    new IfThen(()->!failsafe, new PedroCommand(
                                            b->{
                                                Pose pos1 = getClusterPose(middleX,0);
                                                return b.addPath(new BezierLine(getPose("shoot"),pos1))
                                                            .setConstantHeadingInterpolation(visionHeading)
                                                        .addPath(new BezierLine(()->pos1,()->getClusterPose(wallX,wallXOffset)))
                                                            .addParametricCallback(slowDownT,()->follower.setMaxPower(slowDownAmount))
                                                            .addCallback(()->(follower.getChainIndex()==1&&timer.time()-firstPathStartTime>5),()->failsafe=true)
                                                        .addPath(new BezierLine(follower::getPose, getPose("shoot")))
                                                            .setConstantHeadingInterpolation(visionHeading)
                                                            .addParametricCallback(speedUpT,()->follower.setMaxPower(1.0))
                                                            .addParametricCallback(stopIntakeT,()->setState(Inferno.RobotState.STOPPED).run())
                                                            .addParametricCallback(shootSlowT,()->follower.setMaxPower(shootSlowAmount))
                                                            .addParametricCallback(0.94,()->follower.setMaxPower(1.0));},
                                            true
                                    )),
                                    new IfThen(()->failsafe,new PedroCommand(
                                            b->b.addPath(new BezierLine(follower::getPose, getPose("shoot")))
                                                    .setConstantHeadingInterpolation(visionHeading)
                                                    .addParametricCallback(speedUpT,()->follower.setMaxPower(1.0))
                                                    .addParametricCallback(stopIntakeT,()->setState(Inferno.RobotState.STOPPED).run())
                                                    .addParametricCallback(shootSlowT,()->follower.setMaxPower(shootSlowAmount))
                                                    .addParametricCallback(0.94,()->follower.setMaxPower(1.0)),true
                                    ))
                            ),
                            shoot
                    )
            );
        }
    }
    public static Command visionIntake = new VisionIntake();
    public static Command park = new ParallelCommand(
            setState(null),
            new PedroCommand(
                    b->b.addPath(new BezierLine(getPose("shoot"),getPose("park")))
                            .setConstantHeadingInterpolation(getHeading("park")),true
            )
    );
    public static void runOpMode(Inferno.Alliance alliance, LinearOpMode opMode){
        initialize(opMode,new Inferno(),true,true);
        findMotif.reset();
        turretOffsetFromAuto = 0;
        Inferno.motifDetected = false;
        Components.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        flag = 0;
        Inferno.alliance = alliance;
        if (alliance == Inferno.Alliance.BLUE) {wallX = 9; wallXOffset = 2; visionHeading = Math.toRadians(180);} else {wallX = 135; wallXOffset = -2; visionHeading = Math.toRadians(0);}
        middleX = (getPose("shoot").getX()+wallX)/2;
        gamePhase = Inferno.GamePhase.AUTO;
        Pedro.createFollower(getPose("start"));
        executor.setWriteToTelemetry(()->{
            telemetry.addData("Target Flywheel Velocity",0);
            telemetry.addData("Flywheel Velocity",0);
            telemetry.addData("pos",turretYaw.get("turretYawTop").getCurrentPosition());
            telemetry.addData("botX",follower.getPose().getX());
            telemetry.addData("botY",follower.getPose().getY());
            telemetry.addData("botHeading",follower.getPose().getHeading());
            telemetry.addLine("");
            telemetry.addLine("Please, Speed, we need this.");
        });
        executor.setCommands(
                turretYaw.command((Components.BotServo servo)->servo.triggeredDynamicOffsetCommand(()->gamepad1.left_trigger>0.2,()->gamepad1.right_trigger>0.2,0.05))
        );
        loadingZoneOverride = false;
        setTargetPoint();
        executor.runLoop(opMode::opModeInInit);
        turretOffsetFromAuto = turretYaw.get("turretYawTop").getOffset();
        Components.activateActuatorControl();
        executor.setWriteToTelemetry(()->{
            telemetry.addData("Motif",Arrays.asList(motif));
            telemetry.addLine("");
            telemetry.addData("Target Flywheel Velocity", targetFlywheelVelocity);
            telemetry.addData("Flywheel Velocity", flywheel.get("flywheelLeft").getVelocity());
            telemetry.addData("Flywheel Error", targetFlywheelVelocity - flywheel.get("flywheelLeft").getVelocity());
            telemetry.addLine("");
            Components.telemetry.addData("Yaw Pos",turretYaw.get("turretYawTop").getCurrentPosition());
            Components.telemetry.addData("Yaw Target",turretYaw.get("turretYawTop").getTarget());
            Components.telemetry.addData("Yaw Angle",yawDesired);
            Components.telemetry.addData("Yaw Error", turretYaw.get("turretYawTop").getTarget() - turretYaw.get("turretYawTop").getCurrentPosition());
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
        Command mainPath = new SequentialCommand(
                preloadShoot,
                new InstantCommand(()->flag=1),
                firstSpike,
                new InstantCommand(()->flag=2),
                loadingZone,
                new InstantCommand(()->loadingZoneOverride = false),
                visionIntake,
                visionIntake,
                visionIntake,
                visionIntake
        );
        executor.setCommands(
                new ParallelCommand(findMotif, new SequentialCommand(new SleepCommand(5),new InstantCommand(findMotif::stop))),
                new SequentialCommand(
                        new ParallelCommand(
                                mainPath,
                                new SequentialCommand(
                                        new SleepUntilTrue(mainPath::isFinished,29),
                                        new InstantCommand(mainPath::stop)
                                )
                        ),
                        park
                ),
                Pedro.updateCommand(),
                loopFSM,
                new RunResettingLoop(new InstantCommand(
                        ()->{
                            if (flag==2 && !shoot.isBusy() && !loadingZoneOverride) turretYaw.call(servo->servo.setTarget(Math.toDegrees(atan2(targetPoint[1]-getPose("shoot").getY(),targetPoint[0]-getPose("shoot").getX())-getHeading("shoot"))));
                            else if (flag==1 && !shoot.isBusy()  && !loadingZoneOverride) turretYaw.call(servo->servo.setTarget(Math.toDegrees(atan2(targetPoint[1]-getPose("firstShoot").getY(),targetPoint[0]-getPose("firstShoot").getX())-getHeading("firstShoot"))));
                        }
                    ),
                    new InstantCommand(
                            ()->{if (flag==0) targetFlywheelVelocity = Inferno.VelRegression.regressFormula(getPose("start").distanceFrom(new Pose(targetPoint[0],targetPoint[1])));
                                else targetFlywheelVelocity = Inferno.VelRegression.regressFormula(getPose("shoot").distanceFrom(new Pose(targetPoint[0],targetPoint[1])));}
                    )
                )
        );
        executor.runLoop(opMode::opModeIsActive);
    }
}
