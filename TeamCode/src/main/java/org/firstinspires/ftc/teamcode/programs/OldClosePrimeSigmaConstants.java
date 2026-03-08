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
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.currentBallPath;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.flywheel;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.gamePhase;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.hoodDesired;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.initEncoderError;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.leftFront;
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
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.turretPitch;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.turretYaw;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.PathBuilder;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.base.Commands.*;
import org.firstinspires.ftc.teamcode.base.Components;
import org.firstinspires.ftc.teamcode.pedroPathing.Pedro;
import org.firstinspires.ftc.teamcode.pedroPathing.Pedro.*;
import org.firstinspires.ftc.teamcode.robotconfigs.Inferno;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.Objects;

public class OldClosePrimeSigmaConstants {
    public static final double INITIAL_WAIT = 0.01;
    public static final double PRE_SHOT_TIME = 0.3;
    public static final double SHOT_TIME = 1;
    public static final double slowDownT = 0.73;
    public static final double speedUpT = 0.05;
    public static final double stopIntakeT = 0.15;
    public static final double slowDownAmount = 0.67;
    public static final double gateWait = 0;
    public static final double gateIntakeTimeout = 1;
    public static final double fourthShootSlowT = 0.73;
    public static final double fourthShootSlowAmount = 0.76;
    public static final double secondShootSlowT = 0.85;
    public static final double secondShootSlowAmount = 0.76;
    public static Command shoot = new SequentialCommand(new SleepCommand(PRE_SHOT_TIME),
            setState(Inferno.RobotState.SHOOTING),
            new SleepCommand(SHOT_TIME),
            new ConditionalCommand(
                    new IfThen(
                            ()->{
                                readBallStorage();
                                boolean allNull = true;
                                for (Inferno.Color color : ballStorage){
                                    if (!Objects.isNull(color)) allNull = false;
                                }
                                return !allNull;
                            },
                            new SequentialCommand(
                                    new InstantCommand(transfer::reset),
                                    new SleepCommand(SHOT_TIME)
                            )
                    )
            ),
            intake());
    public static HeadingInterpolator tangent;
    public static final HashMap<String, Pose> poses = new HashMap<>();
    public static Pose mirrorPose(Pose input){
        return new Pose(144-input.getX(),input.getY(),mirrorHeading(input.getHeading()));
    }
    public static double mirrorHeading(double input){return (2*Math.PI - input)%2*Math.PI;}
    static {
        poses.put("start",new Pose(19.68, 121.72, Math.toRadians(144)));
        poses.put("firstShoot",new Pose(53.312,90.398,Math.toRadians(144)));
        poses.put("shoot",new Pose(57.85,77.42,Math.toRadians(0)));
        poses.put("secondSpikeCtrl",new Pose(41.07, 58.06));
        poses.put("secondSpike",new Pose(16.329, 58.805));
        poses.put("gateOpen",new Pose(14.97, 58.72,Math.toRadians(0)));
        poses.put("gateIntake",new Pose(14.97, 56.16,Math.toRadians(-50)));
        poses.put("thirdShootCtrl",new Pose(44.169, 52.575));
        poses.put("firstSpike",new Pose(22.773, 79.829,Math.toRadians(0)));
        poses.put("fourthShoot",new Pose(53.451, 87.203,Math.toRadians(0)));
        poses.put("thirdSpikeCtrl",new Pose(80.067, 27.483));
        poses.put("thirdSpike",new Pose(13.504, 36.131,Math.toRadians(0)));
        poses.put("park",new Pose(45,79,Math.toRadians(360)));
    }
    public static Pose getPose(String input){if (alliance==Inferno.Alliance.BLUE) return poses.get(input); else return mirrorPose(Objects.requireNonNull(poses.get(input)));}
    public static double getHeading(String input){if (alliance==Inferno.Alliance.BLUE) return Objects.requireNonNull(poses.get(input)).getHeading(); else return mirrorHeading(Objects.requireNonNull(poses.get(input)).getHeading());}
    public static Command intake(){return new InstantCommand(()->{if (alliance==Inferno.Alliance.BLUE) setState(Inferno.RobotState.INTAKE_BACK).run(); else setState(Inferno.RobotState.INTAKE_FRONT).run();});}
    public static Command preloadShoot = new SequentialCommand(new SleepCommand(INITIAL_WAIT),
            new PedroCommand(b->
                    b.addPath(new BezierLine(follower::getPose,getPose("firstShoot")))
                    .setConstantHeadingInterpolation(getHeading("firstShoot")),
    true),
    shoot);
    public static Command secondSpike = new SequentialCommand(new PedroCommand(
            (PathBuilder b)->b.addPath(
                    new BezierCurve(
                            follower::getPose,
                            getPose("secondSpikeCtrl"),
                            getPose("secondSpike")
                    ))
                .setHeadingInterpolation(tangent)
                .addParametricCallback(slowDownT,()->follower.setMaxPower(slowDownAmount))
            .addPath(
                    new BezierLine(
                            follower::getPose,
                            getPose("shoot")
                    )
            ).setConstantHeadingInterpolation(getHeading("shoot"))
                .addParametricCallback(speedUpT,()->follower.setMaxPower(1.0))
                .addParametricCallback(stopIntakeT,()->setState(Inferno.RobotState.STOPPED).run())
                .addParametricCallback(secondShootSlowT,()->follower.setMaxPower(secondShootSlowAmount))
                .addParametricCallback(0.93,()->follower.setMaxPower(1.0)), true), shoot);
    public static Command gate = new SequentialCommand(new PedroCommand(
            (PathBuilder b)->b.addPath(
                    new BezierLine(
                            follower::getPose,
                            getPose("gateOpen")
                    )
                    ).setLinearHeadingInterpolation(follower.getHeading(), getHeading("gateOpen"))
                    .addPath(
                            new BezierLine(
                                    follower::getPose,
                                    getPose("gateIntake")
                            )
                    )
                    .setHeadingInterpolation(HeadingInterpolator.linear(getHeading("gateOpen"),getHeading("gateIntake"))),
            true),
            new Inferno.CheckFull(gateIntakeTimeout),
            new PedroCommand(
                    (PathBuilder b)->b.addPath(
                                    new BezierCurve(
                                            follower::getPose,
                                            getPose("thirdShootCtrl"),
                                            getPose("shoot")
                                    )
                            ).setHeadingInterpolation(HeadingInterpolator.linear(getHeading("gateIntake"), getHeading("shoot"),0.7))
                            .addParametricCallback(stopIntakeT,()->setState(Inferno.RobotState.STOPPED).run()),
                    true), shoot);
    public static Command firstSpike = new SequentialCommand(
            new PedroCommand((PathBuilder b)->b
                    .addPath(
                            new BezierLine(
                                    follower::getPose,
                                    getPose("firstSpike")
                            )
                    ).setLinearHeadingInterpolation(follower.getHeading(), getHeading("firstSpike"))
                    .addParametricCallback(slowDownT,()->follower.setMaxPower(slowDownAmount))
                    .addPath(
                            new BezierLine(
                                    follower::getPose,
                                    getPose("shoot")
                            )
                    ).setConstantHeadingInterpolation(getHeading("shoot"))
                    .addParametricCallback(speedUpT,()->follower.setMaxPower(1.0))
                    .addParametricCallback(stopIntakeT,()->setState(Inferno.RobotState.STOPPED).run())
                    .addParametricCallback(fourthShootSlowT,()->follower.setMaxPower(fourthShootSlowAmount))
                    .addParametricCallback(0.93,()->follower.setMaxPower(1.0)),true), shoot
    );
    public static Command thirdSpike =  new SequentialCommand(new PedroCommand(
            (PathBuilder b)->b.addPath(
                            new BezierCurve(
                                    follower::getPose,
                                    getPose("thirdSpikeCtrl"),
                                    getPose("thirdSpike")
                            )
                    ).setConstantHeadingInterpolation(getHeading("thirdSpike"))
                    .addParametricCallback(slowDownT,()->follower.setMaxPower(slowDownAmount))
                    .addPath(
                            new BezierLine(
                                    follower::getPose,
                                    getPose("shoot")
                            )
                    ).setConstantHeadingInterpolation(getHeading("shoot"))
                    .addParametricCallback(speedUpT,()->follower.setMaxPower(1.0))
                    .addParametricCallback(stopIntakeT,()->setState(Inferno.RobotState.STOPPED).run()),true), shoot);
    public static Command park = new SequentialCommand(setState(Inferno.RobotState.STOPPED),
            new PedroCommand(b->
                    b.addPath(new BezierLine(follower::getPose,getPose("park")))
                            .setLinearHeadingInterpolation(getHeading("shoot"),getHeading("park")),
    true));
    public static ArrayList<Command> pathList = new ArrayList<>(Arrays.asList(
            preloadShoot,
            secondSpike,
            gate,
            gate,
            firstSpike
    ));
    public static ArrayList<String> pathListDisplay = new ArrayList<>(Arrays.asList(
            "preloadShoot",
            "secondSpike",
            "gate",
            "gate",
            "firstSpike"
    ));
    public static int selectionIndex = 0;
    public static boolean isInserting = true;
    public static void addAction(Command command, String label){
        if (isInserting) {
            pathListDisplay.add(selectionIndex,label);
            pathList.add(selectionIndex,command);
            selectionIndex+=1;
        } else {
            pathListDisplay.set(selectionIndex,label);
            pathList.set(selectionIndex,command);
        }
    }
    public static void removeAction(){
        if (!isInserting){
            pathList.remove(selectionIndex);
            pathListDisplay.remove(selectionIndex);
        }
    }
    public static void runOpMode(Inferno.Alliance alliance, LinearOpMode opMode){
        initialize(opMode,new Inferno(),true,true);
        Inferno.motifDetected = false;
        Components.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        leftFront.resetEncoder();
        preloadShoot.reset();
        Inferno.alliance = alliance;
        if (alliance == Inferno.Alliance.BLUE) tangent = HeadingInterpolator.tangent.reverse(); else tangent = HeadingInterpolator.tangent;
        gamePhase = Inferno.GamePhase.AUTO;
        Pedro.createFollower(getPose("start"));
        initEncoderError = 0;
        executor.setWriteToTelemetry(()->{
                telemetry.addData("pos",turretYaw.get("turretYawTop").getCurrentPosition());
                telemetry.addData("Target Flywheel Velocity",targetFlywheelVelocity);
                telemetry.addData("Flywheel Velocity",0);
                telemetry.addLine("");
                telemetry.addLine("A: First Spike");
                telemetry.addLine("B: Second Spike");
                telemetry.addLine("X: Third Spike");
                telemetry.addLine("Y: Gate");
                telemetry.addLine("DPAD_LEFT: Preload Shoot");
                telemetry.addLine("BACK: Delete");
                telemetry.addLine("");
                for (int i=0;i<pathListDisplay.size();i++){
                    if (i==selectionIndex && isInserting){
                        telemetry.addLine("> [    ]");
                        telemetry.addLine(pathListDisplay.get(i));
                    } else if (i==selectionIndex){
                        telemetry.addLine("> ["+pathListDisplay.get(i)+"]");
                    } else telemetry.addLine(pathListDisplay.get(i));
                }
                if (selectionIndex==pathListDisplay.size()) telemetry.addLine("> [    ]");
                telemetry.addLine("");
                telemetry.addLine("Please, Speed, we need this.");
        });
        executor.setCommands(
                new RunResettingLoop(new PressCommand(
                        new IfThen(()->gamepad1.dpad_up,new InstantCommand(()->{
                                if (!isInserting){
                                    isInserting=true;
                                } else {
                                    if (selectionIndex>0) {
                                        selectionIndex -= 1;
                                        isInserting = false;
                                    }
                                }
                        })),
                        new IfThen(()->gamepad1.dpad_down,new InstantCommand(()->{
                                if (isInserting && selectionIndex!=pathList.size()) {
                                    isInserting = false;
                                } else {
                                    if (selectionIndex < pathList.size()) {
                                        selectionIndex += 1;
                                        isInserting = true;
                                    }
                                }
                        })),
                        new IfThen(()->gamepad1.a,new InstantCommand(()->addAction(firstSpike,"firstSpike"))),
                        new IfThen(()->gamepad1.b,new InstantCommand(()->addAction(secondSpike,"secondSpike"))),
                        new IfThen(()->gamepad1.x,new InstantCommand(()->addAction(thirdSpike,"thirdSpike"))),
                        new IfThen(()->gamepad1.y,new InstantCommand(()->addAction(gate,"gate"))),
                        new IfThen(()->gamepad1.dpad_left,new InstantCommand(()->addAction(preloadShoot,"preloadShoot"))),
                        new IfThen(()->gamepad1.back,new InstantCommand(OldClosePrimeSigmaConstants::removeAction))
                ))
        );
        setTargetPoint();
        initEncoderError = Math.toDegrees(atan2(targetPoint[1]-getPose("start").getY(),targetPoint[0]-getPose("start").getX())-getHeading("start")) - turretYaw.get("turretYawTop").getCurrentPosition();
        executor.runLoop(opMode::opModeInInit);
        Components.activateActuatorControl();
        executor.setWriteToTelemetry(()->{
            telemetry.addData("Yaw Target", turretYaw.get("turretYawTop").getTarget());
            telemetry.addData("Target Point", Arrays.asList(targetPoint));
            telemetry.addData("Pose Heading", follower.getHeading());
            telemetry.addData("Motif",Arrays.asList(motif));
            telemetry.addLine("");
            telemetry.addData("Target Flywheel Velocity", targetFlywheelVelocity);
            telemetry.addData("Flywheel Velocity", flywheel.get("flywheelLeft").getVelocity());
            telemetry.addData("Flywheel Error", targetFlywheelVelocity - flywheel.get("flywheelLeft").getVelocity());
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
        SequentialCommand mainPath = new SequentialCommand(pathList.toArray(new Command[0]));
        executor.setCommands(
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
                new RunResettingLoop(
                    new InstantCommand(
                            ()->{if (preloadShoot.isBusy()) targetFlywheelVelocity = Inferno.VelRegression.regressFormula(getPose("firstShoot").distanceFrom(new Pose(targetPoint[0],targetPoint[1])));
                                else targetFlywheelVelocity = Inferno.VelRegression.regressFormula(getPose("shoot").distanceFrom(new Pose(targetPoint[0],targetPoint[1])));}
                    )
                )
        );
        executor.runLoop(opMode::opModeIsActive);
    }
}
