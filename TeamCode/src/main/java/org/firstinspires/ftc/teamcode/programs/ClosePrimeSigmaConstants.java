package org.firstinspires.ftc.teamcode.programs;

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

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.PathBuilder;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.base.Commands.Command;
import org.firstinspires.ftc.teamcode.base.Commands.ConditionalCommand;
import org.firstinspires.ftc.teamcode.base.Commands.IfThen;
import org.firstinspires.ftc.teamcode.base.Commands.InstantCommand;
import org.firstinspires.ftc.teamcode.base.Commands.ParallelCommand;
import org.firstinspires.ftc.teamcode.base.Commands.PressCommand;
import org.firstinspires.ftc.teamcode.base.Commands.RunResettingLoop;
import org.firstinspires.ftc.teamcode.base.Commands.SequentialCommand;
import org.firstinspires.ftc.teamcode.base.Commands.SleepCommand;
import org.firstinspires.ftc.teamcode.base.Commands.SleepUntilTrue;
import org.firstinspires.ftc.teamcode.base.Components;
import org.firstinspires.ftc.teamcode.pedroPathing.Pedro;
import org.firstinspires.ftc.teamcode.pedroPathing.Pedro.PedroCommand;
import org.firstinspires.ftc.teamcode.robotconfigs.Inferno;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.Objects;

public class ClosePrimeSigmaConstants {
    public static final double INITIAL_WAIT = 0.01;
    public static final double PRE_SHOT_TIME = 0.3;
    public static final double SHOT_TIME = 1;
    public static final double slowDownT = 0.73;
    public static final double speedUpT = 0.05;
    public static final double stopIntakeT = 0.17;
    public static final double slowDownAmount = 0.67;
    public static final double gateIntakeTimeout = 1;
    public static final double secondShootSlowT = 0.75;
    public static final double fourthShootSlowT = 0.75;
    public static final double shootSlowT = 0.8;
    public static final double shootSlowAmount = 0.7;
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
    public static Pose mirrorPose(Pose input){
        return new Pose(144-input.getX(),input.getY(),mirrorHeading(input.getHeading()));
    }
    public static double mirrorHeading(double input){return Math.PI - input;}
    static {
        poses.put("start",new Pose(19.68, 121.72, Math.toRadians(144)));
        poses.put("shoot",new Pose(57.85,77.42,Math.toRadians(180)));
        poses.put("secondSpikeCtrl",new Pose(41.07, 58.06));
        poses.put("secondSpike",new Pose(14.829, 58.805));
        poses.put("gateOpen",new Pose(14.97, 59.02,Math.toRadians(180)));
        poses.put("gateIntake",new Pose(14.97, 56.16,Math.toRadians(130)));
        poses.put("firstSpike",new Pose(22.773, 79.829,Math.toRadians(180)));
        poses.put("thirdSpikeCtrl",new Pose(80.067, 27.483));
        poses.put("thirdSpike",new Pose(13.504, 36.131,Math.toRadians(180)));
        poses.put("thirdShootCtrl",new Pose(44.169, 52.575));
        poses.put("park",new Pose(45,79,Math.toRadians(180)));
    }
    public static Pose getPose(String input){if (alliance==Inferno.Alliance.BLUE) return poses.get(input); else return mirrorPose(Objects.requireNonNull(poses.get(input)));}
    public static double getHeading(String input){if (alliance==Inferno.Alliance.BLUE) return Objects.requireNonNull(poses.get(input)).getHeading(); else return mirrorHeading(Objects.requireNonNull(poses.get(input)).getHeading());}
    public static Command preloadShoot = new SequentialCommand(new SleepCommand(INITIAL_WAIT),
            new PedroCommand(b->
                    b.addPath(new BezierLine(follower::getPose,getPose("shoot")))
                    .setHeadingInterpolation(HeadingInterpolator.linear(getHeading("start"),getHeading("shoot"),0.5)),
    true),
    shoot);
    public static Command secondSpike = new SequentialCommand(new PedroCommand(
            (PathBuilder b)->b.addPath(
                    new BezierCurve(
                            follower::getPose,
                            getPose("secondSpikeCtrl"),
                            getPose("secondSpike")
                    ))
                .setConstantHeadingInterpolation(getHeading("shoot"))
                .addParametricCallback(slowDownT,()->follower.setMaxPower(slowDownAmount))
            .addPath(
                    new BezierLine(
                            follower::getPose,
                            getPose("shoot")
                    )
            ).setConstantHeadingInterpolation(getHeading("shoot"))
                .addParametricCallback(speedUpT,()->follower.setMaxPower(1.0))
                .addParametricCallback(stopIntakeT,()->setState(Inferno.RobotState.STOPPED).run())
                .addParametricCallback(secondShootSlowT,()->follower.setMaxPower(shootSlowAmount))
                .addParametricCallback(0.93,()->follower.setMaxPower(1.0)), true), shoot);
    public static Command gate = new SequentialCommand(setState(null), new PedroCommand(
            (PathBuilder b)->b.addPath(
                    new BezierLine(
                            follower::getPose,
                            getPose("gateOpen")
                    )
                    ).setConstantHeadingInterpolation(getHeading("gateOpen"))
                    .addPath(
                            new BezierLine(
                                    follower::getPose,
                                    getPose("gateIntake")
                            )
                    )
                    .setHeadingInterpolation(HeadingInterpolator.linear(getHeading("gateOpen"),getHeading("gateIntake"),0.5))
                    .addParametricCallback(0.15,setState(Inferno.RobotState.INTAKE_FRONT)::run),
            true),
            new Inferno.CheckFull(gateIntakeTimeout),
            new PedroCommand(
                    (PathBuilder b)->b.addPath(
                                    new BezierLine(
                                            follower::getPose,
                                            getPose("shoot")
                                    )
                            ).setHeadingInterpolation(HeadingInterpolator.linear(getHeading("gateIntake"), getHeading("shoot"),0.6))
                            .addParametricCallback(stopIntakeT,()->setState(Inferno.RobotState.STOPPED).run())
                            .addParametricCallback(shootSlowT,()->follower.setMaxPower(shootSlowAmount))
                            .addParametricCallback(0.93,()->follower.setMaxPower(1.0)),
                    true), shoot);
    public static Command firstSpike = new SequentialCommand(
            new PedroCommand((PathBuilder b)->b
                    .addPath(
                            new BezierLine(
                                    follower::getPose,
                                    getPose("firstSpike")
                            )
                    ).setConstantHeadingInterpolation(getHeading("firstSpike"))
                    .addParametricCallback(slowDownT,()->follower.setMaxPower(slowDownAmount))
                    .addPath(
                            new BezierLine(
                                    follower::getPose,
                                    getPose("shoot")
                            )
                    ).setConstantHeadingInterpolation(getHeading("shoot"))
                    .addParametricCallback(speedUpT,()->follower.setMaxPower(1.0))
                    .addParametricCallback(stopIntakeT,()->setState(Inferno.RobotState.STOPPED).run())
                    .addParametricCallback(fourthShootSlowT,()->follower.setMaxPower(shootSlowAmount))
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
    public static Command park = new SequentialCommand(setState(null),
            new PedroCommand(b->
                    b.addPath(new BezierLine(follower::getPose,getPose("park")))
                            .setLinearHeadingInterpolation(getHeading("shoot"),getHeading("park")),
    true));
    public static ArrayList<Command> pathList = new ArrayList<>(Arrays.asList(
            preloadShoot,
            secondSpike,
            gate,
            gate,
            gate,
            firstSpike
    ));
    public static ArrayList<String> pathListDisplay = new ArrayList<>(Arrays.asList(
            "preloadShoot",
            "secondSpike",
            "gate",
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
        turretOffsetFromAuto = 0;
        Components.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        preloadShoot.reset();
        Inferno.alliance = alliance;
        gamePhase = Inferno.GamePhase.AUTO;
        Pedro.createFollower(getPose("start"));
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
                turretYaw.command((Components.BotServo servo)->servo.triggeredDynamicOffsetCommand(()->gamepad1.left_trigger>0.2,()->gamepad1.right_trigger>0.2,0.05)),
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
                        new IfThen(()->gamepad1.back,new InstantCommand(ClosePrimeSigmaConstants::removeAction))
                ))
        );
        setTargetPoint();
        executor.runLoop(opMode::opModeInInit);
        turretOffsetFromAuto = turretYaw.get("turretYawTop").getOffset();
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
                            ()->targetFlywheelVelocity = Inferno.VelRegression.regressFormula(getPose("shoot").distanceFrom(new Pose(targetPoint[0],targetPoint[1])))
                    )
                )
        );
        executor.runLoop(opMode::opModeIsActive);
    }
}
