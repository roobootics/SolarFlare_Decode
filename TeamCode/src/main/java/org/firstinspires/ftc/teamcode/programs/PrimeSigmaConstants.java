package org.firstinspires.ftc.teamcode.programs;

import static org.firstinspires.ftc.teamcode.base.Commands.executor;
import static org.firstinspires.ftc.teamcode.base.Components.gamepad1;
import static org.firstinspires.ftc.teamcode.base.Components.gamepad2;
import static org.firstinspires.ftc.teamcode.base.Components.initialize;
import static org.firstinspires.ftc.teamcode.base.Components.telemetry;
import static org.firstinspires.ftc.teamcode.pedroPathing.Pedro.follower;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.alliance;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.ballStorage;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.classifierBallCount;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.clearIntegralAtPeak;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.currentBallPath;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.flywheel;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.gamePhase;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.loopFSM;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.robotState;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.setShooter;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.setState;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.shotType;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.targetFlywheelVelocity;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.turretOffsetFromAuto;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.turretYaw;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
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

public class PrimeSigmaConstants {
    public static final double INITIAL_WAIT = 0.01;
    public static final double SHOT_TIME = 1.2;
    public static final double slowDownT = 0.73;
    public static final double speedUpT = 0.13;
    public static final double stopIntakeT = 0.5;
    public static final double slowDownAmount = 0.67;
    public static final double gateWait = 0.4;
    public static final double gateIntakeTimeout = 1;
    public static final double fourthShootSlowT = 0.73;
    public static final double fourthShootSlowAmount = 0.76;
    public static Command shoot = new SequentialCommand(setState(Inferno.RobotState.SHOOTING), new SleepCommand(SHOT_TIME), intake());
    public static final HashMap<String, Pose> poses = new HashMap<>();
    public static Pose mirrorPose(Pose input){
        return new Pose(144-input.getX(),input.getY(),mirrorHeading(input.getHeading()));
    }
    public static double mirrorHeading(double input){return 2*Math.PI - input;}
    static {
        poses.put("start",new Pose(20, 122.62, Math.toRadians(143.5)));
        poses.put("firstShoot",new Pose(53.312,90.398,Math.toRadians(136)));
        poses.put("shoot",new Pose(53.312,90.398,Math.toRadians(0)));
        poses.put("secondSpikeCtrl",new Pose(88.046, 59.413));
        poses.put("secondSpike",new Pose(16.329, 58.805));
        poses.put("secondShootCtrl",new Pose(54.282, 64.593));
        poses.put("gateOpenCtrl",new Pose(39.404, 37.087));
        poses.put("gateOpen",new Pose(17.669, 60.486,Math.toRadians(-25)));
        poses.put("gateIntake",new Pose(16.201, 52.334,Math.toRadians(-50)));
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
    public static Command preloadShoot = new SequentialCommand(new SleepCommand(INITIAL_WAIT), new PedroLinearCommand(getPose("firstShoot"),true), shoot);
    public static Command secondSpike = new SequentialCommand(new PedroCommand(
            (PathBuilder b)->b.addPath(
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
                            getPose("shoot")
                    )
            ).setConstantHeadingInterpolation(getHeading("shoot"))
            .addParametricCallback(speedUpT,()->follower.setMaxPower(1.0))
            .addParametricCallback(stopIntakeT,()->setState(Inferno.RobotState.STOPPED).run()), true), shoot);
    public static Command gate = new SequentialCommand(new PedroCommand(
            (PathBuilder b)->b.addPath(
                    new BezierCurve(
                            follower::getPose,
                            getPose("gateOpenCtrl"),
                            getPose("gateOpen")
                    )
            ).setLinearHeadingInterpolation(follower.getHeading(), getHeading("gateOpen")),
            true),
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
                    true),
            new Inferno.CheckFull(gateIntakeTimeout),
            new PedroCommand(
                    (PathBuilder b)->b.addPath(
                                    new BezierCurve(
                                            follower::getPose,
                                            getPose("thirdShootCtrl"),
                                            getPose("shoot")
                                    )
                            ).setLinearHeadingInterpolation(getHeading("gateIntake"), getHeading("shoot"))
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
    public static Command park = new SequentialCommand(setState(Inferno.RobotState.STOPPED), new PedroLinearCommand(getPose("park"),true));
    public static ArrayList<Command> pathList = new ArrayList<>(Arrays.asList(
            preloadShoot,
            secondSpike,
            gate,
            firstSpike,
            thirdSpike,
            park
    ));
    public static ArrayList<String> pathListDisplay = new ArrayList<>(Arrays.asList(
            "preloadShoot",
            "secondSpike",
            "gate",
            "firstSpike",
            "thirdSpike",
            "park"
    ));
    public static int selectionIndex = 0;
    public static boolean isInserting = true;
    public static void addAction(Command command, String label){
        if (isInserting) {
            pathListDisplay.add(selectionIndex,label);
            pathList.add(selectionIndex,command);
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
        Inferno.alliance = alliance;
        turretOffsetFromAuto = 0;
        gamePhase = Inferno.GamePhase.AUTO;
        Pedro.createFollower(getPose("start"));
        turretYaw.call((Components.CRBotServo servo)->servo.switchControl("PID"));
        executor.setWriteToTelemetry(()->{
                telemetry.addData("offset",turretYaw.get("turretYawFront").getOffset());
                telemetry.addLine("");
                for (int i=0;i<pathListDisplay.size();i++){
                    if (i==selectionIndex && isInserting){
                        telemetry.addLine("> [    ]");
                        telemetry.addLine(pathListDisplay.get(i));
                    } else if (i==selectionIndex){
                        telemetry.addLine("> ["+pathListDisplay.get(i)+"]");
                    } else telemetry.addLine(pathListDisplay.get(i));
                }
                telemetry.addLine("");
                telemetry.addLine("Please, Speed, we need this.");
        });
        executor.setCommands(
                new InstantCommand(setShooter::run),
                turretYaw.command((Components.CRBotServo servo)->servo.triggeredDynamicOffsetCommand(()->gamepad1.right_trigger>0.4,()->gamepad2.left_trigger>0.4,0.05)),
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
                        new IfThen(()->gamepad1.dpad_right,new InstantCommand(()->addAction(park,"park"))),
                        new IfThen(()->gamepad1.back,new InstantCommand(PrimeSigmaConstants::removeAction))
                ))
        );
        executor.runLoop(opMode::opModeInInit);
        turretOffsetFromAuto = turretYaw.get("turretYawFront").getOffset();
        Components.activateActuatorControl();
        executor.setWriteToTelemetry(()->{
            telemetry.addData("is busy",follower.isBusy());
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
            telemetry.addData("Target Flywheel Velocity",targetFlywheelVelocity);
            telemetry.addData("Flywheel Velocity",flywheel.get("flywheelLeft").getVelocity());
            telemetry.addLine("");
            telemetry.addData("Yaw Target",turretYaw.get("turretYawFront").getTarget());
            telemetry.addData("Yaw Desired",-(turretYaw.get("turretYawFront").getTarget()-180)+Math.toDegrees(follower.getHeading()));
            telemetry.addLine("");
            telemetry.addData("PoseX",follower.getPose().getX());
            telemetry.addData("PoseY",follower.getPose().getY());
            telemetry.addData("PoseHeading",Math.toDegrees(follower.getHeading()));
            telemetry.addLine("");
            telemetry.addData("Flywheel Left Power",flywheel.get("flywheelLeft").getPower());
            telemetry.addData("Flywheel Right Power",flywheel.get("flywheelRight").getPower());
        });
        executor.setCommands(
                new SequentialCommand(pathList.toArray(new Command[0])),
                clearIntegralAtPeak,
                Pedro.updateCommand(),
                loopFSM
        );
        executor.runLoop(opMode::opModeIsActive);
    }
}
