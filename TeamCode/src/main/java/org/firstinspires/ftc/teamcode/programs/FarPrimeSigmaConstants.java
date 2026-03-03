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
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.leftFront;
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

public class FarPrimeSigmaConstants {
    public static final double INITIAL_WAIT = 0.01;
    public static final double SHOT_TIME = 1.2;
    public static final double slowDownT = 0.73;
    public static final double speedUpT = 0.13;
    public static final double stopIntakeT = 0.5;
    public static final double slowDownAmount = 1.0;
    public static Command shoot = new SequentialCommand(setState(Inferno.RobotState.SHOOTING), new SleepCommand(SHOT_TIME), setState(Inferno.RobotState.INTAKE_FRONT));
    public static final HashMap<String, Pose> poses = new HashMap<>();
    public static Pose mirrorPose(Pose input){
        return new Pose(144-input.getX(),input.getY(),mirrorHeading(input.getHeading()));
    }
    public static double mirrorHeading(double input){return 2*Math.PI - input;}
    static {
        poses.put("start",new Pose(60, 9.1, Math.toRadians(90)));
        poses.put("firstSpikeCtrl",new Pose(60.6,36.4,Math.toRadians(136)));
        poses.put("firstSpike",new Pose(10.7,35.1,Math.toRadians(180)));
        poses.put("firstShoot",new Pose(56.2, 21.4));
        poses.put("loadingZoneCtrl",new Pose(10.5, 35.8));
        poses.put("loadingZone",new Pose(9.4, 11.9));
        poses.put("secondShootCtrl",new Pose(31.2, 20.3));
        poses.put("shoot",new Pose(54.7, 17.6,Math.toRadians(180)));
    }
    public static Pose getPose(String input){if (alliance==Inferno.Alliance.BLUE) return poses.get(input); else return mirrorPose(Objects.requireNonNull(poses.get(input)));}
    public static double getHeading(String input){if (alliance==Inferno.Alliance.BLUE) return Objects.requireNonNull(poses.get(input)).getHeading(); else return mirrorHeading(Objects.requireNonNull(poses.get(input)).getHeading());}
    public static Command firstSpike = new SequentialCommand(
            new PedroCommand(
                    b->b.addPath(new BezierCurve(follower::getPose, getPose("firstSpikeCtrl"), getPose("firstSpike")))
                            .setTangentHeadingInterpolation()
                            .addParametricCallback(slowDownT,()->follower.setMaxPower(slowDownAmount))
                        .addPath(new BezierLine(follower::getPose, getPose("firstShoot")))
                            .setTangentHeadingInterpolation().setReversed()
                            .addParametricCallback(speedUpT,()->follower.setMaxPower(1.0))
                            .addParametricCallback(stopIntakeT,()->setState(Inferno.RobotState.STOPPED).run()),
                    true
            ), shoot
    );
    public static ArrayList<Command> pathList = new ArrayList<>();
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
        leftFront.resetEncoder();
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
                        new IfThen(()->gamepad1.back,new InstantCommand(ClosePrimeSigmaConstants::removeAction))
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
            telemetry.addData("Yaw Target",turretYaw.get("turretYawTop").getTarget());
            telemetry.addData("Yaw Desired",-(turretYaw.get("turretYawTop").getTarget()-180)+Math.toDegrees(follower.getHeading()));
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
