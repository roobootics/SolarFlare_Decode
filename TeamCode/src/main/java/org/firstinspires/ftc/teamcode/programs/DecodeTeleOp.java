package org.firstinspires.ftc.teamcode.programs;

import static org.firstinspires.ftc.teamcode.base.Commands.executor;
import static org.firstinspires.ftc.teamcode.base.Components.initialize;
import static org.firstinspires.ftc.teamcode.base.Components.timer;
import static org.firstinspires.ftc.teamcode.pedroPathing.Pedro.follower;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.TURRET_PITCH_OFFSET;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.TURRET_PITCH_RATIO;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.autoGateIntake;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.ballStorage;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.classifierBallCount;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.findMotif;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.motif;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.flywheel;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.gamePhase;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.hoodDesired;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.leftFront;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.leftRear;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.loopFSM;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.panic;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.physicsTime;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.rightFront;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.rightRear;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.robotState;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.setState;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.shotType;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.targetFlywheelVelocity;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.targetPoint;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.toggleShotType;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.transfer;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.turretOffsetFromAuto;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.turretPitch;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.turretYaw;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.useVelFeedforward;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.yawDesired;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import org.firstinspires.ftc.teamcode.base.Commands;
import org.firstinspires.ftc.teamcode.base.Commands.*;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


import org.firstinspires.ftc.teamcode.base.Components;
import org.firstinspires.ftc.teamcode.base.Components.*;
import org.firstinspires.ftc.teamcode.pedroPathing.Pedro;
import org.firstinspires.ftc.teamcode.robotconfigs.Inferno;
import org.firstinspires.ftc.teamcode.robotconfigs.Inferno.*;

import java.util.Arrays;
import java.util.Objects;

@TeleOp
@Config
public class DecodeTeleOp extends LinearOpMode {
    private boolean holdingPosition = false;
    private double lastTime = 0;
    private double previousBallCount = -1;
    private void breakFollowing(){
        holdingPosition = false;
        follower.breakFollowing();
        setMotorsToBrake();
    }
    private void setMotorsToBrake(){
        leftFront.getDevice().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.getDevice().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.getDevice().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.getDevice().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    boolean followerMade = false;
    @Override
    public void runOpMode(){
        gamePhase = GamePhase.TELEOP;
        useVelFeedforward = true;
        initialize(this,new Inferno(),false,true);
        Components.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        if (Objects.isNull(follower)) {Pedro.createFollower(new Pose(96,7.5,0)); followerMade = true; Inferno.motifDetected = false; turretOffsetFromAuto = 0; leftFront.resetEncoder();}
        else {Pedro.createFollower(follower.getPose());}
        executor.setCommands(
                new RunResettingLoop(new InstantCommand(()->{if (gamepad1.back && !followerMade) {follower.setPose(new Pose(96,7.5,0)); followerMade = true; Inferno.motifDetected = false;  turretOffsetFromAuto = 0; leftFront.resetEncoder();}})),
                new RunResettingLoop(new InstantCommand(()->{if (gamepad1.dpad_left) {Inferno.alliance = Alliance.BLUE;}})),
                new RunResettingLoop(new InstantCommand(()->{if (gamepad1.dpad_right) {Inferno.alliance = Alliance.RED;}}))
        );
        turretYaw.call(servo->servo.setOffset(turretOffsetFromAuto));
        executor.runLoop(this::opModeInInit);
        Components.activateActuatorControl();
        breakFollowing();
        executor.setCommands(
                findMotif,
                new RunResettingLoop(
                        new PressCommand(
                                new IfThen(()->gamepad1.right_bumper, setState(RobotState.INTAKE_FRONT)),
                                new IfThen(()->gamepad1.left_bumper, setState(RobotState.INTAKE_BACK)),
                                new IfThen(()->gamepad1.right_trigger>0.5, new InstantCommand(()->{transfer.reset(); setState(RobotState.SHOOTING).run();})),
                                new IfThen(()->gamepad1.left_trigger>0.3, setState(RobotState.STOPPED)),
                                new IfThen(()->gamepad2.b, setState(RobotState.INTAKE_FRONT_AND_SHOOT)),
                                new IfThen(()->gamepad2.x, setState(RobotState.INTAKE_BACK_AND_SHOOT)),
                                new IfThen(()->gamepad1.a, autoGateIntake),
                                new IfThen(()->gamepad1.y, setState(RobotState.EXPEL))
                        ),
                        new PressCommand(
                                new IfThen(()->gamepad2.y,toggleShotType()),
                                new IfThen(()->gamepad2.x,new InstantCommand(()->classifierBallCount=0)),
                                new IfThen(()->gamepad2.a,new InstantCommand(()->{if (classifierBallCount<9) {classifierBallCount+=1;}})),
                                new IfThen(()->gamepad2.b,new InstantCommand(()->{if (classifierBallCount>0){classifierBallCount-=1;}})),
                                new IfThen(()->gamepad2.left_bumper,new AprilTagRelocalize())
                        ),
                        Commands.triggeredToggleCommand(()->gamepad2.left_stick_button,new ContinuousCommand(()->{}),panic),
                        turretYaw.command(servo->servo.triggeredDynamicOffsetCommand(()->gamepad2.left_trigger>0.2,()->gamepad2.right_trigger>0.2,0.5)),
                        new PressCommand(
                                new IfThen(()->robotState==RobotState.SHOOTING && !(Math.sqrt(gamepad1.left_stick_x*gamepad1.left_stick_x + gamepad1.left_stick_y*gamepad1.left_stick_y)>0.1 || Math.abs(gamepad1.right_stick_x)>0.1),
                                        new SequentialCommand(
                                                new InstantCommand(()->{this.breakFollowing(); holdingPosition = true;}),
                                                new SleepCommand(0.2),
                                                new InstantCommand(()->{follower.holdPoint(follower.getPose()); setMotorsToBrake();})
                                        )
                                ),
                                new IfThen(()->!(robotState==RobotState.SHOOTING && !(Math.sqrt(gamepad1.left_stick_x*gamepad1.left_stick_x + gamepad1.left_stick_y*gamepad1.left_stick_y)>0.1 || Math.abs(gamepad1.right_stick_x)>0.1)), new InstantCommand(()->{if (!follower.isBusy()){this.breakFollowing();}}))
                        ),
                        new InstantCommand(this::setMotorsToBrake),
                        new ConditionalCommand(
                                new IfThen(
                                        ()->!(follower.isBusy() || holdingPosition),
                                        new ParallelCommand(
                                                new RobotCentricMecanumCommand(
                                                        new BotMotor[]{leftFront,leftRear,rightFront,rightRear},
                                                        ()-> (double) gamepad1.left_stick_x, ()-> (double) gamepad1.left_stick_y, ()-> (double) gamepad1.right_stick_x
                                                ),
                                                Pedro.updatePoseCommand()
                                        )
                                ),
                                new IfThen(
                                        ()->(follower.isBusy() || holdingPosition),
                                        Pedro.updateCommand()
                                )
                        )
                        /*new InstantCommand(()->{
                            if (315-turretYaw.get("turretYawFront").getTarget()<15 && !gamepad1.isRumbling()){gamepad1.rumble(1000000000);}
                            else if (turretYaw.get("turretYawFront").getTarget()-0<15 && !gamepad1.isRumbling()){gamepad1.rumble(1000000000);}
                            else if (turretYaw.get("turretYawFront").getTarget()-0>15 && 315-turretYaw.get("turretYawFront").getTarget()>15 && gamepad1.isRumbling()){gamepad1.stopRumble();}
                            long count = Arrays.stream(ballStorage).filter(Objects::isNull).count();
                            if (count==3 && count!=previousBallCount){
                                gamepad1.setLedColor(0,0,0,1000000000);
                            } else if (count==2 && count!=previousBallCount){
                                gamepad1.setLedColor(1,0,0,1000000000);
                            } else if (count==1 && count!=previousBallCount){
                                gamepad1.setLedColor(0,0,1,1000000000);
                            } else if (count==0 && count!=previousBallCount){
                                gamepad1.setLedColor(1,1,1,1000000000);
                            }
                            previousBallCount = count;
                        })*/
                ),
                loopFSM

        );
        executor.setWriteToTelemetry(()->{
            Components.telemetry.addData("Classifier Count",classifierBallCount);
            Components.telemetry.addLine("");
            Components.telemetry.addData("Shot Type",shotType);
            Components.telemetry.addLine("");
            Components.telemetry.addData("Motif",Arrays.asList(motif));
            Components.telemetry.addLine("");
            Components.telemetry.addData("Ball Storage", Arrays.asList(ballStorage));
            Components.telemetry.addLine("");
            Components.telemetry.addData("Robot State",robotState);
            Components.telemetry.addLine("");
            Components.telemetry.addData("Target Flywheel Velocity",targetFlywheelVelocity);
            Components.telemetry.addData("Flywheel Velocity",flywheel.get("flywheelLeft").getVelocity());
            Components.telemetry.addData("Flywheel Error", targetFlywheelVelocity - flywheel.get("flywheelLeft").getVelocity());
            Components.telemetry.addLine("");
            Components.telemetry.addData("Hood Angle",(turretPitch.get("turretPitchLeft").getTarget()-TURRET_PITCH_OFFSET)/TURRET_PITCH_RATIO);
            Components.telemetry.addData("Hood Desired",hoodDesired);
            Components.telemetry.addLine("");
            Components.telemetry.addData("Yaw Pos",turretYaw.get("turretYawTop").getCurrentPosition());
            Components.telemetry.addData("Yaw Target",turretYaw.get("turretYawTop").getTarget());
            Components.telemetry.addData("Yaw Angle",yawDesired);
            Components.telemetry.addData("Yaw Error", turretYaw.get("turretYawTop").getTarget() - turretYaw.get("turretYawTop").getCurrentPosition());
            Components.telemetry.addLine("");
            Pose pos = follower.getPose();
            Components.telemetry.addData("Distance",Math.sqrt((targetPoint[0]-pos.getX())*(targetPoint[0]-pos.getX()) + (targetPoint[1]-pos.getY())*(targetPoint[1]-pos.getY())));
            Components.telemetry.addData("PoseX",follower.getPose().getX());
            Components.telemetry.addData("PoseY",follower.getPose().getY());
            Components.telemetry.addData("PoseHeading",Math.toDegrees(follower.getHeading()));
            Components.telemetry.addData("VelX",follower.getVelocity().getXComponent());
            Components.telemetry.addData("VelY",follower.getVelocity().getYComponent());
            Components.telemetry.addData("Angular Vel",Math.toDegrees(follower.getAngularVelocity()));
            Components.telemetry.addLine("");
            Components.telemetry.addData("Loop Time",timer.time()-lastTime);
            Components.telemetry.addData("Physics Time", physicsTime);
            lastTime = timer.time();
        });
        executor.runLoop(this::opModeIsActive);
    }
}
