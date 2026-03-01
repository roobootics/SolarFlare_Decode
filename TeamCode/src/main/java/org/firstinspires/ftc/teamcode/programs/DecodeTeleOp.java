package org.firstinspires.ftc.teamcode.programs;

import static org.firstinspires.ftc.teamcode.base.Commands.executor;
import static org.firstinspires.ftc.teamcode.base.Components.initialize;
import static org.firstinspires.ftc.teamcode.base.Components.timer;
import static org.firstinspires.ftc.teamcode.pedroPathing.Pedro.follower;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.TURRET_PITCH_OFFSET;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.TURRET_PITCH_RATIO;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.aprilTagRelocalize;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.autoGateIntake;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.ballStorage;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.classifierBallCount;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.clearIntegralAtPeak;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.currentBallPath;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.flywheel;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.gamePhase;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.hoodDesired;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.leftFront;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.leftRear;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.loopFSM;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.rightFront;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.rightRear;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.robotState;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.setState;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.shotType;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.targetFlywheelVelocity;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.toggleShotType;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.transfer;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.turretOffsetFromAuto;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.turretPitch;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.turretYaw;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.yawDesired;

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
    private void stopDrivetrain(){leftFront.setPower(0); leftRear.setPower(0); rightFront.setPower(0); rightRear.setPower(0);}
    boolean followerMade = false;
    @Override
    public void runOpMode(){
        gamePhase = GamePhase.TELEOP;
        initialize(this,new Inferno(),false,true);
        if (Objects.isNull(follower)) {Pedro.createFollower(new Pose(96,7.5,0)); leftFront.resetEncoder(); followerMade = true;}
        else {Pedro.createFollower(follower.getPose());}
        executor.setCommands(
                new RunResettingLoop(new InstantCommand(()->{if (gamepad1.back && !followerMade) {Pedro.createFollower(new Pose(96,7.5,0)); leftFront.resetEncoder(); followerMade = true;}})),
                new RunResettingLoop(new InstantCommand(()->{if (gamepad1.dpad_left) {Inferno.alliance = Alliance.BLUE;}})),
                new RunResettingLoop(new InstantCommand(()->{if (gamepad1.dpad_right) {Inferno.alliance = Alliance.RED;}}))
        );
        executor.runLoop(this::opModeInInit);
        turretYaw.call((CRBotServo servo)->servo.setOffset(turretOffsetFromAuto));
        Components.activateActuatorControl();
        breakFollowing();
        executor.setCommands(
                new RunResettingLoop(
                        new PressCommand(
                                new IfThen(()->gamepad1.right_bumper, setState(RobotState.INTAKE_FRONT)),
                                new IfThen(()->gamepad1.left_bumper, setState(RobotState.INTAKE_BACK)),
                                new IfThen(()->gamepad1.right_trigger>0.8, new InstantCommand(()->{transfer.reset(); setState(RobotState.SHOOTING).run();})),
                                new IfThen(()->gamepad1.y, setState(RobotState.STOPPED)),
                                new IfThen(()->gamepad1.a, autoGateIntake)
                        ),
                        new PressCommand(
                                new IfThen(()->gamepad2.y,toggleShotType()),
                                new IfThen(()->gamepad2.x,new InstantCommand(()->classifierBallCount=0)),
                                new IfThen(()->gamepad2.a,new InstantCommand(()->{if (classifierBallCount<9) {classifierBallCount+=1;}})),
                                new IfThen(()->gamepad2.b,new InstantCommand(()->{if (classifierBallCount>0){classifierBallCount-=1;}})),
                                new IfThen(()->gamepad2.dpad_up, setState(RobotState.INTAKE_FRONT_AND_SHOOT)),
                                new IfThen(()->gamepad2.dpad_down, setState(RobotState.INTAKE_BACK_AND_SHOOT)),
                                new IfThen(()->gamepad2.back,setState(RobotState.EXPEL)),
                                new IfThen(()->gamepad2.left_bumper,new ParallelCommand(aprilTagRelocalize,new SequentialCommand(new SleepCommand(0.5),new InstantCommand(aprilTagRelocalize::stop))))
                        ),
                        turretYaw.command((CRBotServo servo)->servo.triggeredDynamicOffsetCommand(()->gamepad2.right_trigger>0.4,()->gamepad2.left_trigger>0.4,0.5)),
                        new PressCommand(
                                new IfThen(()->robotState==RobotState.SHOOTING && !(Math.sqrt(gamepad1.left_stick_x*gamepad1.left_stick_x + gamepad1.left_stick_y*gamepad1.left_stick_y)>0.35 || Math.abs(gamepad1.right_stick_x)>0.35),
                                        new SequentialCommand(
                                                new InstantCommand(()->{this.stopDrivetrain(); holdingPosition = true;}),
                                                new SleepCommand(0.15),
                                                new InstantCommand(()->{follower.holdPoint(follower.getPose()); setMotorsToBrake();})
                                        )
                                ),
                                new IfThen(()->robotState!=RobotState.SHOOTING,new InstantCommand(()->{if (!follower.isBusy()){this.breakFollowing(); this.stopDrivetrain();}}))
                        ),
                        new ConditionalCommand(
                                new IfThen(
                                        ()->!(follower.isBusy() || holdingPosition),
                                        new ParallelCommand(
                                                new RobotCentricMecanumCommand(
                                                        new BotMotor[]{leftFront,leftRear,rightFront,rightRear},
                                                        ()-> (double) gamepad1.left_stick_x, ()-> (double) gamepad1.left_stick_y, ()-> (double) gamepad1.right_stick_x,
                                                        ()->{if (gamepad1.left_trigger > 0.3) return 0.75; else return 1.0;}
                                                ),
                                                Pedro.updatePoseCommand()
                                        )
                                ),
                                new IfThen(
                                        ()->(follower.isBusy() || holdingPosition),
                                        Pedro.updateCommand()
                                )
                        ),
                        Commands.triggeredDynamicCommand(()->gamepad1.dpad_right,()->gamepad1.dpad_left,new InstantCommand(()->targetFlywheelVelocity+=2),new InstantCommand(()->targetFlywheelVelocity-=2)),
                        loopFSM
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
                clearIntegralAtPeak
        );
        executor.setWriteToTelemetry(()->{
            telemetry.addData("Target Flywheel",targetFlywheelVelocity);
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
            telemetry.addData("Hood Angle",(turretPitch.get("turretPitchLeft").getTarget()-TURRET_PITCH_OFFSET)/TURRET_PITCH_RATIO);
            telemetry.addData("Hood Desired",hoodDesired);
            telemetry.addLine("");
            telemetry.addData("Yaw Pos",turretYaw.get("turretYawTop").getCurrentPosition());
            telemetry.addData("Yaw Target",turretYaw.get("turretYawTop").getTarget());
            telemetry.addData("Yaw Angle",yawDesired);
            telemetry.addData("Yaw Error", turretYaw.get("turretYawTop").getTarget() - turretYaw.get("turretYawTop").getCurrentPosition());
            telemetry.addLine("");
            telemetry.addData("PoseX",follower.getPose().getX());
            telemetry.addData("PoseY",follower.getPose().getY());
            telemetry.addData("PoseHeading",Math.toDegrees(follower.getHeading()));
            telemetry.addData("VelX",follower.getVelocity().getXComponent());
            telemetry.addData("VelY",follower.getVelocity().getYComponent());
            telemetry.addLine("");
            telemetry.addData("Flywheel Left Power",flywheel.get("flywheelLeft").getPower());
            telemetry.addData("Flywheel Right Power",flywheel.get("flywheelRight").getPower());
            telemetry.addData("Loop Time",timer.time()-lastTime);
            lastTime = timer.time();
        });
        executor.runLoop(this::opModeIsActive);
    }
}