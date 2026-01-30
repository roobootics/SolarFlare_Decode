package org.firstinspires.ftc.teamcode.programs;

import static org.firstinspires.ftc.teamcode.base.Commands.executor;
import static org.firstinspires.ftc.teamcode.base.Components.initialize;
import static org.firstinspires.ftc.teamcode.base.Components.initializeConfig;
import static org.firstinspires.ftc.teamcode.base.Components.telemetryAddData;
import static org.firstinspires.ftc.teamcode.base.Components.telemetryAddLine;
import static org.firstinspires.ftc.teamcode.base.Components.timer;
import static org.firstinspires.ftc.teamcode.pedroPathing.Pedro.follower;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.aprilTagRelocalize;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.backIntakeGate;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.ballStorage;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.classifierBallCount;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.clearIntegralAtPeak;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.colorSensorReads;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.currentBallPath;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.flywheel;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.frontIntakeGate;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.gamePhase;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.getTargetPoint;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.leftFront;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.leftRear;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.loopFSM;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.motifShootAll;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.rightFront;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.rightRear;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.robotState;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.setState;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.shotType;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.targetFlywheelVelocity;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.toggleShotType;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.turretOffsetFromAuto;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.turretYaw;

import org.firstinspires.ftc.teamcode.base.Commands.*;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.control.PIDFCoefficients;
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
    private double targetVelocity = 1800;
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
    @Override
    public void runOpMode(){
        initialize(hardwareMap,telemetry);
        gamePhase = GamePhase.TELEOP;
        initializeConfig(new Inferno(),true);
        executor.setCommands(
                new RunResettingLoop(new InstantCommand(()->{if (gamepad1.dpad_left) {Inferno.alliance = Alliance.BLUE;}})),
                new RunResettingLoop(new InstantCommand(()->{if (gamepad1.dpad_right) {Inferno.alliance = Alliance.RED;}}))
        );
        executor.runLoop(this::opModeInInit);
        turretYaw.call((BotServo servo)->servo.setOffset(turretOffsetFromAuto));
        Components.activateActuatorControl();
        breakFollowing();
        follower.setTranslationalPIDFCoefficients(new PIDFCoefficients(0.4, 0.0007, 0, 0));
        follower.setHeadingPIDFCoefficients(new PIDFCoefficients(1.8,0.001,0,0));
        executor.setCommands(
                new RunResettingLoop(
                        new ConditionalCommand(
                                new IfThen(()->gamepad1.right_bumper, setState(RobotState.INTAKE_FRONT)),
                                new IfThen(()->gamepad1.left_bumper, setState(RobotState.INTAKE_BACK)),
                                new IfThen(()->gamepad1.right_trigger>0.8, setState(RobotState.SHOOTING)),
                                new IfThen(()->gamepad1.y, setState(RobotState.STOPPED))
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
                        turretYaw.command((BotServo servo)->servo.triggeredDynamicOffsetCommand(()->gamepad2.right_trigger>0.4,()->gamepad2.left_trigger>0.4,3)),
                        loopFSM,
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
                        new PressCommand(
                                new IfThen(()->robotState==RobotState.SHOOTING,
                                        new SequentialCommand(
                                            new InstantCommand(()->{this.stopDrivetrain(); holdingPosition = true; follower.holdPoint(follower.getPose()); setMotorsToBrake();}),
                                            new SleepCommand(1),
                                            new InstantCommand(()->{this.stopDrivetrain(); breakFollowing(); frontIntakeGate.instantSetTargetCommand("closed"); backIntakeGate.instantSetTargetCommand("closed");})
                                        )
                                ),
                                new IfThen(()->robotState!=RobotState.SHOOTING,new InstantCommand(()->{this.breakFollowing(); this.stopDrivetrain();}))
                        ),
                        //Commands.triggeredDynamicCommand(()->gamepad1.dpad_up,()->gamepad1.dpad_down,new InstantCommand(()->targetVelocity+=2),new InstantCommand(()->targetVelocity-=2)),
                        new InstantCommand(()->{
                            if (315-turretYaw.get("turretYawFront").getTarget()<15 && !gamepad1.isRumbling()){gamepad1.rumble(1000000000);}
                            else if (turretYaw.get("turretYawFront").getTarget()-0<15 && !gamepad1.isRumbling()){gamepad1.rumble(1000000000);}
                            else if (turretYaw.get("turretYawFront").getTarget()-0>15 && 315-turretYaw.get("turretYawFront").getTarget()>15 && gamepad1.isRumbling()){gamepad1.stopRumble();}
                            long count = Arrays.stream(ballStorage).filter(Objects::isNull).count();
                            if (count==3 && count!=previousBallCount){
                                gamepad1.setLedColor(0,0,0,1000);
                            } else if (count==2 && count!=previousBallCount){
                                gamepad1.setLedColor(1,0,0,1000);
                            } else if (count==1 && count!=previousBallCount){
                                gamepad1.setLedColor(0,0,1,1000);
                            } else if (count==0 && count!=previousBallCount){
                                gamepad1.setLedColor(1,1,1,1000);
                            }
                            previousBallCount = count;
                            //targetFlywheelVelocity = targetVelocity;
                        })
                ),
                clearIntegralAtPeak
        );
        executor.setWriteToTelemetry(()->{
            telemetryAddLine("");
            telemetryAddData("Ball Storage:", Arrays.asList(ballStorage));
            telemetryAddData("sensor1",Arrays.asList(colorSensorReads.get(0).get()));
            telemetryAddData("sensor2",Arrays.asList(colorSensorReads.get(1).get()));
            telemetryAddData("sensor3",Arrays.asList(colorSensorReads.get(2).get()));
            telemetryAddLine("");
            telemetryAddData("Robot State:",robotState);
            telemetryAddLine("");
            telemetryAddData("Shot Type:",shotType);
            telemetryAddLine("");
            telemetryAddData("Classifier Count:",classifierBallCount);
            telemetryAddData("Current Shot Height:",currentBallPath);
            telemetryAddData("Shoot All Motif:",motifShootAll);
            telemetryAddLine("");
            telemetryAddData("Target Flywheel Velocity",targetFlywheelVelocity);
            telemetryAddLine("");
            telemetryAddData("Yaw Target",turretYaw.get("turretYawFront").getTarget());
            telemetryAddData("Yaw Desired",-(turretYaw.get("turretYawFront").getTarget()-180)+Math.toDegrees(follower.getHeading()));
            telemetryAddData("Yaw Raw Pos",turretYaw.get("turretYawFront").getDevice().getPosition()*355);
            telemetryAddLine("");
            telemetryAddData("Distance", Math.sqrt((follower.getPose().getX() - getTargetPoint()[0])*(follower.getPose().getX() - getTargetPoint()[0]) + (follower.getPose().getY() - getTargetPoint()[1])*(follower.getPose().getY() - getTargetPoint()[1])));
            telemetryAddData("Flywheel Velocity",flywheel.get("flywheelLeft").getVelocity());
            telemetryAddData("PoseX",follower.getPose().getX());
            telemetryAddData("PoseY",follower.getPose().getY());
            telemetryAddData("PoseHeading",Math.toDegrees(follower.getHeading()));
            telemetryAddData("Flywheel Left Power",flywheel.get("flywheelLeft").getPower());
            telemetryAddData("Flywheel Right Power",flywheel.get("flywheelRight").getPower());
            telemetryAddData("looptime",timer.time()-lastTime);
            telemetryAddData("leftFront",leftFront.getDevice().getPower());
            telemetryAddData("rightFront",rightFront.getDevice().getPower());
            telemetryAddData("leftRear",leftRear.getDevice().getPower());
            telemetryAddData("rightRear",rightRear.getDevice().getPower());
            lastTime = timer.time();
        });
        executor.runLoop(this::opModeIsActive);
    }
}