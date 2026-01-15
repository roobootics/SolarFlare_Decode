package org.firstinspires.ftc.teamcode.programs;

import static org.firstinspires.ftc.teamcode.base.Commands.executor;
import static org.firstinspires.ftc.teamcode.base.Components.initialize;
import static org.firstinspires.ftc.teamcode.base.Components.initializeConfig;
import static org.firstinspires.ftc.teamcode.base.Components.telemetryAddData;
import static org.firstinspires.ftc.teamcode.base.Components.telemetryAddLine;
import static org.firstinspires.ftc.teamcode.pedroPathing.Pedro.follower;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.ballStorage;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.classifierBallCount;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.clearIntegralAtPeak;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.currentBallPath;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.flywheel;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.getTargetPoint;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.leftFront;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.leftRear;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.loopFSM;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.motif;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.motifShootAll;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.readBallStorage;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.rightFront;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.rightRear;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.robotState;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.setState;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.shotType;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.targetFlywheelVelocity;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.toggleShotType;

import org.firstinspires.ftc.teamcode.base.Commands;
import org.firstinspires.ftc.teamcode.base.Commands.*;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


import org.firstinspires.ftc.teamcode.base.Components.*;
import org.firstinspires.ftc.teamcode.pedroPathing.Pedro;
import org.firstinspires.ftc.teamcode.robotconfigs.Inferno;
import org.firstinspires.ftc.teamcode.robotconfigs.Inferno.*;

import java.util.Arrays;

@TeleOp
public class DecodeTeleOp extends LinearOpMode {
    boolean autoReset = true;
    @Override
    public void runOpMode(){
        initialize(hardwareMap,telemetry);
        ballStorage = new Color[]{Color.GREEN,Color.PURPLE,Color.PURPLE};
        motif = new Color[]{Color.PURPLE,Color.GREEN,Color.PURPLE};
        executor.setCommands(new RunResettingLoop(
            new PressCommand(
                new IfThen(
                      ()->(autoReset || gamepad1.x),
                      new InstantCommand(()->{autoReset=true; initializeConfig(new Inferno(new Pose(102.5, 7, 0)),true);})
                )
            )
        ));
        executor.setWriteToTelemetry(()->telemetryAddData("Reset Odometry Pos",autoReset));
        executor.runLoop(this::opModeInInit);
        initializeConfig(new Inferno(new Pose(102.5, 7, 0)),false);
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
                                new IfThen(()->gamepad2.a,new InstantCommand(()->classifierBallCount+=1)),
                                new IfThen(()->gamepad2.b,new InstantCommand(()->classifierBallCount-=1)),
                                new IfThen(()->gamepad2.dpad_up, setState(RobotState.INTAKE_FRONT_AND_SHOOT)),
                                new IfThen(()->gamepad2.dpad_down, setState(RobotState.INTAKE_BACK_AND_SHOOT)),
                                new IfThen(()->gamepad2.back,setState(RobotState.EXPEL))
                        ),
                        new InstantCommand(()->{if (classifierBallCount>9) classifierBallCount=9; else if (classifierBallCount<0) classifierBallCount=0;}),
                        new ConditionalCommand(
                            new IfThen(
                                    ()->!follower.isBusy(),
                                    new FieldCentricMecanumCommand(
                                            new BotMotor[]{leftFront,leftRear,rightFront,rightRear},
                                            ()->(0.0),1,
                                            ()-> (double) gamepad1.left_stick_x, ()-> (double) gamepad1.left_stick_y, ()-> (double) gamepad1.right_stick_x,
                                            ()->{if (gamepad1.left_trigger > 0.3) return 0.75; else return 1.0;}
                                    )
                            )
                        ),
                        loopFSM
                ),
                clearIntegralAtPeak,
                Pedro.updateCommand()
        );
        executor.setWriteToTelemetry(()->{
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
        executor.runLoop(this::opModeIsActive);
    }
}