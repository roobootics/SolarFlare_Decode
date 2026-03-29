package org.firstinspires.ftc.teamcode.programs;

import static org.firstinspires.ftc.teamcode.base.Commands.executor;
import static org.firstinspires.ftc.teamcode.base.Components.initialize;
import static org.firstinspires.ftc.teamcode.pedroPathing.Pedro.follower;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.backIntake;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.backIntakeGate;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.ballStorage;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.flywheel;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.frontIntake;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.frontIntakeGate;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.leftFront;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.leftRear;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.readBallStorage;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.rightFront;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.rightRear;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.transfer;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.transferGate;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.turret;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.turretPitch;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.turretYaw;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.vision;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.base.Commands;
import org.firstinspires.ftc.teamcode.base.Components;
import org.firstinspires.ftc.teamcode.pedroPathing.Pedro;
import org.firstinspires.ftc.teamcode.robotconfigs.Inferno;

import java.util.ArrayList;
import java.util.Arrays;

@TeleOp
public class Diagnostic extends LinearOpMode {
    private int stage = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        initialize(this, new Inferno(), false, false);
        Pedro.createFollower(new Pose(72,72,0));
        leftFront.resetEncoder();
        executor.setCommands(
                new Commands.PressCommand(new Commands.IfThen(()->gamepad1.a,new Commands.InstantCommand(()-> stage++))),
                Pedro.updatePoseCommand(),
                new Commands.SequentialCommand(
                    new Commands.ParallelCommand(
                            frontIntake.setPowerCommand(0.2),
                            backIntake.setPowerCommand(0.2),
                            flywheel.command(motor->motor.setPowerCommand(0.15))
                    ),
                    new Commands.LoopUntilTrue(()-> stage ==1,
                            new Commands.SequentialCommand(transferGate.moveToTargetCommand("open"),new Commands.SleepCommand(0.5),transferGate.moveToTargetCommand("closed"),new Commands.SleepCommand(0.5)),
                            new Commands.SequentialCommand(frontIntakeGate.moveToTargetCommand("open"),new Commands.SleepCommand(0.5),frontIntakeGate.moveToTargetCommand("closed"),new Commands.SleepCommand(0.5)),
                            new Commands.SequentialCommand(backIntakeGate.moveToTargetCommand("open"),new Commands.SleepCommand(0.5),backIntakeGate.moveToTargetCommand("closed"),new Commands.SleepCommand(0.5)),
                            turretPitch.command(servo->new Commands.SequentialCommand(servo.moveToTargetCommand(130),new Commands.SleepCommand(0.5),servo.moveToTargetCommand(110),new Commands.SleepCommand(0.5))),
                            turretYaw.command(servo->new Commands.SequentialCommand(servo.setPowerCommand(0.2),new Commands.SleepCommand(0.5),servo.moveToTargetCommand(-0.2),new Commands.SleepCommand(0.5)))
                    ),
                    new Commands.ParallelCommand(
                            frontIntake.setPowerCommand(0),
                            backIntake.setPowerCommand(0),
                            flywheel.command(motor->motor.setPowerCommand(0)),
                            leftFront.setPowerCommand(0.1),
                            leftRear.setPowerCommand(0.1),
                            rightFront.setPowerCommand(-0.1),
                            rightRear.setPowerCommand(-0.1)
                    ),
                    new Commands.SleepUntilTrue(()->stage==2),
                    new Commands.ParallelCommand(
                        leftFront.setPowerCommand(0),
                        leftRear.setPowerCommand(0),
                        rightFront.setPowerCommand(0),
                        rightRear.setPowerCommand(0),
                        flywheel.command(motor->motor.setPowerCommand(0.3)),
                        new Commands.InstantCommand(leftFront::setZeroPowerFloat),
                        new Commands.InstantCommand(leftRear::setZeroPowerFloat),
                        new Commands.InstantCommand(rightFront::setZeroPowerFloat),
                        new Commands.InstantCommand(rightRear::setZeroPowerFloat)
                    ),
                    new Commands.RunLoop(
                            new Commands.InstantCommand(Inferno::readBallStorage),
                            turretYaw.command(servo->new Commands.SequentialCommand(servo.setPowerCommand(0.2),new Commands.SleepCommand(0.5),servo.moveToTargetCommand(-0.2),new Commands.SleepCommand(0.5)))
                    )
                )
        );
        executor.setWriteToTelemetry(
                ()->{
                    if (stage>=2){
                        telemetry.addData("X", follower.getPose().getX());
                        telemetry.addData("Y", follower.getPose().getY());
                        telemetry.addData("Heading", Math.toDegrees(follower.getPose().getHeading()));
                        telemetry.addData("Velocity", flywheel.get("flywheelLeft").getVelocity());
                        telemetry.addData("Turret", turretYaw.get("turretYawTop").getCurrentPosition());
                        telemetry.addData("Ball Storage", Arrays.asList(ballStorage));
                        telemetry.addData("Limelight Ball Count", vision.getArtifactDescriptors(follower.getPose(), Arrays.asList("green","purple")).size());
                    }
                }
        );
        waitForStart();
        Components.activateActuatorControl();
        flywheel.call(motor->motor.switchControl("controlOff"));
        turretYaw.call(servo->servo.switchControl("controlOff"));
        executor.runLoop(this::opModeIsActive);
        leftFront.setZeroPowerBrake();
        rightFront.setZeroPowerBrake();
        leftRear.setZeroPowerBrake();
        rightFront.setZeroPowerBrake();
    }
}
