package org.firstinspires.ftc.teamcode.programs;

import static org.firstinspires.ftc.teamcode.base.Commands.executor;
import static org.firstinspires.ftc.teamcode.base.Components.initialize;
import static org.firstinspires.ftc.teamcode.base.Components.telemetryAddData;
import static org.firstinspires.ftc.teamcode.base.Components.telemetryAddLine;
import static org.firstinspires.ftc.teamcode.pedroPathing.Pedro.follower;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.backIntake;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.ballStorage;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.classifierBallCount;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.currentBallPath;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.frontIntake;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.leftFront;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.leftRear;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.loopFSM;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.motif;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.motifShootAll;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.readBallStorage;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.rightFront;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.rightRear;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.robotState;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.sensors;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.setState;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.shotType;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.toggleShotType;

import org.firstinspires.ftc.teamcode.base.Commands.*;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


import org.firstinspires.ftc.teamcode.base.Components.*;
import org.firstinspires.ftc.teamcode.robotconfigs.Inferno;
import org.firstinspires.ftc.teamcode.robotconfigs.Inferno.*;

import java.util.Arrays;

@TeleOp
public class DecodeTeleOp extends LinearOpMode {
    boolean autoReset = true;
    @Override
    public void runOpMode(){
        initialize(hardwareMap,telemetry,new Inferno(),false);
        ballStorage = new Color[]{Color.GREEN,Color.PURPLE,Color.PURPLE};
        motif = new Color[]{Color.PURPLE,Color.GREEN,Color.PURPLE};
        executor.setCommands(new RunResettingLoop(
            new ConditionalCommand(
                new IfThen(
                      ()->(!autoReset&&gamepad1.x),
                      new InstantCommand(()->autoReset=true)
                )
            )
        ));
        executor.setWriteToTelemetry(()->telemetryAddData("Reset Odometry Pos",autoReset));
        executor.runLoop(this::opModeInInit);

        initialize(hardwareMap,telemetry,new Inferno(),autoReset);

        executor.setCommands(
                new RunResettingLoop(
                        new ConditionalCommand(
                                new IfThen(()->gamepad1.right_bumper, setState(RobotState.INTAKE_FRONT)),
                                new IfThen(()->gamepad1.left_bumper, setState(RobotState.INTAKE_BACK)),
                                new IfThen(()->gamepad1.right_trigger>0.8, setState(RobotState.SHOOTING)),
                                new IfThen(()->gamepad1.y, setState(RobotState.NONE))
                        ),
                        new PressCommand(
                                new IfThen(()->gamepad2.y,toggleShotType()),
                                new IfThen(()->gamepad2.x,new InstantCommand(()->classifierBallCount=0)),
                                new IfThen(()->gamepad2.a,new InstantCommand(()->classifierBallCount+=1)),
                                new IfThen(()->gamepad2.b,new InstantCommand(()->classifierBallCount-=1)),
                                new IfThen(()->gamepad2.dpad_up, setState(RobotState.INTAKE_FRONT_AND_SHOOT)),
                                new IfThen(()->gamepad2.dpad_down, setState(RobotState.INTAKE_BACK_AND_SHOOT)),
                                new IfThen(()->gamepad2.back,new InstantCommand(()->motifShootAll=!motifShootAll))
                        ),
                        new InstantCommand(()->{if (classifierBallCount>9) classifierBallCount=9; else if (classifierBallCount<0) classifierBallCount=0;}),
                        new FieldCentricMecanumCommand(
                                new BotMotor[]{leftFront,leftRear,rightFront,rightRear},
                                ()->(follower.getHeading()),1,
                                ()-> (double) gamepad1.left_stick_x, ()-> (double) gamepad1.left_stick_y, ()-> (double) gamepad1.right_stick_x,
                                ()->{if (gamepad1.left_trigger > 0.8) return 0.75; else return 1.0;}
                        ),
                        loopFSM
                )
        );
        executor.setWriteToTelemetry(()->{
            readBallStorage();
            telemetryAddData("Ball Storage:", Arrays.asList(ballStorage));
            telemetryAddData("Robot State:",robotState);
            telemetryAddData("Shot Type:",shotType);
            telemetryAddLine("");
            telemetryAddData("Classifier Count:",classifierBallCount);
            telemetryAddData("Current Shot Height:",currentBallPath);
            telemetryAddData("Shoot All Motif:",motifShootAll);
            telemetryAddData("Front Velocity:",frontIntake.getVelocity());
            telemetryAddData("Back Velocity:",backIntake.getVelocity());
        });
        executor.runLoop(this::opModeIsActive);
    }
}