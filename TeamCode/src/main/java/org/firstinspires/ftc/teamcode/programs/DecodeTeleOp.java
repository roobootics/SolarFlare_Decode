package org.firstinspires.ftc.teamcode.programs;

import static org.firstinspires.ftc.teamcode.base.Commands.executor;
import static org.firstinspires.ftc.teamcode.base.Components.telemetryAddData;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.loopFSM;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.setState;

import org.firstinspires.ftc.teamcode.base.Commands.*;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


import org.firstinspires.ftc.teamcode.base.Components;
import org.firstinspires.ftc.teamcode.robotconfigs.Inferno;

@TeleOp
public class DecodeTeleOp extends LinearOpMode {
    boolean autoReset = true;
    @Override
    public void runOpMode(){
        executor.setCommands(
            new ConditionalCommand(
                new IfThen(
                      ()->(!autoReset&&gamepad1.a),
                      new InstantCommand(()->autoReset=true)
                )
            ),
            new ConditionalCommand(
                new IfThen()
            )
        );
        executor.runLoop(this::opModeInInit);
        executor.setWriteToTelemetry(()->telemetryAddData("Reset Odometry Pos",autoReset));
        new InstantCommand(()-> Components.initialize(hardwareMap,telemetry,new Inferno(),autoReset));

        executor.setCommands(
                new RunResettingLoop(
                        new ConditionalCommand(
                                new IfThen(()->gamepad1.right_bumper, setState(Inferno.RobotState.INTAKE_FRONT)),
                                new IfThen(()->gamepad1.left_bumper, setState(Inferno.RobotState.INTAKE_BACK)),
                                new IfThen(()->gamepad1.right_trigger>0.8, setState(Inferno.RobotState.INTAKE_FRONT_AND_SHOOT)),
                                new IfThen(()->gamepad1.left_trigger>0.8, setState(Inferno.RobotState.INTAKE_BACK_AND_SHOOT)),
                                new IfThen(()->gamepad1.x, setState(Inferno.RobotState.SHOOTING)),
                                new IfThen(()->gamepad1.y, setState(Inferno.RobotState.NONE))
                        ),
                        new ConditionalCommand(

                        ),
                        loopFSM
                )
        );
    }
}
