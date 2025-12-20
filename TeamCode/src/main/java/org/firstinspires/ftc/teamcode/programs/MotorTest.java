package org.firstinspires.ftc.teamcode.programs;
import static org.firstinspires.ftc.teamcode.base.Components.*;
import static org.firstinspires.ftc.teamcode.base.Commands.*;

import org.firstinspires.ftc.teamcode.robotconfigs.Inferno;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.*;

import org.firstinspires.ftc.teamcode.base.Components.*;
import org.firstinspires.ftc.teamcode.base.Commands.*;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class MotorTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        initialize(hardwareMap,telemetry,new Inferno(),true);
        waitForStart();
        executor.setCommands(new RunResettingLoop(
                new PressCommand(
                        new IfThen(
                                ()->gamepad1.dpad_up,
                                leftFront.setPowerCommand(1.0)
                        ),
                        new IfThen(
                                ()->gamepad1.dpad_down,
                                leftRear.setPowerCommand(1.0)
                        ),
                        new IfThen(
                                ()->gamepad1.dpad_right,
                                rightFront.setPowerCommand(1.0)
                        ),
                        new IfThen(
                                ()->gamepad1.dpad_left,
                                rightRear.setPowerCommand(1.0)
                        )
                )
        ));
        executor.runLoop(this::opModeIsActive);
    }
}
