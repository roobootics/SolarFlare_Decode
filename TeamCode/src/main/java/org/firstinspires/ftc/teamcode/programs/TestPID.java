package org.firstinspires.ftc.teamcode.programs;

import static org.firstinspires.ftc.teamcode.base.Commands.executor;
import static org.firstinspires.ftc.teamcode.base.Components.telemetryAddData;
import static org.firstinspires.ftc.teamcode.robotconfigs.Motor.motor;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.base.Components;
import org.firstinspires.ftc.teamcode.robotconfigs.Motor;

@TeleOp
public class TestPID extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Components.initialize(hardwareMap,telemetry,new Motor(),true);
        Components.activateActuatorControl();
        executor.setCommands(
                motor.triggeredToggleTargetCommand(()->gamepad1.b,0,2000)
        );
        waitForStart();
        executor.setWriteToTelemetry(()->telemetryAddData("position",motor.getCurrentPosition()));
        executor.runLoop(this::opModeIsActive);
    }
}
