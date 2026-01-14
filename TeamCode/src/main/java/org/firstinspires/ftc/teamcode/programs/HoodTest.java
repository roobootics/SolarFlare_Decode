package org.firstinspires.ftc.teamcode.programs;

import static org.firstinspires.ftc.teamcode.base.Commands.executor;
import static org.firstinspires.ftc.teamcode.base.Commands.triggeredDynamicCommand;
import static org.firstinspires.ftc.teamcode.base.Components.initialize;
import static org.firstinspires.ftc.teamcode.base.Components.initializeConfig;
import static org.firstinspires.ftc.teamcode.base.Components.telemetryAddData;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.turretPitch;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.turretYaw;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.base.Commands;
import org.firstinspires.ftc.teamcode.base.Components;
import org.firstinspires.ftc.teamcode.robotconfigs.Inferno;

@TeleOp
public class HoodTest extends LinearOpMode {
    public double targetYaw = 228;

    @Override
    public void runOpMode() throws InterruptedException {
        initialize(hardwareMap,telemetry);
        initializeConfig(new Inferno(),true);
        executor.setCommands(
                turretPitch.command((Components.BotServo servo)->servo.triggeredDynamicTargetCommand(()->gamepad1.right_bumper,()->gamepad1.left_bumper,0.1)),
                Commands.triggeredDynamicCommand(()->gamepad1.right_trigger>0.3,()->gamepad1.left_trigger>0.3,new Commands.InstantCommand(()->targetYaw-=1),new Commands.InstantCommand(()->targetYaw+=1)),
                new Commands.RunResettingLoop(
                        turretYaw.command((Components.BotServo servo)-> servo.instantSetTargetCommand(()->228-targetYaw))
                )

        );
        executor.setWriteToTelemetry(()->{
            telemetryAddData("hood",turretPitch.getActuators().get("turretPitchLeft").getTarget());
            telemetryAddData("yaw pos",turretYaw.getActuators().get("turretYawFront").getTarget());
            telemetryAddData("yaw angle",targetYaw);
        });
        waitForStart();
        executor.runLoop(this::opModeIsActive);
    }
}
