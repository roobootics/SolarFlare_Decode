package org.firstinspires.ftc.teamcode.programs;

import static org.firstinspires.ftc.teamcode.base.Commands.executor;
import static org.firstinspires.ftc.teamcode.base.Commands.triggeredToggleCommand;
import static org.firstinspires.ftc.teamcode.base.Components.telemetryAddData;
import static org.firstinspires.ftc.teamcode.robotconfigs.Motor.motor;
import static org.firstinspires.ftc.teamcode.robotconfigs.Motor.profile;
import static org.firstinspires.ftc.teamcode.robotconfigs.Motor.targetVelocity;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.base.Commands.*;
import org.firstinspires.ftc.teamcode.base.Components;
import org.firstinspires.ftc.teamcode.robotconfigs.Motor;

@TeleOp
public class tets extends LinearOpMode {
    @Override
    public void runOpMode() {
        Components.initialize(hardwareMap,telemetry,new Motor(),false);
        motor.switchControl("PID");
        executor.setCommands(
                motor.triggeredToggleTargetCommand(()->gamepad1.a,0,2000)
        );
        executor.setWriteToTelemetry(()->{
            telemetryAddData("pos",motor.getCurrentPosition());
        });
        waitForStart();
        executor.runLoop(this::opModeIsActive);
    }
}
