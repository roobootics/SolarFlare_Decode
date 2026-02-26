package org.firstinspires.ftc.teamcode.programs;

import static org.firstinspires.ftc.teamcode.base.Commands.executor;
import static org.firstinspires.ftc.teamcode.base.Components.initialize;
import static org.firstinspires.ftc.teamcode.pedroPathing.Pedro.follower;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.flywheel;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.leftFront;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.turretPitch;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.turretYaw;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.base.Commands;
import org.firstinspires.ftc.teamcode.base.Components;
import org.firstinspires.ftc.teamcode.pedroPathing.Pedro;
import org.firstinspires.ftc.teamcode.robotconfigs.Inferno;

@TeleOp
public class HoodTest extends LinearOpMode {
    public double targetYaw = 180;

    @Override
    public void runOpMode(){
        initialize(this,new Inferno(),true,true);
        leftFront.resetEncoder();
        Pedro.createFollower(new Pose(0,0,0));
        executor.setCommands(
                turretPitch.command((Components.BotServo servo)->servo.triggeredDynamicTargetCommand(()->gamepad1.right_bumper,()->gamepad1.left_bumper,0.1)),
                Commands.triggeredDynamicCommand(()->gamepad1.right_trigger>0.3,()->gamepad1.left_trigger>0.3,new Commands.InstantCommand(()->targetYaw-=0.05),new Commands.InstantCommand(()->targetYaw+=0.05)),
                new Commands.RunResettingLoop(
                        new Commands.InstantCommand(()->{
                            if (targetYaw>180) targetYaw=180;
                            else if (targetYaw<-135) targetYaw = -135;
                        }),
                        turretYaw.command(servo->servo.instantSetTargetCommand(()->targetYaw))
                )
        );
        executor.setWriteToTelemetry(()->{
            telemetry.addData("hood",turretPitch.get("turretPitchLeft").getTarget());
            telemetry.addData("yaw pos",turretYaw.get("turretYawTop").getCurrentPosition());
            telemetry.addData("encoder count",leftFront.getCurrentPosition());
            telemetry.addData("yaw target",turretYaw.get("turretYawTop").getTarget());
            telemetry.addData("yaw angle",targetYaw);
            telemetry.addData("top yaw power",turretYaw.get("turretYawTop").getPower());
            telemetry.addData("bottom yaw power",turretYaw.get("turretYawBottom").getPower());
        });
        waitForStart();
        Components.activateActuatorControl();
        flywheel.call((Components.BotMotor motor)->motor.switchControl("controlOff"));
        executor.runLoop(this::opModeIsActive);
    }
}
