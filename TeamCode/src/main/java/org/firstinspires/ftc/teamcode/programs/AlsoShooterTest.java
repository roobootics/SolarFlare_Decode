package org.firstinspires.ftc.teamcode.programs;

import static org.firstinspires.ftc.teamcode.base.Commands.executor;
import static org.firstinspires.ftc.teamcode.base.Components.initialize;
import static org.firstinspires.ftc.teamcode.pedroPathing.Pedro.follower;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.backIntake;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.backIntakeGate;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.flywheel;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.frontIntake;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.frontIntakeGate;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.setTargetPoint;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.targetFlywheelVelocity;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.targetPoint;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.transferGate;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.turretPitch;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.turretYaw;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.base.Commands;
import org.firstinspires.ftc.teamcode.base.Components;
import org.firstinspires.ftc.teamcode.pedroPathing.Pedro;
import org.firstinspires.ftc.teamcode.robotconfigs.Inferno;

public class AlsoShooterTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        initialize(this, new Inferno(), false,false);
        Components.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Pedro.createFollower(new Pose(72,72,0));
        turretYaw.call(servo->servo.switchControl("controlOff"));
        waitForStart();
        frontIntake.setPower(1.0);
        backIntake.setPower(1.0);
        frontIntakeGate.setPosition(180);
        backIntakeGate.setPosition(180);
        transferGate.setPosition(148.5);
        executor.setCommands(turretPitch.command(servo->servo.triggeredDynamicTargetCommand(()->gamepad1.right_bumper,()->gamepad1.right_bumper,0.1)),
                new Commands.ContinuousCommand(
                        ()->{
                            setTargetPoint();
                            Pose pos = follower.getPose();
                            targetFlywheelVelocity = Inferno.VelRegression.regressFormula(Math.sqrt((targetPoint[0]-pos.getX())*(targetPoint[0]-pos.getX()) + (targetPoint[1]-pos.getY())*(targetPoint[1]-pos.getY())));
                            targetFlywheelVelocity = Math.min(Math.max(targetFlywheelVelocity,800),1425);
                        }
                ));
        executor.setWriteToTelemetry(
                ()->{
                    Components.telemetry.addData("Hood Target",turretPitch.get("turretPitchLeft").getTarget());
                    Components.telemetry.addData("Flywheel Error",targetFlywheelVelocity-flywheel.get("flywheelLeft").getVelocity());
                }
        );
        executor.runLoop(this::opModeIsActive);
    }
}
