package org.firstinspires.ftc.teamcode.programs;

import static org.firstinspires.ftc.teamcode.base.Commands.executor;
import static org.firstinspires.ftc.teamcode.base.Components.telemetryAddData;
import static org.firstinspires.ftc.teamcode.robotconfigs.Motor.motor;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.base.Commands;
import org.firstinspires.ftc.teamcode.base.Components;
import org.firstinspires.ftc.teamcode.presets.PresetControl.GenericPIDF;
import org.firstinspires.ftc.teamcode.robotconfigs.Motor;

@TeleOp
@Config
public class VelocityPIDTets extends LinearOpMode {
    public static double kP;
    public static double kI;
    public static double kD;
    public static double kF=0.00035;
    public double averageVel;
    public double meanError;
    public double velocity;
    public long loop;
    public double targetVelocity=1000;
    public GenericPIDF velocityPID = new GenericPIDF(kP,kI,kD,kF);

    @Override
    public void runOpMode() throws InterruptedException {
        Components.initialize(hardwareMap,telemetry,new Motor(),true);
        waitForStart();
        executor.setCommands(
                new Commands.RunResettingLoop(
                        motor.setPowerCommand(()->velocityPID.getPIDFOutput(targetVelocity,motor.getVelocity())),
                        new Commands.ConditionalCommand(
                                new Commands.IfThen(
                                        ()->gamepad1.a,
                                        new Commands.InstantCommand(()->{
                                            loop+=1;
                                            double vel = motor.getVelocity();
                                            velocity=vel;
                                            double error = Math.abs(targetVelocity-vel);
                                            averageVel *= (double) (loop - 1) /(loop);
                                            averageVel += (vel/(loop));
                                            meanError *= (double) (loop - 1) /(loop);
                                            meanError += (error/(loop));
                                        })
                                )

                        ),
                        Commands.triggeredToggleCommand(()->gamepad1.b,
                                new Commands.InstantCommand(()->targetVelocity=2000),
                                new Commands.InstantCommand(()->targetVelocity=1000)
                        )
                )

        );
        executor.setWriteToTelemetry(
                ()->{
                    telemetryAddData("power",motor.getPower());
                    telemetryAddData("vel",velocity);
                    telemetryAddData("mean vel",averageVel);
                    telemetryAddData("mean error",meanError);
                }
        );
        executor.runLoop(this::opModeIsActive);
    }
}
