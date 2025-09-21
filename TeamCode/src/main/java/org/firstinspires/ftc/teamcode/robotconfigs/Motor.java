package org.firstinspires.ftc.teamcode.robotconfigs;

import org.firstinspires.ftc.teamcode.base.Components.*;
import org.firstinspires.ftc.teamcode.presets.PresetControl.*;

import java.util.List;

public class Motor implements RobotConfig {
    public static BotMotor motor;
    public static double targetVelocity;
    @Override
    public void init() {
        motor = new BotMotor(
                "motor", List.of(new DcMotorExData("motor")),
                ()->2000.0,()->0.0,20.0,5.0,
                new String[]{"PID","setVelocity","VelocityPIDF"},
                new ControlSystem<>(new TrapezoidalMotionProfile<>(10000,1000),new PositionPID<>(0.015,0.00001,0.0001)),
                new ControlSystem<>(
                        new String[]{"targetVelocity"},
                        List.of((BotMotor motor) -> targetVelocity),
                        (String name, Double vel)->motor.setVelocity(vel,name),
                        new SetVelocity()
                ),
                new ControlSystem<>(
                        new String[]{"targetVelocity"},
                        List.of((BotMotor motor) -> targetVelocity),
                        new BasicFeedforward<>(new double[]{0.000357185},new String[]{"targetVelocity"}),
                        new VelocityPID<>(0.0001,0,0)
                )
        );
    }
}
