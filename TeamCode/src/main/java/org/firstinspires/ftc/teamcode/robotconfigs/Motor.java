package org.firstinspires.ftc.teamcode.robotconfigs;

import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.base.Components.*;
import org.firstinspires.ftc.teamcode.presets.PresetControl.*;

import java.util.List;

public class Motor implements RobotConfig {
    public static BotMotor motor;
    public static TrapezoidalMotionProfile<BotMotor> profile;
    public static double targetVelocity;
    @Override
    public void init() {
        profile=new TrapezoidalMotionProfile<>(10000,1000);
        motor = new BotMotor(
                "motor", DcMotorSimple.Direction.FORWARD,
                ()->2000.0,()->0.0,20.0,5.0,
                new String[]{"PID","setVelocity","VelocityPIDF"},
                new ControlSystem<>(profile,new PositionPID<>(0.015,0.0,0.0)),
                new ControlSystem<>(
                        new String[]{"targetVelocity"},
                        List.of(() -> targetVelocity),
                        (Double vel)->motor.setVelocity(vel),
                        new SetVelocity()
                ),
                new ControlSystem<>(
                        new String[]{"targetVelocity"},
                        List.of(() -> targetVelocity),
                        new BasicFeedforward<>(new double[]{0.000357185},new String[]{"targetVelocity"}),
                        new VelocityPID(0.0001,0,0)
                )
        );
    }
}
