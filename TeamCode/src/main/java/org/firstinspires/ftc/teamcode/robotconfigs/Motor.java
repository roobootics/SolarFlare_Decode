package org.firstinspires.ftc.teamcode.robotconfigs;

import org.firstinspires.ftc.teamcode.base.Components.*;
import org.firstinspires.ftc.teamcode.presets.PresetControl.PIDF.PIDFConstants;
import org.firstinspires.ftc.teamcode.presets.PresetControl.*;

import java.util.Arrays;
import java.util.Collections;

public class Motor implements RobotConfig {
    public static BotMotor motor;
    @Override
    public void init() {
        motor = new BotMotor(
                "motor", Arrays.asList(new DcMotorExData("motor")),
                ()->2000.0,()->0.0,20,5,
                new String[]{"PID"},
                Collections.singletonList(new PIDF<>(new PIDFConstants(0.015,0.0001,0.005,0)))
        );
    }
}
