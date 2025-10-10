package org.firstinspires.ftc.teamcode.robotconfigs;

import com.qualcomm.hardware.rev.RevColorSensorV3;

import org.firstinspires.ftc.teamcode.base.Components.*;

public class Inferno implements RobotConfig{
    public static BotMotor leftFront;
    public static BotMotor leftRear;
    public static BotMotor rightFront;
    public static BotMotor rightBack;
    public static SyncedActuators<BotMotor> flywheel;
    public static SyncedActuators<BotServo> turretYaw;
    public static SyncedActuators<BotServo> turretPitch;
    public static BotMotor frontIntake;
    public static BotMotor backIntake;
    public static RevColorSensorV3[] sensors = new RevColorSensorV3[3];
    @Override
    public void init() {

    }
}
