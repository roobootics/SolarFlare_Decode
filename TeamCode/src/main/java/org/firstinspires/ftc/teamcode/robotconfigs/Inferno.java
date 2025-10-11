package org.firstinspires.ftc.teamcode.robotconfigs;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.base.Components;
import org.firstinspires.ftc.teamcode.base.Components.*;
import org.firstinspires.ftc.teamcode.pedroPathing.Pedro;
import org.firstinspires.ftc.teamcode.presets.PresetControl.*;

import java.util.List;

public class Inferno implements RobotConfig{
    public static BotMotor leftFront;
    public static BotMotor leftRear;
    public static BotMotor rightFront;
    public static BotMotor rightRear;
    public static SyncedActuators<BotMotor> flywheel;
    public static double targetFlywheelVelocity;
    public static SyncedActuators<BotServo> turretYaw;
    public static SyncedActuators<BotServo> turretPitch;
    public static BotMotor frontIntake;
    public static BotMotor backIntake;
    public static RevColorSensorV3[] sensors = new RevColorSensorV3[3];
    public static Limelight3A limelight;
    @Override
    public void init() {
        Pedro.createFollower(new Pose(0,0,0));
        leftFront = new BotMotor("leftFront", DcMotorSimple.Direction.FORWARD);
        leftRear = new BotMotor("leftRear", DcMotorSimple.Direction.FORWARD);
        rightFront = new BotMotor("rightFront", DcMotorSimple.Direction.REVERSE);
        rightRear = new BotMotor("rightRear", DcMotorSimple.Direction.REVERSE);
        flywheel = new SyncedActuators<>(
              new BotMotor("flywheelLeft",DcMotorSimple.Direction.FORWARD,0,0, new String[]{"VelocityPID"},
                      new ControlSystem<>(new String[]{"targetVelocity"},List.of(()->targetFlywheelVelocity),
                              new VelocityPID(0,0,0),new BasicFeedforward(0,"targetVelocity"))),
              new BotMotor("flywheelRight",DcMotorSimple.Direction.REVERSE,0,0, new String[]{"VelocityPID"},
                      new ControlSystem<>(new String[]{"targetVelocity"},List.of(()->targetFlywheelVelocity),
                              new VelocityPID(0,0,0),new BasicFeedforward(0,"targetVelocity")))
        );
        turretPitch = new SyncedActuators<>(
                new BotServo("turretPitchLeft",Servo.Direction.FORWARD,422,5,270,90),
                new BotServo("turretPitchRight",Servo.Direction.REVERSE,422,5,270,90)
        );
        turretYaw = new SyncedActuators<>(
                new BotServo("turretYawLeft",Servo.Direction.FORWARD,422,5,270,90),
                new BotServo("turretYawRight",Servo.Direction.FORWARD,422,5,270,90)
        );
        frontIntake = new BotMotor("frontIntake",DcMotorSimple.Direction.FORWARD);
        backIntake = new BotMotor("backIntake",DcMotorSimple.Direction.REVERSE);
        frontIntake.setKeyPowers(
                new String[]{"thisSideIntake","transfer","otherSideIntake","expel"},
                new double[]{}
        );
        backIntake.setKeyPowers(
                new String[]{"thisSideIntake","transfer","otherSideIntake","expel"},
                new double[]{}
        );
        sensors[0] = Components.getHardwareMap().get(RevColorSensorV3.class,"sensor1");
        sensors[1] = Components.getHardwareMap().get(RevColorSensorV3.class,"sensor2");
        sensors[2] = Components.getHardwareMap().get(RevColorSensorV3.class,"sensor3");
        limelight=Components.getHardwareMap().get(Limelight3A.class,"limelight");
    }
}
