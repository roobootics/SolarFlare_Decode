package org.firstinspires.ftc.teamcode.robotconfigs;

import static org.firstinspires.ftc.teamcode.pedroPathing.Pedro.follower;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


import org.firstinspires.ftc.teamcode.base.Commands.*;
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
    public static double targetTurretYaw;
    public static SyncedActuators<BotServo> turretPitch;
    public static BotMotor frontIntake;
    public static BotMotor backIntake;
    public static BotServo frontIntakeGate;
    public static BotServo backIntakeGate;
    public static RevColorSensorV3[] sensors = new RevColorSensorV3[3];
    public static Limelight3A limelight;
    public static BotServo limelightPitch;
    public enum Color{
        PURPLE,
        GREEN,
    }
    public enum ShotType{
        MOTIF,
        NORMAL
    }
    public enum RobotState{
        NONE,
        INTAKE_FRONT,
        INTAKE_BACK,
        SHOOTING,
        INTAKE_FRONT_AND_SHOOT,
        INTAKE_BACK_AND_SHOOT
    }
    public static Color[] ballStorage = new Color[3];
    public static RobotState robotState = RobotState.NONE;
    public static ShotType shotType = ShotType.NORMAL;
    public static double ballShotTiming;
    public static Color[] motif = new Color[3];
    public static Color[] shotSequence = new Color[3];

    public static void colorSensorRead(){
        for (int i=0;i<3;i++){
            RevColorSensorV3 sensor = sensors[i];
            double [] greenCenter = new double[]{0,255,0};
            double [] purpleCenter = new double[]{255,0,255};
            double greenTolerance = 75;
            double purpleTolerance = 75;
            Color color = null;
            if ((sensor.red()-greenCenter[0])*(sensor.red()-greenCenter[0]) + (sensor.green()-greenCenter[1])*(sensor.green()-greenCenter[1]) + (sensor.blue()-greenCenter[2])*(sensor.blue()-greenCenter[2])>greenTolerance*greenTolerance){
                color = Color.GREEN;
            }
            else if ((sensor.red()-purpleCenter[0])*(sensor.red()-purpleCenter[0]) + (sensor.red()-purpleCenter[1])*(sensor.red()-purpleCenter[1]) + (sensor.red()-purpleCenter[2])*(sensor.red()-purpleCenter[2])>purpleTolerance*purpleTolerance){
                color = Color.PURPLE;
            }
            ballStorage[i] = color;
        }
    }

    public static ParallelCommand frontIntakeAction;
    public static ParallelCommand backIntakeAction;
    public static ParallelCommand stopIntake;
    public static ParallelCommand frontTransfer;
    public static ParallelCommand backTransfer;
    public static ParallelCommand frontIntakeAndTransfer;
    public static ParallelCommand backIntakeAndTransfer;
    public static ConditionalCommand transfer;
    public static void setTargetTurretYaw(){
        Pose currentRobotPose = follower.getPose();

    }
    @Override
    public void init() {
        Pedro.createFollower(new Pose(0, 0, 0));
        leftFront = new BotMotor("leftFront", DcMotorSimple.Direction.FORWARD);
        leftRear = new BotMotor("leftRear", DcMotorSimple.Direction.FORWARD);
        rightFront = new BotMotor("rightFront", DcMotorSimple.Direction.REVERSE);
        rightRear = new BotMotor("rightRear", DcMotorSimple.Direction.REVERSE);
        flywheel = new SyncedActuators<>(
                new BotMotor("flywheelLeft", DcMotorSimple.Direction.FORWARD, 0, 0, new String[]{"VelocityPID"},
                        new ControlSystem<>(new String[]{"targetVelocity"}, List.of(() -> targetFlywheelVelocity),
                                new VelocityPID(0, 0, 0), new BasicFeedforward(0, "targetVelocity"))),
                new BotMotor("flywheelRight", DcMotorSimple.Direction.REVERSE, 0, 0, new String[]{"VelocityPID"},
                        new ControlSystem<>(new String[]{"targetVelocity"}, List.of(() -> targetFlywheelVelocity),
                                new VelocityPID(0, 0, 0), new BasicFeedforward(0, "targetVelocity")))
        );
        turretPitch = new SyncedActuators<>(
                new BotServo("turretPitchLeft", Servo.Direction.FORWARD, 422, 5, 270, 90),
                new BotServo("turretPitchRight", Servo.Direction.REVERSE, 422, 5, 270, 90)
        );
        turretYaw = new SyncedActuators<>(
                new BotServo("turretYawLeft", Servo.Direction.FORWARD, 422, 5, 355, 90),
                new BotServo("turretYawRight", Servo.Direction.FORWARD, 422, 5, 355, 90)
        );
        turretYaw.call((BotServo servo) -> servo.setTargetBounds(() -> 355.0, () -> 0.0));
        frontIntake = new BotMotor("frontIntake", DcMotorSimple.Direction.FORWARD);
        backIntake = new BotMotor("backIntake", DcMotorSimple.Direction.REVERSE);
        frontIntake.setKeyPowers(
                new String[]{"intake", "otherSideIntake", "thisSideTransfer", "otherSideTransfer", "stopped", "expel"},
                new double[]{}
        );
        backIntake.setKeyPowers(
                new String[]{"intake", "otherSideIntake", "thisSideTransfer", "otherSideTransfer", "stopped", "expel"},
                new double[]{}
        );
        frontIntakeGate = new BotServo("frontIntakeGate", Servo.Direction.FORWARD, 422, 5, 270, 90);
        backIntakeGate = new BotServo("backIntakeGate", Servo.Direction.REVERSE, 422, 5, 270, 90);
        frontIntakeGate.setKeyPositions(new String[]{"open", "closed"}, new double[]{});
        backIntakeGate.setKeyPositions(new String[]{"open", "closed"}, new double[]{});
        sensors[0] = Components.getHardwareMap().get(RevColorSensorV3.class, "sensor1");
        sensors[1] = Components.getHardwareMap().get(RevColorSensorV3.class, "sensor2");
        sensors[2] = Components.getHardwareMap().get(RevColorSensorV3.class, "sensor3");
        limelight = Components.getHardwareMap().get(Limelight3A.class, "limelight");
        limelightPitch = new BotServo("limelightPitch", Servo.Direction.FORWARD, 422, 5, 270, 90);
        limelightPitch.setKeyPositions(new String[]{"balls", "apriltag", "obelisk"}, new double[]{});
        limelightPitch.setTarget(limelightPitch.getPos("obelisk"));

        frontIntakeAction = new ParallelCommand(
                frontIntake.setPowerCommand("intake"),
                backIntake.setPowerCommand("otherSideIntake"),
                frontIntakeGate.instantSetTargetCommand("open"),
                backIntakeGate.instantSetTargetCommand("closed")
        );
        backIntakeAction = new ParallelCommand(
                backIntake.setPowerCommand("intake"),
                frontIntake.setPowerCommand("otherSideIntake"),
                backIntakeGate.instantSetTargetCommand("open"),
                frontIntakeGate.instantSetTargetCommand("closed")
        );
        stopIntake = new ParallelCommand(
                frontIntake.setPowerCommand("stopped"),
                backIntake.setPowerCommand("stopped"),
                frontIntakeGate.instantSetTargetCommand("open"),
                backIntakeGate.instantSetTargetCommand("open")
        );
        frontTransfer = new ParallelCommand(
                frontIntake.setPowerCommand("transfer"),
                backIntake.setPowerCommand("otherSideTransfer"),
                frontIntakeGate.instantSetTargetCommand("closed"),
                backIntakeGate.instantSetTargetCommand("closed")
        );
        backTransfer = new ParallelCommand(
                backIntake.setPowerCommand("transfer"),
                frontIntake.setPowerCommand("otherSideTransfer"),
                frontIntakeGate.instantSetTargetCommand("closed"),
                backIntakeGate.instantSetTargetCommand("closed")
        );
        frontIntakeAndTransfer = new ParallelCommand(
                frontIntake.setPowerCommand("transfer"),
                backIntake.setPowerCommand("otherSideTransfer"),
                frontIntakeGate.instantSetTargetCommand("open"),
                backIntakeGate.instantSetTargetCommand("closed")
        );
        backIntakeAndTransfer = new ParallelCommand(
                backIntake.setPowerCommand("transfer"),
                frontIntake.setPowerCommand("otherSideTransfer"),
                backIntakeGate.instantSetTargetCommand("open"),
                frontIntakeGate.instantSetTargetCommand("closed")
        );
        transfer = new ConditionalCommand(
                new IfThen(
                        ()->shotType==ShotType.NORMAL,
                        frontTransfer
                ),
                new IfThen(
                        ()->shotType==ShotType.MOTIF,
                        new SequentialCommand(
                                new InstantCommand(Inferno::colorSensorRead),
                                new ConditionalCommand(
                                        new IfThen(
                                                ()->shotSequence[1]==ballStorage[2],
                                                backTransfer
                                        ),
                                        new IfThen(
                                                ()->true,
                                                frontTransfer
                                        )
                                )
                        )
                )
        );
    }
}
