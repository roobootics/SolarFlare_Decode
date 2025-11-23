package org.firstinspires.ftc.teamcode.robotconfigs;

import static org.firstinspires.ftc.teamcode.base.Components.timer;
import static org.firstinspires.ftc.teamcode.pedroPathing.Pedro.follower;

import org.apache.commons.lang3.tuple.Triple;

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

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Objects;

public class Inferno implements RobotConfig{
    public static BotMotor leftFront;
    public static BotMotor leftRear;
    public static BotMotor rightFront;
    public static BotMotor rightRear;
    public static SyncedActuators<BotMotor> flywheel;
    public static double targetFlywheelVelocity;
    public static SyncedActuators<BotServo> turretYaw;
    private static final double TURRET_YAW_RATIO = 1;
    public static SyncedActuators<BotServo> turretPitch;
    private static final double TURRET_PITCH_RATIO = 1;
    public static BotMotor frontIntake;
    public static BotMotor backIntake;
    private static final double TRANSFER_VEL = 2000;
    private static final double OPPOSITE_TRANSFER_VEL = 1500;
    public static BotServo frontIntakeGate;
    public static BotServo backIntakeGate;
    public static RevColorSensorV3[] sensors = new RevColorSensorV3[3];
    public static Limelight3A limelight;
    public static BotServo limelightPitch;
    public enum Color{
        PURPLE,
        GREEN,
    }
    public static Color opposite(Color color){
        if (color == Color.GREEN) return Color.PURPLE; else return Color.GREEN;
    }
    public enum BallPath {
        LOW,
        HIGH
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
    public static BallPath currentBallPath = BallPath.LOW;
    public static RobotState robotState = RobotState.NONE;
    public static ShotType shotType = ShotType.NORMAL;
    private final static double BALL_SHOT_TIMING = 5;
    private final static double SLOWED_BALL_SHOT_TIMING = 5;
    private final static double TRANSFER_SLOWDOWN = 11;
    public static Color[] motif = new Color[3];
    public static double classifierBallCount = 0;
    private static Color colorSensorRead(int index){
        RevColorSensorV3 sensor = sensors[index];
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
        return color;
    }
    private static void readBallStorage(){
        for (int i=0;i<3;i++){
            ballStorage[i] = colorSensorRead(i);
        }
    }
    public static Triple<ArrayList<BallPath>,Boolean,Boolean> findMotifShotPlan(){
        //readBallStorage();
        ArrayList<BallPath> ballPaths = new ArrayList<>();
        Color[] shotSequence = new Color[3];
        if (classifierBallCount%3==0){
            shotSequence = motif;
        }
        else if (classifierBallCount%3==1){
            shotSequence[0] = motif[1];
            shotSequence[1] = motif[2];
            shotSequence[2] = motif[0];
        }
        else{
            shotSequence[0] = motif[2];
            shotSequence[1] = motif[0];
            shotSequence[2] = motif[1];
        }
        int shotLength = 0;
        boolean transferDirection = true;
        boolean leaveRollersOn;
        ArrayList<Color> balls = new ArrayList<>(Arrays.asList(ballStorage));
        for (Color color : shotSequence){
            if (balls.contains(color)){
                shotLength+=1;
                balls.remove(color);
            }
            else{ break;}
        }
        if (shotLength>0 && Objects.isNull(ballStorage[1])){shotLength+=1;}
        leaveRollersOn=!(shotLength==0 || balls.contains(Color.PURPLE)||balls.contains(Color.GREEN));

        ArrayList<Color> shotChecklist = new ArrayList<>(Arrays.asList(shotSequence));
        if (shotLength>=1){
            if (ballStorage[1]==shotSequence[0]){
                ballPaths.add(BallPath.LOW);
                shotChecklist.remove(0);
            } else if (Objects.isNull(ballStorage[1])){
                ballPaths.add(null);
            } else{ ballPaths.add(BallPath.HIGH);}
            if (shotLength>=2){
                transferDirection = !(ballStorage[2]==shotChecklist.get(0));
                ballPaths.add(BallPath.LOW);
                shotChecklist.remove(0);
                if (shotLength==3){
                    int lastBall;
                    if (transferDirection) lastBall=2; else lastBall=0;
                    if (ballStorage[lastBall] == shotChecklist.get(0)){
                        ballPaths.add(BallPath.LOW);
                    } else {ballPaths.add(BallPath.HIGH);}
                }
            }
        }
        return Triple.of(ballPaths,transferDirection,leaveRollersOn);
    }
    public static void aprilTagRelocalize(){}
    public static void findBalls(){}
    public static void countClassifier(){}
    private static void findMotif(){}
    private void calcFlywheelVelocity(){
        targetFlywheelVelocity = 2000;
    }
    private void calcTurretPos(){
        Pose currPose = follower.getPose();
        Pose targetPoint = new Pose(0,0,0);
        double absoluteAngle = Math.toDegrees(Math.atan2(targetPoint.getX()-currPose.getX(),targetPoint.getY()-currPose.getY()));
        double relativeAngle = absoluteAngle - Math.toDegrees(currPose.getHeading());
        turretYaw.call((BotServo servo)->servo.setTarget(relativeAngle*TURRET_YAW_RATIO));
        if (currentBallPath==BallPath.LOW){
            turretPitch.call((BotServo servo)->servo.setTarget(50));
        } else {
            turretPitch.call((BotServo servo)->servo.setTarget(120));
        }
    }
    public static Command setState(RobotState robotState){
        return new InstantCommand(()->Inferno.robotState=robotState);
    }
    public static Command setShotType(ShotType shotType){
        return new InstantCommand(()->Inferno.shotType=shotType);
    }
    public static Command toggleShotType(){
        return new InstantCommand(()->{if (Inferno.shotType==ShotType.MOTIF) shotType=ShotType.NORMAL; else shotType=ShotType.MOTIF;});
    }
    public static Command loopFSM;
    private static ParallelCommand frontTransfer;
    private static ParallelCommand backTransfer;
    public static class MotifShoot extends Command{
        private double startTime;
        private ArrayList<BallPath> ballPaths; private boolean leaveRollersOn; private boolean transferDirection;
        private double currentBallShotTiming = BALL_SHOT_TIMING;
        @Override
        protected boolean runProcedure() {

            if (isStart()) {
                startTime = -9999;
                Triple<ArrayList<BallPath>, Boolean, Boolean> plan = findMotifShotPlan();
                ballPaths = plan.getLeft(); transferDirection = plan.getMiddle(); leaveRollersOn = plan.getRight();
                if (!ballPaths.isEmpty()){
                    if (transferDirection){frontTransfer.reset(); frontTransfer.run();} else{backTransfer.reset(); backTransfer.run();}
                }
            }

            if (timer.time() - startTime > currentBallShotTiming && !ballPaths.isEmpty()) {
                startTime = timer.time();
                if (ballPaths.get(0)!=currentBallPath && !Objects.isNull(ballPaths.get(0))) {
                    if (transferDirection) {
                        //frontIntake.setVelocity(TRANSFER_VEL * TRANSFER_SLOWDOWN);
                        //backIntake.setVelocity(OPPOSITE_TRANSFER_VEL * TRANSFER_SLOWDOWN);
                        frontIntake.setPower(frontIntake.getKeyPower("transfer")*TRANSFER_SLOWDOWN);
                        backIntake.setPower(backIntake.getKeyPower("otherSideTransfer")*TRANSFER_SLOWDOWN);
                    } else {
                        backIntake.setPower(backIntake.getKeyPower("transfer")*TRANSFER_SLOWDOWN);
                        frontIntake.setPower(frontIntake.getKeyPower("otherSideTransfer")*TRANSFER_SLOWDOWN);
                        //backIntake.setVelocity(TRANSFER_VEL * TRANSFER_SLOWDOWN);
                        //frontIntake.setVelocity(OPPOSITE_TRANSFER_VEL * TRANSFER_SLOWDOWN);
                    }
                    currentBallShotTiming = SLOWED_BALL_SHOT_TIMING;
                }
                else{
                    if (transferDirection) {
                        //frontIntake.setVelocity(TRANSFER_VEL);
                        //backIntake.setVelocity(OPPOSITE_TRANSFER_VEL);
                        frontIntake.setPower(frontIntake.getKeyPower("transfer"));
                        backIntake.setPower(backIntake.getKeyPower("otherSideTransfer"));
                    } else {
                        //backIntake.setVelocity(TRANSFER_VEL);
                        //frontIntake.setVelocity(OPPOSITE_TRANSFER_VEL);
                        backIntake.setPower(backIntake.getKeyPower("transfer"));
                        frontIntake.setPower(frontIntake.getKeyPower("otherSideTransfer"));
                    }
                    currentBallShotTiming = BALL_SHOT_TIMING;
                }
                if (!Objects.isNull(ballPaths.get(0))) currentBallPath = ballPaths.get(0); else currentBallPath = ballPaths.get(1);
                ballPaths.remove(0);
            }
            else if (timer.time() - startTime > currentBallShotTiming && ballPaths.isEmpty() && !leaveRollersOn){
                robotState = RobotState.NONE;
            }
            return true;
        }
    }

    @Override
    public void init() {
        //Pedro.createFollower(new Pose(0, 0, 0));
        leftFront = new BotMotor("leftFront", DcMotorSimple.Direction.REVERSE);
        leftRear = new BotMotor("leftRear", DcMotorSimple.Direction.REVERSE);
        rightFront = new BotMotor("rightFront", DcMotorSimple.Direction.FORWARD);
        rightRear = new BotMotor("rightRear", DcMotorSimple.Direction.FORWARD);
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
        backIntake = new BotMotor("backIntake", DcMotorSimple.Direction.FORWARD);
        frontIntake.setKeyPowers(
                new String[]{"intake","otherSideIntake","transfer","otherSideTransfer","stopped","expel"},
                new double[]{1.0,-1.0,1.0,0.7,0.0,-1.0}
        );
        backIntake.setKeyPowers(
                new String[]{"intake","otherSideIntake","transfer","otherSideTransfer","stopped","expel"},
                new double[]{1.0,-1.0,1.0,0.7,0.0,-1.0}
        );
        frontIntakeGate = new BotServo("frontIntakeGate", Servo.Direction.FORWARD, 422, 5, 180, 50);
        backIntakeGate = new BotServo("backIntakeGate", Servo.Direction.FORWARD, 422, 5, 180, 50);
        frontIntakeGate.setKeyPositions(new String[]{"open", "closed"}, new double[]{40,0});
        backIntakeGate.setKeyPositions(new String[]{"open", "closed"}, new double[]{50,10});
        //sensors[0] = Components.getHardwareMap().get(RevColorSensorV3.class, "sensor1");
        //sensors[1] = Components.getHardwareMap().get(RevColorSensorV3.class, "sensor2");
        //sensors[2] = Components.getHardwareMap().get(RevColorSensorV3.class, "sensor3");
        //limelight = Components.getHardwareMap().get(Limelight3A.class, "limelight");
        limelightPitch = new BotServo("limelightPitch", Servo.Direction.FORWARD, 422, 5, 270, 90);
        limelightPitch.setKeyPositions(new String[]{"balls", "apriltag", "obelisk","classifier"}, new double[]{0,0,0,0});
        limelightPitch.setTarget(limelightPitch.getPos("obelisk"));

        frontTransfer = new ParallelCommand(
                //new InstantCommand(()->frontIntake.setVelocity(TRANSFER_VEL)),
                //new InstantCommand(()->backIntake.setVelocity(OPPOSITE_TRANSFER_VEL)),
                frontIntake.setPowerCommand("transfer"),
                backIntake.setPowerCommand("otherSideTransfer"),
                frontIntakeGate.instantSetTargetCommand("open"),
                backIntakeGate.instantSetTargetCommand("open")
        );
        backTransfer = new ParallelCommand(
                //new InstantCommand(()->backIntake.setVelocity(TRANSFER_VEL)),
                //new InstantCommand(()->frontIntake.setVelocity(OPPOSITE_TRANSFER_VEL)),
                backIntake.setPowerCommand("transfer"),
                frontIntake.setPowerCommand("otherSideTransfer"),
                frontIntakeGate.instantSetTargetCommand("open"),
                backIntakeGate.instantSetTargetCommand("open")
        );
        ParallelCommand midTransfer = new ParallelCommand(
                backIntake.setPowerCommand("transfer"),
                frontIntake.setPowerCommand("transfer"),
                frontIntakeGate.instantSetTargetCommand("open"),
                backIntakeGate.instantSetTargetCommand("open")
        );
        ParallelCommand frontIntakeAction = new ParallelCommand(
                frontIntake.setPowerCommand("intake"),
                backIntake.setPowerCommand("otherSideIntake"),
                frontIntakeGate.instantSetTargetCommand("open"),
                backIntakeGate.instantSetTargetCommand("closed")
                /*
                new SequentialCommand(
                        new SleepUntilTrue(()->!Objects.isNull(colorSensorRead(2))),
                        backIntake.setPowerCommand("stopped")
                )
                */
        );
        ParallelCommand backIntakeAction = new ParallelCommand(
                backIntake.setPowerCommand("intake"),
                frontIntake.setPowerCommand("otherSideIntake"),
                backIntakeGate.instantSetTargetCommand("open"),
                frontIntakeGate.instantSetTargetCommand("closed")
                /*
                new SequentialCommand(
                        new SleepUntilTrue(()->!Objects.isNull(colorSensorRead(0))),
                        frontIntake.setPowerCommand("stopped")
                )
                */
        );
        ParallelCommand stopIntake = new ParallelCommand(
                frontIntake.setPowerCommand("stopped"),
                backIntake.setPowerCommand("stopped"),
                frontIntakeGate.instantSetTargetCommand("open"),
                backIntakeGate.instantSetTargetCommand("open")
        );
        ParallelCommand frontIntakeAndTransfer = new ParallelCommand(
                new InstantCommand(() -> shotType = ShotType.NORMAL),
                midTransfer,
                frontIntakeGate.instantSetTargetCommand("open"),
                backIntakeGate.instantSetTargetCommand("closed")
        );
        ParallelCommand backIntakeAndTransfer = new ParallelCommand(
                new InstantCommand(() -> shotType = ShotType.NORMAL),
                midTransfer,
                backIntakeGate.instantSetTargetCommand("open"),
                frontIntakeGate.instantSetTargetCommand("closed")
        );
        ConditionalCommand transfer = new ConditionalCommand(
                new IfThen(
                        () -> shotType == ShotType.NORMAL,
                        new ParallelCommand(midTransfer,new InstantCommand(()-> currentBallPath = BallPath.LOW))
                ),
                new IfThen(
                        () -> shotType == ShotType.MOTIF,
                        new MotifShoot()
                )
        );
        ContinuousCommand physics = new ContinuousCommand(()->{
            //calcTurretPos();
            if (robotState==RobotState.SHOOTING || robotState==RobotState.INTAKE_BACK_AND_SHOOT || robotState==RobotState.INTAKE_FRONT_AND_SHOOT){calcFlywheelVelocity();}
            else {targetFlywheelVelocity=0;}
        });

        loopFSM = new RunResettingLoop(
                /*
                new PressCommand(
                        new IfThen(
                                ()->limelightPitch.getTarget()==limelightPitch.getPos("obelisk"),
                                new InstantCommand(()->limelight.pipelineSwitch(0))
                        ),
                        new IfThen(
                                ()->limelightPitch.getTarget()==limelightPitch.getPos("apriltag"),
                                new InstantCommand(()->limelight.pipelineSwitch(1))
                        ),
                        new IfThen(
                                ()->limelightPitch.getTarget()==limelightPitch.getPos("balls"),
                                new InstantCommand(()->limelight.pipelineSwitch(2))
                        )
                ),
                */
                new PressCommand(
                        new IfThen(()->robotState==RobotState.NONE, stopIntake),
                        new IfThen(()->robotState==RobotState.INTAKE_BACK, backIntakeAction),
                        new IfThen(()->robotState==RobotState.INTAKE_FRONT, frontIntakeAction),
                        new IfThen(()->robotState==RobotState.INTAKE_BACK_AND_SHOOT, backIntakeAndTransfer),
                        new IfThen(()->robotState==RobotState.INTAKE_FRONT_AND_SHOOT, frontIntakeAndTransfer),
                        new IfThen(()->robotState==RobotState.SHOOTING, transfer)
                ),
                new InstantCommand(()->{if (robotState!=RobotState.SHOOTING){currentBallPath=BallPath.LOW;}}),
                physics
        );
        findMotif();
        classifierBallCount=0;
        Components.activateActuatorControl();
    }
    public void reset(){
        shotType=ShotType.NORMAL;
        robotState=RobotState.NONE;
        currentBallPath = BallPath.LOW;
    }
}
