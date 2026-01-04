package org.firstinspires.ftc.teamcode.robotconfigs;

import static org.firstinspires.ftc.teamcode.base.Components.timer;
import static org.firstinspires.ftc.teamcode.pedroPathing.Pedro.follower;

import static java.lang.Math.atan2;
import static java.lang.Math.sqrt;

import org.apache.commons.lang3.tuple.Triple;

import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
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
    public static BotServo transferGate;
    public enum Color{
        PURPLE,
        GREEN,
    }
    public static Color opposite(Color color){
        if (color == Color.GREEN) return Color.PURPLE; else if (color == Color.PURPLE) return Color.GREEN; else return null;
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
    public static boolean motifShootAll = false;
    private final static double BALL_SHOT_TIMING = 5;
    private final static double SLOWED_BALL_SHOT_TIMING = 5;
    private final static double TRANSFER_SLOWDOWN = 0.67;
    public static Color[] motif = new Color[3];
    public static double classifierBallCount = 0;
    private static final double[] targetPoint = new double[]{132,141.5,38.7};

    private static Color colorSensorRead(int index){
        RevColorSensorV3 sensor = sensors[index];
        double [] greenCenter = new double[]{0,255,0};
        double [] purpleCenter = new double[]{255,0,255};
        double greenTolerance = 75;
        double purpleTolerance = 75;
        Color color = null;
        NormalizedRGBA normal = sensor.getNormalizedColors();
        double red = normal.red*255; double green = normal.green*255; double blue = normal.blue*255;
        if ((red-greenCenter[0])*(red-greenCenter[0]) + (green-greenCenter[1])*(green-greenCenter[1]) + (blue-greenCenter[2])*(blue-greenCenter[2])<=greenTolerance*greenTolerance){
            color = Color.GREEN;
        }
        else if ((red-purpleCenter[0])*(red-purpleCenter[0]) + (green-purpleCenter[1])*(green-purpleCenter[1]) + (blue-purpleCenter[2])*(blue-purpleCenter[2])<=purpleTolerance*purpleTolerance){
            color = Color.PURPLE;
        }
        return color;
    }
    public static void readBallStorage(){
        for (int i=0;i<3;i++){
            ballStorage[i] = colorSensorRead(i);
        }
    }
    public static Triple<ArrayList<BallPath>,Integer,Boolean> findMotifShotPlan(boolean shootAll){
        readBallStorage();
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
        int transferDirection = 0;
        boolean leaveRollersOn;
        ArrayList<Color> balls = new ArrayList<>(Arrays.asList(ballStorage));
        for (Color color : shotSequence){
            if (balls.contains(color)){
                shotLength+=1;
                balls.remove(color);
            }
            else{ break;}
        }
        if (shootAll) {
            shotLength=0;
            for (Color color : shotSequence){
                if (Objects.nonNull(color)){shotLength+=1;}
            }
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
                if (ballStorage[0]!=opposite(shotChecklist.get(0)) && ballStorage[2]!=opposite(shotChecklist.get(0))){
                    transferDirection = 1;
                    ballPaths.add(BallPath.LOW);
                    shotChecklist.remove(0);
                }
                else if (shootAll && ballStorage[0]==opposite(shotChecklist.get(0)) && ballStorage[2]==opposite(shotChecklist.get(0))){
                    transferDirection = 1;
                    ballPaths.add(BallPath.HIGH);
                }
                else if (ballStorage[2]==shotChecklist.get(0)){
                    transferDirection = 2;
                    ballPaths.add(BallPath.LOW);
                    shotChecklist.remove(0);
                } else if (ballStorage[0]==shotChecklist.get(0)){
                    ballPaths.add(BallPath.LOW);
                    shotChecklist.remove(0);
                }
                if (shotLength==3){
                    int lastBall;
                    if (transferDirection==0) lastBall=2; else lastBall=0;
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
    private static SequentialCommand frontTransfer;
    private static SequentialCommand backTransfer;
    public static class MotifShoot extends Command{
        private double startTime; private double firstStartTime;
        private ArrayList<BallPath> ballPaths; private boolean leaveRollersOn; private int transferDirection;
        private double currentBallShotTiming = BALL_SHOT_TIMING;
        @Override
        protected boolean runProcedure() {

            if (isStart()) {
                startTime = -9999;
                firstStartTime = timer.time();
                Triple<ArrayList<BallPath>, Integer, Boolean> plan = findMotifShotPlan(motifShootAll);
                ballPaths = plan.getLeft(); transferDirection = plan.getMiddle(); leaveRollersOn = plan.getRight();
                frontIntakeGate.setTarget(frontIntakeGate.getPos("push"));
                backIntakeGate.setTarget(backIntakeGate.getPos("push"));
            }

            if (timer.time() - startTime > currentBallShotTiming && !ballPaths.isEmpty()) {
                startTime = timer.time();
                if (ballPaths.get(0)!=currentBallPath && !Objects.isNull(ballPaths.get(0))) {
                    if (transferDirection==0 || transferDirection == 1) {
                        //frontIntake.setVelocity(TRANSFER_VEL * TRANSFER_SLOWDOWN);
                        //backIntake.setVelocity(OPPOSITE_TRANSFER_VEL * TRANSFER_SLOWDOWN);
                        frontIntake.setPower(frontIntake.getKeyPower("transfer")*TRANSFER_SLOWDOWN);
                        backIntake.setPower(backIntake.getKeyPower("otherSideTransfer")*TRANSFER_SLOWDOWN);
                    } else if (transferDirection==2){
                        backIntake.setPower(backIntake.getKeyPower("transfer")*TRANSFER_SLOWDOWN);
                        frontIntake.setPower(frontIntake.getKeyPower("otherSideTransfer")*TRANSFER_SLOWDOWN);
                        //backIntake.setVelocity(TRANSFER_VEL * TRANSFER_SLOWDOWN);
                        //frontIntake.setVelocity(OPPOSITE_TRANSFER_VEL * TRANSFER_SLOWDOWN);
                    }
                    currentBallShotTiming = SLOWED_BALL_SHOT_TIMING;
                }
                else{
                    if (transferDirection==0 || transferDirection == 1) {
                        frontTransfer.reset(); frontTransfer.run();
                    } else if (transferDirection==2){
                        backTransfer.reset(); backTransfer.run();
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
    private abstract static class Fisiks {
        final static double GRAVITY = -386.089;
        final static double FRICTION = 0.67;
        final static double HEIGHT = 10.748;
        final static double TICKS_TO_RAD = 151;
        final static double WHEEL_RAD = 1.41732;
        final static double BALL_RAD = 2.5;
        public static double[] runPhysics(){
            Pose pos = follower.getPose();
            Vector vel = follower.getVelocity();
            double xPos = pos.getX();
            double yPos = pos.getY();
            double xVel = vel.getXComponent();
            double yVel = vel.getYComponent();
            double currWheelVel = TICKS_TO_RAD*(flywheel.getActuators().get("flywheelLeft").getVelocity() + flywheel.getActuators().get("flywheelRight").getVelocity())/2;
            double initSpeed = FRICTION*currWheelVel*BALL_RAD;
            double xDist = sqrt((targetPoint[0]-xPos)*(targetPoint[0]-xPos) + (targetPoint[1]-yPos)*(targetPoint[1]-yPos));
            double yDist = targetPoint[2] - HEIGHT;
            double TOTAL_RAD = WHEEL_RAD+BALL_RAD;
            double discriminant = sqrt(initSpeed*initSpeed*initSpeed*initSpeed + 2*yDist*GRAVITY*initSpeed*initSpeed + GRAVITY*GRAVITY*(TOTAL_RAD*TOTAL_RAD - (xDist - TOTAL_RAD)*(xDist - TOTAL_RAD)));
            if (currentBallPath == BallPath.LOW) discriminant*=-1;
            double shotTime = sqrt(2*(initSpeed*initSpeed + yDist*GRAVITY + discriminant)/(GRAVITY*GRAVITY));
            double turretPitch = atan2(
                    initSpeed*shotTime*(yDist - 0.5*GRAVITY*shotTime*shotTime) - TOTAL_RAD*(xDist - TOTAL_RAD),
                    initSpeed*shotTime*(xDist - TOTAL_RAD) + TOTAL_RAD*(yDist - 0.5*GRAVITY*shotTime*shotTime)
            );
            double turretYaw = atan2(targetPoint[0] - shotTime*xVel - xPos, targetPoint[1] - shotTime*yVel - yPos);
            if (turretYaw<=0) turretYaw+=2*Math.PI; if (turretPitch<=0) turretPitch+=2*Math.PI;
            return new double[]{Math.toDegrees(turretPitch),Math.toDegrees(turretYaw)};
        }
    }

    @Override
    public void init() {
        Pedro.createFollower(new Pose(0, 0, 0));
        leftFront = new BotMotor("leftFront", DcMotorSimple.Direction.REVERSE);
        leftRear = new BotMotor("leftRear", DcMotorSimple.Direction.REVERSE);
        rightFront = new BotMotor("rightFront", DcMotorSimple.Direction.FORWARD);
        rightRear = new BotMotor("rightRear", DcMotorSimple.Direction.FORWARD);
        flywheel = new SyncedActuators<>(
                new BotMotor("flywheelLeft", DcMotorSimple.Direction.FORWARD, 0, 0, new String[]{"setVelocity","VelocityPID"},
                        new ControlSystem<>(new String[]{"targetVelocity"}, List.of(() -> targetFlywheelVelocity), new SetVelocity()),
                        new ControlSystem<>(new String[]{"targetVelocity"}, List.of(() -> targetFlywheelVelocity), new VelocityPID(0, 0, 0), new BasicFeedforward(0, "targetVelocity"))
                ),
                new BotMotor("flywheelRight", DcMotorSimple.Direction.REVERSE, 0, 0, new String[]{"setVelocity","VelocityPID"},
                        new ControlSystem<>(new String[]{"targetVelocity"}, List.of(() -> targetFlywheelVelocity), new SetVelocity()),
                        new ControlSystem<>(new String[]{"targetVelocity"}, List.of(() -> targetFlywheelVelocity), new VelocityPID(0, 0, 0), new BasicFeedforward(0, "targetVelocity"))
                )
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
                new double[]{1.0,-0.5,1.0,0.61,-0.08,-1.0}
        );
        backIntake.setKeyPowers(
                new String[]{"intake","otherSideIntake","transfer","otherSideTransfer","stopped","expel"},
                new double[]{1.0,-0.5,1.0,0.61,-0.08,-1.0}
        );
        frontIntakeGate = new BotServo("frontIntakeGate", Servo.Direction.FORWARD, 422, 5, 180, 50);
        backIntakeGate = new BotServo("backIntakeGate", Servo.Direction.FORWARD, 422, 5, 180, 50);
        frontIntakeGate.setKeyPositions(new String[]{"open", "closed","push"}, new double[]{99,13.8,0});
        backIntakeGate.setKeyPositions(new String[]{"open", "closed","push"}, new double[]{93.6,8.4,0});
        sensors[0] = Components.getHardwareMap().get(RevColorSensorV3.class, "sensor1");
        sensors[1] = Components.getHardwareMap().get(RevColorSensorV3.class, "sensor2");
        sensors[2] = Components.getHardwareMap().get(RevColorSensorV3.class, "sensor3");
        //limelight = Components.getHardwareMap().get(Limelight3A.class, "limelight");
        transferGate = new BotServo("transferGate", Servo.Direction.FORWARD,422,5,270,90);
        transferGate.setKeyPositions(new String[]{"open","closed"},new double[]{148.5,86.4});
        frontTransfer = new SequentialCommand(
                new ParallelCommand(
                    //new InstantCommand(()->frontIntake.setVelocity(TRANSFER_VEL)),
                    //new InstantCommand(()->backIntake.setVelocity(OPPOSITE_TRANSFER_VEL)),
                    transferGate.instantSetTargetCommand("open"),
                    frontIntake.setPowerCommand("transfer"),
                    backIntake.setPowerCommand("otherSideTransfer"),
                    frontIntakeGate.instantSetTargetCommand("push"),
                    backIntakeGate.instantSetTargetCommand("push")
                )
        );
        backTransfer = new SequentialCommand(
                new ParallelCommand(
                    //new InstantCommand(()->backIntake.setVelocity(TRANSFER_VEL)),
                    //new InstantCommand(()->frontIntake.setVelocity(OPPOSITE_TRANSFER_VEL)),
                    transferGate.instantSetTargetCommand("open"),
                    backIntake.setPowerCommand("transfer"),
                    frontIntake.setPowerCommand("otherSideTransfer"),
                    frontIntakeGate.instantSetTargetCommand("push"),
                    backIntakeGate.instantSetTargetCommand("push")
                )
        );
        ParallelCommand frontIntakeAction = new ParallelCommand(
                transferGate.instantSetTargetCommand("closed"),
                frontIntake.setPowerCommand("intake"),
                backIntake.setPowerCommand("otherSideIntake"),
                frontIntakeGate.instantSetTargetCommand("open"),
                backIntakeGate.instantSetTargetCommand("closed"),
                new SequentialCommand(
                        new SleepUntilTrue(()->!Objects.isNull(colorSensorRead(2))),
                        backIntake.setPowerCommand("stopped"),
                        new SleepUntilTrue(()->!Objects.isNull(colorSensorRead(1))),
                        new SleepUntilTrue(()->!Objects.isNull(colorSensorRead(0))),
                        setState(RobotState.NONE)
                )
        );
        ParallelCommand backIntakeAction = new ParallelCommand(
                transferGate.instantSetTargetCommand("closed"),
                backIntake.setPowerCommand("intake"),
                frontIntake.setPowerCommand("otherSideIntake"),
                backIntakeGate.instantSetTargetCommand("open"),
                frontIntakeGate.instantSetTargetCommand("closed"),
                new SequentialCommand(
                        new SleepUntilTrue(()->!Objects.isNull(colorSensorRead(0))),
                        frontIntake.setPowerCommand("stopped"),
                        new SleepUntilTrue(()->!Objects.isNull(colorSensorRead(1))),
                        new SleepUntilTrue(()->!Objects.isNull(colorSensorRead(2))),
                        setState(RobotState.NONE)
                )
        );
        ParallelCommand stopIntake = new ParallelCommand(
                transferGate.instantSetTargetCommand("open"),
                frontIntake.setPowerCommand("stopped"),
                backIntake.setPowerCommand("stopped"),
                frontIntakeGate.instantSetTargetCommand("closed"),
                backIntakeGate.instantSetTargetCommand("closed"),
                new InstantCommand(()->{if (!findMotifShotPlan(motifShootAll).getLeft().isEmpty()) {currentBallPath=findMotifShotPlan(motifShootAll).getLeft().get(0);} else {currentBallPath=BallPath.LOW;}})
        );
        ParallelCommand frontIntakeAndTransfer = new ParallelCommand(
                new InstantCommand(() -> shotType = ShotType.NORMAL),
                transferGate.instantSetTargetCommand("open"),
                frontTransfer,
                frontIntakeGate.instantSetTargetCommand("open"),
                backIntakeGate.instantSetTargetCommand("closed")
        );
        ParallelCommand backIntakeAndTransfer = new ParallelCommand(
                new InstantCommand(() -> shotType = ShotType.NORMAL),
                transferGate.instantSetTargetCommand("open"),
                backTransfer,
                backIntakeGate.instantSetTargetCommand("open"),
                frontIntakeGate.instantSetTargetCommand("closed")
        );
        ConditionalCommand transfer = new ConditionalCommand(
                new IfThen(
                        () -> shotType == ShotType.NORMAL,
                        new ParallelCommand(frontTransfer,new InstantCommand(()-> currentBallPath = BallPath.LOW))
                ),
                new IfThen(
                        () -> shotType == ShotType.MOTIF,
                        new MotifShoot()
                )
        );
        ContinuousCommand setShooter = new ContinuousCommand(()->{
            //targetFlywheelVelocity = 3000;
            flywheel.call((BotMotor motor)->motor.setPower(1.0));
            if (robotState != RobotState.INTAKE_FRONT && robotState!= RobotState.INTAKE_BACK){
                double heading = Math.toDegrees(follower.getHeading());
                double[] turret = Fisiks.runPhysics();
                turretPitch.command((BotServo servo)->servo.instantSetTargetCommand(turret[0]*TURRET_PITCH_RATIO)).run();
                turretYaw.command((BotServo servo)->servo.instantSetTargetCommand((turret[1] - heading)*TURRET_YAW_RATIO)).run();
            }
        });

        loopFSM = new RunResettingLoop(
                new PressCommand(
                        new IfThen(()->robotState==RobotState.NONE, stopIntake),
                        new IfThen(()->robotState==RobotState.INTAKE_BACK, backIntakeAction),
                        new IfThen(()->robotState==RobotState.INTAKE_FRONT, frontIntakeAction),
                        new IfThen(()->robotState==RobotState.INTAKE_BACK_AND_SHOOT, backIntakeAndTransfer),
                        new IfThen(()->robotState==RobotState.INTAKE_FRONT_AND_SHOOT, frontIntakeAndTransfer),
                        new IfThen(()->robotState==RobotState.SHOOTING, transfer)
                ),
                new InstantCommand(()->{if ((robotState!=RobotState.SHOOTING && robotState!=RobotState.NONE) || shotType==ShotType.NORMAL){currentBallPath=BallPath.LOW;}}),
                flywheel.command((BotMotor motor)->motor.setPowerCommand(1.0))
                //setShooter
        );
        findMotif();
        Components.activateActuatorControl();
        flywheel.call((BotMotor motor) -> motor.switchControl("controlOff"));
    }
    public void reset(){
        shotType=ShotType.NORMAL;
        robotState=RobotState.NONE;
        currentBallPath = BallPath.LOW;
        motifShootAll = false;
        ballStorage = new Color[3];
        classifierBallCount=0;
    }
}
