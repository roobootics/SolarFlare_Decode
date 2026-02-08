package org.firstinspires.ftc.teamcode.robotconfigs;

import static org.firstinspires.ftc.teamcode.base.Components.getHardwareMap;
import static org.firstinspires.ftc.teamcode.base.Components.telemetryAddData;
import static org.firstinspires.ftc.teamcode.base.Components.timer;
import static org.firstinspires.ftc.teamcode.pedroPathing.Pedro.follower;

import static java.lang.Math.atan2;
import static java.lang.Math.sqrt;

import org.apache.commons.lang3.tuple.Triple;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.pedropathing.paths.PathBuilder;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;


import org.firstinspires.ftc.teamcode.base.Commands.*;
import org.firstinspires.ftc.teamcode.base.Components;
import org.firstinspires.ftc.teamcode.base.Components.*;
import org.firstinspires.ftc.teamcode.pedroPathing.Pedro.*;
import org.firstinspires.ftc.teamcode.presets.PresetControl.*;
import org.firstinspires.ftc.teamcode.vision.Vision;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Objects;
import java.util.function.Supplier;

public class Inferno implements RobotConfig{
    public static BotMotor leftFront = new BotMotor("leftFront", DcMotorSimple.Direction.REVERSE);
    public static BotMotor leftRear = new BotMotor("leftRear", DcMotorSimple.Direction.REVERSE);
    public static BotMotor rightFront = new BotMotor("rightFront", DcMotorSimple.Direction.FORWARD);
    public static BotMotor rightRear = new BotMotor("rightRear", DcMotorSimple.Direction.FORWARD);
    public static SyncedActuators<BotMotor> flywheel;
    public static double targetFlywheelVelocity;
    public static SyncedActuators<BotServo> turretYaw  = new SyncedActuators<>(
            new BotServo("turretYawFront", Servo.Direction.FORWARD, 422, 5, 355,0),
            new BotServo("turretYawBack", Servo.Direction.FORWARD, 422, 5, 355,0)
    );
    private static final double TURRET_YAW_RATIO = 1.0;
    private static final double TURRET_YAW_OFFSET = 0;
    public static SyncedActuators<BotServo> turretPitch = new SyncedActuators<>(
            new BotServo("turretPitchLeft", Servo.Direction.REVERSE, 422, 5, 180, 180),
            new BotServo("turretPitchRight", Servo.Direction.FORWARD, 422, 5, 180, 180)
    );
    private static final double TURRET_PITCH_RATIO = (double) 48/40;
    private static final double TURRET_PITCH_OFFSET = 80.2;
    public static BotMotor frontIntake  = new BotMotor("frontIntake", DcMotorSimple.Direction.FORWARD);
    public static BotMotor backIntake = new BotMotor("backIntake", DcMotorSimple.Direction.FORWARD);
    public static BotServo frontIntakeGate = new BotServo("frontIntakeGate", Servo.Direction.FORWARD, 422, 5, 180, 90.8);
    public static BotServo backIntakeGate = new BotServo("backIntakeGate", Servo.Direction.FORWARD, 422, 5, 180, 99.5);
    public static NormalizedColorSensor[] sensors = new NormalizedColorSensor[3];
    public static BotServo transferGate  = new BotServo("transferGate", Servo.Direction.FORWARD,422,5,270,148.5);
    public static VoltageSensor voltageSensor;
    public static Vision vision;

    public static Command autoGateIntake;
    private static boolean isSpinningUp = true;
    public static final ArrayList<Supplier<Double[]>> colorSensorReads = new ArrayList<>();
    public static double turretOffsetFromAuto = 0;
    public static Color[] ballStorage = new Color[3];
    public static BallPath currentBallPath = BallPath.LOW;
    public static RobotState robotState = RobotState.STOPPED;
    public static ShotType shotType = ShotType.NORMAL;
    public static boolean motifShootAll = true;
    private final static double BALL_SHOT_TIMING = 0.17;
    private final static double TRANSFER_SELECT_DELAY = 0.1;
    private final static double TRANSFER_REBOOST_DELAY = 0.3;
    public static Color[] motif = new Color[]{Color.PURPLE,Color.GREEN,Color.PURPLE};
    public static double classifierBallCount = 0;
    public static Alliance alliance = Alliance.RED;
    public static GamePhase gamePhase = GamePhase.AUTO;
    public static VelocityPID leftVelocityPID = new VelocityPID(false,BotMotor::getVelocity,0.0012, 0.0012, 0.00003);
    public static VelocityPID rightVelocityPID = new VelocityPID(false,(BotMotor motor)->flywheel.get("flywheelLeft").getVelocity(),0.0012, 0.0012 , 0.00003);

    static {
        for (int i=0;i<3;i++){
            int finalI = i;
            colorSensorReads.add(new CachedReader<>(
                    ()->{
                        NormalizedColorSensor sensor = sensors[finalI];
                        NormalizedRGBA normal = sensor.getNormalizedColors();
                        double red = normal.red*256; double green = normal.green*256; double blue = normal.blue*256;
                        double brightness = red+green+blue; red/=brightness; green/=brightness; blue/=brightness;
                        return new Double[]{red,green,blue};
                    },2
            )::cachedRead);
        }
        flywheel = new SyncedActuators<>(
                new BotMotor("flywheelLeft", DcMotorSimple.Direction.REVERSE, 0, 0, new String[]{"VelocityPIDF"},
                        new ControlSystem<>(new String[]{"targetVelocity"}, List.of(() -> targetFlywheelVelocity), leftVelocityPID, new CustomFeedforward(1, ()->targetFlywheelVelocity/(80/0.58 * voltageSensor.getVoltage() + 673)), new Clamp(),
                                new CustomFeedforward(1, ()->{
                                    if (isSpinningUp || ((robotState==RobotState.SHOOTING || robotState==RobotState.INTAKE_FRONT_AND_SHOOT || robotState==RobotState.INTAKE_BACK_AND_SHOOT)&&flywheel.get("flywheelLeft").getVelocity()<targetFlywheelVelocity)) {return 1.0;}
                                    if ((robotState==RobotState.SHOOTING || robotState==RobotState.INTAKE_FRONT_AND_SHOOT || robotState==RobotState.INTAKE_BACK_AND_SHOOT)&&flywheel.get("flywheelLeft").getVelocity()>targetFlywheelVelocity+54){return -0.5;}
                                    else if (flywheel.get("flywheelLeft").getVelocity()>targetFlywheelVelocity+85){return -0.5;}
                                    else {return 0.0;}})
                        )),
                new BotMotor("flywheelRight", DcMotorSimple.Direction.FORWARD, 0, 0, new String[]{"VelocityPIDF"},
                        new ControlSystem<>(new String[]{"targetVelocity"}, List.of(() -> targetFlywheelVelocity), rightVelocityPID, new CustomFeedforward(1, ()->targetFlywheelVelocity/(80/0.58 * voltageSensor.getVoltage() + 673)), new Clamp(),
                                new CustomFeedforward(1, ()->{
                                    if (isSpinningUp || ((robotState==RobotState.SHOOTING || robotState==RobotState.INTAKE_FRONT_AND_SHOOT || robotState==RobotState.INTAKE_BACK_AND_SHOOT)&&flywheel.get("flywheelLeft").getVelocity()<targetFlywheelVelocity)) {return 1.0;}
                                    else if ((robotState==RobotState.SHOOTING || robotState==RobotState.INTAKE_FRONT_AND_SHOOT || robotState==RobotState.INTAKE_BACK_AND_SHOOT)&&flywheel.get("flywheelLeft").getVelocity()>targetFlywheelVelocity+54){return -0.5;}
                                    else if (flywheel.get("flywheelLeft").getVelocity()>targetFlywheelVelocity+85){return -0.5;}
                                    else {return 0.0;}})
                        ))
        );
        turretYaw.call((BotServo servo) -> servo.setTargetBounds(() -> 315*TURRET_YAW_RATIO, () -> 0.0));
        turretPitch.call((BotServo servo) -> servo.setTargetBounds(() -> 173.0, () -> 150-5*TURRET_PITCH_RATIO));
        frontIntake.setKeyPowers(
                new String[]{"intake","otherSideIntake","transfer","otherSideTransfer","stopped","expel","frontDrive","sideSelect"},
                new double[]{1.0,-1.0,1.0,0.9,0,-0.8,0.75,-1.0}
        );
        backIntake.setKeyPowers(
                new String[]{"intake","otherSideIntake","transfer","otherSideTransfer","stopped","expel","frontDrive","sideSelect"},
                new double[]{1.0,-1.0,1.0,0.9,0,-1.0,0.75,-1.0}
        );
        frontIntakeGate.setKeyPositions(new String[]{"open", "closed","push"}, new double[]{180,60.9,55.9});
        backIntakeGate.setKeyPositions(new String[]{"open", "closed","push"}, new double[]{180,75.9,70.9});
        transferGate.setKeyPositions(new String[]{"open","closed"},new double[]{148.5,86.4});
        frontIntake.setZeroPowerFloat();
        backIntake.setZeroPowerFloat();
    }
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
        STOPPED,
        EXPEL,
        INTAKE_FRONT,
        INTAKE_BACK,
        SHOOTING,
        INTAKE_FRONT_AND_SHOOT,
        INTAKE_BACK_AND_SHOOT
    }
    public enum Alliance{
        RED,
        BLUE
    }
    public enum GamePhase{
        AUTO,
        TELEOP
    }
    private static final SequentialCommand frontTransfer  = new SequentialCommand(
            new ParallelCommand(
                    transferGate.instantSetTargetCommand("open"),
                    frontIntake.setPowerCommand("transfer"),
                    backIntake.setPowerCommand("otherSideTransfer"),
                    frontIntakeGate.instantSetTargetCommand("push"),
                    backIntakeGate.instantSetTargetCommand("push")
            ),
            new SleepCommand(TRANSFER_SELECT_DELAY),
            new ParallelCommand(
                    frontIntake.setPowerCommand("transfer"),
                    backIntake.setPowerCommand("sideSelect")
            ),
            new SleepCommand(TRANSFER_REBOOST_DELAY),
            new ParallelCommand(
                    frontIntake.setPowerCommand("transfer"),
                    backIntake.setPowerCommand("transfer")
            )
    );
    private static final SequentialCommand backTransfer = new SequentialCommand(
            new ParallelCommand(
                    transferGate.instantSetTargetCommand("open"),
                    frontIntake.setPowerCommand("otherSideTransfer"),
                    backIntake.setPowerCommand("transfer"),
                    frontIntakeGate.instantSetTargetCommand("push"),
                    backIntakeGate.instantSetTargetCommand("push")
            ),
            new SleepCommand(TRANSFER_SELECT_DELAY),
            new ParallelCommand(
                    backIntake.setPowerCommand("transfer"),
                    frontIntake.setPowerCommand("sideSelect")
            ),
            new SleepCommand(TRANSFER_REBOOST_DELAY),
            new ParallelCommand(
                    frontIntake.setPowerCommand("transfer"),
                    backIntake.setPowerCommand("transfer")
            )
    );
    public static final Command frontIntakeAction = new SequentialCommand(
            new ParallelCommand(
                    transferGate.instantSetTargetCommand("closed"),
                    frontIntakeGate.instantSetTargetCommand("open"),
                    backIntakeGate.instantSetTargetCommand("closed"),
                    frontIntake.setPowerCommand("stopped"),
                    backIntake.setPowerCommand("stopped")
            ),
            new SleepCommand(0.3),
            new ParallelCommand(
                    frontIntake.setPowerCommand("intake"),
                    backIntake.setPowerCommand("otherSideIntake"),
                    new SequentialCommand(
                            new SleepCommand(0.5),
                            new CheckFull(),
                            setState(RobotState.STOPPED)
                    )
            )
    );
    public static final Command backIntakeAction = new SequentialCommand(
            new ParallelCommand(
                    transferGate.instantSetTargetCommand("closed"),
                    backIntakeGate.instantSetTargetCommand("open"),
                    frontIntakeGate.instantSetTargetCommand("closed"),
                    frontIntake.setPowerCommand("stopped"),
                    backIntake.setPowerCommand("stopped")
            ),
            new SleepCommand(0.3),
            new ParallelCommand(
                    backIntake.setPowerCommand("intake"),
                    frontIntake.setPowerCommand("otherSideIntake"),
                    new SequentialCommand(
                            new SleepCommand(0.5),
                            new CheckFull(),
                            setState(RobotState.STOPPED)
                    )
            )
    );
    public static final SequentialCommand stopIntake = new SequentialCommand(
            new ParallelCommand(
                    frontIntakeGate.instantSetTargetCommand("closed"),
                    backIntakeGate.instantSetTargetCommand("closed"),
                    frontIntake.setPowerCommand("frontDrive"),
                    backIntake.setPowerCommand("frontDrive")
            ),
            new SleepCommand(0.5),
            new ParallelCommand(
                    frontIntake.setPowerCommand("stopped"),
                    backIntake.setPowerCommand("stopped"),
                    transferGate.instantSetTargetCommand("open")
            ),
            new ConditionalCommand(new IfThen(()->gamePhase==GamePhase.AUTO,new InstantCommand(Inferno::readBallStorage)))
    );
    public static final ParallelCommand expel = new ParallelCommand(
            frontIntake.setPowerCommand("expel"),
            backIntake.setPowerCommand("expel"),
            frontIntakeGate.instantSetTargetCommand("open"),
            backIntakeGate.instantSetTargetCommand("open"),
            transferGate.instantSetTargetCommand("open")
    );
    public static final ParallelCommand frontIntakeAndTransfer = new ParallelCommand(
            new InstantCommand(() -> shotType = ShotType.NORMAL),
            transferGate.instantSetTargetCommand("open"),
            frontIntake.setPowerCommand("transfer"),
            backIntake.setPowerCommand("transfer"),
            frontIntakeGate.instantSetTargetCommand("open"),
            backIntakeGate.instantSetTargetCommand("closed")
    );
    public static final ParallelCommand backIntakeAndTransfer = new ParallelCommand(
            new InstantCommand(() -> shotType = ShotType.NORMAL),
            transferGate.instantSetTargetCommand("open"),
            frontIntake.setPowerCommand("transfer"),
            backIntake.setPowerCommand("transfer"),
            backIntakeGate.instantSetTargetCommand("open"),
            frontIntakeGate.instantSetTargetCommand("closed")
    );
    public static final Command transfer = new ParallelCommand(
        new InstantCommand(()->ballStorage = new Color[3]),
        new ConditionalCommand(
            new IfThen(
                () -> shotType == ShotType.NORMAL,
                new ParallelCommand(backTransfer,new InstantCommand(()-> currentBallPath = BallPath.LOW))
            ),
            new IfThen(
                () -> shotType == ShotType.MOTIF,
                new SemiSort()
            )
        )
    );
    public static final Command setShooter = new ContinuousCommand(()->{
        double[] targetPoint = getTargetPoint();
        Pose pos = follower.getPose();
        double xPos = pos.getX();
        double yPos = pos.getY();
        double dist = sqrt((targetPoint[0]-xPos)*(targetPoint[0]-xPos) + (targetPoint[1]-yPos)*(targetPoint[1]-yPos));
        targetFlywheelVelocity = VelRegression.regressFormula(dist);
        if (targetFlywheelVelocity<1500) targetFlywheelVelocity = 1500;
        double heading = Math.toDegrees(follower.getHeading());
        double vel = flywheel.get("flywheelLeft").getVelocity();
        double[] turret = new double[]{HoodRegression.regressFormula(dist,vel),Math.toDegrees(atan2(targetPoint[1] - yPos,targetPoint[0] - xPos))};
        if ((turret[1]-heading)<=-225) turret[1] += 360;
        else if ((turret[1]-heading)>225) turret[1] -= 360;
        turretPitch.call((BotServo servo)->servo.setTarget((turret[0]+TURRET_PITCH_OFFSET)*TURRET_PITCH_RATIO));
        turretYaw.call((BotServo servo)->servo.setTarget((180-(turret[1]-heading)+TURRET_YAW_OFFSET)*TURRET_YAW_RATIO));
    });
    public static final Command stopAll = new ParallelCommand(
            frontIntake.setPowerCommand("stopped"),
            backIntake.setPowerCommand("stopped"),
            frontIntakeGate.instantSetTargetCommand("closed"),
            backIntakeGate.instantSetTargetCommand("closed"),
            transferGate.instantSetTargetCommand("open")
    );
    public static final Command loopFSM = new RunResettingLoop(
            new PressCommand(
                new IfThen(()->robotState==RobotState.STOPPED, stopIntake),
                new IfThen(()->robotState==RobotState.EXPEL, expel),
                new IfThen(()->robotState==RobotState.INTAKE_BACK, backIntakeAction),
                new IfThen(()->robotState==RobotState.INTAKE_FRONT, frontIntakeAction),
                new IfThen(()->robotState==RobotState.INTAKE_BACK_AND_SHOOT, backIntakeAndTransfer),
                new IfThen(()->robotState==RobotState.INTAKE_FRONT_AND_SHOOT, frontIntakeAndTransfer),
                new IfThen(()->robotState==RobotState.SHOOTING, transfer),
                new IfThen(()->Objects.isNull(robotState),stopAll)
            ),
        //new InstantCommand(()->{if ((robotState!=RobotState.SHOOTING && robotState!=RobotState.STOPPED) || shotType==ShotType.NORMAL){currentBallPath=BallPath.LOW;}}),
        setShooter
    );
    private static void colorSensorRead(int index){
        double [] greenCenter = new double[]{0.23,0.54,0.23};
        double [] purpleCenter = new double[]{0.4,0.2,0.4};
        double greenTolerance = 0.17;
        double purpleTolerance = 0.17;
        Double[] output = colorSensorReads.get(index).get();
        Color color = null;
        double red = output[0]; double green = output[1]; double blue = output[2];
        if ((red-greenCenter[0])*(red-greenCenter[0]) + (green-greenCenter[1])*(green-greenCenter[1]) + (blue-greenCenter[2])*(blue-greenCenter[2])<=greenTolerance*greenTolerance){
            color = Color.GREEN;
        }
        else if ((red-purpleCenter[0])*(red-purpleCenter[0]) + (green-purpleCenter[1])*(green-purpleCenter[1]) + (blue-purpleCenter[2])*(blue-purpleCenter[2])<=purpleTolerance*purpleTolerance){
            color = Color.PURPLE;
        }
        ballStorage[index] = color;
    }
    public static void readBallStorage(){
        for (int i=0;i<3;i++){
            colorSensorRead(i);
        }
    }
    private static Triple<ArrayList<BallPath>,Integer,Boolean> findMotifShotPlan(boolean shootAll){
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
    public static final Command aprilTagRelocalize = new LambdaCommand(
            ()->{
                Pose pose = vision.getBotPoseMT1();
                if (!Objects.isNull(pose)){
                    follower.setPose(pose);
                    return false;
                } else {
                    return true;
                }
            }
    );
    public static final Command findMotif = new LambdaCommand(
            ()->{
                Integer id = vision.getObeliskID();
                if (!Objects.isNull(id)){
                    if (id == 23){
                        motif = new Color[]{Color.PURPLE,Color.PURPLE,Color.GREEN};
                    } else if (id==22){
                        motif = new Color[]{Color.PURPLE,Color.GREEN,Color.PURPLE};
                    } else if (id==21){
                        motif = new Color[]{Color.GREEN,Color.PURPLE,Color.PURPLE};
                    }
                    return false;
                } else {
                    return true;
                }
            }
    );
    public static void findBalls(){}
    public static void countClassifier(){}
    public static Command setState(RobotState robotState){
        return new InstantCommand(()->Inferno.robotState=robotState);
    }
    public static Command setShotType(ShotType shotType){
        return new InstantCommand(()->Inferno.shotType=shotType);
    }
    public static Command toggleShotType(){
        return new InstantCommand(()->{if (Inferno.shotType==ShotType.MOTIF) shotType=ShotType.NORMAL; else shotType=ShotType.MOTIF;});
    }
    public static Command clearIntegralAtPeak = new SequentialCommand(
            new SleepUntilTrue(()->flywheel.get("flywheelLeft").getVelocity()>=targetFlywheelVelocity-25),
            new InstantCommand(()->{leftVelocityPID.clearIntegral();rightVelocityPID.clearIntegral();isSpinningUp=false;})
    );
    private static abstract class MotifShoot{
        private static ArrayList<BallPath> ballPaths; private static boolean leaveRollersOn; private static int transferDirection;
        public static void getMotifShotPlan(){
            Triple<ArrayList<BallPath>, Integer, Boolean> plan = findMotifShotPlan(motifShootAll);
            ballPaths = plan.getLeft(); transferDirection = plan.getMiddle(); leaveRollersOn = plan.getRight();
        }
        public static class TransferCommand extends CompoundCommand{
            public TransferCommand(){
                setGroup(new ConditionalCommand(
                        new IfThen(()->transferDirection==0, frontTransfer),
                        new IfThen(()->transferDirection==2 || transferDirection==1, backTransfer)
                ));
            }
        }
        public static class HoodCommand extends Command{
            private double startTime;
            @Override
            protected boolean runProcedure() {
                if (isStart()) {
                    startTime = -9999;
                    frontIntakeGate.setTarget(frontIntakeGate.getPos("push"));
                    backIntakeGate.setTarget(backIntakeGate.getPos("push"));
                }
                if (timer.time() - startTime > BALL_SHOT_TIMING && !ballPaths.isEmpty()) {
                    startTime = timer.time();
                    if (!Objects.isNull(ballPaths.get(0))) currentBallPath = ballPaths.get(0); else currentBallPath = ballPaths.get(1);
                    ballPaths.remove(0);
                }
                else if (timer.time() - startTime > BALL_SHOT_TIMING && ballPaths.isEmpty() && !leaveRollersOn){
                    robotState = RobotState.STOPPED;
                }
                return true;
            }
        }
        public static Command getFullMotifCommand(){
            return new ParallelCommand(
                new InstantCommand(MotifShoot::getMotifShotPlan),
                new TransferCommand(),
                new HoodCommand()
            );
        }
    }
    private static class SemiSort extends CompoundCommand{
        private int transferDirection;
        public SemiSort(){
            setGroup(new ParallelCommand(
                new InstantCommand(()->{
                    readBallStorage();
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
                    if (shotSequence[1]==ballStorage[0] && shotSequence[1]!=ballStorage[2]){
                        transferDirection = 0;
                    } else transferDirection = 2;
                }),
                new ConditionalCommand(
                        new IfThen(()->transferDirection==0,frontTransfer),
                        new IfThen(()->transferDirection==2,backTransfer)
                )
            ));
        }
    }
    private static class CheckBallPresent extends Command{
        private final static int COUNT = 3;
        private int counter = 0;
        private final ArrayList<Integer> sensorIndices;
        public CheckBallPresent(Integer...indices){
            sensorIndices = new ArrayList<>(Arrays.asList(indices));
        }
        @Override
        protected boolean runProcedure() {
            if (isStart()) {counter = 0;}
            boolean allFull = true;
            for (Integer i : sensorIndices){
                colorSensorRead(i);
                if (Objects.isNull(ballStorage[i])){
                    allFull = false;
                }
            }
            if (allFull) counter+=1; else counter = 0;
            return counter < COUNT;
        }
    }
    public static class CheckFull extends Command{
        private double startTime;
        private final double timeout;
        private final SequentialCommand frontIntakeCheck = new SequentialCommand(
                new CheckBallPresent(2),
                new CheckBallPresent(1,0)
        );
        private final SequentialCommand backIntakeCheck = new SequentialCommand(
                new CheckBallPresent(0),
                new CheckBallPresent(1,2)
        );
        public CheckFull(double timeout){
            this.timeout = timeout;
        }
        public CheckFull(){
            this(Double.POSITIVE_INFINITY);
        }
        @Override
        protected boolean runProcedure() {
            if (isStart()) {startTime = timer.time(); frontIntakeCheck.reset(); backIntakeCheck.reset();}
            if (robotState==RobotState.INTAKE_FRONT){
                frontIntakeCheck.run();
                return frontIntakeCheck.isBusy() && timer.time()-startTime<timeout;
            } else {
                backIntakeCheck.run();
                return backIntakeCheck.isBusy() && timer.time()-startTime<timeout;
            }
        }
    }
    private abstract static class Fisiks {
        final static double GRAVITY = -386.089;
        final static double FRICTION = 0.5;
        final static double HEIGHT = 10.748;
        final static double TICKS_TO_RAD = 2*Math.PI/28;
        final static double WHEEL_RAD = 1.41732;
        final static double BALL_RAD = 2.5;
        public static double[] runPhysics(BallPath currentBallPath){
            double[] targetPoint = getTargetPoint();
            Pose pos = follower.getPose();
            Vector vel = follower.getVelocity();
            double xPos = pos.getX();
            double yPos = pos.getY();
            double xVel = vel.getXComponent();
            double yVel = vel.getYComponent();
            double currWheelVel = TICKS_TO_RAD*Math.max(-flywheel.get("flywheelLeft").getVelocity(),flywheel.get("flywheelRight").getVelocity());
            double initSpeed = FRICTION*currWheelVel*WHEEL_RAD;
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
            double turretYaw = atan2(targetPoint[1] - shotTime*yVel - yPos,targetPoint[0] - shotTime*xVel - xPos);
            if (Double.isNaN(turretYaw)) turretYaw =  atan2(targetPoint[1] - yPos,targetPoint[0] - xPos); if (Double.isNaN(turretPitch)) turretPitch = Math.toRadians(69.8);
            if (turretYaw>=Math.PI) turretYaw-=2*Math.PI; if (turretPitch<=0) turretPitch+=2*Math.PI;
            telemetryAddData("NaN",Double.isNaN(shotTime));
            return new double[]{Math.toDegrees(turretPitch),Math.toDegrees(turretYaw)};
        }
    }
    public abstract static class VelRegression {
        private static final double M_1 = 7.081273246;
        private static final double B = 1132.298935;
        private static double regressFormula(double dist){
            return M_1*dist+B;
        }
    }
    public abstract static class HoodRegression {
        private static final double A = 0.00002359858;
        private static final double B = -0.00001730411;
        private static final double C = -0.00061919547;
        private static final double D = -0.116957588;
        private static final double E = 0.343627627;
        private static final double F = 153.71021674734098;
        private static double regressFormula(double dist,double vel){
            return F+E*dist+D*vel+C*dist*dist+B*dist*vel+A*vel*vel;
        }
    }
    public static double[] getTargetPoint(){
        if (alliance==Alliance.RED){
            if (follower.getPose().getY()<=108) return new double[]{141.5,141.5,44};
            else return new double[]{141.5,139.5,44};
        } else {
            if (follower.getPose().getY()<=108) return new double[]{2.5,141.5,44};
            else return new double[]{2.5,139.5,44};
        }
    }
    public static class Clamp extends ControlFunc<BotMotor>{
        @Override
        protected void runProcedure() {
            double output = system.getOutput();
            if (output>1) output=1; else if (output<-1) output=-1;
            system.setOutput(output);
        }
    }

    @Override
    public ArrayList<Actuator<?>> getActuators() {
        return new ArrayList<>(Arrays.asList(leftFront, leftRear, rightFront, rightRear, frontIntake, backIntake, frontIntakeGate, backIntakeGate, transferGate,
            flywheel.get("flywheelLeft"), flywheel.get("flywheelRight"), turretYaw.get("turretYawFront"), turretYaw.get("turretYawBack"), turretPitch.get("turretPitchLeft"), turretPitch.get("turretPitchRight")));
    }

    @Override
    public void generalInit() {
        sensors[0] = Components.getHardwareMap().get(NormalizedColorSensor.class, "sensor1");
        sensors[1] = Components.getHardwareMap().get(NormalizedColorSensor.class, "sensor2");
        sensors[2] = Components.getHardwareMap().get(NormalizedColorSensor.class, "sensor3");
        voltageSensor = getHardwareMap().get(VoltageSensor.class,"Control Hub");
        vision = new Vision(Components.getHardwareMap(),Components.getTelemetry());
        targetFlywheelVelocity = 0;
        shotType=ShotType.NORMAL;
        robotState=null;
        currentBallPath = BallPath.LOW;
        motifShootAll = true;
        ballStorage = new Color[3];
        classifierBallCount=0;
        isSpinningUp = true;
        if (alliance == Alliance.RED) autoGateIntake = new ParallelCommand(setState(RobotState.STOPPED),
                new PedroCommand(
                        (PathBuilder b)->{RobotState intakeDirection = RobotState.INTAKE_FRONT; double targetHeading = 0;
                            if (Math.toDegrees(follower.getHeading())>90 || Math.toDegrees(follower.getHeading())<-90) {intakeDirection = RobotState.INTAKE_BACK; targetHeading = 180;}
                            return b.addPath(new BezierLine(follower::getPose,new Pose(128,70)))
                                .setConstantHeadingInterpolation(Math.toRadians(targetHeading))
                                .addPath(new BezierCurve(follower::getPose,new Pose(126,58),new Pose(128,55)))
                                .setLinearHeadingInterpolation(Math.toRadians(targetHeading),Math.toRadians(targetHeading+45))
                                .addParametricCallback(0,setState(intakeDirection)::run);},false
                )
        );
        else autoGateIntake = new ParallelCommand(setState(RobotState.STOPPED),
                new PedroCommand(
                        (PathBuilder b)->{RobotState intakeDirection = RobotState.INTAKE_FRONT; double targetHeading = 180;
                            if (Math.toDegrees(follower.getHeading())<90 && Math.toDegrees(follower.getHeading())>-90) {intakeDirection = RobotState.INTAKE_BACK; targetHeading = 0;}
                            return b.addPath(new BezierLine(follower::getPose,new Pose(16,70)))
                                .setConstantHeadingInterpolation(Math.toRadians(targetHeading))
                                .addPath(new BezierCurve(follower::getPose,new Pose(18,58),new Pose(16,55)))
                                .setLinearHeadingInterpolation(Math.toRadians(targetHeading),Math.toRadians(targetHeading-45))
                                .addParametricCallback(0,setState(intakeDirection)::run);},false
                )
        );
    }
}
