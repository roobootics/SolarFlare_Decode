package org.firstinspires.ftc.teamcode.base;

import static org.firstinspires.ftc.teamcode.base.Commands.executor;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.base.Commands.Command;
import org.firstinspires.ftc.teamcode.base.Commands.CompoundCommand;
import org.firstinspires.ftc.teamcode.base.Commands.ConditionalCommand;
import org.firstinspires.ftc.teamcode.base.Commands.IfThen;
import org.firstinspires.ftc.teamcode.base.Commands.InstantCommand;
import org.firstinspires.ftc.teamcode.base.Commands.PressCommand;
import org.firstinspires.ftc.teamcode.base.Commands.RunResettingLoop;
import org.firstinspires.ftc.teamcode.base.Commands.SequentialCommand;
import org.firstinspires.ftc.teamcode.base.Commands.SleepUntilTrue;
import org.firstinspires.ftc.teamcode.presets.PresetControl.ServoControl;
import org.firstinspires.ftc.teamcode.presets.TimeBasedLocalizers;

import java.lang.annotation.ElementType;
import java.lang.annotation.Target;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;
import java.util.Collections;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Objects;
import java.util.function.Function;
import java.util.function.Supplier;
import java.util.stream.Collectors;

public abstract class Components {
    private static RobotConfig config;
    private static HardwareMap hardwareMap;
    public static HardwareMap getHardwareMap(){
        return hardwareMap;
    }
    private static Telemetry telemetry;
    private static final LinkedHashMap<String,Object> telemetryOutput=new LinkedHashMap<>();
    private static LinkedHashMap<String,Object> prevTelemetryOutput = new LinkedHashMap<>();
    public static void telemetryAddData(String caption, Object data){
        telemetryOutput.put(caption, data);
    }
    public static void telemetryAddLine(String line){
        telemetryOutput.put(line,null);
    }
    public static void updateTelemetry(){
        if (!prevTelemetryOutput.equals(telemetryOutput)){
            prevTelemetryOutput=new LinkedHashMap<>(telemetryOutput);
            for (String caption: telemetryOutput.keySet()){
                if (Objects.isNull(telemetryOutput.get(caption))){
                    telemetry.addLine(caption);
                }
                else{
                    telemetry.addData(caption,telemetryOutput.get(caption));
                }
            }
            telemetry.update();
        }
        telemetryOutput.clear();
    }
    public static final ElapsedTime timer = new ElapsedTime(); //Central timer used by everything (e.g. sleep  command, motion profile)
    public static final HashMap<String,Actuator<?>> actuators = new HashMap<>(); //Map of all actuators, each accessible through its name
    public static void activateActuatorControl(){
        for (Actuator<?> actuator: actuators.values()){
            actuator.switchControl(actuator.getDefaultControlKey());
        }
    }
    public static void resetMotorEncoders(){
        for (Actuator<?> actuator: actuators.values()){
            if (actuator instanceof BotMotor){
                BotMotor castedActuator=(BotMotor) actuator;
                castedActuator.resetEncoders();
            }
        }
    }
    public interface RobotConfig {
        void init();
    }
    public static class CachedReader<E>{
        //Allows for the optimized reading of values. The return of a read is cached and re-returned every time the read is called, until the cache is cleared so fresh values can be obtained.
        public static final ArrayList<CachedReader<?>> readers = new ArrayList<>(); //Stores all CachedReaders.
        private final Supplier<E> read;
        private int resetCacheCounter = 1;
        private final int resetCacheLoopInterval; //If this is set to n, the cache is reset every nth iteration.
        private E storedReadValue = null;
        public CachedReader(Supplier<E> read, int resetCacheLoopInterval){
            this.read=read;
            this.resetCacheLoopInterval = resetCacheLoopInterval;
            readers.add(this);
        }
        public E cachedRead(){
            if (Objects.isNull(storedReadValue) || resetCacheCounter%resetCacheLoopInterval==0){
                storedReadValue=read.get();
            }
            return storedReadValue;
        }
        public void resetCache(){
            resetCacheCounter=1;
            storedReadValue=null;
        }
        protected static void updateResetAllCaches(){
            for (CachedReader<?> reader : readers){
                reader.resetCacheCounter+=1;
                if (reader.resetCacheCounter> reader.resetCacheLoopInterval){
                    reader.resetCache();
                }
            }
        }
    }
    @Target(ElementType.METHOD)
    public @interface Actuate{} //Used to denote methods that actually move a part, like setPower or setPosition
    public static void initialize(HardwareMap hardwareMap, Telemetry telemetry, RobotConfig config, boolean alwaysReInit){ //Method to initialize hardwareMap and telemetry.
        Components.hardwareMap=hardwareMap;
        Components.telemetry=telemetry;
        timer.reset(); //Static variables are preserved between runs, so timer needs to be reset
        telemetryOutput.clear();
        prevTelemetryOutput.clear();
        executor.clearCommands();
        if (Objects.isNull(Components.config) || alwaysReInit || config.getClass()!=Components.config.getClass()){
            actuators.clear();
            Components.config=config;
            config.init();
        }
    }
    public abstract static class ControlFunction<E extends Actuator<?>>{ //The subclasses of this are methods that are called to control actuators and get them to the target, such as PID or motion profiles. Each function works with a specific type of actuator. Multiple can run at once
        protected E parentActuator; //Each function has access to the actuator it runs on
        private boolean isStart; //Indicates if the control function has just started running
        public boolean isStart(){
            return isStart;
        }
        public void registerToParent(E parentActuator){
            this.parentActuator=parentActuator;
        }
        public void run(){
            runProcedure();
            isStart=false;
        }
        protected abstract void runProcedure(); //This method is where the control function does its job
        public void stopAndReset(){stopProcedure(); isStart=true;} //The function can stop running if the control mode of the actuator is switched
        public void stopProcedure(){} //Takes care of anything that needs to occur when the control function stops
    }
    public static class DcMotorExData{
        private final String name;
        private final DcMotorSimple.Direction direction;
        public DcMotorExData(String name, DcMotorSimple.Direction direction){
            this.name=name;
            this.direction=direction;
        }
        public DcMotorExData(String name){
            this.name=name;
            this.direction=DcMotorSimple.Direction.FORWARD;
        }
        public DcMotorEx getMotor(){
            DcMotorEx motor = Components.getHardwareMap().get(DcMotorEx.class,name);
            motor.setDirection(direction);
            return motor;
        }
    }
    public static class ServoData{
        private final String name;
        private final Servo.Direction direction;
        public ServoData(String name, Servo.Direction direction){
            this.name=name;
            this.direction=direction;
        }
        public ServoData(String name){
            this.name=name;
            this.direction=Servo.Direction.FORWARD;
        }
        public Servo getServo(){
            Servo servo = Components.getHardwareMap().get(Servo.class,name);
            servo.setDirection(direction);
            return servo;
        }
    }
    public static class CRServoData{
        private final String name;
        private final DcMotorSimple.Direction direction;
        public CRServoData(String name, DcMotorSimple.Direction direction){
            this.name=name;
            this.direction=direction;
        }
        public CRServoData(String name){
            this.name=name;
            this.direction=DcMotorSimple.Direction.FORWARD;
        }
        public CRServo getCRServo(){
            CRServo crservo = Components.getHardwareMap().get(CRServo.class,name);
            crservo.setDirection(direction);
            return crservo;
        }
    }
    public abstract static class Actuator<E extends HardwareDevice>{ //Actuators are enhanced hardware classes that have more state and functionality. Each Actuator instance is parametrized with a specific type, like DcMotorEx or Servo/.
        private final String name;
        public final HashMap<String,E> parts = new HashMap<>(); public final String[] partNames; //Since two hardware devices can be synchronized on one mechanism, Actuators can have multiple inner parts, each referenced by its hardwareMap name
        private double target;
        private double instantTarget;
        private boolean newTarget=false; //Set to true when setTarget is called. Set to false after the end of each loop.
        private boolean newActuation=false; //Set to true when a method tagged with @Actuator is called. Set to false after the end of each loop.
        private double offset; //In case a part skips or something, this allows us to offset all the targets we set to compensate
        public final Supplier<Double> maxTargetFunc;
        public final Supplier<Double> minTargetFunc;
        //Max and min targets. They are dynamic functions since the max position for an actuator may not be the same. An in-game extension limit may not apply based on the direction of the actuator, for example.
        private Supplier<Double> maxOffsetFunc = ()->(Double.POSITIVE_INFINITY);
        private Supplier<Double> minOffsetFunc = ()->(Double.NEGATIVE_INFINITY);
        //Max and min offsets. They are dynamic functions since the max position for an actuator may not be the same.
        private final double errorTol; //Error tolerance for when the actuator is commanded to a position
        private final double defaultMovementTimeout; //Default time waited when an actuator is commanded to a position before ending the  command.
        protected boolean actuationStateUnlocked = true; //If set to false, methods tagged with @Actuate should not have an effect; it locks the actuator in whatever power/position state it's in.
        private boolean targetStateUnlocked = true; //If set to false, the actuator's target cannot change.
        private final HashMap<String,Double> keyPositions = new HashMap<>(); //Stores key positions, like 'transferPosition,' etc.
        private final HashMap<String,Supplier<Double>> getCurrentPositions = new HashMap<>(); //Map of methods to get the current positions of each of the actuator's parts. (They may have slightly different positions each)
        private final Runnable resetCurrentPositionCaches;
        protected ControlFuncRegister<?> funcRegister;
        private String currControlFuncKey;
        private String defaultControlKey;
        private boolean timeBasedLocalization = false; //Indicates whether the getCurrentPosition method of the actuator calculates the position based on time as opposed to an encoder, which is important to know.
        public class ControlFuncRegister<T extends Actuator<E>>{ //Registers control functions. Parametrized to the subclass of Actuator that is using it. The functions cannot be stored directly in the actuator because of generic type erasure and generic invariance. This approach is cleaner
            private final HashMap<String, List<ControlFunction<T>>> controlFuncsMap = new HashMap<>(); //Map with lists of control functions paired with names.
            @SafeVarargs
            public ControlFuncRegister(T instance, //The function to find the current position of the actuator accepts one of the actuator's parts
                                       String[] controlFuncKeys, List<ControlFunction<T>>... controlFuncs){

                for (int i=0;i<controlFuncKeys.length;i++) {
                    for (ControlFunction<T> func : controlFuncs[i]) {
                        func.registerToParent(instance);
                    }
                    controlFuncsMap.put(controlFuncKeys[i],controlFuncs[i]);
                }
                controlFuncsMap.put("controlOff",new ArrayList<>()); //Add a control mode without any control functions.
                currControlFuncKey="controlOff";
                if (controlFuncKeys.length>0){
                    defaultControlKey=controlFuncKeys[0];
                }
                else{
                    defaultControlKey="controlOff";
                }
            }
        }
        public Actuator(String actuatorName, List<E> parts, Function<E, Double> getCurrentPosition, int currentPosPollingInterval,
                        Supplier<Double> maxTargetFunc, Supplier<Double> minTargetFunc,
                        double errorTol, double defaultMovementTimeout){
            currentPosPollingInterval=Math.max(0,currentPosPollingInterval);
            this.name=actuatorName;
            this.partNames=new String[parts.size()];
            for (int i=0;i<parts.size();i++){
                partNames[i]=parts.get(i).getDeviceName();
            }
            this.maxTargetFunc = ()->(maxTargetFunc.get()+offset);
            this.minTargetFunc = ()->(minTargetFunc.get()+offset);
            this.defaultMovementTimeout = defaultMovementTimeout;
            this.errorTol=errorTol;
            ArrayList<CachedReader<Double>> readers = new ArrayList<>();
            for (int i=0;i<parts.size();i++){
                this.parts.put(partNames[i],parts.get(i));
                int finalI = i;
                CachedReader<Double> reader = new CachedReader<>(()->(getCurrentPosition.apply(parts.get(finalI))),currentPosPollingInterval);
                readers.add(reader);
                this.getCurrentPositions.put(partNames[i],reader::cachedRead); //The getCurrentPosition function is copied, one for each of the actuator's parts. Position conversion is also applied
            }
            resetCurrentPositionCaches = ()->{
                for (CachedReader<Double> reader:readers){
                    reader.resetCache();
                }
            };
            actuators.put(name,this);
        }
        public String getName(){
            return name;
        }
        public void setOffsetBoundFuncs(Supplier<Double> maxFunc,Supplier<Double> minFunc){
            this.maxOffsetFunc=maxFunc;
            this.minOffsetFunc=minFunc;
        }
        public double getErrorTol(){
            return errorTol;
        }
        public String getCurrControlFuncKey(){
            return currControlFuncKey;
        }
        public String getDefaultControlKey(){
            return defaultControlKey;
        }
        public void setTimeBasedLocalization(boolean timeBasedLocalization){
            this.timeBasedLocalization=timeBasedLocalization;
        }
        public boolean getTimeBasedLocalization(){
            return this.timeBasedLocalization;
        }
        public boolean isNewTarget(){
            return newTarget;
        }
        public void resetNewTarget(){
            newTarget=false;
        }
        public boolean isNewActuation(){
            return newActuation;
        }
        public void resetNewActuation(){
            newActuation=false;
        }
        public void setNewActuation(){
            newActuation=true;
        }
        public void setTarget(double target){
            if (targetStateUnlocked){
                target=target+offset;
                target=Math.max(minTargetFunc.get(),Math.min(target, maxTargetFunc.get()));
                if (target!=this.target) {
                    this.target = target;
                    this.instantTarget = target;
                    newTarget = true;
                }
            }
        }
        public double getTargetMinusOffset(){
            return target-offset;
        }
        public double getTarget(){
            return target;
        }
        public void setInstantTarget(double instantTarget){
            this.instantTarget=Math.max(minTargetFunc.get(),Math.min(instantTarget, maxTargetFunc.get()));
        }
        public double getInstantTarget(){
            return instantTarget;
        }
        public double getCurrentPosition(String name){ //Gets the position of a specific part
            return Objects.requireNonNull(getCurrentPositions.get(name)).get();
        }
        public double getCurrentPosition(){ //Gets the avg position of all synchronized parts
            double avg = 0;
            for (String name : partNames){
                avg+=getCurrentPosition(name);
            }
            return avg/getCurrentPositions.values().size();
        }
        public void setOffset(double offset){
            offset=Math.max(minOffsetFunc.get(),Math.min(maxOffsetFunc.get(),offset));
            if (offset!=this.offset){
                double oldOffset=this.offset;
                this.offset=offset;
                setTarget(target-oldOffset);
            }
        }
        public void runControl(){
            for (ControlFunction<? extends Actuator<E>> func : Objects.requireNonNull(this.funcRegister.controlFuncsMap.get(currControlFuncKey))) {
                func.run();
            }
        }
        public double getPos(String key){ //Returns one of the key positions based on the inputted key
            return Objects.requireNonNull(keyPositions.get(key));
        }
        public void switchControl(String key){
            for (ControlFunction<?> func: Objects.requireNonNull(this.funcRegister.controlFuncsMap.get(currControlFuncKey))){
                func.stopAndReset();
            }
            currControlFuncKey=key;
        }
        public void lockActuationState(){
            actuationStateUnlocked=false;
        }
        public void unlockActuationState(){
            actuationStateUnlocked=true;
        }
        public void lockTargetState(){
            targetStateUnlocked=false;
        }
        public void unlockTargetState(){
            targetStateUnlocked=true;
        }
        protected void resetCurrentPositionCaches(){
            resetCurrentPositionCaches.run();
        }
        public void setKeyPositions(String[] keyPositionKeys, double[] keyPositionValues){
            for (int i=0; i<keyPositionKeys.length; i++){
                keyPositions.put(keyPositionKeys[i],keyPositionValues[i]);
            }
        }
        public E getPart(String partName){
            return Objects.requireNonNull(this.parts.get(partName));
        }
        public Collection<E> getParts(){
            return this.parts.values();
        }
        public String[] getPartNames(){
            return this.partNames;
        }
        public class MoveToTargetCommand extends CompoundCommand { //Command to set the target, then wait until the position of the actuator is a certain distance from the target, or until a set timeout
            public MoveToTargetCommand(Supplier<Double> targetFunc, double timeout){
                group = new SequentialCommand(
                        new InstantCommand(()->setTarget(targetFunc.get())),
                        new SleepUntilTrue(
                                ()->(Math.abs(getCurrentPosition()-target)<errorTol),
                                timeout
                        )
                );
            }
            public MoveToTargetCommand(double target, double timeout){
                this(()->(target), timeout);
            }
            public MoveToTargetCommand(Supplier<Double> targetFunc){
                this(targetFunc, defaultMovementTimeout);
            }
            public MoveToTargetCommand(double target){
                this(()->(target), defaultMovementTimeout);
            }
        }
        public class SetOffsetCommand extends CompoundCommand { //Command to set the offset
            public SetOffsetCommand(Supplier<Double> offsetFunc){
                group = new InstantCommand(()-> setOffset(offsetFunc.get()));
            }
            public SetOffsetCommand(double offset){
                this(()->(offset));
            }
        }
        public InstantCommand instantSetTargetCommand(double target){
            return new InstantCommand(()->setTarget(target));
        }
        public InstantCommand instantSetTargetCommand(Supplier<Double> targetFunc){
            return new InstantCommand(()->setTarget(targetFunc.get()));
        }
        public InstantCommand instantSetTargetCommand(String position){
            return new InstantCommand(()->setTarget(getPos(position)));
        }
        public MoveToTargetCommand moveToTargetCommand(double target){
            return new MoveToTargetCommand(target);
        }
        public MoveToTargetCommand moveToTargetCommand(Supplier<Double> targetFunc){
            return new MoveToTargetCommand(targetFunc);
        }
        public MoveToTargetCommand moveToTargetCommand(double target, double timeout){
            return new MoveToTargetCommand(target,timeout);
        }
        public MoveToTargetCommand moveToTargetCommand(Supplier<Double> targetFunc, double timeout){
            return new MoveToTargetCommand(targetFunc);
        }
        public MoveToTargetCommand moveToTargetCommand(String position){
            return new MoveToTargetCommand(getPos(position));
        }
        public MoveToTargetCommand moveToTargetCommand(String position,double timeout){
            return new MoveToTargetCommand(getPos(position),timeout);
        }
        public MoveToTargetCommand toggleTargetCommand(double target1, double target2){
            return moveToTargetCommand(()->{
                if (getTargetMinusOffset()==target1) return target2; else if (getTargetMinusOffset()==target2) return target1; else return getTargetMinusOffset();
            });
        }
        public MoveToTargetCommand upwardFSMTargetCommand(double...targets){
            Arrays.sort(targets);
            return moveToTargetCommand(()->{
                for (double target: targets){
                    if (getTargetMinusOffset()<target){
                        return target;
                    }
                }
                return getTargetMinusOffset();
            });
        }
        public MoveToTargetCommand downwardFSMTargetCommand(double...targets){
            Arrays.sort(targets);
            return moveToTargetCommand(()->{
                for (int i = targets.length-1; i>=0; i--){
                    if (getTargetMinusOffset()>targets[i]){
                        return targets[i];
                    }
                }
                return getTargetMinusOffset();
            });
        }
        public SetOffsetCommand setOffsetCommand(double offset){
            return new SetOffsetCommand(offset);
        }
        public SetOffsetCommand setOffsetCommand(Supplier<Double> offsetFunc){
            return new SetOffsetCommand(offsetFunc);
        }
        public RunResettingLoop triggeredMoveToTargetCommand(Supplier<Boolean> condition, double target){
            return new RunResettingLoop(new PressCommand(new IfThen(condition, new MoveToTargetCommand(target))));
        }
        public RunResettingLoop triggeredMoveToTargetCommand(Supplier<Boolean> condition, Supplier<Double> targetFunc) {
            return new RunResettingLoop(new PressCommand(new IfThen(condition, moveToTargetCommand(targetFunc))));
        }
        public RunResettingLoop triggeredMoveToTargetCommand(Supplier<Boolean> condition, double target, double timeout){
            return new RunResettingLoop(new PressCommand(new IfThen(condition, moveToTargetCommand(target,timeout))));
        }
        public RunResettingLoop triggeredMoveToTargetCommand(Supplier<Boolean> condition, Supplier<Double> targetFunc, double timeout){
            return new RunResettingLoop(new PressCommand(new IfThen(condition, moveToTargetCommand(targetFunc,timeout))));
        }
        public RunResettingLoop triggeredMoveToTargetCommand(Supplier<Boolean> condition, String position){
            return new RunResettingLoop(new PressCommand(new IfThen(condition, moveToTargetCommand(position))));
        }
        public RunResettingLoop triggeredMoveToTargetCommand(Supplier<Boolean> condition, String position,double timeout){
            return new RunResettingLoop(new PressCommand(new IfThen(condition, moveToTargetCommand(position,timeout))));
        }
        public RunResettingLoop triggeredToggleTargetCommand(Supplier<Boolean> condition, double target1, double target2){
            return new RunResettingLoop(new PressCommand(new IfThen(condition, toggleTargetCommand(target1,target2))));
        }
        public RunResettingLoop triggeredDynamicTargetCommand(Supplier<Boolean> upCondition, Supplier<Boolean> downCondition, double change){
            return new RunResettingLoop(new ConditionalCommand(new IfThen(upCondition, moveToTargetCommand(()->(getTargetMinusOffset()+change))),new IfThen(downCondition, moveToTargetCommand(()->(getTargetMinusOffset()-change)))));
        }
        public RunResettingLoop triggeredFSMTargetCommand(Supplier<Boolean> upCondition, Supplier<Boolean> downCondition, double...targets){
            return new RunResettingLoop(new PressCommand(new IfThen(upCondition, upwardFSMTargetCommand(targets)),new IfThen(downCondition, downwardFSMTargetCommand(targets))));
        }
        public RunResettingLoop triggeredSetOffsetCommand(Supplier<Boolean> condition, double offset){
            return new RunResettingLoop(new PressCommand(new IfThen(condition, new SetOffsetCommand(offset))));
        }
        public RunResettingLoop triggeredDynamicOffsetCommand(Supplier<Boolean> upCondition, Supplier<Boolean> downCondition, double offsetChange){
            return new RunResettingLoop(new ConditionalCommand(new IfThen(upCondition, setOffsetCommand(()->(offset+offsetChange))),new IfThen(downCondition, setOffsetCommand(()->(offset-offsetChange)))));
        }
        public InstantCommand switchControlCommand(String controlKey){
            return new InstantCommand(()->this.switchControl(controlKey));
        }
        public InstantCommand lockActuationCommand(){
            return new InstantCommand(this::lockActuationState);
        }
        public InstantCommand unlockActuationCommand(){
            return new InstantCommand(this::unlockActuationState);
        }
        public InstantCommand lockTargetCommand(){
            return new InstantCommand(this::lockTargetState);
        }
        public InstantCommand unlockTargetCommand(){
            return new InstantCommand(this::unlockTargetState);
        }
    }
    //Each of the subclasses of Actuator will have some generic constructors and some constructors where information is preset.
    public abstract static class CRActuator<E extends DcMotorSimple> extends Actuator<E>{ //Type of Actuator that works for continuous rotation parts, like DcMotorEx and CRServo
        private final Supplier<Double> maxPowerFunc;
        private final Supplier<Double> minPowerFunc;
        //Max and min power boundaries
        private final HashMap<String,Double> keyPowers = new HashMap<>(); //Stores key powers, like 'intakePower,' etc.
        public CRActuator(String name, List<E> parts, Function<E, Double> getCurrentPosition, int pollingRate, Supplier<Double> maxTargetFunc, Supplier<Double> minTargetFunc, Supplier<Double> maxPowerFunc, Supplier<Double> minPowerFunc, double errorTol, double defaultTimeout) {
            super(name, parts, getCurrentPosition, pollingRate, maxTargetFunc, minTargetFunc, errorTol, defaultTimeout);
            this.maxPowerFunc=maxPowerFunc;
            this.minPowerFunc=minPowerFunc;
            this.setTarget(0);
        }
        public CRActuator(String name, List<E> parts, Function<E, Double> getCurrentPosition, int pollingRate, Supplier<Double> maxTargetFunc, Supplier<Double> minTargetFunc, double errorTol, double defaultTimeout) {
            this(name,parts,getCurrentPosition,pollingRate,maxTargetFunc,minTargetFunc,()->(1.0),()->(-1.0),errorTol,defaultTimeout);
        }
        public CRActuator(String name, List<E> parts, Supplier<Double> maxPowerFunc, Supplier<Double> minPowerFunc) { //For CRActuators that don't set targets and only use setPower, like drivetrain motors.
            this(name,parts,(E e)->(0.0), 1,()->(Double.POSITIVE_INFINITY),()->(Double.NEGATIVE_INFINITY),()->(1.0),()->(-1.0),0,0);
        }
        public double getKeyPower(String key){
            return Objects.requireNonNull(keyPowers.get(key));
        }
        public void setKeyPowers(String[] keyPowerKeys, double[] keyPowerValues){
            for (int i=0; i<keyPowerKeys.length; i++){
                keyPowers.put(keyPowerKeys[i],keyPowerValues[i]);
            }
        }
        @Actuate
        public void setPower(double power, String name){ //Sets power to a specific part
            if (actuationStateUnlocked){
                power=Math.max(Math.min(power, maxPowerFunc.get()), minPowerFunc.get());
                E part = parts.get(name);
                assert part != null;
                if (Math.abs(power-part.getPower())>0.03) {
                    part.setPower(power);
                    if (getTimeBasedLocalization()){ //If current position is calculated by time, it needs to be updated everytime the actuator moves
                        resetCurrentPositionCaches();
                        getCurrentPosition(name);
                    }
                    setNewActuation();
                }
            }
        }
        @Actuate
        public void setPower(double power){ //Sets power to all synchronized parts at once
            if (actuationStateUnlocked) {
                for (String name:partNames) {
                    setPower(power,name);
                }
            }
        }
        public double getPower(String name){
            return getPart(name).getPower();
        }
        public double getPower(){
            double avg=0;
            for (String name:partNames){
                avg+=getPower(name);
            }
            return avg/partNames.length;
        }
        public class SetPowerCommand extends InstantCommand{ //Command to set the power of all synchronized parts
            public SetPowerCommand(Supplier<Double> powerFunc) {
                super(()-> setPower(powerFunc.get()));
            }
            public SetPowerCommand(double power) {
                super(()-> setPower(power));
            }
        }
        public SetPowerCommand setPowerCommand(Supplier<Double> powerFunc){
            return new SetPowerCommand(powerFunc);
        }
        public SetPowerCommand setPowerCommand(double power){
            return new SetPowerCommand(power);
        }
        public SetPowerCommand setPowerCommand(String key){
            return new SetPowerCommand(getKeyPower(key));
        }
        public SetPowerCommand togglePowerCommand(double power1, double power2){
            return new SetPowerCommand(()->{
                if (getPart(partNames[0]).getPower()==power1) return power2; else if (getPart(partNames[0]).getPower()==power2) return power1; else return getPart(partNames[0]).getPower();
            });
        }
        public SetPowerCommand upwardFSMPowerCommand(double...powersList){
            Arrays.sort(powersList);
            return setPowerCommand(()->{
                for (double power: powersList){
                    if (getPart(partNames[0]).getPower()<power){
                        return power;
                    }
                }
                return getPart(partNames[0]).getPower();
            });
        }
        public SetPowerCommand downwardFSMPowerCommand(double...powersList){
            Arrays.sort(powersList);
            return setPowerCommand(()->{
                for (int i = powersList.length-1; i>=0; i--){
                    if (getPart(partNames[0]).getPower()>powersList[i]){
                        return powersList[i];
                    }
                }
                return getPart(partNames[0]).getPower();
            });
        }
        public RunResettingLoop triggeredSetPowerCommand(Supplier<Boolean> condition, Supplier<Double> powerFunc){
            return new RunResettingLoop(new PressCommand(new IfThen(condition, new SetPowerCommand(powerFunc))));
        }
        public RunResettingLoop triggeredSetPowerCommand(Supplier<Boolean> condition, String key){
            return new RunResettingLoop(new PressCommand(new IfThen(condition, new SetPowerCommand(getKeyPower(key)))));
        }
        public RunResettingLoop triggeredSetPowerCommand(Supplier<Boolean> condition, double power){
            return new RunResettingLoop(new PressCommand(new IfThen(condition, new SetPowerCommand(power))));
        }
        public RunResettingLoop triggeredDynamicPowerCommand(Supplier<Boolean> upCondition, Supplier<Boolean> downCondition, double change){
            return new RunResettingLoop(new ConditionalCommand(new IfThen(upCondition, setPowerCommand(()->(getPart(partNames[0]).getPower()+change))),new IfThen(downCondition, setPowerCommand(()->(getPart(partNames[0]).getPower()-change)))));
        }
        public RunResettingLoop triggeredTogglePowerCommand(Supplier<Boolean> condition, double power1, double power2){
            return new RunResettingLoop(new PressCommand(new IfThen(condition, togglePowerCommand(power1,power2))));
        }
        public RunResettingLoop triggeredFSMPowerCommand(Supplier<Boolean> upCondition, Supplier<Boolean> downCondition, double...powers){
            return new RunResettingLoop(new PressCommand(
                    new IfThen(upCondition, upwardFSMPowerCommand(powers)),
                    new IfThen(downCondition, downwardFSMPowerCommand(powers))
            ));
        }
    }
    //Each of the bottom-level subclass constructors will accept getCurrentPosition functions and control functions, since those cater to a specific subclass.
    public static class BotMotor extends CRActuator<DcMotorEx> {
        private boolean isStallResetting;
        private final HashMap<String, Supplier<Double>> velocityReaders = new HashMap<>();
        private final HashMap<String, Supplier<Double>> currentReaders = new HashMap<>();
        private final HashMap<String,Double> keyVelocities = new HashMap<>(); //Stores key velocities, like 'intakeVelocity,' etc.
        private final Supplier<Double> maxVelocityFunc;
        private final Supplier<Double> minVelocityFunc;
        @SafeVarargs
        public BotMotor(String name, List<DcMotorExData> motors, Function<DcMotorEx, Double> getCurrentPosition, int currentPosPollingInterval, Supplier<Double> maxTargetFunc, Supplier<Double> minTargetFunc, Supplier<Double> maxPowerFunc, Supplier<Double> minPowerFunc, Supplier<Double> maxVelocityFunc, Supplier<Double> minVelocityFunc, double errorTol, double defaultTimeout, String[] controlFuncKeys, List<ControlFunction<BotMotor>>... controlFuncs) {
            super(name, motors.stream().map(DcMotorExData::getMotor).collect(Collectors.toList()),getCurrentPosition, currentPosPollingInterval, maxTargetFunc, minTargetFunc, maxPowerFunc, minPowerFunc, errorTol, defaultTimeout);
            this.minVelocityFunc=minVelocityFunc;
            this.maxVelocityFunc=maxVelocityFunc;
            for (String partName : getPartNames()) {
                velocityReaders.put(partName, new CachedReader<>(Objects.requireNonNull(parts.get(partName))::getVelocity, 1)::cachedRead);
                currentReaders.put(partName, new CachedReader<>(() -> Objects.requireNonNull(parts.get(partName)).getCurrent(CurrentUnit.AMPS), 3)::cachedRead);
            }
            for (DcMotorEx part : parts.values()) {
                part.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                part.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
            this.funcRegister = new ControlFuncRegister<BotMotor>(this, controlFuncKeys, controlFuncs);
        }

        @SafeVarargs
        public BotMotor(String name, List<DcMotorExData> motors, Supplier<Double> maxTargetFunc, Supplier<Double> minTargetFunc, Supplier<Double> maxPowerFunc, Supplier<Double> minPowerFunc, Supplier<Double> maxVelocityFunc, Supplier<Double> minVelocityFunc, double errorTol, double defaultTimeout, String[] controlFuncKeys, List<ControlFunction<BotMotor>>... controlFuncs) {
            this(name, motors, (DcMotorEx motor) -> ((double) motor.getCurrentPosition()), 1, maxTargetFunc, minTargetFunc, maxPowerFunc, minPowerFunc, maxVelocityFunc, minVelocityFunc, errorTol, defaultTimeout,controlFuncKeys,controlFuncs);
        }

        @SafeVarargs
        public BotMotor(String name, List<DcMotorExData> motors, Supplier<Double> maxTargetFunc, Supplier<Double> minTargetFunc, double errorTol, double defaultTimeout, String[] controlFuncKeys, List<ControlFunction<BotMotor>>... controlFuncs) {
            this(name, motors, (DcMotorEx motor) -> ((double) motor.getCurrentPosition()), 1, maxTargetFunc, minTargetFunc, ()->(1.0), ()->-1.0, ()->Double.POSITIVE_INFINITY, ()->Double.NEGATIVE_INFINITY, errorTol, defaultTimeout,controlFuncKeys,controlFuncs);
        }

        public BotMotor(String name, List<DcMotorExData> motors, Supplier<Double> maxPowerFunc, Supplier<Double> minPowerFunc, Supplier<Double> maxVelocityFunc, Supplier<Double> minVelocityFunc) {
            this(name, motors, ()->(1.0), ()->-1.0, maxPowerFunc, minPowerFunc, maxVelocityFunc, minVelocityFunc, 0.0, 0.0, new String[]{});
        }

        public double getKeyVelocity(String key){
            return Objects.requireNonNull(keyVelocities.get(key));
        }
        public void setKeyVelocities(String[] keyVelocityKeys, double[] keyVelocityValues){
            for (int i=0; i<keyVelocityKeys.length; i++){
                keyVelocities.put(keyVelocityKeys[i],keyVelocityValues[i]);
            }
        }

        public double getVelocity(String name) {
            return Objects.requireNonNull(velocityReaders.get(name)).get();
        }

        public double getVelocity() { //Returns avg velocity of all parts
            double avg = 0;
            for (String name : partNames) {
                avg += getVelocity(name);
            }
            return avg / parts.size();
        }

        public void resetEncoders() {
            for (DcMotorEx part : parts.values()) {
                part.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                part.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
            resetCurrentPositionCaches();
        }

        public void setZeroPowerFloat() {
            for (DcMotorEx part : parts.values()) {
                part.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            }
        }

        public void setZeroPowerBrake() {
            for (DcMotorEx part : parts.values()) {
                part.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }
        }

        public double getCurrentAmps(String partName) {
            return Objects.requireNonNull(currentReaders.get(partName)).get();
        }

        public double getCurrentAmps() {
            double maxCurrent = 0;
            for (String partName : getPartNames()) {
                double current = getCurrentAmps(partName);
                if (current > maxCurrent) {
                    maxCurrent = current;
                }
            }
            return maxCurrent;
        }

        public boolean isStallResetting() {
            return isStallResetting;
        }

        public void setMode(DcMotorEx.RunMode mode) {
            for (DcMotorEx part : parts.values()) {
                part.setMode(mode);
            }
        }

        public class StallResetCommand extends Command { //Stall resets encoders, and offsets the position if you want to reset at a non-zero position.
            double resetPosition;
            double stallVolts;

            public StallResetCommand(double resetPosition, double stallVolts) {
                this.resetPosition = resetPosition;
                this.stallVolts = stallVolts;
            }

            @Override
            protected boolean runProcedure() {
                if (isStart()) {
                    isStallResetting = true;
                    setPower(-0.2);
                }
                if (getCurrentAmps() > stallVolts) {
                    setPower(0);
                    setOffset(getCurrentPosition() - resetPosition);
                    setTarget(resetPosition);
                    isStallResetting = false;
                }
                return isStallResetting;
            }
        }

        public StallResetCommand stallResetCommand(double resetPosition, double stallVolts) {
            return new StallResetCommand(resetPosition, stallVolts);
        }

        public PressCommand triggeredStallResetCommand(Supplier<Boolean> condition, double resetPosition, double stallVolts) {
            return new PressCommand(new IfThen(condition, stallResetCommand(resetPosition, stallVolts)));
        }
        public class SetPowerForDistance extends CompoundCommand{ //Makes the motor set a power until it travels a certain distance.
            private double startPosition;
            public SetPowerForDistance(double power, double distance){
                setGroup(new SequentialCommand(
                        new InstantCommand(()->startPosition=getCurrentPosition()),
                        new Commands.ParallelCommand(
                                setPowerCommand(power)
                        ),
                        new SleepUntilTrue(()->(getCurrentPosition()-startPosition)>50),
                        new Commands.ParallelCommand(
                                setPowerCommand(0)
                        )
                ));
            }
        }
        public SetPowerForDistance setPowerForDistance(double power, double distance){
            return new SetPowerForDistance(power,distance);
        }
        @Actuate
        public void setVelocity(double velocity, String name){ //Sets power to a specific part
            if (actuationStateUnlocked){
                velocity=Math.max(minVelocityFunc.get(),Math.min(velocity, maxVelocityFunc.get()));
                if (Math.abs(velocity-getVelocity(name))>1) {
                    getPart(name).setVelocity(velocity);
                    setNewActuation();
                }
            }
        }
        @Actuate
        public void setVelocity(double velocity){ //Sets power to all synchronized parts at once
            if (actuationStateUnlocked) {
                for (String name:partNames) {
                    setVelocity(velocity,name);
                }
            }
        }
        public class SetVelocityCommand extends InstantCommand{ //Command to set the power of all synchronized parts
            public SetVelocityCommand(Supplier<Double> velFunc) {
                super(()-> setVelocity(velFunc.get()));
            }
            public SetVelocityCommand(double velocity) {
                super(()-> setVelocity(velocity));
            }
        }
        public SetVelocityCommand setVelocityCommand(Supplier<Double> velocityFunc){
            return new SetVelocityCommand(velocityFunc);
        }
        public SetVelocityCommand setVelocityCommand(double velocity){
            return new SetVelocityCommand(velocity);
        }
        public SetVelocityCommand setVelocityCommand(String key){
            return new SetVelocityCommand(getKeyVelocity(key));
        }
        public SetVelocityCommand toggleVelocityCommand(double velocity1, double velocity2){
            return new SetVelocityCommand(()->{
                if (getPart(partNames[0]).getVelocity()==velocity1) return velocity2; else if (getPart(partNames[0]).getVelocity()==velocity2) return velocity1; else return getPart(partNames[0]).getVelocity();
            });
        }
        public RunResettingLoop triggeredSetVelocityCommand(Supplier<Boolean> condition, Supplier<Double> velocityFunc){
            return new RunResettingLoop(new PressCommand(new IfThen(condition, new SetVelocityCommand(velocityFunc))));
        }
        public RunResettingLoop triggeredSetVelocityCommand(Supplier<Boolean> condition, double velocity){
            return new RunResettingLoop(new PressCommand(new IfThen(condition, new SetVelocityCommand(velocity))));
        }
        public RunResettingLoop triggeredSetVelocityCommand(Supplier<Boolean> condition, String key){
            return new RunResettingLoop(new PressCommand(new IfThen(condition, new SetVelocityCommand(getKeyVelocity(key)))));
        }
        public RunResettingLoop triggeredDynamicVelocityCommand(Supplier<Boolean> upCondition, Supplier<Boolean> downCondition, double change){
            return new RunResettingLoop(new ConditionalCommand(new IfThen(upCondition, setVelocityCommand(()->(getPart(partNames[0]).getVelocity()+change))),new IfThen(downCondition, setPowerCommand(()->(getPart(partNames[0]).getVelocity()-change)))));
        }
        public RunResettingLoop triggeredToggleVelocityCommand(Supplier<Boolean> condition, double velocity1, double velocity2){
            return new RunResettingLoop(new PressCommand(new IfThen(condition, toggleVelocityCommand(velocity1,velocity2))));
        }
    }
    public static class BotServo extends Actuator<Servo>{
        private double currCommandedPos;
        private boolean ignoreSetPosCaching = false;
        private final double range;
        private Function<Double, Double> setPositionConversion;
        @SafeVarargs
        public BotServo(String name, List<ServoData> servos, Function<Servo, Double> getCurrentPosition, int currentPosPollingInterval, Supplier<Double> maxTargetFunc, Supplier<Double> minTargetFunc, double errorTol, double defaultTimeout, double range, //Degree range that servo is programmed to
                        double initialTarget, String[] controlFuncKeys, List<ControlFunction<BotServo>>... controlFuncs) {
            super(name, servos.stream().map(ServoData::getServo).collect(Collectors.toList()), getCurrentPosition, currentPosPollingInterval, maxTargetFunc, minTargetFunc, errorTol, defaultTimeout);
            setTarget(initialTarget);
            this.range=range;
            this.setPositionConversion=(Double pos)->pos/range;
            this.funcRegister=new ControlFuncRegister<BotServo>(this,controlFuncKeys, controlFuncs);
        }
        public BotServo(String name, List<ServoData> servos, Supplier<Double> maxTargetFunc, Supplier<Double> minTargetFunc, double servoSpeedDPS, double defaultTimeout, double range, double initialTarget) {
            this(name,servos,new TimeBasedLocalizers.ServoTimeBasedLocalizer(servoSpeedDPS/range,initialTarget,range)::getCurrentPosition,1,maxTargetFunc,minTargetFunc,1.5,defaultTimeout,range, initialTarget, new String[]{"setPos"}, new ArrayList<>(Collections.singleton(new ServoControl())));
            setTimeBasedLocalization(true);
        }
        public void setPositionConversion(Function<Double,Double> setPositionConversion){
            this.setPositionConversion=setPositionConversion;
        }
        @Actuate
        public void setPosition(double position){ //Accepts position in degrees
            position=Math.max(minTargetFunc.get(),Math.min(position, maxTargetFunc.get()));
            if (actuationStateUnlocked && (Math.abs(currCommandedPos-position)>0.07||ignoreSetPosCaching)){
                currCommandedPos=position;
                for (Servo part:parts.values()){part.setPosition(setPositionConversion.apply(position));}
                if (getTimeBasedLocalization()){
                    resetCurrentPositionCaches();
                    getCurrentPosition();
                }
                setNewActuation();
            }
        }
        public double getPosition(){
            return currCommandedPos;
        }
        public double getRange(){return range;}
        public boolean isIgnoreSetPosCaching(){
            return ignoreSetPosCaching;
        }
        public void setIgnoreSetPosCaching(boolean bool){
            ignoreSetPosCaching=bool;
        }
        public InstantCommand toggleIgnoreSetPosCaching(){
            return new InstantCommand(()->setIgnoreSetPosCaching(!isIgnoreSetPosCaching()));
        }
    }
    public static class CRBotServo extends CRActuator<CRServo>{
        @SafeVarargs
        public CRBotServo(String name, List<CRServoData> crservos, Function<CRServo, Double> getCurrentPosition, int pollingRate,Supplier<Double> maxTargetFunc, Supplier<Double> minTargetFunc, Supplier<Double> maxPowerFunc, Supplier<Double> minPowerFunc, double errorTol, double defaultTimeout, String[] controlFuncKeys, List<ControlFunction<CRBotServo>>... controlFuncs) {
            super(name, crservos.stream().map(CRServoData::getCRServo).collect(Collectors.toList()), getCurrentPosition, pollingRate, maxTargetFunc, minTargetFunc, maxPowerFunc, minPowerFunc, errorTol, defaultTimeout);
            this.funcRegister=new ControlFuncRegister<CRBotServo>(this, controlFuncKeys, controlFuncs);
        }
        @SafeVarargs
        public CRBotServo(String name, List<CRServoData> crservos, Supplier<Double> maxTargetFunc, Supplier<Double> minTargetFunc, Supplier<Double> maxPowerFunc, Supplier<Double> minPowerFunc, double servoSpeed, String[] controlFuncKeys, List<ControlFunction<CRBotServo>>... controlFuncs) {
            this(name, crservos, new TimeBasedLocalizers.CRTimeBasedLocalizer<CRServo>(servoSpeed)::getCurrentPosition, 1, maxTargetFunc, minTargetFunc, maxPowerFunc,minPowerFunc,0, Double.POSITIVE_INFINITY,controlFuncKeys,controlFuncs);
        }
        @SafeVarargs
        public CRBotServo(String name,List<CRServoData> crservos, Function<CRServo, Double> getCurrentPosition, int pollingRate, Supplier<Double> maxTargetFunc, Supplier<Double> minTargetFunc, double errorTol, double defaultTimeout, String[] controlFuncKeys, List<ControlFunction<CRBotServo>>... controlFuncs) {
            this(name, crservos, getCurrentPosition, pollingRate, maxTargetFunc, minTargetFunc,()->(1.0),()->(-1.0),errorTol,defaultTimeout,controlFuncKeys,controlFuncs);
        }
        @SafeVarargs
        public CRBotServo(String name, List<CRServoData> crservos, Supplier<Double> maxTargetFunc, Supplier<Double> minTargetFunc, double servoSpeed, String[] controlFuncKeys, List<ControlFunction<CRBotServo>>... controlFuncs) {
            this(name, crservos, new TimeBasedLocalizers.CRTimeBasedLocalizer<CRServo>(servoSpeed)::getCurrentPosition, 1, maxTargetFunc, minTargetFunc, ()->(1.0),()->(-1.0),0, Double.POSITIVE_INFINITY,controlFuncKeys,controlFuncs);
        }
        public CRBotServo(String name, List<CRServoData> crservos, Supplier<Double> maxPowerFunc, Supplier<Double> minPowerFunc, double servoSpeed) {
            this(name, crservos, ()->(0.0), ()->(0.0), maxPowerFunc,minPowerFunc,servoSpeed,new String[]{});
        }
    }
}
