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
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Objects;
import java.util.function.Consumer;
import java.util.function.Function;
import java.util.function.Supplier;

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
    static void updateTelemetry(){
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
                castedActuator.resetEncoder();
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
        public void resetCache(){ //Reset the cache so a new value is taken the next time cachedRead is called.
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
    public static void initialize(HardwareMap hardwareMap, Telemetry telemetry, RobotConfig config, boolean alwaysReInit){ //Method to initialize hardwareMap, telemetry, and a RobotConfig.
        Components.hardwareMap=hardwareMap;
        Components.telemetry=telemetry;
        timer.reset(); //Static variables are preserved between runs, so timer needs to be reset
        telemetryOutput.clear();
        prevTelemetryOutput.clear();
        executor.clearCommands();
        if (Objects.isNull(Components.config) || alwaysReInit || config.getClass()!=Components.config.getClass()){
            CachedReader.readers.clear();
            actuators.clear();
            Components.config=config;
            config.init();
        }
    }
    public abstract static class ControlFunc<E extends Actuator<?>>{ //Control functions extend this subclass
        protected E parentActuator;
        protected ControlSystem<? extends E> system; //Each function has access to the system it's part of
        private void registerToSystem(ControlSystem<? extends E> system){
            this.system=system;
            this.parentActuator=system.getParentActuator();
        }
        protected abstract void runProcedure(); //This method is where the control function does its job
        public void stopProcedure(){} //Takes care of anything that needs to occur when the control function stops running.
    }
    public static class ControlSystem<E extends Actuator<?>>{ //Holds control functions, as well as the necessary plant reference values and the function by which the actuator actuates
        private E parentActuator;
        private boolean isStart=true; //Indicates if the control system has just started running
        private final HashMap<String,Supplier<Double>> globalReferences=new HashMap<>();
        private final HashMap<String,Double> storedGlobalReferences=new HashMap<>();
        private final HashMap<String,Boolean> isNewGlobalReferences=new HashMap<>();
        private final HashMap<String,Double> instantReferences=new HashMap<>();
        private Consumer<Double> outputFunc;
        private double output =0;
        private final List<ControlFunc<? super E>> controlFuncs;
        @SafeVarargs
        public ControlSystem(String[] referenceKeys, List<Supplier<Double>> referenceValues, Consumer<Double> outputFunc, ControlFunc<? super E>...controlFuncs) {
            this.outputFunc = outputFunc;
            this.controlFuncs=Arrays.asList(controlFuncs);
            globalReferences.put("targetPosition",()->parentActuator.getTarget());
            storedGlobalReferences.put("targetPosition",0.0);
            isNewGlobalReferences.put("targetPosition",false);
            instantReferences.put("targetPosition",0.0);
            for (int i=0;i<referenceKeys.length;i++){ //The default reference, or target, is the position-based target. You can also set a target velocity or a target acceleration. Input a label for your custom reference and a Supplier by which to access it through the constructor.
                globalReferences.put(referenceKeys[i],referenceValues.get(i));
                storedGlobalReferences.put(referenceKeys[i],0.0);
                isNewGlobalReferences.put(referenceKeys[i],false);
                instantReferences.put(referenceKeys[i],0.0);
            }
        }
        @SafeVarargs
        public ControlSystem(String[] referenceKeys, List<Supplier<Double>> referenceValues, ControlFunc<? super E>...controlFuncs) {
            this(referenceKeys,referenceValues,null,controlFuncs);
        }
        @SafeVarargs
        public ControlSystem(ControlFunc<? super E>...controlFuncs) {
            this(new String[]{}, List.of(),null,controlFuncs);
        }
        private void registerToActuator(E parentActuator){
            this.parentActuator=parentActuator;
            for (ControlFunc<? super E> func:controlFuncs){
                func.registerToSystem(this);
            }
            if (Objects.isNull(outputFunc)){
                if (parentActuator instanceof CRActuator){
                    CRActuator<?> castedActuator = (CRActuator<?>) parentActuator;
                    outputFunc = castedActuator::setPower;
                }
                else if (parentActuator instanceof BotServo){
                    BotServo castedActuator = (BotServo) parentActuator;
                    outputFunc = castedActuator::setPosition;
                }
            }
        }
        public boolean isStart(){
            return isStart;
        }
        private void run(){
            if (controlFuncs.isEmpty()){
                return;
            }
            output=0;
            isNewGlobalReferences.replaceAll((r, v) -> false);
            for (String label:storedGlobalReferences.keySet()){
                readReference(label);
            }
            for (String label:storedGlobalReferences.keySet()){
                setInstantReference(label,getReference(label));
            }
            for (ControlFunc<?> func:controlFuncs){
                func.runProcedure();
            }
            isStart=false;
            outputFunc.accept(output);
        }
        private void stopAndReset(){ //Stop and reset the system, if control is switched to another.
            for (ControlFunc<?> func:controlFuncs){
                func.stopProcedure();
            }
            isStart=true;
        }
        public double getReference(String label){ //Return the value of a chosen reference.
            try{
                return Objects.requireNonNull(storedGlobalReferences.get(label));
            }
            catch (NullPointerException n){
                throw new NullPointerException("No reference "+label+" exists. Add the reference to the control system.");
            }
        }
        private void readReference(String label){
            double reference = Objects.requireNonNull(globalReferences.get(label)).get();
            if (reference!=Objects.requireNonNull(storedGlobalReferences.get(label))){
                isNewGlobalReferences.put(label,true);
            }
            storedGlobalReferences.put(label,reference);
        }
        public double getInstantReference(String label){ //Return the value of an instantaneous reference. In systems like motion profiling, this is distinct from the global reference but builds to it over time.
            try{
                return Objects.requireNonNull(instantReferences.get(label));
            }
            catch (NullPointerException n){
                throw new NullPointerException("No reference "+label+" exists. Add the reference to the control system.");
            }
        }
        public void setInstantReference(String label, double value){
            instantReferences.put(label,value);
        }
        public void setOutput(double output){
            this.output=output;
        }
        public double getOutput(){
            return output;
        }
        public E getParentActuator(){
            return parentActuator;
        }
        public boolean isNewReference(String label){ //Returns true if the global reference has just changed.
            return Boolean.TRUE.equals(isNewGlobalReferences.get(label));
        }
    }
    public static class SyncedActuators<E extends Actuator<?>>{ //Holds multiple synchronized actuators and allows you to call methods or actions on them in sync.
        private final HashMap<String, E> actuators = new HashMap<>();
        @SafeVarargs
        public SyncedActuators(E...actuators){
            for (E actuator:actuators){
                this.actuators.put(actuator.getName(),actuator);
            }
        }
        public void call(Consumer<E> method){ //Call a method on all synced actuators. Provide a function that accepts an actuator and calls the method on it.
            for (E actuator:this.actuators.values()){
                method.accept(actuator);
            }
        }
        public Command command(Function<E,Command> commandMethod){ //Get a ParallelAction that performs an action on all actuators. Provide a function that accepts an actuator and returns the desired action.
            return new Commands.ParallelCommand((Command[]) actuators.values().stream().map(commandMethod).toArray());
        }
        public HashMap<String,E> getActuators(){
            return actuators;
        }
    }
    public abstract static class Actuator<E extends HardwareDevice>{ //Actuators are enhanced hardware classes that have more state and functionality. Each Actuator instance is parametrized with a specific type, like DcMotorEx or Servo, and holds a HardwareDevice of that type.
        private final String name; //The actuator's hardwareMap name.
        private final E device; //This is the actuator's internal HardwareDevice, from hardwareMap.
        private double target; //Global target of the actuator
        private boolean newTarget=false; //Set to true when setTarget is called. Set to false after the end of each loop.
        private boolean newActuation=false; //Set to true when a method tagged with @Actuator is called. Set to false after the end of each loop.
        private double offset; //In case a part skips or something, this allows us to offset all the targets we set to compensate for the skip.
        public Supplier<Double> maxTargetFunc = ()->Double.POSITIVE_INFINITY;
        public Supplier<Double> minTargetFunc = ()->Double.NEGATIVE_INFINITY;
        //Max and min targets. They are dynamic functions since the max position for an actuator may not be the same. An in-game extension limit may not apply based on the direction of the actuator, for example.
        private Supplier<Double> maxOffsetFunc = ()->(Double.POSITIVE_INFINITY);
        private Supplier<Double> minOffsetFunc = ()->(Double.NEGATIVE_INFINITY);
        //Max and min offsets. They are dynamic functions since the max position for an actuator may not be the same.
        private final double errorTol; //Error tolerance for when the actuator is commanded to a position
        private final double defaultMovementTimeout; //Default time waited when an actuator is commanded to a position before ending the  command.
        protected boolean actuationStateUnlocked = true; //If set to false, methods tagged with @Actuate should not have an effect; it locks the actuator in whatever power/position state it's in.
        private boolean targetStateUnlocked = true; //If set to false, the actuator's target cannot change.
        private final HashMap<String,Double> keyPositions = new HashMap<>(); //Stores key positions, like 'transferPosition,' etc. The keys are labels for positions, and the values are the positions themselves. Useful because you only have to adjust the value corresponding to a certain position in one place.
        private final Supplier<Double> getCurrentPosition;
        private final Runnable resetCurrentPositionCache;
        private final HashMap<String, ControlSystem<? extends Actuator<E>>> controlSystemMap = new HashMap<>();
        private String currControlFuncKey;
        private final String defaultControlKey;
        private boolean timeBasedLocalization = false; //Indicates whether the getCurrentPosition method of the actuator calculates the position based on time as opposed to an encoder, which is important to know.
        @SafeVarargs
        public Actuator(String actuatorName, Function<E, Double> getCurrentPosition, int currentPosPollingInterval,
                        double errorTol, double defaultMovementTimeout, String[] controlFuncKeys, ControlSystem<? extends Actuator<E>>... controlFuncs){
            currentPosPollingInterval=Math.max(0,currentPosPollingInterval);
            this.name=actuatorName;
            this.device = (E) hardwareMap.get(actuatorName);
            this.defaultMovementTimeout = defaultMovementTimeout;
            this.errorTol=errorTol;
            CachedReader<Double> reader = new CachedReader<>(()->(getCurrentPosition.apply(device)),currentPosPollingInterval);
            this.getCurrentPosition=reader::cachedRead;
            resetCurrentPositionCache = reader::resetCache;
            for (int i=0;i< controlFuncKeys.length;i++){
                controlSystemMap.put(controlFuncKeys[i],controlFuncs[i]);
            }
            controlSystemMap.put("controlOff",new ControlSystem<>());
            currControlFuncKey="controlOff";
            if (controlFuncKeys.length>0){
                defaultControlKey=controlFuncKeys[0];
            } else{
                defaultControlKey=currControlFuncKey;
            }
            actuators.put(name,this);
        }
        public void setTargetBounds(Supplier<Double> maxTargetFunc, Supplier<Double> minTargetFunc){ //Sets the maximum and minimum target that can be set to the Actuator. They are functions because the maximum and minimum may change depending on other factors.
            this.maxTargetFunc = ()->(maxTargetFunc.get()+offset);
            this.minTargetFunc = ()->(minTargetFunc.get()+offset);
        }
        public String getName(){
            return name;
        }
        public void setOffsetBoundFuncs(Supplier<Double> maxFunc,Supplier<Double> minFunc){ //Sets bounds on the offset that can be set.
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
        public void setTarget(double target){ //Set the target position of the actuator.
            if (targetStateUnlocked){
                target=target+offset;
                target=Math.max(minTargetFunc.get(),Math.min(target, maxTargetFunc.get()));
                if (target!=this.target) {
                    this.target = target;
                    newTarget = true;
                }
            }
        }
        public double getTargetMinusOffset(){
            return target-offset;
        } //Returns the target minus the offset
        public double getTarget(){
            return target;
        } //Returns the target including the offset
        public double getCurrentPosition(){ //Gets the position of a specific part
            return getCurrentPosition.get();
        }
        public void setOffset(double offset){ //Set the offset of an actuator. Whenever you set the target position, it will add the offset. Useful if a part skips.
            offset=Math.max(minOffsetFunc.get(),Math.min(maxOffsetFunc.get(),offset));
            if (offset!=this.offset){
                double oldOffset=this.offset;
                this.offset=offset;
                setTarget(target-oldOffset);
            }
        }
        public void runControl(){
            Objects.requireNonNull(controlSystemMap.get(currControlFuncKey)).run();
        }
        public double getPos(String key){ //Returns one of the key positions based on the inputted key
            return Objects.requireNonNull(keyPositions.get(key));
        }
        public void switchControl(String key){ // Switch control of actuator.
            Objects.requireNonNull(controlSystemMap.get(currControlFuncKey)).stopAndReset();
            currControlFuncKey=key;
        }
        public void lockActuationState(){
            actuationStateUnlocked=false;
        }
        //Prevent the actuator from actuating (so no new setPower calls, for example)
        public void unlockActuationState(){
            actuationStateUnlocked=true;
        }
        public void lockTargetState(){
            targetStateUnlocked=false;
        }
        //Prevent the actuator from setting a new target
        public void unlockTargetState(){
            targetStateUnlocked=true;
        }
        protected void resetCurrentPositionCache(){
            resetCurrentPositionCache.run();
        }
        public void setKeyPositions(String[] keyPositionKeys, double[] keyPositionValues){ //Set key targets. Input an array of labels and an array of values.
            for (int i=0; i<keyPositionKeys.length; i++){
                keyPositions.put(keyPositionKeys[i],keyPositionValues[i]);
            }
        }
        public E getDevice(){
            return this.device;
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
        public InstantCommand instantSetTargetCommand(double target){ //Action to set the target but without waiting for it to get there
            return new InstantCommand(()->setTarget(target));
        }
        public InstantCommand instantSetTargetCommand(Supplier<Double> targetFunc){
            return new InstantCommand(()->setTarget(targetFunc.get()));
        }
        public InstantCommand instantSetTargetCommand(String position){ //Whenever a setTarget, setPower, or setVelocity method takes a String label, it will get the value corresponding to that label in the keyPositions/keyPowers/keyVelocities hashmap.
            return new InstantCommand(()->setTarget(getPos(position)));
        }
        public MoveToTargetCommand moveToTargetCommand(double target){ //Command to move to a target and wait for it to get there
            return new MoveToTargetCommand(target);
        }
        public MoveToTargetCommand moveToTargetCommand(Supplier<Double> targetFunc){
            return new MoveToTargetCommand(targetFunc);
        }
        public MoveToTargetCommand moveToTargetCommand(double target, double timeout){ //Timeout refers to a timeout on which the action stops waiting for the device to reach the target
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
        public SetOffsetCommand setOffsetCommand(double offset){ //Sets the offset of the actuator
            return new SetOffsetCommand(offset);
        }
        public SetOffsetCommand setOffsetCommand(Supplier<Double> offsetFunc){
            return new SetOffsetCommand(offsetFunc);
        }
        public RunResettingLoop triggeredMoveToTargetCommand(Supplier<Boolean> condition, double target){ //Command to move to a target on a button press
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
        public RunResettingLoop triggeredToggleTargetCommand(Supplier<Boolean> condition, double target1, double target2){ //Command to toggle between two targets on a button press
            return new RunResettingLoop(new PressCommand(new IfThen(condition, toggleTargetCommand(target1,target2))));
        }
        public RunResettingLoop triggeredDynamicTargetCommand(Supplier<Boolean> upCondition, Supplier<Boolean> downCondition, double change){ //Command to increment/decrement the target by a certain amount per loop when up/down buttons are held
            return new RunResettingLoop(new ConditionalCommand(new IfThen(upCondition, moveToTargetCommand(()->(getTargetMinusOffset()+change))),new IfThen(downCondition, moveToTargetCommand(()->(getTargetMinusOffset()-change)))));
        }
        public RunResettingLoop triggeredFSMTargetCommand(Supplier<Boolean> upCondition, Supplier<Boolean> downCondition, double...targets){ //Command to scroll the target through a list of targets when up/down scrolling buttons are pressed
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
        private Supplier<Double> maxPowerFunc = ()->Double.POSITIVE_INFINITY;
        private Supplier<Double> minPowerFunc = ()->Double.NEGATIVE_INFINITY;
        //Max and min power boundaries
        private final HashMap<String,Double> keyPowers = new HashMap<>(); //Stores key powers, like 'intakePower,' etc.
        @SafeVarargs
        public CRActuator(String name, DcMotorSimple.Direction direction, Function<E, Double> getCurrentPosition, int pollingRate, double errorTol, double defaultTimeout, String[] controlFuncKeys, ControlSystem<? extends CRActuator<E>>... controlFuncs) {
            super(name, getCurrentPosition, pollingRate, errorTol, defaultTimeout,controlFuncKeys,controlFuncs);
            this.setTarget(0);
            this.getDevice().setDirection(direction);
        }
        public CRActuator(String name, DcMotorSimple.Direction direction, Function<E, Double> getCurrentPosition) { //For CRActuators that don't set targets and only use setPower, like drivetrain motors.
            this(name,direction,getCurrentPosition, 1,0,0,new String[]{});
        }
        public void setPowerBounds(Supplier<Double> maxPowerFunc, Supplier<Double> minPowerFunc){
            this.maxPowerFunc=maxPowerFunc;
            this.minPowerFunc=minPowerFunc;
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
        public void setPower(double power){
            if (actuationStateUnlocked){
                power=Math.max(Math.min(power, maxPowerFunc.get()), minPowerFunc.get());
                E device=getDevice();
                if (Math.abs(power-device.getPower())>0.03) {
                    device.setPower(power);
                    if (getTimeBasedLocalization()){ //If current position is calculated by time, it needs to be updated everytime the actuator moves
                        resetCurrentPositionCache();
                        getCurrentPosition();
                    }
                    setNewActuation();
                }
            }
        }
        public double getPower(){
            return getDevice().getPower();
        }
        public class SetPowerCommand extends InstantCommand{ //Command to set the power of all synchronized parts
            public SetPowerCommand(Supplier<Double> powerFunc) {
                super(()-> setPower(powerFunc.get()));
            }
            public SetPowerCommand(double power) {
                super(()-> setPower(power));
            }
        }
        //The following methods for commands for setting powers are like their counterparts for setting targets.
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
                if (getDevice().getPower()==power1) return power2; else if (getDevice().getPower()==power2) return power1; else return getDevice().getPower();
            });
        }
        public SetPowerCommand upwardFSMPowerCommand(double...powersList){
            Arrays.sort(powersList);
            return setPowerCommand(()->{
                for (double power: powersList){
                    if (getDevice().getPower()<power){
                        return power;
                    }
                }
                return getDevice().getPower();
            });
        }
        public SetPowerCommand downwardFSMPowerCommand(double...powersList){
            Arrays.sort(powersList);
            return setPowerCommand(()->{
                for (int i = powersList.length-1; i>=0; i--){
                    if (getDevice().getPower()>powersList[i]){
                        return powersList[i];
                    }
                }
                return getDevice().getPower();
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
            return new RunResettingLoop(new ConditionalCommand(new IfThen(upCondition, setPowerCommand(()->(getDevice().getPower()+change))),new IfThen(downCondition, setPowerCommand(()->(getDevice().getPower()-change)))));
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
        private Supplier<Double> velocityReader;
        private final Supplier<Double> currentReader;
        private final HashMap<String,Double> keyVelocities = new HashMap<>(); //Stores key velocities, like 'intakeVelocity,' etc.
        private Supplier<Double> maxVelocityFunc = ()->Double.POSITIVE_INFINITY;
        private Supplier<Double> minVelocityFunc = ()->Double.NEGATIVE_INFINITY;
        @SafeVarargs
        public BotMotor(String name, DcMotorSimple.Direction direction, Function<DcMotorEx, Double> getCurrentPosition, int currentPosPollingInterval, double errorTol, double defaultTimeout, String[] controlFuncKeys, ControlSystem<BotMotor>... controlFuncs) {
            super(name,direction,getCurrentPosition, currentPosPollingInterval, errorTol, defaultTimeout,controlFuncKeys,controlFuncs);
            DcMotorEx device=getDevice();
            velocityReader=new CachedReader<>(device::getVelocity,1)::cachedRead;
            currentReader=new CachedReader<>(()->device.getCurrent(CurrentUnit.AMPS),3)::cachedRead;
            device.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            device.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            for (ControlSystem<BotMotor> system:controlFuncs){
                system.registerToActuator(this);
            }
        }

        @SafeVarargs
        public BotMotor(String name, DcMotorSimple.Direction direction, double errorTol, double defaultTimeout, String[] controlFuncKeys, ControlSystem<BotMotor>... controlFuncs) {
            this(name, direction, (DcMotorEx motor) -> ((double) motor.getCurrentPosition()), 1, errorTol, defaultTimeout,controlFuncKeys,controlFuncs);
        }

        public BotMotor(String name, DcMotorSimple.Direction direction) {
            this(name, direction, 0.0, 0.0, new String[]{});
        }
        public void setVelocityBounds(Supplier<Double> maxVelocityFunc, Supplier<Double> minVelocityFunc){
            this.minVelocityFunc=minVelocityFunc;
            this.maxVelocityFunc=maxVelocityFunc;
        }
        public double getKeyVelocity(String key){
            return Objects.requireNonNull(keyVelocities.get(key));
        }
        public void setKeyVelocities(String[] keyVelocityKeys, double[] keyVelocityValues){
            for (int i=0; i<keyVelocityKeys.length; i++){
                keyVelocities.put(keyVelocityKeys[i],keyVelocityValues[i]);
            }
        }
        public void setVelocityFilter(Function<BotMotor,Double> velFilter){ //Allows you to set a custom function, like a Kalman filter, to determine the velocity of the motor.
            velocityReader=new CachedReader<>(()->velFilter.apply(this),1)::cachedRead;
        }
        public double getVelocity() { //Get the motor's velocity
            return velocityReader.get();
        }
        public void resetEncoder() { //Reset motor encoder to 0
            getDevice().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            getDevice().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            resetCurrentPositionCache();
        }

        public void setZeroPowerFloat() { //Set ZeroPowerBehavior to float
            getDevice().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }

        public void setZeroPowerBrake() {
            getDevice().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        public double getCurrentAmps() { //Get the current of the motor in amps
            return currentReader.get();
        }
        public boolean isStallResetting() {
            return isStallResetting;
        }

        public void setMode(DcMotorEx.RunMode mode) {
            getDevice().setMode(mode);
        }

        public class StallResetCommand extends Command { //Stall resets encoders, and offsets the position if you want to reset at a non-zero position.
            //The way stall resetting works is: you set power to the motor until it hits a hard stop. The motor will stall because it is trying to get past the hard stop but can't. We detect this stall and reset the motor to the position it should be at on the hard stop.
            //Good for if a part skips on a motor.
            double resetPosition;
            double stallAmps;

            public StallResetCommand(double resetPosition, double stallAmps) { //Provide the amp threshold to determine of the motor is stalling, and provide a position to reset to
                this.resetPosition = resetPosition;
                this.stallAmps = stallAmps;
            }

            @Override
            protected boolean runProcedure() {
                if (isStart()) {
                    isStallResetting = true;
                    setPower(-0.2);
                }
                if (getCurrentAmps() > stallAmps) {
                    setPower(0);
                    setOffset(getCurrentPosition() - resetPosition);
                    setTarget(resetPosition);
                    isStallResetting = false;
                }
                return isStallResetting;
            }
        }

        public StallResetCommand stallResetCommand(double resetPosition, double stallAmps) {
            return new StallResetCommand(resetPosition, stallAmps);
        }

        public PressCommand triggeredStallResetCommand(Supplier<Boolean> condition, double resetPosition, double stallAmps) { //Command to initiate a stall reset on a button press
            return new PressCommand(new IfThen(condition, stallResetCommand(resetPosition, stallAmps)));
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
        public void setVelocity(double velocity){ //Sets the velocity of the motor. Like setPower, but with velocity control
            if (actuationStateUnlocked){
                velocity=Math.max(minVelocityFunc.get(),Math.min(velocity, maxVelocityFunc.get()));
                if (Math.abs(velocity-getVelocity())>1) {
                    getDevice().setVelocity(velocity);
                    setNewActuation();
                }
            }
        }
    }
    public static class BotServo extends Actuator<Servo>{
        private double currCommandedPos;
        private boolean ignoreSetPosCaching = false; //If this is true, then even if a commanded position is the same as its current commanded position, the actuator will call setPosition on the hardwareMap Servos it controls. Useful to counteract Axon nudge.
        private final double range; //Stores the range of the servo positions (e.g. 0-180 degrees)
        private Function<Double, Double> setPositionConversion;
        @SafeVarargs
        public BotServo(String name, Servo.Direction direction, Function<Servo, Double> getCurrentPosition, int currentPosPollingInterval, double errorTol, double defaultTimeout, double range, //Degree range that servo is programmed to
                        double initialTarget, String[] controlFuncKeys, ControlSystem<BotServo>... controlFuncs) {
            super(name, getCurrentPosition, currentPosPollingInterval, errorTol, defaultTimeout, controlFuncKeys, controlFuncs);
            setTarget(initialTarget);
            this.range=range;
            this.setPositionConversion=(Double pos)->pos/range;
            for (ControlSystem<BotServo> system:controlFuncs){
                system.registerToActuator(this);
            }
            getDevice().setDirection(direction);
        }
        public BotServo(String name, Servo.Direction direction, double servoSpeedDPS, double defaultTimeout, double range, double initialTarget) {
            this(name,direction, new TimeBasedLocalizers.ServoTimeBasedLocalizer(servoSpeedDPS/range,initialTarget,range)::getCurrentPosition,1,1.5,defaultTimeout,range, initialTarget, new String[]{"setPos"}, new ControlSystem<>(new ServoControl()));
            setTimeBasedLocalization(true);
        }
        public void setPositionConversion(Function<Double,Double> setPositionConversion){
            this.setPositionConversion=setPositionConversion;
        }
        @Actuate
        public void setPosition(double position){ //Accepts position in degrees by default and sets the servos position to that.
            position=Math.max(minTargetFunc.get(),Math.min(position, maxTargetFunc.get()));
            if (actuationStateUnlocked && (Math.abs(currCommandedPos-position)>0.07||ignoreSetPosCaching)){
                currCommandedPos=position;
                getDevice().setPosition(setPositionConversion.apply(position));
                if (getTimeBasedLocalization()){
                    resetCurrentPositionCache();
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
        public CRBotServo(String name, DcMotorSimple.Direction direction, Function<CRServo, Double> getCurrentPosition, int pollingRate, double errorTol, double defaultTimeout, String[] controlFuncKeys, ControlSystem<CRBotServo>... controlFuncs) {
            super(name, direction, getCurrentPosition, pollingRate, errorTol, defaultTimeout,controlFuncKeys,controlFuncs);
            for (ControlSystem<CRBotServo> system:controlFuncs){
                system.registerToActuator(this);
            }
        }
        @SafeVarargs
        public CRBotServo(String name, DcMotorSimple.Direction direction, double servoSpeedDPS, String[] controlFuncKeys, ControlSystem<CRBotServo>... controlFuncs) {
            this(name, direction, new TimeBasedLocalizers.CRTimeBasedLocalizer<CRServo>(servoSpeedDPS)::getCurrentPosition, 1,0, Double.POSITIVE_INFINITY,controlFuncKeys,controlFuncs);
        }
        public CRBotServo(String name, DcMotorSimple.Direction direction, double servoSpeedDPS) {
            this(name, direction, servoSpeedDPS,new String[]{});
        }
    }
}
