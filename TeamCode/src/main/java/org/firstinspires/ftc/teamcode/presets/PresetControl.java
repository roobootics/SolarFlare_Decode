package org.firstinspires.ftc.teamcode.presets;

import static org.firstinspires.ftc.teamcode.base.Components.timer;

import org.firstinspires.ftc.teamcode.base.Components.Actuator;
import org.firstinspires.ftc.teamcode.base.Components.BotMotor;
import org.firstinspires.ftc.teamcode.base.Components.BotServo;
import org.firstinspires.ftc.teamcode.base.Components.CRActuator;
import org.firstinspires.ftc.teamcode.base.Components.ControlFunc;
import org.firstinspires.ftc.teamcode.base.Components.ControlSystem;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Objects;
import java.util.function.Supplier;

public abstract class PresetControl { //Holds control functions that actuators can use. Note that more control functions, like other types of motion profiling, can be coded and used.
    public static class GenericPID{ //A class that creates a PIDF controller for any purpose.
        private final double kP;
        private final double kI;
        private final double kD;
        private double integralSum;
        private double previousLoop;
        private double previousError;
        private final ArrayList<Double> previousFiveLoopTimes = new ArrayList<>();
        private final ArrayList<Double> previousFiveErrors = new ArrayList<>();
        public GenericPID(double kP, double kI, double kD){ //Allows for a custom feedforward, such as one that takes acceleration into account
            this.kP=kP;
            this.kI=kI;
            this.kD=kD;
        }
        public GenericPID(double kP, double kI, double kD, double kF){
            this(kP,kI,kD);
        }
        public double getPIDOutput(double target, double current, double velocity){ //Give it the target value and the current value
            double error=target-current;
            double time=timer.time();
            double loopTime=time-previousLoop;

            integralSum+=loopTime*error;
            previousFiveLoopTimes.add(loopTime);
            previousFiveErrors.add(error);
            if (previousFiveLoopTimes.size()>5){
                previousFiveErrors.remove(0);
                previousFiveLoopTimes.remove(0);
            }

            double pOutput=kP*error;
            double iOutput=kI*integralSum;
            double dOutput;
            if (previousFiveErrors.size()==5){
                double dtAvg=0;
                for (double dt:previousFiveLoopTimes){
                    dtAvg+=dt;
                }
                dtAvg=dtAvg/5;
                dOutput=kD*(-previousFiveErrors.get(4)+8*previousFiveErrors.get(3)-8*previousFiveErrors.get(1)+previousFiveErrors.get(0))/(12*dtAvg);
            } else{
                dOutput=kD*(error-previousError)/loopTime;
            }
            if (!Double.isNaN(velocity)){
                dOutput=-kD*velocity;
            }

            previousLoop=time;
            previousError=error;

            return pOutput+iOutput+dOutput;
        }
        public double getPIDOutput(double target, double current){
            return getPIDOutput(target,current,Double.NaN);
        }
        public void clearIntegral(){
            integralSum=0;
        } //Clear the accumulating integral term. Do this when a global target is changed
        public void clearFivePointStencil(){ //Clear the five point stencil for derivative approximation
            previousLoop=timer.time();
            previousError=0;
            previousFiveLoopTimes.clear();
            previousFiveErrors.clear();
        }
    }
    public static class PositionPID<E extends CRActuator<?>> extends ControlFunc<E>{ //Position PIDF controller for CRActuators
        private final ArrayList<GenericPID> PIDs = new ArrayList<>();
        private final double kP;
        private final double kI;
        private final double kD;
        public PositionPID(double kP, double kI, double kD){
            this.kP=kP;
            this.kI=kI;
            this.kD=kD;
        }
        @Override
        public void registerToSystem(ControlSystem<? extends E> system){
            super.registerToSystem(system);
            for (String name: system.getParentActuator().getPartNames()){
                PIDs.add(new GenericPID(kP,kI,kD));
            }
        }
        @Override
        public void runProcedure(){
            for (int i=0;i<parentActuator.getPartNames().length;i++){
                String name=parentActuator.getPartNames()[i];
                GenericPID PID=PIDs.get(i);
                if (system.isStart()){
                    PID.clearIntegral();
                    PID.clearFivePointStencil();
                }
                if (system.isNewReference("targetPosition")){
                    PID.clearIntegral();
                }
                double output=PID.getPIDOutput(system.getInstantReference("targetPosition"), parentActuator.getCurrentPosition(name));
                system.setOutput(system.getOutput(name)+output,name);
            }
        }
        @Override
        public void stopProcedure(){
            parentActuator.setPower(0);
        }
    }
    public static class VelocityPID<E extends BotMotor> extends ControlFunc<E>{ //Position PIDF controller for CRActuators
        private final ArrayList<GenericPID> PIDs = new ArrayList<>();
        private final double kP;
        private final double kI;
        private final double kD;
        public VelocityPID(double kP, double kI, double kD){
            this.kP=kP;
            this.kI=kI;
            this.kD=kD;
        }
        @Override
        public void registerToSystem(ControlSystem<? extends E> system){
            super.registerToSystem(system);
            for (String name: system.getParentActuator().getPartNames()){
                PIDs.add(new GenericPID(kP,kI,kD));
            }
        }
        @Override
        public void runProcedure(){
            for (int i=0;i<parentActuator.getPartNames().length;i++){
                String name=parentActuator.getPartNames()[i];
                GenericPID PID=PIDs.get(i);
                if (system.isStart()){
                    PID.clearIntegral();
                    PID.clearFivePointStencil();
                }
                if (system.isNewReference("targetVelocity")){
                    PID.clearIntegral();
                }
                double output=PID.getPIDOutput(system.getInstantReference("targetVelocity"), parentActuator.getVelocity(name));
                system.setOutput(system.getOutput(name)+output,name);
            }
        }
        @Override
        public void stopProcedure(){
            parentActuator.setPower(0);
        }
    }
    public static class BasicFeedforward<E extends CRActuator<?>> extends ControlFunc<E>{
        private final double[] kFs;
        private final String[] references;
        public BasicFeedforward(double[] kFs, String[] references){
            this.kFs = kFs;
            this.references = references;
        }
        @Override
        protected void runProcedure() {
            for (String name: parentActuator.getPartNames()){
                double output=0;
                for (int i=0;i< kFs.length;i++){
                    output+=kFs[i]*system.getInstantReference(references[i]);
                }
                system.setOutput(system.getOutput(name)+output,name);
            }
        }
    }
    public static class ElevatorFeedforward<E extends CRActuator<?>> extends ControlFunc<E>{
        public double kF;
        public ElevatorFeedforward(double kG){
            this.kF = kF;
        }
        @Override
        protected void runProcedure() {
            for (String name: parentActuator.getPartNames()){
                system.setOutput(system.getOutput(name)+kF,name);
            }
        }
    }
    public static class ArmFeedforward<E extends CRActuator<?>> extends ControlFunc<E>{
        public double kF;
        public double referenceToRad;
        public double angleAtZero;
        public ArmFeedforward(double kF, double unitsPerRevolution, double angleAtZero){
            this.kF = kF;
            referenceToRad=(2*Math.PI)/unitsPerRevolution;
            this.angleAtZero=angleAtZero;
        }
        @Override
        protected void runProcedure() {
            for (String name: parentActuator.getPartNames()){
                system.setOutput(kF*Math.cos(system.getOutput(name)*referenceToRad+angleAtZero),name);
            }
        }
    }
    public static class SQUID<E extends CRActuator<?>> extends ControlFunc<E>{ //SQUID controller for CRActuators
        public double kP;
        public SQUID(double kP){
            this.kP=kP;
        }
        @Override
        protected void runProcedure() {
            for (int i=0;i<parentActuator.partNames.length;i++){
                double error = system.getInstantReference("targetPosition")- parentActuator.getCurrentPosition(parentActuator.getPartNames()[i]);
                system.setOutput(
                        kP * Math.sqrt(Math.abs(error))*Math.signum(error)+system.getOutput(parentActuator.getPartNames()[i]),
                        parentActuator.getPartNames()[i]
                );
            }
        }
        @Override
        public void stopProcedure(){
            parentActuator.setPower(0);
        }
    }
    public static class TrapezoidalMotionProfile<E extends Actuator<?>> extends ControlFunc<E>{ //Trapezoidal motion profile for any actuator.
        public enum Phase{
            ACCEL,
            CRUISE,
            DECEL,
            IDLE,
            OFF
        }
        private double currentTarget;
        private boolean newParams=true;
        private double currentMaxVelocity;
        private double currentAcceleration;
        private double currentDeceleration;
        private double accelDT;
        private double decelDT;
        private double cruiseDT;
        private double accelDistance;
        private double decelDistance;
        private double cruiseDistance;
        private double MAX_VELOCITY;
        private double ACCELERATION;
        private double lastLoopTime=0;
        private double profileStartPos;
        private double startVelocity;
        private double targetVelocity;
        private double instantTarget;
        private Phase phase = Phase.IDLE;
        private double elapsedTime;
        public TrapezoidalMotionProfile(double maxVelocity, double acceleration){
            this.MAX_VELOCITY=maxVelocity;
            this.ACCELERATION=acceleration;
        }
        public void setNewParams(double maxVelocity, double acceleration){
            this.MAX_VELOCITY=maxVelocity;
            this.ACCELERATION=acceleration;
            newParams=true;
        }
        @Override
        protected void runProcedure() {
            if (system.isNewReference("targetPosition")||newParams||system.isStart()){
                if (system.isStart()){
                    instantTarget=parentActuator.getCurrentPosition();
                    lastLoopTime=timer.time();
                }
                newParams=false;
                createMotionProfile(system.getReference("targetPosition"));
            }
            system.setInstantReference("targetPosition", runMotionProfileOnce());
        }
        public void createMotionProfile(double target){
            elapsedTime=0;
            currentTarget = target;
            profileStartPos = instantTarget;
            double distance = target - profileStartPos;
            if (distance!=0) {
                startVelocity=targetVelocity;
                currentMaxVelocity = MAX_VELOCITY * Math.signum(distance);
                double accelSign=Math.signum(currentMaxVelocity - startVelocity);
                if (accelSign==0){
                    accelSign=Math.signum(distance);
                }
                currentAcceleration = ACCELERATION * accelSign;
                currentDeceleration = -ACCELERATION * Math.signum(distance);
                accelDT = (currentMaxVelocity - startVelocity) / currentAcceleration;
                decelDT = (0 - currentMaxVelocity) / currentDeceleration;
                accelDistance = startVelocity * accelDT + 0.5 * currentAcceleration * accelDT * accelDT;
                decelDistance = currentMaxVelocity * decelDT + 0.5 * currentDeceleration * decelDT * decelDT;
                cruiseDistance = Math.abs(distance - accelDistance - decelDistance) * Math.signum(currentMaxVelocity);
                if (Math.abs(accelDistance + cruiseDistance + decelDistance) > Math.abs(distance)) {
                    cruiseDistance=0;
                    double halfExceededDistance = (distance - accelDistance - decelDistance) / 2;
                    accelDistance = accelDistance + halfExceededDistance;
                    accelDT = Math.max(
                            (-startVelocity + Math.sqrt(Math.abs(startVelocity * startVelocity + 2 * currentAcceleration * accelDistance))) / (currentAcceleration),
                            (-startVelocity - Math.sqrt(Math.abs(startVelocity * startVelocity + 2 * currentAcceleration * accelDistance))) / (currentAcceleration)
                    );
                    currentMaxVelocity = currentAcceleration * accelDT + startVelocity;
                    decelDistance = decelDistance + halfExceededDistance;
                    decelDT = Math.max(
                            (-currentMaxVelocity + Math.sqrt(Math.abs(currentMaxVelocity * currentMaxVelocity + 2 * currentDeceleration * decelDistance))) / (currentDeceleration),
                            (-currentMaxVelocity - Math.sqrt(Math.abs(currentMaxVelocity * currentMaxVelocity + 2 * currentDeceleration * decelDistance))) / (currentDeceleration)
                    );
                }
                cruiseDT = cruiseDistance / currentMaxVelocity;
                if (Double.isNaN(accelDT) || Double.isNaN(accelDistance) || Double.isNaN(decelDT) || Double.isNaN(decelDistance) || Double.isNaN(cruiseDT) || Double.isNaN(cruiseDistance) || accelDT < 0 || decelDT < 0 || cruiseDT < 0) {
                    accelDT = 0;
                    cruiseDT = 0;
                    decelDT = 0;
                    accelDistance = 0;
                    cruiseDistance = 0;
                    decelDistance = 0;
                }
            }
            else{
                accelDT=0;
                cruiseDT=0;
                decelDT=0;
                accelDistance=0;
                cruiseDistance=0;
                decelDistance=0;
            }
        }
        public double runMotionProfileOnce(){
            double time=timer.time();
            elapsedTime+=time-lastLoopTime;
            lastLoopTime=time;
            if (elapsedTime < accelDT){
                phase= Phase.ACCEL;
                targetVelocity = startVelocity + currentAcceleration * elapsedTime;
                instantTarget = profileStartPos + startVelocity * elapsedTime + 0.5 * currentAcceleration * elapsedTime*elapsedTime;
            }
            else if (elapsedTime < accelDT+cruiseDT){
                phase= Phase.CRUISE;
                double cruiseCurrentDT = elapsedTime - accelDT;
                targetVelocity = currentMaxVelocity;
                instantTarget = profileStartPos + accelDistance + currentMaxVelocity * cruiseCurrentDT;
            }
            else if (elapsedTime < accelDT+cruiseDT+decelDT){
                phase= Phase.DECEL;
                double decelCurrentDT = elapsedTime - accelDT - cruiseDT;
                targetVelocity = currentMaxVelocity + currentDeceleration * decelCurrentDT;
                instantTarget = profileStartPos + accelDistance + cruiseDistance + currentMaxVelocity * decelCurrentDT + 0.5 * currentDeceleration * decelCurrentDT*decelCurrentDT;
            }
            else{
                phase= Phase.IDLE;
                targetVelocity=0;
                instantTarget = currentTarget;
            }
            return instantTarget;
        }
        @Override
        public void stopProcedure() {
            phase= Phase.OFF; targetVelocity=0; instantTarget=0;
        }
        public HashMap<String,Double> getProfileData(){ //Returns data on the motion profile for debugging.
            HashMap<String,Double> data = new HashMap<>();
            data.put("instantTarget",instantTarget);
            data.put("currentMaxVelocity",currentMaxVelocity);
            data.put("accelDistance",accelDistance);
            data.put("decelDistance",decelDistance);
            data.put("cruiseDistance",cruiseDistance);
            data.put("accelDT",accelDT);
            data.put("cruiseDT",cruiseDT);
            data.put("decelDT",decelDT);
            data.put("targetVelocity",targetVelocity);
            data.put("elapsedTime",elapsedTime);
            data.put("profileStartPos",profileStartPos);
            return data;
        }
        public double getProfileValue(String label){
            return Objects.requireNonNull(getProfileData().get(label));
        }
        public Phase getPhase(){
            return phase;
        } //Gives the phase that the motion profile is in.
    }


    public static class ServoControl extends ControlFunc<BotServo>{ //Control function to get servos to their targets by calling setPosition. Automatically given to BotServos depending on the constructor you call.
        @Override
        protected void runProcedure() {
            for (String name: parentActuator.getPartNames()){
                system.setOutput(system.getInstantReference("targetPosition"),name);
            }
        }
    }
    public static class SetVelocity extends ControlFunc<BotMotor>{
        @Override
        protected void runProcedure() {
            for (String name: parentActuator.getPartNames()){
                system.setOutput(system.getInstantReference("targetVelocity"),name);
            }
        }
    }
    public static class CRBangBangControl<E extends CRActuator<?>> extends ControlFunc<E>{ //Likely will be used to get CRServos to their targets if they have no encoders with them. Sets a positive or negative power to the servo depending on where it is relative to the target. May create oscillations
        private final Supplier<Double> powerFunc; //This control function moves the CRActuator to the target at a given power, which can change. That is stored here.
        public CRBangBangControl(double power){
            this.powerFunc=()->(power);
        }
        public CRBangBangControl(Supplier<Double> powerFunc){
            this.powerFunc=powerFunc;
        }
        @Override
        protected void runProcedure() {
            for (String name: parentActuator.getPartNames()){
                double currentPosition = parentActuator.getCurrentPosition(name);
                if (Math.abs(system.getInstantReference("targetPosition")-currentPosition)>parentActuator.getErrorTol()){
                    system.setOutput(system.getOutput(name)+ powerFunc.get()*Math.signum(system.getInstantReference("targetPosition")-currentPosition),name);
                }
            }
        }
    }
}
