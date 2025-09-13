package org.firstinspires.ftc.teamcode.presets;

import static org.firstinspires.ftc.teamcode.base.Components.timer;

import org.firstinspires.ftc.teamcode.base.Components.Actuator;
import org.firstinspires.ftc.teamcode.base.Components.BotServo;
import org.firstinspires.ftc.teamcode.base.Components.CRActuator;
import org.firstinspires.ftc.teamcode.base.Components.ControlFunction;


import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.Objects;
import java.util.function.Function;
import java.util.function.Supplier;

public abstract class PresetControl { //Holds control functions that actuators can use. Note that more control functions, like other types of motion profiling, can be coded and used.
    public static class GenericPIDF{
        private final double kP;
        private final double kI;
        private final double kD;
        private final Function<Double,Double> fTerm;
        private double integralSum;
        private double previousLoop;
        private double previousError;
        private final ArrayList<Double> previousFiveLoopTimes = new ArrayList<>();
        private final ArrayList<Double> previousFiveErrors = new ArrayList<>();
        public GenericPIDF(double kP, double kI, double kD, Function<Double,Double> fTerm){ //Allows for a custom feedforward, such as one that takes acceleration into account
            this.kP=kP;
            this.kI=kI;
            this.kD=kD;
            this.fTerm=fTerm;
        }
        public GenericPIDF(double kP, double kI, double kD, double kF){
            this(kP,kI,kD,(Double target)->kF*target);
        }
        public double getPIDFOutput(double target, double current){
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
            double fOutput=fTerm.apply(target);

            previousLoop=time;
            previousError=error;

            return pOutput+iOutput+dOutput+fOutput;
        }
        public void clearIntegral(){
            integralSum=0;
        }
        public void clearKalmanFilter(){
            previousLoop=timer.time();
            previousError=0;
            previousFiveLoopTimes.clear();
            previousFiveErrors.clear();
        }
    }
    public static class PIDF<E extends CRActuator<?>> extends ControlFunction<E>{

        private final ArrayList<PIDFConstants> constants;
        private final ArrayList<GenericPIDF> PIDFs = new ArrayList<>();
        public static class PIDFConstants{ //Stores PIDF coefficients
            public double kP;
            public double kI;
            public double kD;
            public Function<Double,Double> feedForwardFunc;
            public PIDFConstants(double kP, double kI, double kD, Function<Double,Double> feedForwardFunc){
                this.kP=kP;
                this.kI=kI;
                this.kD=kD;
                this.feedForwardFunc = feedForwardFunc;
            }
            public PIDFConstants(double kP, double kI, double kD, double kF){
                this.kP=kP;
                this.kI=kI;
                this.kD=kD;
                this.feedForwardFunc = (Double target)->(kF*target);
            }
        }
        public PIDF(PIDFConstants...constants){ //The PIDF can accept multiple sets of coefficients, since if two synchronized CR components have different loads, they will need to produce different power outputs
            this.constants=new ArrayList<>(Arrays.asList(constants));
        }
        @Override
        public void registerToParent(E actuator){
            super.registerToParent(actuator);
            if (actuator.parts.values().size()>this.constants.size()){
                for (int i=0;i<actuator.parts.values().size()-this.constants.size();i++){
                    constants.add(constants.get(constants.size()-1));
                }
            }
            for (PIDFConstants constant:this.constants){
                PIDFs.add(new GenericPIDF(constant.kP, constant.kI, constant.kD,constant.feedForwardFunc));
            }
        }
        @Override
        public void runProcedure(){
            for (int i=0;i<parentActuator.getPartNames().length;i++){
                GenericPIDF pidf = PIDFs.get(i);
                if (isStart()){
                    pidf.clearIntegral();
                    pidf.clearKalmanFilter();
                }
                if (parentActuator.isNewTarget()){
                    pidf.clearIntegral();
                }
                parentActuator.setPower(
                        pidf.getPIDFOutput(parentActuator.getInstantTarget(), parentActuator.getCurrentPosition()),
                        parentActuator.getPartNames()[i]
                );
            }
        }
        @Override
        public void stopProcedure(){
            parentActuator.setPower(0);
        }
    }
    public static class SQUID<E extends CRActuator<?>> extends ControlFunction<E>{
        public ArrayList<Double> kPs;
        public SQUID(double...kPs){
            Double[] tempKPs=new Double[kPs.length];
            Arrays.setAll(tempKPs,(int i)->(kPs[i]));
            this.kPs=new ArrayList<>(Arrays.asList(tempKPs));
        }
        @Override
        public void registerToParent(E actuator){
            super.registerToParent(actuator);
            for (int i=0;i<parentActuator.partNames.length-kPs.size();i++){
                kPs.add(kPs.get(kPs.size()-1));
            }
        }
        @Override
        protected void runProcedure() {
            for (int i=0;i<parentActuator.partNames.length;i++){
                double error = parentActuator.getInstantTarget()- parentActuator.getCurrentPosition();
                parentActuator.setPower(
                        kPs.get(i) * Math.sqrt(Math.abs(error))*Math.signum(error),
                        parentActuator.getPartNames()[i]
                );
            }
        }
        @Override
        public void stopProcedure(){
            parentActuator.setPower(0);
        }
    }
    public static class TrapezoidalMotionProfile<E extends Actuator<?>> extends ControlFunction<E>{
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
        @Override
        public void registerToParent(E actuator){
            super.registerToParent(actuator);
            currentTarget=actuator.getTarget();
        }
        public void setNewParams(double maxVelocity, double acceleration){
            this.MAX_VELOCITY=maxVelocity;
            this.ACCELERATION=acceleration;
            newParams=true;
        }
        @Override
        protected void runProcedure() {
            if (parentActuator.isNewTarget()||newParams||isStart()){
                if (isStart()){
                    instantTarget=parentActuator.getCurrentPosition();
                    lastLoopTime=timer.time();
                }
                newParams=false;
                createMotionProfile(parentActuator.getTarget());
            }
            parentActuator.setInstantTarget(runMotionProfileOnce());
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
        public HashMap<String,Double> getProfileData(){
            HashMap<String,Double> data = new HashMap<>();
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
        }
    }


    public static class ServoControl extends ControlFunction<BotServo>{ //Control function to get servos to their targets
        @Override
        protected void runProcedure() {
            parentActuator.setPosition(parentActuator.getInstantTarget());
        }
    }
    public static class CRBangBangControl<E extends CRActuator<?>> extends ControlFunction<E>{ //Likely will be used to get CRServos to their targets if they have no encoders with them
        private final Supplier<Double> powerFunc; //This control function moves the CRActuator to the target at a given power, which can change. That is stored here.
        public CRBangBangControl(double power){
            this.powerFunc=()->(power);
        }
        public CRBangBangControl(Supplier<Double> powerFunc){
            this.powerFunc=powerFunc;
        }
        @Override
        protected void runProcedure() {
            double currentPosition = parentActuator.getCurrentPosition();
            if (Math.abs(parentActuator.getInstantTarget()-currentPosition)>parentActuator.getErrorTol()){
                parentActuator.setPower(powerFunc.get()*Math.signum(parentActuator.getInstantTarget()-currentPosition));
            }
        }
    }
}
