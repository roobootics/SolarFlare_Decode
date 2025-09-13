package org.firstinspires.ftc.teamcode.base;

import static org.firstinspires.ftc.teamcode.base.Components.BotMotor;
import static org.firstinspires.ftc.teamcode.base.Components.actuators;
import static org.firstinspires.ftc.teamcode.base.Components.timer;
import static org.firstinspires.ftc.teamcode.base.Components.updateTelemetry;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.Objects;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.function.Supplier;
import java.util.stream.Collectors;

public abstract class Commands { //Command-based system
    public static final ParallelCommandExecutor executor=new ParallelCommandExecutor(); //This runs Commands
    public abstract static class Command { //Base class for any command
        private boolean isBusy = false; //Indicates whether the command is active or not
        private boolean isStart = true; //Commands know if they've just started running or not
        private boolean isEnabled = true;
        public void reset() {
            isStart = true;
        }

        final public boolean run() { //Actual method called to run the command, called repeatedly in a loop. Returns true if the command is not complete and false if it is.
            if (isEnabled){
                isBusy = runProcedure();
                isStart = false;
                if (!isBusy) {
                    reset(); //Auto-reset after command is completed or stopped.
                }
                return isBusy;
            }
            else{
                return false;
            }
        }

        protected abstract boolean runProcedure(); //This is where one codes what the command does

        public void stopProcedure() {
        } //This is where code is made for if the command is interrupted

        final public void stop() { //Actual method called to stop the command
            if (isBusy) {
                stopProcedure();
                isBusy = false;
                reset();
            }
        }
        final public void pause(){
            if (isBusy) {
                stopProcedure();
                isBusy = false;
            }
        }
        final public boolean isBusy(){
            return isBusy;
        }
        final public boolean isStart(){
            return isStart;
        }
        final public boolean isEnabled(){
            return isEnabled;
        }
        final public void enable(){
            isEnabled=true;
        }
        final public void disable(){
            pause();
            isEnabled=false;
        }
    }

    //TEMPLATE commandS

    public abstract static class PersistentCommand extends Command { //Command but it can't be reset if not completed
        @Override
        public void reset() {
            if (!isBusy()) {
                super.reset();
            }
        }
    }

    public static class InstantCommand extends Command { //Command that completes in one loop iteration
        private final Runnable procedure;

        public InstantCommand(Runnable procedure) {
            this.procedure = procedure;
        }

        @Override
        public boolean runProcedure() {
            procedure.run();
            return false;
        }
    }

    public static class ContinuousCommand extends Command { //Command that constantly runs the same way each loop iteration
        private final Runnable procedure;

        public ContinuousCommand(Runnable procedure) {
            this.procedure = procedure;
        }

        @Override
        public boolean runProcedure() {
            procedure.run();
            return true;
        }
    }

    public static class LambdaCommand extends Command { //This allows one to create a Command without making it its own class
        private final Supplier<Boolean> command;

        public LambdaCommand(Supplier<Boolean> command) {
            this.command = command;
        }

        @Override
        public boolean runProcedure() {
            return command.get();
        }
    }

    public abstract static class CompoundCommand extends Command { //Allows one to represent a sequence of commands as one atomic command. The main difference between this and a sequential command is that you can code custom stop functionality.
        Command group; //A subclass will assign a command to this.
        @Override
        protected boolean runProcedure() {
            if (isStart()) {
                group.reset();
            }
            return group.run();
        }
        protected void setGroup(Command group){
            this.group=group;
        }
        @Override
        public void stopProcedure() {
            group.stop();
        }
    }

    //PRELOADED commandS
    public static class CommandHolder extends Command{ //An command that can run a given command when run. The command it runs can be changed.
        //The idea behind this is that you can build sequences using an CommandHolder, and set, change, or remove the command inside the CommandHolder later on without having to rebuild the entire sequence again.
        private Command command = null;
        public CommandHolder(Command initialCommand){
            setCommand(initialCommand);
        }
        public CommandHolder(){}
        @Override
        protected boolean runProcedure() {
            if (Objects.nonNull(command)){
                return command.run();
            }
            else{
                return false;
            }
        }
        @Override
        public void stopProcedure(){
            command.stop();
        }
        public void setCommand(Command command){
            if (Objects.nonNull(this.command)){
                this.command.stop();
            }
            command.reset();
            this.command=command;
        }
        public Command getCommand(){
            return this.command;
        }
        public void removeCommand(){
            if (Objects.nonNull(this.command)){
                command.stop();
                command=null;
            }
        }
        public InstantCommand setCommandCommand(Command command){
            return new InstantCommand(()->this.setCommand(command));
        }
        public InstantCommand removeCommandCommand(){
            return new InstantCommand(this::removeCommand);
        }
    }
    public static class ContinuousCommandHolder extends CommandHolder{ //CommandHolder that won't stop running if it has no command inside or if the command inside has finished running; it will wait for another command to be inserted.
        @Override
        protected boolean runProcedure(){
            if (Objects.nonNull(getCommand())){
                if (!getCommand().run()){
                    removeCommand();
                }
            }
            return true;
        }
    }
    public static class PersistentContinuousCommandHolder extends ContinuousCommandHolder{ //ContinuousCommandHolder where one can only set its command if there is no command inside or if the command inside has finished running.
        @Override
        public void setCommand(Command command){
            if (Objects.isNull(getCommand())){
                super.setCommand(command);
            }
        }
        @Override
        final public void removeCommand(){}
    }
    public static class PowerOnCommand extends Command { //This command automatically activates each actuator's default control functions when they are first commanded
        private final HashMap<String, Boolean> actuatorsCommanded = new HashMap<>();
        @Override
        protected boolean runProcedure() {
            if (actuatorsCommanded.size()<actuators.size()) {
                for (String key : actuators.keySet()) {
                    if (Objects.requireNonNull(actuators.get(key)).getTarget() != 0 && !actuatorsCommanded.containsKey(key)) {
                        Objects.requireNonNull(actuators.get(key)).switchControl(Objects.requireNonNull(actuators.get(key)).getDefaultControlKey());
                        actuatorsCommanded.put(key, true);
                    }
                }
                return true;
            }
            else{
                disable();
                return false;
            }

        }
    }
    public static class SleepUntilTrue extends Command { //Sleeps until a is met or until an optional timeout time is reached
        private final Supplier<Boolean> condition;
        private final double timeout;
        private double startTime;

        public SleepUntilTrue(Supplier<Boolean> condition, double timeout) {
            this.condition = condition;
            this.timeout = timeout;
        }

        public SleepUntilTrue(Supplier<Boolean> condition) {
            this.condition = condition;
            this.timeout = Double.POSITIVE_INFINITY;
        }

        @Override
        protected boolean runProcedure() {
            if (isStart() && timeout != Double.POSITIVE_INFINITY) {
                startTime = timer.time();
            }
            return !condition.get() && (timer.time() - startTime) < timeout;
        }
    }

    public static class SleepCommand extends Command { //Sleeps for a set time
        private final double time;
        private double startTime;

        public SleepCommand(double time) {
            this.time = time;
        }

        @Override
        protected boolean runProcedure() {
            if (isStart()) {
                startTime = timer.time();
            }
            return (timer.time() - startTime) < time;
        }
    }
    public static class InterruptOnTimeout extends Command{ //Command to run the command passed to it, but interrupt it after it's been running for a given time
        private final Command command;
        private final SleepCommand sleepCommand;
        boolean commandEnded=false;
        public InterruptOnTimeout(double time, Command command){
            this.command=command;
            this.sleepCommand=new SleepCommand(time);
        }
        @Override
        protected boolean runProcedure() {
            if (isStart()){
                sleepCommand.reset();
                command.reset();
                commandEnded=false;
            }
            if (sleepCommand.run()){
                if (!commandEnded && !command.run()){
                    commandEnded=true;
                }
            }
            else{
                commandEnded=true;
                command.stop();
            }
            return !commandEnded;
        }
        @Override
        public void stopProcedure(){
            command.stop();
        }
    }
    public static class InterruptOnCondition extends Command{ //Command to run the command passed to it, but interrupt it after a given Supplier<Boolean> is met
        private final Command command;
        private final SleepUntilTrue sleepCommand;
        private boolean commandEnded=false;
        public InterruptOnCondition(Supplier<Boolean> condition, Command command){
            this.command=command;
            this.sleepCommand=new SleepUntilTrue(condition);
        }
        @Override
        protected boolean runProcedure() {
            if (isStart()){
                sleepCommand.reset();
                command.reset();
                commandEnded=false;
            }
            if (sleepCommand.run()){
                if (!commandEnded && !command.run()){
                    commandEnded=true;
                }
            }
            else{
                commandEnded=true;
                command.stop();
            }
            return !commandEnded;
        }
        @Override
        public void stopProcedure(){
            command.stop();
        }
    }
    public static class SequentialCommand extends Command{ //Runs commands sequentially
        private ArrayList<Command> remainingCommands;
        private final ArrayList<Command> commands;
        private final ArrayList<Boolean> isStarts;

        public SequentialCommand(Command... commands) {
            this.commands = new ArrayList<>(Arrays.asList(commands));
            this.remainingCommands = new ArrayList<>(this.commands);
            isStarts = new ArrayList<>(commands.length);
            for (Command command : commands) {
                isStarts.add(true);
            }
        }

        @Override
        public boolean runProcedure() {
            if (isStart()) {
                remainingCommands = new ArrayList<>(commands);
                for (int i = 0; i < commands.size(); i++) {
                    isStarts.set(i,true);
                }
            }
            if (isStarts.get(commands.size() - remainingCommands.size())) {
                remainingCommands.get(0).reset();
                isStarts.set(commands.size() - remainingCommands.size(),false);
            }
            if (!remainingCommands.get(0).run()) {
                remainingCommands.remove(0);
            }
            return !remainingCommands.isEmpty();
        }

        @Override
        public void stopProcedure() {
            remainingCommands.get(0).stop();
        }
        public Command getCurrentAction(){
            if (remainingCommands.isEmpty()){
                return commands.get(0);
            }
            return remainingCommands.get(0);
        }
        public int getCurrentActionIndex(){
            if (remainingCommands.isEmpty()){
                return 0;
            }
            return commands.size()-remainingCommands.size();
        }
    }

    public static class ParallelCommand extends Command{ //Runs commands in parallel
        private ArrayList<Command> remainingCommands;
        protected final ArrayList<Command> commands;
        public ParallelCommand(Command... commands) {
            this.commands = new ArrayList<>(Arrays.asList(commands));
            this.remainingCommands = new ArrayList<>(this.commands);
        }

        @Override
        public boolean runProcedure() {
            if (isStart()) {
                remainingCommands = new ArrayList<>(commands);
                for (Command command : remainingCommands) {
                    command.reset();
                }
            }
            ArrayList<Command> removingCommands = new ArrayList<>();
            for (Command command:remainingCommands){
                if(!command.run()){
                    removingCommands.add(command);
                }
            }
            for (Command command:removingCommands){
                remainingCommands.remove(command);
            }
            return !remainingCommands.isEmpty();
        }

        @Override
        public void stopProcedure() {
            for (Command command : remainingCommands) {
                command.stop();
            }
        }
    }

    public static class IfThen { //Holds a Supplier<Boolean> and a command to be executed if it is met
        private final Supplier<Boolean> condition;
        private final Command command;

        public IfThen(Supplier<Boolean> condition, Command command) {
            this.condition = condition;
            this.command = command;
        }
    }

    public static class ConditionalCommand extends Command{ //Executes commands if their respective Conditions are met, in an if,else-if,else manner. Only one command can run at a time. If one command's Supplier<Boolean> stops being met, it will finish, unless another command's Supplier<Boolean> starts being met, in which case it will stop and switch to that command
        protected final LinkedHashMap<Supplier<Boolean>, Command> commands = new LinkedHashMap<>();
        protected Command currentCommand = null;
        public ConditionalCommand(IfThen... ConditionalPairs) {
            for (IfThen ConditionalPair : ConditionalPairs) {
                commands.put(ConditionalPair.condition, ConditionalPair.command);
            }
        }

        @Override
        protected boolean runProcedure() {
            if (isStart()) {
                for (Supplier<Boolean> condition : commands.keySet()) {
                    if (condition.get()) {
                        if (commands.get(condition) != currentCommand) {
                            if (Objects.nonNull(currentCommand)) {
                                currentCommand.stop();
                            }
                            currentCommand=commands.get(condition);
                        }
                        if (Objects.nonNull(currentCommand)) {
                            currentCommand.reset();
                        }
                        break;
                    }
                }
            } else {
                for (Supplier<Boolean> condition : commands.keySet()) {
                    if (condition.get()) {
                        if (commands.get(condition) != currentCommand) {
                            currentCommand.stop();
                            currentCommand = commands.get(condition);
                            assert currentCommand != null;
                            currentCommand.reset();
                        }
                        break;
                    }
                }
            }
            if (Objects.nonNull(currentCommand)) {
                if (!currentCommand.run()) {
                    currentCommand = null;
                    return false;
                } else {
                    return true;
                }
            } else return false;
        }
        public Command getCurrentCommand(){
            return currentCommand;
        }
        @Override
        public void stopProcedure(){
            if (Objects.nonNull(currentCommand)){
                currentCommand.stop();
            }
        }
    }

    public static class PersistentConditionalCommand extends PersistentCommand{ //ConditionalCommand, but a command cannot be interrupted
        protected final LinkedHashMap<Supplier<Boolean>, Command> commands = new LinkedHashMap<>();
        protected Command currentCommand = null;

        public PersistentConditionalCommand(IfThen... ConditionalPairs) {
            for (IfThen ConditionalPair : ConditionalPairs) {
                commands.put(ConditionalPair.condition, ConditionalPair.command);
            }
        }

        @Override
        protected boolean runProcedure() {
            if (Objects.isNull(currentCommand)) {
                for (Supplier<Boolean> condition : commands.keySet()) {
                    if (condition.get()) {
                        currentCommand = commands.get(condition);
                        assert currentCommand != null;
                        currentCommand.reset();
                        break;
                    }
                }
            }
            if (Objects.nonNull(currentCommand)) {
                if (!currentCommand.run()) {
                    currentCommand = null;
                    return false;
                } else {
                    return true;
                }
            } else return false;
        }
        public Command getCurrentCommand(){
            return currentCommand;
        }
        @Override
        public void stopProcedure(){
            if (Objects.nonNull(currentCommand)){
                currentCommand.stop();
            }
        }
    }

    public static class SemiPersistentConditionalCommand extends Command{ //ConditionalCommand, but a command can only be interrupted if the SemiPersistentConditionalCommand is reset()
        private final LinkedHashMap<Supplier<Boolean>, Command> commands = new LinkedHashMap<>();
        private Command currentCommand = null;

        public SemiPersistentConditionalCommand(IfThen... ConditionalPairs) {
            for (IfThen ConditionalPair : ConditionalPairs) {
                commands.put(ConditionalPair.condition, ConditionalPair.command);
            }
        }

        @Override
        protected boolean runProcedure() {
            if (isStart()) {
                for (Supplier<Boolean> condition : commands.keySet()) {
                    if (condition.get()) {
                        if (commands.get(condition) != currentCommand) {
                            if (Objects.nonNull(currentCommand)) {
                                currentCommand.stop();
                            }
                            currentCommand = commands.get(condition);
                            if (Objects.nonNull(currentCommand)) {
                                currentCommand.reset();
                            }
                        }
                        break;
                    }
                }
            }
            if (Objects.nonNull(currentCommand)) {
                if (!currentCommand.run()) {
                    currentCommand = null;
                    return false;
                } else {
                    return true;
                }
            } else return false;
        }
        public Command getCurrentCommand(){
            return currentCommand;
        }
        @Override
        public void stopProcedure(){
            if (Objects.nonNull(currentCommand)){
                currentCommand.stop();
            }
        }
    }

    public static class PressTrigger extends ConditionalCommand { //ConditionalCommand, but all Conditions are converted into button-presses, such that they will not return 'true' two loop-iterations in a row.
        private final ArrayList<Boolean> isPressed;
        public PressTrigger(IfThen... ConditionalPairs) {
            super(ConditionalPairs);
            commands.clear();
            isPressed = new ArrayList<>();
            for (int i = 0; i < ConditionalPairs.length; i++) {
                int finalI = i;
                isPressed.add(false);
                commands.put(
                        () -> {
                            if (ConditionalPairs[finalI].condition.get()) {
                                if (!isPressed.get(finalI)){
                                    isPressed.set(finalI,true);
                                    return true;
                                }
                                else{
                                    return false;
                                }
                            } else {
                                isPressed.set(finalI,false);
                                return false;
                            }
                        },
                        ConditionalPairs[i].command
                );
            }
        }
    }

    public static class PersistentPressTrigger extends PersistentConditionalCommand { //PressTrigger but persistent
        private final ArrayList<Boolean> isPressed;
        public PersistentPressTrigger(IfThen... ConditionalPairs) {
            super(ConditionalPairs);
            commands.clear();
            isPressed = new ArrayList<>();
            for (int i = 0; i < ConditionalPairs.length; i++) {
                int finalI = i;
                isPressed.add(false);
                commands.put(
                        () -> {
                            if (ConditionalPairs[finalI].condition.get()) {
                                isPressed.set(finalI,true);
                                return !isPressed.get(finalI);
                            } else {
                                isPressed.set(finalI,false);
                                return false;
                            }
                        },
                        ConditionalPairs[i].command
                );
            }
        }
    }
    public static class LoopForDuration extends ParallelCommand { //Loops a command for a certain duration
        private double startTime;
        private final double duration;
        public LoopForDuration(double duration, Command...commands) {
            super(commands);
            this.duration = duration;
        }

        @Override
        public boolean runProcedure() {
            if (isStart()) {
                startTime = timer.time();
                reset();
            }
            if ((timer.time() - startTime) < duration) {
                for (Command command : commands){
                    command.run();
                }
                return true;
            } else {
                stop();
                return false;
            }
        }
    }
    public static class LoopUntilTrue extends ParallelCommand { //Loops a command until a Supplier<Boolean> is met, or until an optional timeout is reached
        private double startTime;
        private final Supplier<Boolean> condition;
        private final double timeout;

        public LoopUntilTrue(Supplier<Boolean> condition, double timeout, Command... commands) {
            super(commands);
            this.condition = condition;
            this.timeout=timeout;
        }
        public LoopUntilTrue(Supplier<Boolean> condition, Command... commands) {
            this(condition,Double.POSITIVE_INFINITY,commands);
        }
        @Override
        public boolean runProcedure() {
            if (isStart()) {
                startTime = timer.time();
                reset();
            }
            if (!condition.get() && (timer.time() - startTime) < timeout) {
                for (Command command : commands){
                    command.run();
                }
                return true;
            } else {
                stop();
                return false;
            }
        }
    }
    public static class ResetAndLoopForDuration extends ParallelCommand { //Loops a command for a certain duration
        private double startTime;
        private final double duration;

        public ResetAndLoopForDuration(double duration, Command... commands) {
            super(commands);
            this.duration = duration;
        }

        @Override
        public boolean runProcedure() {
            if (isStart()) {
                startTime = timer.time();
            }
            if ((timer.time() - startTime) < duration) {
                reset(); super.runProcedure();
                return true;
            } else {
                stop();
                return false;
            }
        }
    }
    public static class ResetAndLoopUntilTrue extends ParallelCommand { //Loops a command until a Supplier<Boolean> is met, or until an optional timeout is reached
        private double startTime;
        private final Supplier<Boolean> condition;
        private final double timeout;

        public ResetAndLoopUntilTrue(Supplier<Boolean> condition, double timeout, Command...commands) {
            super(commands);
            this.condition = condition;
            this.timeout=timeout;
        }
        public ResetAndLoopUntilTrue(Supplier<Boolean> condition, Command... commands) {
            this(condition,Double.POSITIVE_INFINITY,commands);
        }
        @Override
        public boolean runProcedure() {
            if (isStart()) {
                startTime = timer.time();
            }
            if (!condition.get() && (timer.time() - startTime) < timeout) {
                reset(); super.runProcedure();
                return true;
            } else {
                stop();
                return false;
            }
        }
    }
    public static class StallResetOnStall extends CompoundCommand{
        public BotMotor[] motors;
        public double[] stallVolts;
        public double[] resetPositions;
        public StallResetOnStall(BotMotor[] motors, double[] stallVolts, double[] resetPositions){
            this.motors=motors;
            this.stallVolts=stallVolts;
            this.resetPositions = resetPositions;
            ConditionalCommand[] commands = new ConditionalCommand[motors.length];
            for (int i=0;i<motors.length;i++){
                int finalI = i;
                commands[i]=new ConditionalCommand(
                        new IfThen(
                                ()->((!motors[finalI].isStallResetting()) && (motors[finalI].getCurrentAmps()>stallVolts[finalI])),
                                motors[finalI].stallResetCommand(resetPositions[finalI],stallVolts[finalI])
                        )
                );
            }
            this.group = new RunLoop(
                    commands
            );
        }
    }
    public abstract static class SleepUntilPose extends SleepUntilTrue { //Sleeps until the drivetrain and heading get a certain distance from a desired position and heading, or until an optional timeout is reached. Meant to be subclassed depending on the pathing library.
        public static Supplier<double[]> getPose; //Subclasses assign this to a method that can get the drivetrain position, returning x, y, and heading.
        public static void setGetPose(Supplier<double[]> getPose){
            SleepUntilPose.getPose=getPose;
        }
        public SleepUntilPose(double x, double y, double heading, double poseDistance, double headingDistance, double timeout) {
            super(() -> {
                double[] pose = getPose.get();
                return Math.sqrt((x - pose[0]) * (x - pose[0]) + (y - pose[1]) * (y - pose[1])) < poseDistance &&
                        Math.abs(heading - pose[2]) < headingDistance;
            }, timeout);
        }

        public SleepUntilPose(double x, double y, double heading, double poseDistance, double headingDistance) {
            super(() -> {
                double[] pose = getPose.get();
                return Math.sqrt((x - pose[0]) * (x - pose[0]) + (y - pose[1]) * (y - pose[1])) < poseDistance &&
                        Math.abs(heading - pose[2]) < headingDistance;
            });
        }
    }

    public abstract static class PathCommand<E> extends Command { //Command for autonomous pathing. Must be subclassed to create an implementation for a specific autonomous library. Parameterized to the actual path object it is based off of.
        private final Supplier<E> buildPath;
        public static boolean buildPathOnInit=false;
        private boolean constructPathOnRuntime=true;
        private E path; //Stores the path this command follows. For RR it would be a TrajectoryCommand, for Pedro it would be a PathChain
        public PathCommand(Supplier<E> buildPath) {
            this.buildPath = buildPath;
            if (buildPathOnInit){
                constructPathOnRuntime=false;
                path=buildPath.get();
            }
        }
        @Override
        protected boolean runProcedure() {
            if (isStart()) {
                preBuild();
                if (constructPathOnRuntime){
                    path = buildPath.get(); //Path is built when the command needs to run (useful for RoadRunner)
                }
            }
            return followPath();
        }

        public abstract boolean followPath(); //Here, one implements the autonomous library's method of following paths. The function must return true if the path is still being followed, and false if it has finished

        public void preBuild() {
        } //Here, one can code anything that must occur right before the path is built.
        public E getPath(){
            return path;
        }
        public void buildPath(){
            path=buildPath.get();
            constructPathOnRuntime=false;
        }
    }

    public static class RobotCentricMecanumCommand extends Command { //Command for robot-centric TeleOp drivetrain control
        private final Supplier<Double> xFun;
        private final Supplier<Double> yFun;
        private final Supplier<Double> rxFun;
        private final Supplier<Double> slowDownFun;
        private final BotMotor[] motors;

        public RobotCentricMecanumCommand(BotMotor[] motors, Supplier<Double> xFun, Supplier<Double> yFun, Supplier<Double> rxFun, Supplier<Double> slowDownFun) {
            this.xFun = xFun;
            this.yFun = yFun;
            this.rxFun = rxFun;
            this.slowDownFun = slowDownFun;
            this.motors = motors;
        }

        public RobotCentricMecanumCommand(BotMotor[] motors, Supplier<Double> xFun, Supplier<Double> yFun, Supplier<Double> rxFun) {
            this(motors, xFun, yFun, rxFun, ()->(1.0));
        }

        @Override
        public boolean runProcedure() {
            double y = -yFun.get();
            double x = xFun.get();
            double rx = -rxFun.get();

            double botHeading = 0;

            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            rotX = rotX * 1.1;  // Counteract imperfect strafing

            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double slowDownAmount=slowDownFun.get();
            double frontLeftPower = slowDownAmount*(rotY + rotX + rx) / denominator;
            double backLeftPower = slowDownAmount*(rotY - rotX + rx) / denominator;
            double frontRightPower = slowDownAmount*(rotY - rotX - rx) / denominator;
            double backRightPower = slowDownAmount*(rotY + rotX - rx) / denominator;

            motors[0].setPower(frontLeftPower);
            motors[1].setPower(backLeftPower);
            motors[2].setPower(frontRightPower);
            motors[3].setPower(backRightPower);
            return true;
        }
    }

    public static class FieldCentricMecanumCommand extends Command { //Command for field-centric TeleOp drivetrain control
        private final Supplier<Double> xFun;
        private final Supplier<Double> yFun;
        private final Supplier<Double> rxFun;
        private final Supplier<Double> slowDownFun;
        private final BotMotor[] motors;
        private final Supplier<Double> getHeading;

        public FieldCentricMecanumCommand(BotMotor[] motors, Supplier<Double> getHeading, int imuPollingRate, Supplier<Double> xFun, Supplier<Double> yFun, Supplier<Double> rxFun, Supplier<Double> slowDownFun) {
            this.xFun = xFun;
            this.yFun = yFun;
            this.rxFun = rxFun;
            this.slowDownFun = slowDownFun;
            this.motors = motors;
            this.getHeading = new Components.CachedReader<>(getHeading,imuPollingRate)::cachedRead;
        }

        public FieldCentricMecanumCommand(BotMotor[] motors, Supplier<Double> getHeading, int imuPollingRate, Supplier<Double> xFun, Supplier<Double> yFun, Supplier<Double> rxFun) {
            this(motors, getHeading, imuPollingRate, xFun, yFun, rxFun, ()->(1.0));
        }

        @Override
        public boolean runProcedure() {
            double y = -yFun.get();
            double x = xFun.get();
            double rx = -rxFun.get();

            double botHeading = getHeading.get();

            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            rotX = rotX * 1.1;  // Counteract imperfect strafing

            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double slowDownAmount=slowDownFun.get();
            double frontLeftPower = slowDownAmount*(rotY + rotX + rx) / denominator;
            double backLeftPower = slowDownAmount*(rotY - rotX + rx) / denominator;
            double frontRightPower = slowDownAmount*(rotY - rotX - rx) / denominator;
            double backRightPower = slowDownAmount*(rotY + rotX - rx) / denominator;

            motors[0].setPower(frontLeftPower);
            motors[1].setPower(backLeftPower);
            motors[2].setPower(frontRightPower);
            motors[3].setPower(backRightPower);
            return true;
        }
    }
    public static RunResettingLoop triggeredToggleCommand(Supplier<Boolean> condition, Command command1, Command command2){
        AtomicBoolean state = new AtomicBoolean(true);
        command1 = new SequentialCommand(new InstantCommand(()->state.set(!state.get())), command1);
        command2 = new SequentialCommand(new InstantCommand(()->state.set(!state.get())), command2);
        return new RunResettingLoop(new PressTrigger(
                new IfThen(condition,
                        new SemiPersistentConditionalCommand(
                                new IfThen(state::get,command1),
                                new IfThen(()->(!state.get()),command2)
                        )
                )
        ));
    }
    public static RunResettingLoop triggeredFSMCommand(Supplier<Boolean> upCondition, Supplier<Boolean> downCondition, int startingState, Command...commands){
        if (startingState>commands.length){
            startingState= commands.length;
        }
        else if (startingState<0){
            startingState=0;
        }
        AtomicInteger state = new AtomicInteger(-1+startingState);
        IfThen[] upIfThens = new IfThen[commands.length];
        for (int i=0;i<upIfThens.length;i++){
            int finalI = i;
            upIfThens[i]=new IfThen(
                    ()->(state.get()==finalI),
                    commands[i]
            );
        }
        IfThen[] downIfThens = new IfThen[commands.length];
        for (int i=0;i<downIfThens.length;i++){
            int finalI = i;
            downIfThens[i]=new IfThen(
                    ()->(state.get()==finalI),
                    commands[i]
            );
        }
        return new RunResettingLoop(new PressTrigger(
                new IfThen(upCondition,
                        new SequentialCommand(
                                new InstantCommand(()->{if (state.get()<commands.length-1){state.set(state.get()+1);}}),
                                new SemiPersistentConditionalCommand(
                                        upIfThens
                                )
                        )
                ),
                new IfThen(downCondition,
                    new SequentialCommand(
                            new InstantCommand(()->{if (state.get()>0){state.set(state.get()-1);}}),
                            new SemiPersistentConditionalCommand(
                                    downIfThens
                            )
                    )
                )
        ));
    }
    public static RunResettingLoop triggeredCycleCommand(Supplier<Boolean> condition, Command...commands){
        AtomicInteger state = new AtomicInteger(0);
        IfThen[] ifThens=new IfThen[commands.length];
        for (int i=0;i< ifThens.length;i++){
            int finalI = i;
            ifThens[i]=new IfThen(
                    ()->(state.get()==finalI),
                    new SequentialCommand(new InstantCommand(()->{
                        if (state.get()==commands.length-1){
                            state.set(0);
                        }
                        else{state.set(state.get()+1);}
                    }), commands[finalI])
            );
        }
        return new RunResettingLoop(new PressTrigger(
                new IfThen(condition,
                    new SemiPersistentConditionalCommand(
                        ifThens
                    )
                )
        ));
    }
    public static RunResettingLoop triggeredDynamicCommand(Supplier<Boolean> upCondition, Supplier<Boolean> downCondition, Command command1, Command command2){
        return new RunResettingLoop(
                new ConditionalCommand(
                        new IfThen(
                                upCondition,
                                command1
                        ),
                        new IfThen(
                                downCondition,
                                command2
                        )
                )
        );
    }
    public static class RunResettingLoop extends ParallelCommand{ //Group of commands that runs commands in parallel in a while loop and resets them each iteration (used for TeleOp)
        public RunResettingLoop(Command...commands){
            super(commands);
        }
        public boolean runProcedure(){
            reset(); super.runProcedure();
            return true;
        }
        public void stopProcedure(){
            for (Command command : commands){
                command.stop();
            }
        }
    }
    public static class RunLoop extends ParallelCommand{ //Group of commands that runs commands in parallel in a while loop
        public RunLoop(Command...commands){
            super(commands);
        }
        public boolean runProcedure(){
            for (Command command : commands){
                command.run();
            }
            return true;
        }
        public void stopProcedure(){
            for (Command command : commands){
                command.stop();
            }
        }
    }
    public static class ParallelCommandExecutor{
        private ArrayList<Command> commands = new ArrayList<>();
        private final ArrayList<Command> commandsToAdd = new ArrayList<>();
        private final ArrayList<Command> commandsToRemove = new ArrayList<>();
        private Runnable writeToTelemetry = ()->{};
        private ParallelCommandExecutor(){
        }
        public void setCommands(Command...commandGroups){
            this.commands=new ArrayList<>(Arrays.asList(commandGroups));
        }
        public void clearCommands(){
            this.commands.clear();
        }
        public ArrayList<Command> getCommands(){
            return new ArrayList<>(this.commands);
        }
        public void setWriteToTelemetry(Runnable procedure){
            this.writeToTelemetry=procedure;
        }
        public void runOnce(){
            this.commands.addAll(commandsToAdd);
            this.commands.removeAll(commandsToRemove);
            commandsToAdd.clear();
            commandsToRemove.clear();
            this.commands = commands.stream().filter(Command::run).collect(Collectors.toCollection(ArrayList::new));
            for (Components.Actuator<?> actuator : actuators.values()) {
                //This ensures that old targets do not fall outside of any new max or min targets.
                actuator.setTarget(actuator.getTargetMinusOffset());
                if (actuator instanceof Components.CRActuator) {
                    Components.CRActuator<?> castedActuator = ((Components.CRActuator<?>) actuator);
                    for (String name:castedActuator.getPartNames()){
                        castedActuator.setPower(castedActuator.getPower(name),name);
                    }
                }
                //This ensures that old powers do not fall outside of any new max or min targets.
                actuator.runControl();
                actuator.resetNewTarget(); actuator.resetNewActuation();
            }
            Components.CachedReader.updateResetAllCaches();
            writeToTelemetry.run();
            updateTelemetry();
        }
        public void runLoop(Supplier<Boolean> condition){
            while (condition.get()){
                runOnce();
            }
            stop();
        }
        public void stop(){
            for (Command command : commands){
                command.stop();
            }
        }
        public void addCommand(Command command){
            command.reset();
            commandsToAdd.add(command);
        }
        public void removeCommand(Command command){
            command.stop();
            if (this.commands.contains(command)) {
                commandsToRemove.add(command);
            }
        }
    }
}
