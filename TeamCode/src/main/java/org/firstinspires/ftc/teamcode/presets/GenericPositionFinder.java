package org.firstinspires.ftc.teamcode.presets;

import static org.firstinspires.ftc.teamcode.base.Components.actuators;
import static org.firstinspires.ftc.teamcode.base.Commands.executor;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.base.Components;

import org.firstinspires.ftc.teamcode.base.Commands;

import java.util.ArrayList;
import java.util.Objects;
//Dpad left/right to switch between actuators. If the actuator is a servo, use left/right bumper to move it and see the position.
public abstract class GenericPositionFinder extends LinearOpMode { //Used to find the specific positions that we will end up setting actuators to. Allows one to select an actuator and move it.
    //Subclass this class, such that in the subclass's runOpMode, all robot state is initialized before the super's runOpMode runs.
    private int selectedActuatorIndex = 0;
    protected double dynamicChangeAmount=0.1;
    private final ArrayList<String> actuatorNames = new ArrayList<>();
    public void updateTelemetry(){
        Components.telemetryAddLine(actuatorNames.get(selectedActuatorIndex));
        Components.telemetryAddData("position", Objects.requireNonNull(actuators.get(actuatorNames.get(selectedActuatorIndex))).getCurrentPosition());
    }
    public void shiftSelectionRight(){
        if (selectedActuatorIndex<actuatorNames.size()-1){
            selectedActuatorIndex+=1;
        }
    }
    public void shiftSelectionLeft(){
        if (selectedActuatorIndex>0){
            selectedActuatorIndex-=1;
        }
    }
    @Override
    public void runOpMode(){
        for (String name:actuators.keySet()){
            if (!(actuators.get(name) instanceof Components.BotMotor)){
                Objects.requireNonNull(actuators.get(name)).switchControl(Objects.requireNonNull(actuators.get(name)).getDefaultControlKey());
            }
            else{
                Components.BotMotor motor = Objects.requireNonNull((Components.BotMotor) actuators.get(name));
                motor.resetEncoders();
                motor.setZeroPowerFloat();
                motor.lockTargetState();
                //Motors are disabled; to find their positions you move them manually and read the encoder
            }
            actuatorNames.add(name);
        }
        Commands.IfThen[] conditions = new Commands.IfThen[actuatorNames.size()];
        for (int i=0;i<actuatorNames.size();i++){
            int finalI = i;
            conditions[i]=new Commands.IfThen(
                    ()->(selectedActuatorIndex==finalI),
                    Objects.requireNonNull(actuators.get(actuatorNames.get(i))).triggeredDynamicTargetCommand(()->(gamepad1.left_bumper),()->(gamepad1.right_bumper),dynamicChangeAmount)
            );
        }

        waitForStart();
        executor.setWriteToTelemetry(this::updateTelemetry);
        executor.setCommands(
                new Commands.RunResettingLoop(
                        new Commands.PressTrigger(new Commands.IfThen(
                                ()->(gamepad1.dpad_left),
                                new Commands.InstantCommand(this::shiftSelectionLeft)
                        )),
                        new Commands.PressTrigger(new Commands.IfThen(
                                ()->(gamepad1.dpad_right),
                                new Commands.InstantCommand(this::shiftSelectionRight)
                        )),
                        new Commands.ConditionalCommand(
                                conditions
                        )
                ),
                new Commands.PowerOnCommand()
        );
        executor.runLoop(this::opModeIsActive);
    }
}
