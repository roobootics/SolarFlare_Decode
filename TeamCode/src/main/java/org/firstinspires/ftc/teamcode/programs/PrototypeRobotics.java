package org.firstinspires.ftc.teamcode.programs;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;

@TeleOp
public class PrototypeRobotics extends LinearOpMode {
    public boolean isOn;
    public void runOpMode(){
        waitForStart();
        CRServo leftSideRoller = hardwareMap.get(CRServo.class,"sideRollerLeft");
        CRServo rightSideRoller = hardwareMap.get(CRServo.class,"sideRollerRight");
        while (opModeIsActive()){
            if (gamepad1.a && !gamepad1.aWasPressed()){
                isOn = !isOn;
            }
            if (isOn){
                leftSideRoller.setPower(1.0);
                rightSideRoller.setPower(1.0);
            } else {
                leftSideRoller.setPower(0.0);
                rightSideRoller.setPower(0.0);
            }
        }
    }
}
