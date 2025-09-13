package org.firstinspires.ftc.teamcode.presets;

import static org.firstinspires.ftc.teamcode.base.Components.timer;


import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

public abstract class TimeBasedLocalizers{
    public static class ServoTimeBasedLocalizer{ //Calculates position of a servo based on time
        private final double ABS_SERVO_SPEED;
        private double prevPosition;
        private double prevTime;
        private final double range;
        public ServoTimeBasedLocalizer(double servoSpeed, double initialTarget,double range){
            this.ABS_SERVO_SPEED=servoSpeed; this.prevPosition=initialTarget; this.range=range;
        }
        public double getCurrentPosition(Servo servo) {
            if (!Double.isNaN(servo.getPosition())){
                double targetPos = servo.getPosition();
                double servoSpeed = Math.signum(targetPos - prevPosition)*ABS_SERVO_SPEED;
                double time=timer.time();
                double change = servoSpeed*(time-prevTime);
                if (Math.signum(change)==1.0){
                    prevPosition=Math.min(targetPos,prevPosition+change);
                }
                else if (Math.signum(change)==-1.0){
                    prevPosition=Math.max(targetPos,prevPosition+change);
                }
                prevTime=time;
            }
            return prevPosition*this.range;
        }
    }
    public static class CRTimeBasedLocalizer<E extends DcMotorSimple>{ //Calculates position of a continuous rotation actuator based on time
        private final double ABS_SERVO_SPEED;
        private double prevPosition;
        private double prevTime;
        private double prevPower;
        public CRTimeBasedLocalizer(double servoSpeed){
            this.ABS_SERVO_SPEED=servoSpeed;
        }
        public double getCurrentPosition(E actuator){
            double servoSpeed = prevPower*ABS_SERVO_SPEED;
            double time=timer.time();
            prevPosition=prevPosition+servoSpeed*(time-prevTime);
            prevTime=time;
            prevPower=actuator.getPower();
            return prevPosition;
        }
    }
}
