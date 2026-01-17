package org.firstinspires.ftc.teamcode.programs;

import static org.firstinspires.ftc.teamcode.base.Components.telemetryAddData;
import static java.lang.Math.atan2;
import static java.lang.Math.sqrt;

import org.firstinspires.ftc.teamcode.robotconfigs.Inferno;

public class PhysicsTets {
    public static double[] target = new double[]{141.5,141.5,50};
    private abstract static class Physics {
        final static double GRAVITY = -386.089;
        final static double FRICTION = 0.3;
        final static double HEIGHT = 10.748;
        final static double TICKS_TO_RAD = 2*Math.PI/28;
        final static double WHEEL_RAD = 1.41732;
        final static double BALL_RAD = 2.5;
        public static double[] runPhysics(Inferno.BallPath currentBallPath){

            double xPos = 72;
            double yPos = 72;
            double xVel = 0;
            double yVel = 0;

            double currWheelVel = TICKS_TO_RAD*2100;
            double initSpeed = FRICTION*currWheelVel*WHEEL_RAD;
            double xDist = sqrt((target[0]-xPos)*(target[0]-xPos) + (target[1]-yPos)*(target[1]-yPos));
            double yDist = target[2] - HEIGHT;
            double TOTAL_RAD = WHEEL_RAD+BALL_RAD;

            double discriminant = sqrt(initSpeed*initSpeed*initSpeed*initSpeed + 2*yDist*GRAVITY*initSpeed*initSpeed + GRAVITY*GRAVITY*(TOTAL_RAD*TOTAL_RAD - (xDist - TOTAL_RAD)*(xDist - TOTAL_RAD)));
            if (currentBallPath == Inferno.BallPath.LOW) discriminant*=-1;
            double shotTime = sqrt(2*(initSpeed*initSpeed + yDist*GRAVITY + discriminant)/(GRAVITY*GRAVITY));
            double turretPitch = atan2(
                    initSpeed*shotTime*(yDist - 0.5*GRAVITY*shotTime*shotTime) - TOTAL_RAD*(xDist - TOTAL_RAD),
                    initSpeed*shotTime*(xDist - TOTAL_RAD) + TOTAL_RAD*(yDist - 0.5*GRAVITY*shotTime*shotTime)
            );

            double turretYaw = atan2(target[1] - shotTime*yVel - yPos, target[0] - shotTime*xVel - xPos);
            if (Double.isNaN(turretYaw)) turretYaw =  atan2(target[1] - yPos, target[0] - xPos); if (Double.isNaN(turretPitch)) turretPitch = Math.toRadians(47);
            if (turretYaw>=Math.PI) turretYaw-=2*Math.PI; if (turretPitch<=0) turretPitch+=2*Math.PI;

            telemetryAddData("NaN",Double.isNaN(shotTime));
            System.out.println(initSpeed/36);
            System.out.println(shotTime);
            System.out.println(Math.toDegrees(turretPitch));
            System.out.println(Math.toDegrees(turretYaw));
            return new double[]{Math.toDegrees(turretPitch),Math.toDegrees(turretYaw)};
        }
    }
    public static void main(String[] args){
        Physics.runPhysics(Inferno.BallPath.LOW);
        Physics.runPhysics(Inferno.BallPath.HIGH);
    }
}
