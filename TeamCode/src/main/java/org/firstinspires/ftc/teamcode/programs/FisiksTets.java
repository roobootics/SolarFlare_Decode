package org.firstinspires.ftc.teamcode.programs;

import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robotconfigs.Fisiks;
import org.firstinspires.ftc.teamcode.robotconfigs.Inferno;


public class FisiksTets {
    public static void main(String[] args) throws InterruptedException {
        /*
        Fisiks.buildPhysics(new double[]{141.5,141.5,48},new Pose(72,72,0), new Vector(0,0), 1700);
        ElapsedTime timer = new ElapsedTime();
        Fisiks.State s = Fisiks.RK4.integrate(Math.toRadians(45),Math.toRadians(45),0.6);
        System.out.println("time "+timer.time());
        System.out.println(Fisiks.initSpeed);
        System.out.println(Fisiks.initSpin);
        System.out.println(s.tPos.x);
        System.out.println(s.tPos.y);
        System.out.println(s.tPos.z);
        */
        ElapsedTime timer = new ElapsedTime();
        double[] out = Fisiks.runPhysics(Inferno.BallPath.LOW,new double[]{141.5,141.5,48},new Pose(72,72,0), new Vector(new Pose(0,0)), 1800);
        Thread.sleep(1000);
        timer.reset();
        out = Fisiks.runPhysics(Inferno.BallPath.LOW,new double[]{141.5,141.5,48},new Pose(72,72,0), new Vector(new Pose(48,-48)), 1800);
        System.out.println("time "+timer.time());
        System.out.println("pitch "+out[0]);
        System.out.println("yaw "+out[1]);
        System.out.println("time "+out[2]);
        System.out.println("dist error "+Fisiks.Error.distError);
        System.out.println("side error "+Fisiks.Error.sideError);
        System.out.println("height error "+Fisiks.Error.heightError);

    }
}
