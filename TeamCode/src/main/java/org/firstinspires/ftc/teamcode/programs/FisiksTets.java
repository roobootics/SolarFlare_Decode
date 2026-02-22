package org.firstinspires.ftc.teamcode.programs;

import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robotconfigs.Fisiks;
import org.firstinspires.ftc.teamcode.robotconfigs.Inferno;


public class FisiksTets {
    public static void main(String[] args) throws InterruptedException {
        ElapsedTime timer = new ElapsedTime();
        Fisiks.runPhysics(Inferno.BallPath.LOW,new double[]{141.5,141.5,48},new Pose(72,72,0), new Vector(new Pose(0,0)), 1800);
        timer.reset();
        double[] out = Fisiks.runPhysics(Inferno.BallPath.HIGH,new double[]{141.5,141.5,48},new Pose(72,0,0), new Vector(new Pose(0,0)), 2100);
        System.out.println("time "+timer.time());
        System.out.println("pitch "+out[0]);
        System.out.println("yaw "+out[1]);
        System.out.println("time "+out[2]);
        System.out.println("dist error "+Fisiks.Error.distError);
        System.out.println("side error "+Fisiks.Error.sideError);
        System.out.println("height error "+Fisiks.Error.heightError);

    }
}
