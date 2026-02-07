package org.firstinspires.ftc.teamcode.programs;

import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.setState;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.base.Commands;
import org.firstinspires.ftc.teamcode.robotconfigs.Inferno;

import java.util.HashMap;
import java.util.Objects;

public class PrimeSigmaConstants {
    public static final double INITIAL_WAIT = 0.01;
    public static final double SHOT_TIME = 1.2;
    public static final double slowDownT = 0.73;
    public static final double speedUpT = 0.13;
    public static final double stopIntakeT = 0.5;
    public static final double slowDownAmount = 0.67;
    public static final double gateWait = 0.4;
    public static final double gateIntakeTimeout = 1;
    public static final double fourthShootSlowT = 0.73;
    public static final double fourthShootSlowAmount = 0.76;
    public static Commands.Command shoot = new Commands.ParallelCommand(setState(Inferno.RobotState.SHOOTING), new Commands.SleepCommand(SHOT_TIME));
    public static final HashMap<String, Pose> poses = new HashMap<>();
    public static Pose mirrorPose(Pose input){
        return new Pose(144-input.getX(),input.getY(),mirrorHeading(input.getHeading()));
    }
    public static double mirrorHeading(double input){
        return 2*Math.PI - input;
    }
    static {
        poses.put("start",new Pose(20, 122.62, Math.toRadians(143.5)));
        poses.put("firstShoot",new Pose(53.312,90.398,Math.toRadians(136)));
        poses.put("secondSpikeCtrl",new Pose(88.046, 59.413));
        poses.put("secondSpike",new Pose(16.329, 58.805));
        poses.put("secondShootCtrl",new Pose(54.282, 64.593));
        poses.put("secondShoot",new Pose(53.076, 87.680,Math.toRadians(0)));
        poses.put("gateOpenCtrl",new Pose(39.404, 37.087));
        poses.put("gateOpen",new Pose(17.669, 60.486,Math.toRadians(-25)));
        poses.put("gateIntake",new Pose(16.201, 52.334,Math.toRadians(-50)));
        poses.put("thirdShootCtrl",new Pose(44.169, 52.575));
        poses.put("thirdShoot",new Pose(53.370, 87.903,Math.toRadians(0)));
        poses.put("firstSpike",new Pose(22.773, 79.829,Math.toRadians(0)));
        poses.put("fourthShoot",new Pose(53.451, 87.203,Math.toRadians(0)));
        poses.put("thirdSpikeCtrl",new Pose(80.067, 27.483));
        poses.put("thirdSpike",new Pose(13.504, 36.131,Math.toRadians(0)));
        poses.put("fifthShoot",new Pose(53.177, 87.918,Math.toRadians(360)));
        poses.put("park",new Pose(45,79,Math.toRadians(360)));
    }
    public static Pose getPose(String input){return poses.get(input);}
    public static Pose getMirroredPose(String input){return Objects.requireNonNull(mirrorPose(Objects.requireNonNull(poses.get(input))));}
    public static double getHeading(String input){return Objects.requireNonNull(poses.get(input)).getHeading();}
    public static double getMirroredHeading(String input){return mirrorHeading(Objects.requireNonNull(poses.get(input)).getHeading());}
}
