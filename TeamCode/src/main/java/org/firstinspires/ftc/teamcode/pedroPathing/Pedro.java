package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.base.Commands;
import org.firstinspires.ftc.teamcode.base.Components;

import java.util.function.Function;

public abstract class Pedro {
    public static Follower follower;

    static{
        PedroSleepUntilPose.setGetPose(()->{
            Pose pose=follower.getPose();
            return new double[]{pose.getX(),pose.getY(),pose.getHeading()};
        });
    }
    public static void createFollower(Pose startingPose){
        follower=Constants.createFollower(Components.getHardwareMap());
        follower.setStartingPose(startingPose);
        Drawing.init();
    }
    public static Commands.RunResettingLoop updateCommand(){
        return new Commands.RunResettingLoop(new Commands.InstantCommand(follower::update));
    }
    public static class PedroCommand extends Commands.PathCommand<PathChain>{
        private final boolean holdEnd;
        public PedroCommand(Function<PathBuilder,PathBuilder> buildPath, boolean holdEnd) {
            super(()->buildPath.apply(follower.pathBuilder()).build());
            this.holdEnd=holdEnd;
        }
        @Override
        public boolean followPath(){
            if (isStart()){
                follower.followPath(getPath(),holdEnd);
            }
            Drawing.drawDebug(follower);
            return follower.isBusy();
        }
        @Override
        public void stopProcedure(){
            follower.breakFollowing();
        }
    }
    public static class PedroLinearCommand extends PedroCommand{
        public PedroLinearCommand(double x, double y, double heading, boolean holdEnd){ //Goes to the position indicated by the inputs
            super(
                    (PathBuilder b)-> b
                            .addPath(new BezierLine(follower::getPose,new Pose(x,y)))
                            .setLinearHeadingInterpolation(follower.getPose().getHeading(),heading),
                    holdEnd
            );
        }
    }
    public static class PedroLinearChainCommand extends PedroCommand{
        public PedroLinearChainCommand(boolean holdEnd, Pose...poses){ //In a path chain, goes to all positions provided one at a time
            super(
                    (PathBuilder b)->{
                            for (int i=0;i<poses.length;i++) {
                                b.addPath(new BezierLine(follower::getPose, poses[i]));
                                if (i==0){
                                    b.setLinearHeadingInterpolation(follower.getPose().getHeading(), poses[i].getHeading());}
                                else{
                                    b.setLinearHeadingInterpolation(poses[i-1].getHeading(), poses[i].getHeading());}
                            }
                            return b;
                    },
                    holdEnd
            );
        }
    }
    public static class PedroLinearTransformCommand extends PedroCommand{
        public PedroLinearTransformCommand(double x, double y, double heading, boolean holdEnd){ //Transforms from current position by the given inputs
            super(
                    (PathBuilder b)-> b
                            .addPath(new BezierLine(follower::getPose,new Pose(follower.getPose().getX()+x,follower.getPose().getY()+y)))
                            .setLinearHeadingInterpolation(follower.getPose().getHeading(),follower.getPose().getHeading()+heading),
                    holdEnd
            );
        }
    }
    public static class PedroCurveCommand extends PedroCommand{
        public PedroCurveCommand(double tension, boolean holdEnd, double heading, Pose...poses) {
            super(
                    (PathBuilder b)-> b
                            .curveThrough(tension,poses)
                            .setLinearHeadingInterpolation(follower.getPose().getHeading(),heading),
                    holdEnd
            );
        }
    }
    public static class PedroInstantCommand extends Commands.InstantCommand { //This action ends instantly and does not wait for the follower to reach its goal
        public PedroInstantCommand(Function<PathBuilder,PathBuilder> buildPath, boolean holdEnd) {
            super(()->follower.followPath(buildPath.apply(follower.pathBuilder()).build(),holdEnd));
        }
    }
    public static class PedroInstantLinearCommand extends PedroInstantCommand{
        public PedroInstantLinearCommand(double x, double y, double heading, boolean holdEnd) {
            super((PathBuilder b)-> b
                            .addPath(new BezierLine(follower::getPose,new Pose(x,y)))
                            .setLinearHeadingInterpolation(follower.getPose().getHeading(),heading),
                    holdEnd
            );
        }
    }
    public static class PedroInstantLinearChainCommand extends PedroInstantCommand{
        public PedroInstantLinearChainCommand(boolean holdEnd, Pose...poses){ //In a path chain, goes to all positions provided one at a time
            super(
                    (PathBuilder b)->{
                        for (int i=0;i<poses.length;i++) {
                            b.addPath(new BezierLine(follower::getPose, poses[i]));
                            if (i==0){
                                b.setLinearHeadingInterpolation(follower.getPose().getHeading(), poses[i].getHeading());}
                            else{
                                b.setLinearHeadingInterpolation(poses[i-1].getHeading(), poses[i].getHeading());}
                        }
                        return b;
                    },
                    holdEnd
            );
        }
    }
    public static class PedroInstantLinearTransformCommand extends PedroInstantCommand{
        public PedroInstantLinearTransformCommand(boolean holdEnd, double x, double y, double heading) {
            super((PathBuilder b)-> b
                            .addPath(new BezierLine(follower::getPose,new Pose(follower.getPose().getX()+x,follower.getPose().getY()+y)))
                            .setLinearHeadingInterpolation(follower.getPose().getHeading(),follower.getPose().getHeading()+heading),
                    holdEnd
            );
        }
    }
    public static class PedroInstantCurveCommand extends PedroInstantCommand{
        public PedroInstantCurveCommand(double tension, boolean holdEnd, double heading, Pose...poses) {
            super(
                    (PathBuilder b)-> b
                            .curveThrough(tension,poses)
                            .setLinearHeadingInterpolation(follower.getPose().getHeading(),heading),
                    holdEnd
            );
        }
    }
    public static class PedroSleepUntilPose extends Commands.SleepUntilPose{
        public PedroSleepUntilPose(double x, double y, double heading, double poseDistance, double headingDistance, double timeout) {
            super(x, y, heading, poseDistance, headingDistance, timeout);
        }
        public PedroSleepUntilPose(double x, double y, double heading, double poseDistance, double headingDistance) {
            super(x, y, heading, poseDistance, headingDistance);
        }
    }
}
