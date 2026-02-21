package org.firstinspires.ftc.teamcode.robotconfigs;

import static org.apache.commons.math3.util.FastMath.atan2;
import static org.apache.commons.math3.util.FastMath.cos;
import static org.apache.commons.math3.util.FastMath.sin;
import static java.lang.Math.PI;
import static java.lang.Math.min;
import static java.lang.Math.sqrt;

import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;

public abstract class Fisiks {
    final static double MASS = 74.8;
    final static double AIR_DENSITY = 0.020;
    final static double GRAVITY = -386.089;
    final static double HEIGHT = 10.748;
    final static double TICKS_TO_RAD = 2*PI/28;
    final static double WHEEL_RAD = 1.41732;
    final static double BALL_RAD = 2.5;
    final static double SURFACE_SPEED_RATIO = 0.75;

    final static double FRICTION = 0.5;
    final static double AUTHORITY = 0.25;
    final static double TRANSLATIONAL_DRAG = 0.0002;
    final static double ANGULAR_DRAG = 0;
    final static double MAGNUS = 0;

    final static double TOTAL_RAD = WHEEL_RAD+BALL_RAD;
    final static double K_DRAG = -1/(2*MASS)*TRANSLATIONAL_DRAG*AIR_DENSITY*PI*BALL_RAD*BALL_RAD;
    final static double K_MAGNUS = MAGNUS/MASS;
    final static double K_SPIN = -3/(4*MASS)*AIR_DENSITY*ANGULAR_DRAG*PI*BALL_RAD;

    final static double yawBracketRange = Math.toRadians(25);
    final static double yawBracketIncrement = Math.toRadians(10);
    final static double VEL_THRESHOLD = 0.2;

    static double[] targetPoint;
    static double botVelX;
    static double botVelY;
    public static double initSpeed;
    public static double initSpin;
    static double distance;
    final static double[] targetNorm = new double[2];
    final static double[] sidewaysNorm = new double[2];
    public static class Vec3 {
        public double x,y,z;
        public Vec3(double x, double y, double z){
            set(x,y,z);
        }
        Vec3 set(double X,double Y,double Z){x=X;y=Y;z=Z;return this;}
        Vec3 set(Vec3 vec){x=vec.x;y=vec.y;z=vec.z;return this;}
        Vec3 addScaled(Vec3 b,double s){x+=b.x*s;y+=b.y*s;z+=b.z*s;return this;}
        Vec3 add(double x, double y, double z){this.x+=x;this.y+=y;this.z+=z;return this;}
        Vec3 scale(double s){x*=s;y*=s;z*=s;return this;}
        double magnitude(){return sqrt(x*x+y*y+z*z);}
    }
    public static class State{
        public final Vec3 tVel = new Vec3(0,0,0);
        public final Vec3 tPos = new Vec3(0,0,0);
        //public final Vec3 aVel = new Vec3(0,0,0);
        public void set(State state){
            tVel.set(state.tVel);
            tPos.set(state.tPos);
            //aVel.set(state.aVel);
        }
    }
    public static class RK4{
        private final static State current = new State();
        private final static State[] derivs = new State[]{new State(),new State(),new State(),new State()};
        private final static State tmp = new State();
        private final static double deltaT = 0.003;
        private static void deriv(State current, State change){
            change.tPos.set(current.tVel);
            double vmag = current.tVel.magnitude();
            change.tVel.set(
                    K_DRAG*vmag*current.tVel.x /*+ K_MAGNUS*(current.aVel.y*current.tVel.z - current.aVel.z*current.tVel.y*)*/,
                    K_DRAG*vmag*current.tVel.y /*+ K_MAGNUS*(current.aVel.z*current.tVel.x - current.aVel.x*current.tVel.z)*/,
                    K_DRAG*vmag*current.tVel.z /*+ K_MAGNUS*(current.aVel.x*current.tVel.y - current.aVel.y*current.tVel.x)*/ + GRAVITY
            );
            /*
            double wmag = current.aVel.magnitude();
            change.aVel.set(
                    K_SPIN*wmag*current.aVel.x,
                    K_SPIN*wmag*current.aVel.y,
                    K_SPIN*wmag*current.aVel.z
            );
            */
        }
        public static State integrate(double pitch, double yaw, double time){
            current.tPos.set((cos(PI/2 + pitch)+1)*cos(yaw), (cos(PI/2 + pitch)+1)*sin(yaw), sin(PI/2 + pitch)).scale(TOTAL_RAD);
            current.tVel.set(cos(yaw)*cos(pitch), sin(yaw)*cos(pitch), sin(pitch)).scale(initSpeed).add(botVelX,botVelY,0);
            //current.aVel.set(cos(yaw + PI/2), sin(yaw + PI/2), 0).scale(initSpin);
            double t = 0;
            double h;
            while (t < time){
                h = min(deltaT, time-t);

                deriv(current, derivs[0]);
                tmp.tPos.set(current.tPos).addScaled(derivs[0].tPos,h*0.5);
                tmp.tVel.set(current.tVel).addScaled(derivs[0].tVel,h*0.5);
                //tmp.aVel.set(current.aVel).addScaled(derivs[0].aVel,h*0.5);
                deriv(tmp, derivs[1]);
                tmp.tPos.set(current.tPos).addScaled(derivs[1].tPos,h*0.5);
                tmp.tVel.set(current.tVel).addScaled(derivs[1].tVel,h*0.5);
                //tmp.aVel.set(current.aVel).addScaled(derivs[1].aVel,h*0.5);
                deriv(tmp, derivs[2]);
                tmp.tPos.set(current.tPos).addScaled(derivs[2].tPos,h);
                tmp.tVel.set(current.tVel).addScaled(derivs[2].tVel,h);
                //tmp.aVel.set(current.aVel).addScaled(derivs[2].aVel,h);
                deriv(tmp, derivs[3]);

                double c = h/6.0;
                current.tPos.addScaled(derivs[0].tPos, c).addScaled(derivs[1].tPos,2*c).addScaled(derivs[2].tPos,2*c).addScaled(derivs[3].tPos,c);
                current.tVel.addScaled(derivs[0].tVel, c).addScaled(derivs[1].tVel,2*c).addScaled(derivs[2].tVel,2*c).addScaled(derivs[3].tVel,c);
                //current.aVel.addScaled(derivs[0].aVel, c).addScaled(derivs[1].aVel,2*c).addScaled(derivs[2].aVel,2*c).addScaled(derivs[3].aVel,c);

                t+=h;
            }
            return current;
        }

    }
    public static class Error{
        public static double distError;
        public static double sideError;
        public static double heightError;
        public static void findError(double pitch, double yaw, double time){
            State s = RK4.integrate(pitch,yaw,time);
            distError = s.tPos.x*targetNorm[0] + s.tPos.y*targetNorm[1] - distance;
            heightError = s.tPos.z - targetPoint[2];
            sideError = sidewaysNorm[0]*(s.tPos.x-targetPoint[0]) + sidewaysNorm[1]*(s.tPos.y-targetPoint[1]);
            System.out.printf(
                    "pitch=%.3f  time=%.3f  yaw=%.3f  distErr=%.2f  heightErr=%.2f  sideErr=%.2f\n",
                    pitch, time, yaw, distError, heightError, sideError
            );
        }
    }
    public static class Solver{
        public static double clampHood(double in){return Math.max(Math.toRadians(22),Math.min(in,Math.toRadians(87)));}
        public static double clampTime(double in){return Math.max(0.2,Math.min(in,2));}
        final public static double[] out = new double[3]; //pitch, yaw, time
        private static final double STAGE1MAXITR = 12;
        private static final double STAGE2MAXITR = 8;
        private static final double DISTERR = 1e-1;
        private static final double HEIGHTERR = 1e-1;
        private static final double YAWERR = 1e-1;
        private static double a,b,c,dj;
        public static boolean resetJacobian = true;
        public static boolean stage1Solve(double initialPitchGuess, double initialTimeGuess, double yaw){
            if (resetJacobian) {
                resetJacobian = false;
                double timeOffset = 0.01;
                double angleOffset = Math.toRadians(1);

                Error.findError(initialPitchGuess + angleOffset, yaw, initialTimeGuess);
                double distErr2 = Error.distError;
                double heightErr2 = Error.heightError;
                Error.findError(initialPitchGuess, yaw, initialTimeGuess + timeOffset);
                double distErr3 = Error.distError;
                double heightErr3 = Error.heightError;
                Error.findError(initialPitchGuess, yaw, initialTimeGuess);
                double distErr1 = Error.distError;
                double heightErr1 = Error.heightError;

                double topLeft = (distErr2 - distErr1) / angleOffset;
                double bottomRight = (heightErr3 - heightErr1) / timeOffset;
                double topRight = (distErr3 - distErr1) / timeOffset;
                double bottomLeft = (heightErr2 - heightErr1) / angleOffset;

                double determinant = topLeft * bottomRight - topRight * bottomLeft;
                a = bottomRight / determinant;
                b = -topRight / determinant;
                c = -bottomLeft / determinant;
                dj = topLeft / determinant;
            }
            else{
                Error.findError(initialPitchGuess, yaw, initialTimeGuess);
            }

            out[0] = initialPitchGuess; out[2] = initialTimeGuess;
            double startDistError;
            double startHeightError;
            double newDistError;
            double newHeightError;

            for(int it=0; it<STAGE1MAXITR; it++)
            {
                startDistError = Error.distError; startHeightError = Error.heightError;

                if (Math.abs(startDistError)<DISTERR && Math.abs(startHeightError)<HEIGHTERR)
                    return true;

                boolean update = true;
                double dx0 = -(a*startDistError + b*startHeightError);
                double dx1 = -(c*startDistError + dj*startHeightError);

                double x0 = clampHood(out[0] + dx0);
                double x1 = clampTime(out[2] + dx1);

                dx0 = x0-out[0];
                dx1 = x1-out[2];

                if (dx0!=-(a*startDistError + b*startHeightError) || dx1!=-(c*startDistError + dj*startHeightError)) update = false;

                Error.findError(x0,yaw,x1);
                newDistError = Error.distError;
                newHeightError = Error.heightError;

                double y0 = newDistError - startDistError;
                double y1 = newHeightError - startHeightError;

                double sty = dx0*y0 + dx1*y1;

                if (Math.abs(sty) > 1e-12 && update) {
                    double r0 = dx0 - (a*y0 + b*y1);
                    double r1 = dx1 - (c*y0 + dj*y1);

                    double k0 = r0/sty;
                    double k1v = r1/sty;

                    a += k0*dx0;  b += k0*dx1;
                    c += k1v*dx0; dj+= k1v*dx1;
                }
                out[0] = x0; out[2] = x1;
            }
            return false;
        }
        public static boolean solve(double initialPitchGuess, double initialTimeGuess, double yawTopBracket, double yawBottomBracket){
            if (Math.abs(sidewaysNorm[0]*botVelX+sidewaysNorm[1]*botVelY)<VEL_THRESHOLD){
                out[1] = atan2(targetPoint[1], targetPoint[0]);
                return stage1Solve(initialPitchGuess,initialTimeGuess,out[1]);
            }
            double fLo, fHi;
            stage1Solve(initialPitchGuess,initialTimeGuess,yawBottomBracket); fLo = Error.sideError;
            stage1Solve(initialPitchGuess,initialTimeGuess,yawTopBracket); fHi = Error.sideError;
            while (fLo*fHi>0){
                System.out.println("faliure");
                if (sidewaysNorm[0]*botVelX+sidewaysNorm[1]*botVelY>0){
                    yawBottomBracket -= yawBracketIncrement;
                    stage1Solve(initialPitchGuess,initialTimeGuess,yawBottomBracket); fLo = Error.sideError;
                } else {
                    yawTopBracket += yawBracketIncrement;
                    stage1Solve(initialPitchGuess,initialTimeGuess,yawTopBracket); fHi = Error.sideError;
                }
            }
            double a=yawBottomBracket, b=yawTopBracket, fa=fLo, fb=fHi;
            double initPitchGuess = initialPitchGuess, initTimeGuess = initialTimeGuess;
            double prevC = (yawTopBracket+yawBottomBracket)/2;
            double c = (a*fb - b*fa)/(fb - fa);
            for(int it=0; it<STAGE2MAXITR; it++) {
                System.out.println("Stage 2");
                boolean success = stage1Solve(initPitchGuess,initTimeGuess,c);
                if (!success){
                    c = prevC + 0.5 * (c - prevC);
                } else{
                    prevC = c;
                    initPitchGuess = out[0];
                    initTimeGuess = out[2];
                    double fc = Error.sideError;
                    if (Math.abs(fc) < YAWERR) {
                        out[1] = c;
                        return true;
                    }
                    if (fa*fc < 0) {
                        b=c; fb=fc;
                        fa*=0.5; // Illinois
                    } else {
                        a=c; fa=fc;
                        fb*=0.5;
                    }
                    c = (a*fb - b*fa)/(fb - fa);
                }
            }
            return false;
        }
    }
    public static void buildPhysics(double[] targetPoint, Pose pos, Vector botVel, double flywheelVel){
        double xPos = pos.getX();
        double yPos = pos.getY();
        targetPoint[0] -= xPos; targetPoint[1] -= yPos; targetPoint[2] -= HEIGHT;
        Fisiks.targetPoint = targetPoint;
        Fisiks.distance = sqrt(targetPoint[0]*targetPoint[0]+targetPoint[1]*targetPoint[1]);
        Fisiks.targetNorm[0] = 1/distance * targetPoint[0];
        Fisiks.targetNorm[1] = 1/distance * targetPoint[1];
        Fisiks.sidewaysNorm[0] = 1/distance * -targetPoint[1];
        Fisiks.sidewaysNorm[1] = 1/distance * targetPoint[0];
        Fisiks.botVelX = botVel.getXComponent();
        Fisiks.botVelY = botVel.getYComponent();
        double flyVel = TICKS_TO_RAD*flywheelVel*WHEEL_RAD;
        double backVel = flyVel*SURFACE_SPEED_RATIO;
        Fisiks.initSpeed = FRICTION*(AUTHORITY * backVel+(1- AUTHORITY)*flyVel);
        Fisiks.initSpin = FRICTION*((min(2*AUTHORITY,1)*backVel + (1-min(2*AUTHORITY,1))*flyVel) - ((1 - min(2-2*AUTHORITY,1))*backVel + min(2-2*AUTHORITY,1)*flyVel))/BALL_RAD;
    }

    public static double[] runPhysics(Inferno.BallPath currentBallPath, double[] targetPoint, Pose pos, Vector botVel, double flywheelVel){
        Solver.resetJacobian = true;
        buildPhysics(targetPoint,pos,botVel,flywheelVel);
        double initialYawGuess = atan2(targetPoint[1],targetPoint[0]);
        double initialPitchGuess;
        double initialTimeGuess;
        double yawTopBracket, yawBottomBracket;
        if (sidewaysNorm[0]*botVelX+sidewaysNorm[1]*botVelY>0){
            yawTopBracket = initialYawGuess;
            yawBottomBracket = initialYawGuess - yawBracketRange;
        } else {
            yawTopBracket = initialYawGuess + yawBracketRange;
            yawBottomBracket = initialYawGuess;
        }
        if (currentBallPath == Inferno.BallPath.HIGH) {initialPitchGuess = Math.toRadians(70); initialTimeGuess = 1.1;} else {initialPitchGuess = Math.toRadians(40); initialTimeGuess = 0.5;}
        boolean success = Solver.solve(initialPitchGuess,initialTimeGuess,yawTopBracket,yawBottomBracket);
        System.out.println(success);
        return Solver.out;
    }
}
