package org.firstinspires.ftc.teamcode.programs;

import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;

public class TestGetVelocity {

    public static void main(String[] args) throws InterruptedException {
        ElapsedTime timer = new ElapsedTime();
        double loopTime= 0.006 + (0.01-0.006)* Math.random();
        double velocity=1500;
        double encoderCount=0;
        double lastLoopTime=0;
        double lastEncoderCount=0;
        double previousEncoderCount=0;
        ArrayList<Double> encoderTimes = new ArrayList<>();
        ArrayList<Double> previousFiveLoopTimes=new ArrayList<>();
        ArrayList<Double> previousFiveEncoders=new ArrayList<>();

        long loopnum=-200;

        double meanSimpleError = 0;
        double meanFivePointError = 0;
        double meanGetVelocityError = 0;

        timer.reset();
        for (int i=0;i<100000000;i++){
            double time=timer.time();
            velocity = velocity+(-0.1+(0.2)*Math.random());
            if (time-lastLoopTime>loopTime){
                loopnum+=1;
                double exactLoopTime=time-lastLoopTime;
                lastLoopTime=time;
                loopTime= 0.006 + (0.007-0.006)* Math.random();

                previousFiveLoopTimes.add(exactLoopTime);
                previousFiveEncoders.add(encoderCount);
                if (previousFiveEncoders.size()>5){
                    previousFiveLoopTimes.remove(0);
                    previousFiveEncoders.remove(0);
                }
                System.out.print("Actual Velocity: ");
                System.out.println(velocity);

                System.out.print("Simple: ");
                System.out.println((encoderCount-previousEncoderCount)/exactLoopTime);
                if (loopnum>0){
                    meanSimpleError = meanSimpleError* ((double) (loopnum-1))/loopnum + Math.abs(velocity-(encoderCount-previousEncoderCount)/exactLoopTime)/((double) loopnum);
                }
                System.out.print("getVelocity: ");
                System.out.println(encoderTimes.size()*20);
                if (loopnum>0){
                    meanGetVelocityError = meanGetVelocityError*((double) (loopnum-1))/loopnum + Math.abs(velocity- encoderTimes.size() * 20)/((double) loopnum);
                }
                if (previousFiveEncoders.size()==5){

                    double dtAvg=0;
                    for (Double loop:previousFiveLoopTimes){
                        dtAvg+=loop;
                    }
                    dtAvg=dtAvg/5.0;

                    System.out.print("Five Point Stencil: ");
                    System.out.println(
                            (-previousFiveEncoders.get(4)+8*previousFiveEncoders.get(3)-8*previousFiveEncoders.get(1)+previousFiveEncoders.get(0))/(12*dtAvg)
                    );

                    if (loopnum>0){
                        meanFivePointError = meanFivePointError*((double) (loopnum-1))/loopnum + Math.abs(velocity-(-previousFiveEncoders.get(4)+8*previousFiveEncoders.get(3)-8*previousFiveEncoders.get(1)+previousFiveEncoders.get(0))/(12*dtAvg))/((double) loopnum);
                    }

                }
                System.out.println("");
                previousEncoderCount=encoderCount;
            }
            if (time-lastEncoderCount>1/velocity){
                lastEncoderCount=time;
                encoderCount+=1;
                encoderTimes.add(time);
            }
            while (!encoderTimes.isEmpty() && time-encoderTimes.get(0)>0.05){
                encoderTimes.remove(0);
            }
        }
        System.out.println("");
        System.out.println(meanSimpleError);
        System.out.println(meanGetVelocityError);
        System.out.println(meanFivePointError);
    }
}
