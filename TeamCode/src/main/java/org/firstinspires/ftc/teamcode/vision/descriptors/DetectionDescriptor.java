package org.firstinspires.ftc.teamcode.vision.descriptors;

import java.util.ArrayList;
import java.util.List;

public class DetectionDescriptor {
    // Describes a detection on the camera side, relative position from the robot
    public DetectionDescriptor() {
        this.tx = 0.0;
        this.ty = 0.0;
        this.className = "none";
        this.leftRightOffset = 0.0;
        this.forwardOffset = 0.0;
        this.corners = new ArrayList<>();
        this.targetPixels = new double[2];
        this.data = new double[10];
    }
    public DetectionDescriptor(double tx,
                               double ty,
                               String className,
                               double leftRightOffset,
                               double forwardOffset,
                               List<List<Double>> corners,
                               double[] targetPixels,
                               double[] data){
        this.tx = tx;
        this.ty = ty;
        this.className = className;
        this.leftRightOffset = leftRightOffset;
        this.forwardOffset = forwardOffset;
        this.corners = corners;
        this.targetPixels = targetPixels;
        this.data = data;
    }
    public DetectionDescriptor(double leftRightOffset, double forwardOffset, String className){
        this.leftRightOffset = leftRightOffset;
        this.forwardOffset = forwardOffset;
        this.className = className;
    }

    double tx;
    double ty;
    String className;
    public double leftRightOffset;
    public double forwardOffset;
    List<List<Double>> corners;
    double[] targetPixels;
    double[] data;
    public double getTx() {return tx;}
    public void setTx(double tx) {this.tx = tx;}
    public double getTy() {return ty;}
    public void setTy(double ty) {this.ty = ty;}
    public String getClassName() {return className;}
    public void setClassName(String className) {this.className = className;}
    public double getLeftRightOffset() {return leftRightOffset;}
    public void setLeftRightOffset(double leftRightOffset) {this.leftRightOffset = leftRightOffset;}
    public double getForwardOffset() {return forwardOffset;}
    public void setForwardOffset(double y) {this.forwardOffset = y;}
    public double[] getData() {return data;}
    public List<List<Double>> getCorners() {return corners;}
    public void setCorners(List<List<Double>> corners) {this.corners = corners;}
    public double[] getTargetPixels() {return targetPixels;}
    public void setTargetPixels(double x, double y) {
        targetPixels[0] = x;
        targetPixels[1] = y;
    }
    public void setTargetPixels(double[] targetPixels) {
        this.targetPixels = targetPixels;
    }
}
