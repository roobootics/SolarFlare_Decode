package org.firstinspires.ftc.teamcode.vision.descriptors;

import org.firstinspires.ftc.teamcode.vision.Vision;

import java.util.ArrayList;
import java.util.List;

public class ArtifactDescriptor {
    double x;
    double y;
    String className;
    double targetXPixels;
    double targetYPixels;
    public ArtifactDescriptor(double x, double y, String className){
        this.x = x;
        this.y = y;
        this.className = className;
    }
    public double getX() {return x;}
    public void setX(double x) {this.x = x;}
    public double getY() {return y;}
    public void setY(double y) {this.y = y;}
    public String getClassName() {return className;}
    public void setClassName(String className) {this.className = className;}
    public void setTargetXPixels(double targetXPixels) {this.targetXPixels = targetXPixels;}
    public void setTargetYPixels(double targetYPixels) {this.targetYPixels = targetYPixels;}
}
