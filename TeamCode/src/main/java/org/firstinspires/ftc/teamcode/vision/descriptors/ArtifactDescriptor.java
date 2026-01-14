package org.firstinspires.ftc.teamcode.vision.descriptors;

public class ArtifactDescriptor {
    // Describes the actual artifact and its real-world coordinates
    double x;
    double y;
    String className;
    public ArtifactDescriptor(){
        this.x = 0;
        this.y = 0;
        this.className = "none";
    }
    public ArtifactDescriptor(double x, double y, String className){
        this.x = x;
        this.y = y;
        this.className = className;
    }
    public double getX() {return x;}
    public void setX(int x) {this.x = x;}
    public double getY() {return y;}
    public void setY(int y) {this.y = y;}
    public String getClassName() {return className;}
    public void setClassName(String className) {this.className = className;}
}
