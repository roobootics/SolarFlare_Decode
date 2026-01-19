package org.firstinspires.ftc.teamcode.vision.descriptors;

public class AprilTagDescriptor {
    // Describes april tag in camera space
    double tx;
    double ty;
    int id;
    public AprilTagDescriptor(){
        this.tx = 0;
        this.ty = 0;
        this.id = 0;
    }
    public AprilTagDescriptor(double tx, double ty, int id){
        this.tx = tx;
        this.ty = ty;
        this.id = id;
    }
    public double getTy() {return ty;}
    public void setTy(double ty) {this.ty = ty;}
    public double getTx() {return tx;}
    public void setTx(double tx) {this.tx = tx;}
    public int getId() {return id;}
    public void setId(int id) {this.id = id;}
}
