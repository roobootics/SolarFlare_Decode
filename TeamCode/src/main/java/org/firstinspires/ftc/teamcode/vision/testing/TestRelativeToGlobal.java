package org.firstinspires.ftc.teamcode.vision.testing;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.vision.descriptors.DetectionDescriptor;

public class TestRelativeToGlobal {
    final double CAMERA_VERTICAL_HEIGHT_INCHES = 5; // Increases up from reference point
    final double CAMERA_OFFSET_X_INCHES = 0; // Increases to the right from reference point
    final double CAMERA_OFFSET_Y_INCHES = 0;
    DetectionDescriptor detection = new DetectionDescriptor(0, 0, "green");
    Pose3D botPose = new Pose3D(new Position(DistanceUnit.INCH, 0, 0, 0, 0), new YawPitchRollAngles(AngleUnit.DEGREES, 90, 0, 0, 0));

    public static void main(String[] args){
        
    }
}
