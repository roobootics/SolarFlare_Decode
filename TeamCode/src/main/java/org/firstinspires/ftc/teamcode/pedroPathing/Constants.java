package org.firstinspires.ftc.teamcode.pedroPathing;
import static org.firstinspires.ftc.teamcode.pedroPathing.Pedro.follower;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.gamePhase;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.robotconfigs.Inferno;

public class Constants {

    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(14.15208)
            .forwardZeroPowerAcceleration(-31.9940432459363933)
            .lateralZeroPowerAcceleration(-68.69912366702141)
            .centripetalScaling(0.001)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.2, 0.0003, 0.001, 0.015))
            .headingPIDFCoefficients(new PIDFCoefficients(1.35,0.001,0,0.01))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.01, 0, 0.00003, 0.6, 0.01))
            .centripetalScaling(0.0004)
            .secondaryDrivePIDFCoefficients(new FilteredPIDFCoefficients(0.005, 0, 0.00003, 0.6, 0.01))
            .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(0.07, 0.0007, 0.0003, 0.015))
            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(1.25,0.001,0,0.01));

    public static MecanumConstants driveConstants = new MecanumConstants()
            .leftFrontMotorName("leftFront")
            .leftRearMotorName("leftRear")
            .rightFrontMotorName("rightFront")
            .rightRearMotorName("rightRear")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(66.20164573849657)
            .yVelocity(49.480625903512546);

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(-3.76925)
            .strafePodX(-2.625)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(
                    GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD
            )
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);

    public static PathConstraints pathConstraints = new PathConstraints(
            0.94,
            0.1,
            0.1,
            0.004,
            7,
            1.0,
            10,
            0.85
    );

    public static Follower createFollower(HardwareMap hardwareMap) {
        if (gamePhase== Inferno.GamePhase.AUTO){
            followerConstants.setUseSecondaryDrivePIDF(true);
            followerConstants.setUseSecondaryTranslationalPIDF(true);
            followerConstants.setUseSecondaryHeadingPIDF(true);
        } else {
            followerConstants.setHoldPointTranslationalScaling(1.4);
            followerConstants.setHoldPointHeadingScaling(0.4);
            followerConstants.setUseSecondaryDrivePIDF(false);
            followerConstants.setUseSecondaryTranslationalPIDF(false);
            followerConstants.setUseSecondaryHeadingPIDF(false);
        }
        Follower follower = new FollowerBuilder(followerConstants, hardwareMap)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .build();
        if (gamePhase== Inferno.GamePhase.TELEOP) {
            follower.setTranslationalPIDFCoefficients(new PIDFCoefficients(0.2, 0, 0.001, 0));
            follower.setHeadingPIDFCoefficients(new PIDFCoefficients(1.35,0.001,0,0));
        }
        return follower;
    }
}
