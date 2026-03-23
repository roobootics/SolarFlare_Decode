package org.firstinspires.ftc.teamcode.pedroPathing;
import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.gamePhase;

import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PredictiveBrakingCoefficients;
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
            .mass(15.05927)
            .forwardZeroPowerAcceleration(-31.9940432459363933)
            .lateralZeroPowerAcceleration(-68.69912366702141)
            .headingPIDFCoefficients(new PIDFCoefficients(1.1,0.001,0.00001,0))
            .centripetalScaling(0)
            .predictiveBrakingCoefficients(new PredictiveBrakingCoefficients(0.045,0.10624854404399096,0.001604334137270121));

    public static MecanumConstants driveConstants = new MecanumConstants()
            .leftFrontMotorName("leftFront")
            .leftRearMotorName("leftRear")
            .rightFrontMotorName("rightFront")
            .rightRearMotorName("rightRear")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(71.55921082984744)
            .yVelocity(55.24798439806841);

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
            0.95,
            0.1,
            0.1,
            0.004,
            7,
            1.0,
            10,
            0.85
    );

    public static Follower createFollower(HardwareMap hardwareMap) {
        followerConstants.usePredictiveBraking = true;
        if (gamePhase== Inferno.GamePhase.TELEOP){
            followerConstants.setHoldPointTranslationalScaling(1.4);
            followerConstants.setHoldPointHeadingScaling(0.4);
        }
        Follower follower = new FollowerBuilder(followerConstants, hardwareMap)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .build();
        if (gamePhase== Inferno.GamePhase.TELEOP) {
            follower.setTranslationalPIDFCoefficients(new PIDFCoefficients(0.2, 0, 0.001, 0));
            follower.setHeadingPIDFCoefficients(new PIDFCoefficients(1.1,0.001,0.00001,0));
        }
        return follower;
    }
}
