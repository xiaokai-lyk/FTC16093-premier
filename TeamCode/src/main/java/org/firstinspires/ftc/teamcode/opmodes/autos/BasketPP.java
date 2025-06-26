package org.firstinspires.ftc.teamcode.opmodes.autos;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Config
@Autonomous(name = "BasketPP path", group = "Autos")
public class BasketPP extends AutoCommandBasePP {
    // For Basket Scoring
//    private final Pose startPose = new Pose(9, 111, Math.toRadians(270));
//    public final Pose Basket = new Pose(-56, -56, 45);
//    public final Pose BasketForSpline = new Pose(-59, -54.5, 60);
    public final Pose S1Basket = new Pose(-51, -57.4, Math.toRadians(60));
    public final Pose PreloadBasket = new Pose(-53.5, -57.5, Math.toRadians(50));
//    public final Pose S2Basket = new Pose(-64.694, -53, Math.toDegrees(1.621));

    // The right sample
    public final Pose S1 = new Pose(47.5, 47, Math.toRadians(-90));
    // The middle sample
    public final Pose S2 = new Pose(40.3, 47, Math.toRadians(-90));
    // The left sample
    public final Pose S3 = new Pose(32.9, 47, Math.toRadians(-90));

    // Middle point for spline
//    public final Pose splinePoint1 = new Pose(-24.5, -5, 0);
//    public static Pose splinePoint2 = new Pose(-24.5, 0, 0);

    public static long basketWaitMs = 630;
    public static long basketWaitForAutoPickMs = 0;
    public static long firstBasketWaitMs = 520;
    public static long pick2Handoff = 0;
    public static long startStowToPath = 200;
    public static long stowedToGrabDelay = 200;
    public static long stopLinearToLLM = 0;
    public static long S3SlideDelay = 0;

    public static double S1TurnPos = 0.35;
    public static double S2TurnPos = 0.25;
    public static double S3TurnPos = 0.17;

    public static Pose startPose = new Pose(-44, -60, Math.toRadians(55));

    @Override
    public Pose getStartPose() {
        return startPose;
    }

    @Override
    public void initializeSuperStructure() {
        follower.setStartingPose(startPose);
//        slide.stow();
//        slide.openIntakeClaw();
//        slide.backwardSlideExtension();
//        liftClaw.closeClaw();
//        liftClaw.foldLiftArm();
    }

    @Override
    public Command runAutoCommand() {
//        Supplier<Command> slideExtendCommand = () -> new InstantCommand(() -> slide.forwardSlideExtension());

        return new SequentialCommandGroup(
                new InstantCommand(() -> follower.setStartingPose(startPose)),
                followPath(new Path(new BezierLine(new Point(follower.getPose()), new Point(PreloadBasket))))
                        .alongWith(),
                new WaitCommand(200),
                followPath(new Path(new BezierLine(new Point(follower.getPose()), new Point(S1)))),
                followPath(new Path(new BezierLine(new Point(follower.getPose()), new Point(S1Basket))))
                        .alongWith(),
                new WaitCommand(basketWaitMs)
                        .alongWith(),
                followPath(new Path(new BezierLine(new Point(follower.getPose()), new Point(S2)))),
//                slide.grabCommand(),
                followPath(new Path(new BezierLine(new Point(follower.getPose()), new Point(S1Basket))))
                        .alongWith(),
                new WaitCommand(basketWaitMs)
                        .alongWith(),
                                new WaitCommand(S3SlideDelay).andThen(),
                followPath(new Path(new BezierLine(new Point(follower.getPose()), new Point(S3))))
//                slide.grabCommand(),
//                followPath(new Path(new BezierLine(new Point(follower.getPose()), new Point(BasketForSpline))))
//                        .alongWith(),
//                new WaitCommand(basketWaitMs)
//                        .alongWith(
//                                new WaitCommand(startStowToPath)
//                                        .andThen(
//                                                followPath(new Path(new BezierLine(new Point(follower.getPose()), new Point(splinePoint1))))
//                                                        .andThen(new WaitCommand(stopLinearToLLM)))),
//                followPath(new Path(new BezierLine(new Point(follower.getPose()), new Point(BasketForSpline))))
//                        .alongWith(new WaitCommand(pick2Handoff)),
//                wait(follower, basketWaitForAutoPickMs)
//                        .alongWith(
//                                new WaitCommand(startStowToPath)
//                                        .andThen(
//                                                followPath(new Path(new BezierLine(new Point(follower.getPose()), new Point(splinePoint2))))
//                                                        .andThen(new WaitCommand(stopLinearToLLM)))),
//                followPath(new Path(new BezierLine(new Point(follower.getPose()), new Point(BasketForSpline))))
//                        .alongWith(new WaitCommand(pick2Handoff)),
//                wait(follower, basketWaitForAutoPickMs)
                );
    }
}