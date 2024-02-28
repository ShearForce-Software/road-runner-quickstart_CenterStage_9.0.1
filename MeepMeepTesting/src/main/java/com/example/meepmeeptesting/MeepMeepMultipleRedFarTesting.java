package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepMultipleRedFarTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(500);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                //TODO Need to figure out the real constraints -- maxAccel seems to have biggest effect
                // Setting constraints to 60,40 helps a lot with timeline, saving approximately 3 seconds
                // Currently robot is actually configured to 40,40
                .setConstraints(40, 40, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->

                        // RedFarStackAuto - MULTIPLES
                        drive.trajectorySequenceBuilder(new Pose2d(-35.5, -62.5, Math.toRadians(90)))
                                // move a little forward to get away from the wall before starting to spline turn
                                .lineTo(new Vector2d(-35.5, -55))

                                // Position - 1 floor
                                //.splineToLinearHeading (new Pose2d(-39.5, -20.5, Math.toRadians(45)), Math.toRadians(45))
                                //TODO deliver to floor here
                                //.waitSeconds(0.9)
                                //.splineToLinearHeading(new Pose2d(-50, -12, Math.toRadians(180)), Math.toRadians(0))

                                // Position - 2 floor
                                //.lineTo(new Vector2d(-35.5, -12.5))
                                //TODO deliver to floor here
                                //.waitSeconds(0.9)
                                //.lineTo(new Vector2d(-40, -12.5))
                                //.lineToLinearHeading(new Pose2d(-50, -12, Math.toRadians(180)))

                                // Position - 3 floor
                                .splineToLinearHeading(new Pose2d(-37.5, -32, Math.toRadians(180)), Math.toRadians(180))
                                .lineTo (new Vector2d(-34.5, -32))
                                //TODO deliver to floor here
                                .waitSeconds(0.9)
                                .lineTo(new Vector2d(-37.5, -32))
                                .splineToConstantHeading(new Vector2d(-50, -12), Math.toRadians(180))

                                // drive straight at the stack
                                .lineTo(new Vector2d(-56, -12))
                                // TODO pickup a pixel here
                                .waitSeconds(1.0)
                                // backup a little  from the stack before allowing turns to start
                                .lineTo(new Vector2d(-50, -12))
                                // Drive beyond the bridge area
                                .splineToLinearHeading(new Pose2d(12, -12, Math.toRadians(180)), Math.toRadians(0))
                                // TODO -- raise the slides/arms in parallel
                                // Drive to the backboard area and swipe out anything in the way
                                // get past the floor drop area for the alliance partner
                                .splineToLinearHeading(new Pose2d(30, -12, Math.toRadians(180)), Math.toRadians(0))
                                // get to the board
                                .splineToLinearHeading(new Pose2d(50, -12, Math.toRadians(180)), Math.toRadians(0))
                                // swipe the area infront of the board clean
                                .lineTo(new Vector2d(50, -40))

                                // Position - 1 board
                                //.lineTo(new Vector2d(50, -28))

                                // Position - 2 board
                                //.lineTo(new Vector2d(50, -33))

                                // Position - 3 board
                                .lineTo(new Vector2d(50, -38.5))

                                // TODO -- deliver to the board here
                                .waitSeconds(0.5)

                                // Return to the bridge
                                // avoid hitting alliance marker
                                .lineTo(new Vector2d(34, -12))
                                // TODO must have arm down by the time get to this bridge position
                                .lineTo(new Vector2d(12, -12))
                                // get to the stack
                                //.splineToLinearHeading(new Pose2d(-50, 12, Math.toRadians(180)), Math.toRadians(0))
                                .lineTo(new Vector2d(-50, -12))
                                // drive straight at the stack
                                .lineTo(new Vector2d(-56, -12))
                                // TODO pickup two pixels here
                                .waitSeconds(1.0)
                                // backup a little  from the stack before allowing turns to start
                                .lineTo(new Vector2d(-50, -12))
                                // Drive beyond the bridge area
                                .splineToLinearHeading(new Pose2d(12, -12, Math.toRadians(180)), Math.toRadians(0))
                                // TODO -- raise the slides/arms in parallel
                                // Drive to the backboard area and swipe out anything in the way
                                // get past the floor drop area for the alliance partner
                                .splineToLinearHeading(new Pose2d(30, -12, Math.toRadians(180)), Math.toRadians(0))
                                // get to the board
                                .splineToLinearHeading(new Pose2d(50, -12, Math.toRadians(180)), Math.toRadians(0))
                                // swipe the area in-front of the board clean
                                .lineTo(new Vector2d(50, -40))
                                // deliver to the center spot on the board
                                .lineTo(new Vector2d(50, -33))
                                // TODO -- deliver to the board here
                                .waitSeconds(0.5)

                                // Parking Position
                                .lineTo(new Vector2d(50, -11)) //lineToX(47)
                                .turn(Math.toRadians(-90))

                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}