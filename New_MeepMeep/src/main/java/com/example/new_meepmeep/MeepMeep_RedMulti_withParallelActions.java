package com.example.new_meepmeep;


import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeep_RedMulti_withParallelActions {
    static Pose2d startPose;
    static Pose2d stackPose;
    static Pose2d deliverToFloorPose;
    static Pose2d deliverToBoardPose;
    static Pose2d deliverToBoardPosition1Pose;
    static Action FloorTraj;
    static Action DriveToStack;
    static Action BoardTraj2;
    static Action BoardTraj3;
    static Action BackToStackTraj;
    static Action ParkingTraj;
    static RoadRunnerBotEntity myBot;
    static int autoPosition = 1;
    static VelConstraint speedUpVelocityConstraint;
    static AccelConstraint speedUpAccelerationConstraint;

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(500);

        startPose = new Pose2d(-35.5,-62.5,Math.toRadians(90));
        stackPose = new Pose2d(-54.5, -11.5, Math.toRadians(180));

        // Define the standard constraints to use for this robot
        myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(40, 40, Math.PI*.8, Math.PI, 15)
                .setDimensions(18, 15)
                .build();

        // Define some custom constraints to use when wanting to go faster than defaults
        speedUpVelocityConstraint = new TranslationalVelConstraint(90.0);
        speedUpAccelerationConstraint = new ProfileAccelConstraint(-50.0, 60.0);

        // ******************************************
        /* Specify which Position will be run */
        // ******************************************
        autoPosition = 3;

        // Build up the floor delivery trajectory
        RedLeftPurplePixelDecision();

        // Create the floor to Stack trajectory
        DriveToStack = myBot.getDrive().actionBuilder(deliverToFloorPose)
                //.splineToLinearHeading(stackPose, Math.toRadians(180))
                .strafeToLinearHeading(new Vector2d(stackPose.position.x, stackPose.position.y), Math.toRadians(180))
                .build();

        // Build up the Stack to Board Trajectory
        RedBoardDecision();

        // Create the second backup trajectory
        Action Backup1 = myBot.getDrive().actionBuilder(deliverToBoardPose)
                .lineToX(46)
                .build();

        // Build up the Board back to Stack Trajectory
        Action DriveBackToStack1 = myBot.getDrive().actionBuilder(new Pose2d(46, deliverToBoardPose.position.y, Math.toRadians(180)))
                .strafeToLinearHeading(new Vector2d(45, -11.5), Math.toRadians(180))
                .build();
        Action DriveBackToStack2 = myBot.getDrive().actionBuilder(new Pose2d(45,-11.5,Math.toRadians(180)))
                .strafeToLinearHeading(new Vector2d(stackPose.position.x, stackPose.position.y), Math.toRadians(180), speedUpVelocityConstraint)
                .build();

        // Build up the Stack to Board Position 1 Trajectory
        Action BoardTrajFinal = myBot.getDrive().actionBuilder(stackPose)
                //.setTangent(0)
                //.splineToLinearHeading(new Pose2d(-30, -11.5, Math.toRadians(180)), Math.toRadians(0))
                //.splineToLinearHeading(new Pose2d(47.5, -11.5, Math.toRadians(180)), Math.toRadians(0))
                //.setTangent(Math.toRadians(270))
                //.splineToLinearHeading(deliverToBoardPose, Math.toRadians(270))
                .strafeToLinearHeading(new Vector2d(47.5, -11.5), Math.toRadians(180), speedUpVelocityConstraint)
                .strafeToLinearHeading(new Vector2d(47.5, -28), Math.toRadians(180)) // Position 1
                .build();

        // Create the second backup trajectory
        //Action Backup2 = myBot.getDrive().actionBuilder(deliverToBoardPose)
        Action Backup2 = myBot.getDrive().actionBuilder(new Pose2d(47.5, -28, Math.toRadians(180))) // position-1
                .lineToX(46)
                .build();

        // Build up the Board position 1 to Parking Trajectory
        //Action Park = myBot.getDrive().actionBuilder(new Pose2d(46, deliverToBoardPose.position.y, Math.toRadians(180)))
        Action Park = myBot.getDrive().actionBuilder(new Pose2d(46, -28, Math.toRadians(180)))
                //.splineToLinearHeading(new Pose2d(48, -9, Math.toRadians(90)), Math.toRadians(90))
                //.strafeToLinearHeading(new Vector2d(48, -9), Math.toRadians(90))
                .turnTo(Math.toRadians(90))
                .build();

        myBot.runAction(new SequentialAction(
                // Drive to Floor Position
                FloorTraj,
                // simulate waiting for purple pixel delivery & resetArm
                new SleepAction(1.0),
                // simulate waiting for slides down
                new SleepAction(0.6),
                DriveToStack,
                // simulate waiting to get to the white pixel pickup location
                new SleepAction(0.7),
                // simulate waiting to pick up the pixels
                new SleepAction(1.05),
                BoardTraj2,
                // simulate waiting for board delivery and april tag correction
                new SleepAction(0.6),
                Backup1,
                // Drive back to the stack - part 1
                DriveBackToStack1,
                // simulate waiting for the slides to come down
                new SleepAction(0.6),
                // Drive back to the stack - part 2
                DriveBackToStack2,
                // simulate waiting to get to the white pixel pickup location
                new SleepAction(0.7),
                // simulate waiting to pick up the pixels
                new SleepAction(1.05),
                // Drive to Board Position 1
                BoardTrajFinal,
                // simulate waiting for board delivery
                new SleepAction(0.6),
                // backup from the board
                Backup2,
                // Drive to the parking spot
                Park
                // simulate waiting for slides down, and servo stop
                //, new SleepAction(0.8)
                ));

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }

    static public void RedLeftPurplePixelDecision() {
        //***POSITION 1***
        if (autoPosition == 1) {
            deliverToFloorPose = new Pose2d(-39.5, -20.5, Math.toRadians(45));
            FloorTraj = myBot.getDrive().actionBuilder(startPose)
                    .splineToLinearHeading(new Pose2d(-38.5, -33, Math.toRadians(90)), Math.toRadians(90))
                    .splineToLinearHeading (deliverToFloorPose, Math.toRadians(45))
                    .build();
        }
        //***POSITION 3***
        else if (autoPosition == 3) {
            deliverToFloorPose = new Pose2d(-34.5, -32, Math.toRadians(180));
            FloorTraj = myBot.getDrive().actionBuilder(startPose)
                    .splineToLinearHeading(new Pose2d(-38.5, -33, Math.toRadians(90)), Math.toRadians(90))
                    //.splineToLinearHeading(new Pose2d(-27, -33, Math.toRadians(180)), Math.toRadians(180))
                    //.splineToLinearHeading(deliverToFloorPose, Math.toRadians(180))
                    .strafeToLinearHeading(new Vector2d(-27, -33), Math.toRadians(180))
                    .strafeToLinearHeading(new Vector2d(deliverToFloorPose.position.x, deliverToFloorPose.position.y), Math.toRadians(180))
                    .build();
        }
        //***POSITION 2***
        else {
            deliverToFloorPose = new Pose2d(-38.5, -12.5, Math.toRadians(90));
            FloorTraj = myBot.getDrive().actionBuilder(startPose)
                    .splineToLinearHeading(new Pose2d(-38.5, -33, Math.toRadians(90)), Math.toRadians(90))
                    .splineToLinearHeading(deliverToFloorPose, Math.toRadians(90))
                    .build();
        }
    }

    static public void RedBoardDecision() {
        // Look for potential errors
        //***POSITION 1***
        if (autoPosition == 1) {
            deliverToBoardPose = new Pose2d(47.5,-28,Math.toRadians(180));
            BoardTraj2 = myBot.getDrive().actionBuilder(stackPose)
                    //.setTangent(0)
                    //.splineToLinearHeading(new Pose2d(-30, -11.5, Math.toRadians(180)), Math.toRadians(0))
                    //.splineToLinearHeading(new Pose2d(47.5, -11.5, Math.toRadians(180)), Math.toRadians(0))
                    .strafeToLinearHeading(new Vector2d(47.5, -11.5), Math.toRadians(180), speedUpVelocityConstraint)
                    .setTangent(Math.toRadians(270))
                    .splineToLinearHeading(deliverToBoardPose, Math.toRadians(270))
                    .build();
            //drive.actionBuilder(drive.  new Pose2d(50+control.rangeError, 36+control.yawError, Math.toRadians(180)))

        }
        //***POSITION 3***
        else if (autoPosition == 3) {
            deliverToBoardPose = new Pose2d(47.5,-38,Math.toRadians(180));
            BoardTraj2 = myBot.getDrive().actionBuilder(stackPose)
                    //.setTangent(0)
                    //.splineToLinearHeading(new Pose2d(47.5, -11.5, Math.toRadians(180)), Math.toRadians(0))
                    //.setTangent(Math.toRadians(270))
                    //.splineToLinearHeading(deliverToBoardPose, Math.toRadians(270))
                    .strafeToLinearHeading(new Vector2d(47.5, -11.5), Math.toRadians(180), speedUpVelocityConstraint)
                    .strafeToLinearHeading(new Vector2d(deliverToBoardPose.position.x, deliverToBoardPose.position.y), Math.toRadians(180))
                    .build();
            //drive.actionBuilder(drive.  new Pose2d(50+control.rangeError, 36+control.yawError, Math.toRadians(180)))
        }
        //***POSITION 2***
        else {
            deliverToBoardPose = new Pose2d(47.5,-33,Math.toRadians(180));
            BoardTraj2 = myBot.getDrive().actionBuilder(stackPose)
                    //.setTangent(0)
                    //.splineToLinearHeading(new Pose2d(-30, -11.5, Math.toRadians(180)), Math.toRadians(0))
                    //.splineToLinearHeading(new Pose2d(47.5, -11.5, Math.toRadians(180)), Math.toRadians(0))
                    .strafeToLinearHeading(new Vector2d(47.5, -11.5), Math.toRadians(180), speedUpVelocityConstraint)
                    .setTangent(Math.toRadians(270))
                    .splineToLinearHeading(deliverToBoardPose, Math.toRadians(270))
                    .build();
            //drive.actionBuilder(drive.  new Pose2d(50+control.rangeError, 36+control.yawError, Math.toRadians(180)))
        }
    }


}