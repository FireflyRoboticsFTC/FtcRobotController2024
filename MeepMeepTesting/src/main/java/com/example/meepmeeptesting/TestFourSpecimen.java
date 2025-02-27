package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class TestFourSpecimen {
    public static double subY = 29;
    public static double specimenY = 55;
    public static double specimenX = -27;
    public static double slidePos = 1200;
    public static double slideDelay = 1200;
    public static double intakeVelocity = 14;
    public static double specimenDelay = 0.1;
    public static double clipAngleOffset1 = Math.toRadians(-90); //to submersible
    public static double clipAngleOffset2 = Math.toRadians(-94); //slow move
    public static double intakeAngleOffset = Math.toRadians(180-10); //to specimen

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(700);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setColorScheme(new ColorSchemeBlueDark())
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 10.471975439233053707565457666)
                .setDimensions(15.25, 17.25)
                .build();

        Pose2d initialPose = new Pose2d(-7.5, 61.375, Math.toRadians(-90));

        //6.12
        /*myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-7.5, subY+2, Math.toRadians(-90)))
                .setTangent(Math.PI / 2)
                .splineToConstantHeading(new Vector2d(-34, 36), Math.PI)
                .setTangent(-Math.PI / 2)
                .splineToLinearHeading(new Pose2d(-46, 16, Math.PI), Math.PI)
                .strafeTo(new Vector2d(-41, 53))
                .setTangent(-Math.PI / 2)
                .splineToLinearHeading(new Pose2d(-56, 16.5, Math.PI / 2), Math.PI, new TranslationalVelConstraint(60))
                .build());*/

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-7.5, subY+2, Math.toRadians(-90)))
                .setTangent(Math.PI / 2)
                .splineToLinearHeading(new Pose2d(-36, 36, Math.PI), Math.PI)
                .setTangent(-Math.PI / 2)
                .splineToLinearHeading(new Pose2d(-46, 16, Math.PI / 2), Math.PI)
                .strafeTo(new Vector2d(-46, 53))
                .setTangent(-Math.PI / 2)
                .splineToConstantHeading(new Vector2d(-56, 16.5), Math.PI, new TranslationalVelConstraint(60))
                .build());


        /*//left wall
        myBot.runAction(myBot.getDrive().actionBuilder(initialPose)
                //toSubmersibleOne
                .lineToY(32.625, new TranslationalVelConstraint(25.0))
                .waitSeconds(0.3)
                //pushTwoBlock
                .strafeTo(new Vector2d(-34, 34))
                .setTangent(-Math.PI / 2)
                .splineTo(new Vector2d(-46, 14), Math.PI)
                .strafeTo(new Vector2d(-46, 52))
                .setTangent(-Math.PI / 2)
                .splineTo(new Vector2d(-56, 14), Math.PI)
                .strafeTo(new Vector2d(-56, 50))
                //specimenTwo
                .strafeTo(new Vector2d(-56, 56))
                .waitSeconds(0.6)
                //toSubmersibleTwo
                .setTangent(-Math.PI/2)
                .splineToLinearHeading(new Pose2d(-2, 32.625, -Math.PI/2), -Math.PI/2)
                .waitSeconds(0.3)
                //specimenThree
                .setTangent(Math.PI/2)
                .splineToLinearHeading(new Pose2d(-50, 60, Math.PI), Math.PI)
                .strafeTo(new Vector2d(-56, 60))
                .waitSeconds(0.6)
                //toSubmersibleThree
                .setTangent(0)
                .splineToLinearHeading(new Pose2d(3, 32.625, -Math.PI/2), -Math.PI/2)
                .waitSeconds(0.3)
                //specimenFour
                .setTangent(Math.PI/2)
                .splineToLinearHeading(new Pose2d(-50, 60, Math.PI), Math.PI)
                .strafeTo(new Vector2d(-56, 60))
                .waitSeconds(0.6)
                //toSubmersibleFour
                .setTangent(0)
                .splineToLinearHeading(new Pose2d(8, 32.625, -Math.PI/2), -Math.PI/2)
                .waitSeconds(0.3)
                //toPark
                .strafeTo(new Vector2d(-40, 61.375))
                .build());
        */

        /*//back wall
        myBot.runAction(myBot.getDrive().actionBuilder(initialPose)
                //toSubmersibleOne
                .lineToY(32.625)
                //pushTwoBlock
                .strafeTo(new Vector2d(-34, 34))
                .setTangent(-Math.PI / 2)
                .splineTo(new Vector2d(-46, 14), Math.PI)
                .strafeTo(new Vector2d(-46, 52))
                .setTangent(-Math.PI / 2)
                .splineTo(new Vector2d(-56, 14), Math.PI)
                .strafeTo(new Vector2d(-56, 50))
                //specimenTwo
                .strafeTo(new Vector2d(-56, 56))
                .waitSeconds(0.6)
                //toSubmersibleTwo
                .setTangent(-Math.PI/2)
                .splineToLinearHeading(new Pose2d(-2, 32.625, -Math.PI/2), -Math.PI/2)
                .waitSeconds(0.3)
                //specimenThree
                .setTangent(Math.PI/2)
                .splineToLinearHeading(new Pose2d(-44, 50, Math.PI/2), Math.PI/2)
                .strafeTo(new Vector2d(-44, 56))
                .waitSeconds(0.6)
                //toSubmersibleThree
                .setTangent(-Math.PI/2)
                .splineToLinearHeading(new Pose2d(3, 32.625, -Math.PI/2), -Math.PI/2)
                .waitSeconds(0.3)
                //specimenFour
                .setTangent(Math.PI/2)
                .splineToLinearHeading(new Pose2d(-44, 50, Math.PI/2), Math.PI/2)
                .strafeTo(new Vector2d(-44, 56))
                .waitSeconds(0.6)
                //toSubmersibleFour
                .setTangent(-Math.PI/2)
                .splineToLinearHeading(new Pose2d(8, 32.625, -Math.PI/2), -Math.PI/2)
                .waitSeconds(0.3)
                //toPark
                .strafeTo(new Vector2d(-40, 61.375))
                .build());
        */

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
