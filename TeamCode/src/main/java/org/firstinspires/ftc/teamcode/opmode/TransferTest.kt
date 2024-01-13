package org.firstinspires.ftc.teamcode.opmode

import com.amarcolini.joos.command.*
import com.amarcolini.joos.dashboard.JoosConfig
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.CSRobot
import org.firstinspires.ftc.teamcode.Intake
import org.firstinspires.ftc.teamcode.Outtake

@TeleOp
class TransferTest : CommandOpMode() {
    private val robot by robot<CSRobot>()

    companion object {
        var armNeutral = 0.9
        var armTransfer = 0.92
        var rightPositions = arrayOf(0.37, 0.5, 0.7)
        var leftPositions = arrayOf(0.98, 0.85, 0.68)
    }

    override fun preInit() {
        robot.outtake.armServo.position = armNeutral
        robot.outtake.rightServo.position = rightPositions[0]
        robot.outtake.leftServo.position = leftPositions[0]

        map(gamepad.p1.a0::isJustActivated, Command.select {
            if (robot.intake.motorState == Intake.MotorState.STOPPED) robot.intake.intake()
            else robot.intake.stop()
        }.requires(robot.intake))

        map(gamepad.p1.y0::isJustActivated, transferCommand(robot.intake, robot.outtake))

        map(gamepad.p1.x0::isJustActivated,
            robot.outtake.armServo.waitForPosition(0.4, 0.5)
                .wait(1.0)
                .then {
                    robot.outtake.leftServo.position = leftPositions[1]
                    robot.outtake.rightServo.position = rightPositions[1]
                }
                .wait(1.0)
                .then(robot.outtake.armServo.waitForPosition(armNeutral, 0.5))
                .requires(robot.outtake)
        )

        schedule(true) {
            telem.addData("intake is free", CommandScheduler.isAvailable(robot.intake.intake()))
        }
    }

    private fun transferCommand(intake: Intake, outtake: Outtake): Command {
        return SequentialCommand(
            InstantCommand {
                outtake.leftServo.position = leftPositions[0]
                outtake.rightServo.position = rightPositions[0]
                intake.motorState = Intake.MotorState.ACTIVE
            },
            (intake.waitForServoState(Intake.ServoState.UP) and outtake.armServo.waitForPosition(armNeutral, 0.5)),
            InstantCommand { intake.motorState = Intake.MotorState.STOPPED },
            WaitCommand(0.3),
            outtake.armServo.waitForPosition(armTransfer, 0.5),
            WaitCommand(1.5).onInit { intake.motorState = Intake.MotorState.REVERSE },
            intake.stop(),
            WaitCommand(0.5),
            InstantCommand {
                outtake.leftServo.position = leftPositions[2]
                outtake.rightServo.position = rightPositions[2]
            },
            WaitCommand(0.5),
            outtake.armServo.waitForPosition(armNeutral, 0.5)
        ).requires(intake, outtake).setInterruptable(false)
    }
}