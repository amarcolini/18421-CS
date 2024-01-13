package org.firstinspires.ftc.teamcode.opmode

import com.amarcolini.joos.command.Command
import com.amarcolini.joos.command.CommandOpMode
import com.amarcolini.joos.dashboard.JoosConfig
import com.amarcolini.joos.geometry.Pose2d
import com.amarcolini.joos.util.rad
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.CSRobot
import org.firstinspires.ftc.teamcode.Intake
import kotlin.math.pow

@JoosConfig
@TeleOp(name = "DriveControl", group = "Official")
class DriveControl : CommandOpMode() {
    private val robot by robot<CSRobot>()
    override fun preInit() {
        robot.pixelPlopper.prime()
        robot.outtake.reset().schedule()
        robot.verticalExtension.initializeEncoders().schedule()
    }

    override fun preStart() {
        schedule(true) {
            val leftStick = gamepad.p1.getLeftStick()
            val rightStick = gamepad.p1.getRightStick()

            robot.drive.setDrivePower(
                Pose2d(
                    -leftStick.y,
                    -leftStick.x,
                    -rightStick.x.rad
                )
            )

            val leftTrigger = gamepad.p1.left_trigger.value
            val rightTrigger = gamepad.p1.right_trigger.value
            var power = (rightTrigger - leftTrigger).pow(3).toDouble() * 1.0 + 0.1
            if (!robot.verticalExtension.bottomSensor.state && power < 0) power = 0.0
            robot.verticalExtension.motors.setPower(
                power
            )
        }

        map(gamepad.p1.y0::isJustActivated, Command.select {
            if (robot.outtake.isExtended) robot.outtake.resetArm()
            else robot.outtake.extend()
        })

        map(gamepad.p1.left_bumper::isJustActivated, robot.outtake::releaseLeft)
        map(gamepad.p1.right_bumper::isJustActivated, robot.outtake::releaseRight)

        map(gamepad.p1.b0::isJustActivated, robot.transfer())

        map(gamepad.p1.a0::isJustActivated, Command.select(robot.intake) {
            when (robot.intake.motorState) {
                Intake.MotorState.ACTIVE -> robot.intake.stop()
                else -> robot.intake.intake()
            }
        })
        map(gamepad.p1.x0::isJustActivated, Command.select(robot.intake) {
            when (robot.intake.servoState) {
                Intake.ServoState.DOWN -> robot.intake.waitForServoState(Intake.ServoState.UP)
                Intake.ServoState.UP -> robot.intake.waitForServoState(Intake.ServoState.DOWN)
            }
        })
        map(gamepad.p1.dpad_left::isJustActivated, robot.intake.reverse())

        map(gamepad.p1.dpad_up::isJustActivated, robot.droneLauncher::launch)
        map(gamepad.p1.dpad_down::isJustActivated, robot.droneLauncher::reset)

//        map(gamepad.p1.dpad_left::isJustActivated) {
//            if (robot.pixelPlopper.isPrimed) robot.pixelPlopper.open()
//            else robot.pixelPlopper.prime()
//        }
        map(gamepad.p1.dpad_right::isJustActivated, robot.outtake.prepareClimb())
    }
}