package org.firstinspires.ftc.teamcode

import com.amarcolini.joos.command.AbstractComponent
import com.amarcolini.joos.command.Component
import com.amarcolini.joos.control.DCMotorFeedforward
import com.amarcolini.joos.control.PIDCoefficients
import com.amarcolini.joos.control.PIDController
import com.amarcolini.joos.dashboard.JoosConfig
import com.amarcolini.joos.hardware.Motor
import com.amarcolini.joos.hardware.MotorGroup

@JoosConfig
class VerticalExtension(val motors: MotorGroup) : AbstractComponent() {
    companion object {
        val coeffs = PIDCoefficients(0.01, 0.002, 0.00011)
        val feedforward = DCMotorFeedforward()
        private val maxTicks = 2350.0
        private val minTicks = -50.0
    }

    private val positionController = PIDController(coeffs)
    var positionControlEnabled = false

    fun setTargetPosition(position: Double) {
        positionController.targetPosition = position
        positionController.update(
            motors.currentPosition,
            motors.velocity
        )
    }

    init {
        subcomponents.add(motors)
        motors.runMode = Motor.RunMode.RUN_WITHOUT_ENCODER
        motors.zeroPowerBehavior = Motor.ZeroPowerBehavior.BRAKE
        motors.feedforward = feedforward
        positionController.setOutputBounds(-1.0, 1.0)
        positionController.tolerance = 20.0
    }

    override fun update() {
        super.update()
        if (positionControlEnabled) {
            motors.setPower(
                if (positionController.isAtSetPoint() && motors.velocity <= 100.0) 0.0 else positionController.update(
                    motors.currentPosition,
                    motors.velocity
                )
            )
            telem.addData("liftTargetPosition", positionController.targetPosition)
            telem.addData("liftPosition", motors.currentPosition)
        }
    }
}