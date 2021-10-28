
void setExternalControlSetpoint(float roll, float pitch, float yaw, float thrust, quint16 buttons, int joystickMode)
{
    if (!_vehicle) {
        return;
    }

    if (!_vehicle->priorityLink()) {
        return;
    }

    // Store the previous manual commands
    static float manualRollAngle = 0.0;
    static float manualPitchAngle = 0.0;
    static float manualYawAngle = 0.0;
    static float manualThrust = 0.0;
    static quint16 manualButtons = 0;
    static quint8 countSinceLastTransmission = 0; // Track how many calls to this function have occurred since the last MAVLink transmission

    // Transmit the external setpoints only if they've changed OR if it's been a little bit since they were last transmit. To make sure there aren't issues with
    // response rate, we make sure that a message is transmit when the commands have changed, then one more time, and then switch to the lower transmission rate
    // if no command inputs have changed.

    // The default transmission rate is 25Hz, but when no inputs have changed it drops down to 5Hz.
    bool sendCommand = false;
    if (countSinceLastTransmission++ >= 5) {
        sendCommand = true;
        countSinceLastTransmission = 0;
    } else if ((!qIsNaN(roll) && roll != manualRollAngle) || (!qIsNaN(pitch) && pitch != manualPitchAngle) ||
             (!qIsNaN(yaw) && yaw != manualYawAngle) || (!qIsNaN(thrust) && thrust != manualThrust) ||
             buttons != manualButtons) {
        sendCommand = true;

        // Ensure that another message will be sent the next time this function is called
        countSinceLastTransmission = 10;
    }

    // Now if we should trigger an update, let's do that
    if (sendCommand) {
        // Save the new manual control inputs
        manualRollAngle = roll;
        manualPitchAngle = pitch;
        manualYawAngle = yaw;
        manualThrust = thrust;
        manualButtons = buttons;

        mavlink_message_t message;

        if (joystickMode == Vehicle::JoystickModeAttitude) {
            // send an external attitude setpoint command (rate control disabled)
            float attitudeQuaternion[4];
            mavlink_euler_to_quaternion(roll, pitch, yaw, attitudeQuaternion);
            uint8_t typeMask = 0x7; // disable rate control
            mavlink_msg_set_attitude_target_pack_chan(mavlink->getSystemId(),
                                                      mavlink->getComponentId(),
                                                      _vehicle->priorityLink()->mavlinkChannel(),
                                                      &message,
                                                      QGC::groundTimeUsecs(),
                                                      this->uasId,
                                                      0,
                                                      typeMask,
                                                      attitudeQuaternion,
                                                      0,
                                                      0,
                                                      0,
                                                      thrust);
        } else if (joystickMode == Vehicle::JoystickModePosition) {
            // Send the the local position setpoint (local pos sp external message)
            static float px = 0;
            static float py = 0;
            static float pz = 0;
            //XXX: find decent scaling
            px -= pitch;
            py += roll;
            pz -= 2.0f*(thrust-0.5);
            uint16_t typeMask = (1<<11)|(7<<6)|(7<<3); // select only POSITION control
            mavlink_msg_set_position_target_local_ned_pack_chan(mavlink->getSystemId(),
                                                                mavlink->getComponentId(),
                                                                _vehicle->priorityLink()->mavlinkChannel(),
                                                                &message,
                                                                QGC::groundTimeUsecs(),
                                                                this->uasId,
                                                                0,
                                                                MAV_FRAME_LOCAL_NED,
                                                                typeMask,
                                                                px,
                                                                py,
                                                                pz,
                                                                0,
                                                                0,
                                                                0,
                                                                0,
                                                                0,
                                                                0,
                                                                yaw,
                                                                0);
        } else if (joystickMode == Vehicle::JoystickModeForce) {
            // Send the the force setpoint (local pos sp external message)
            float dcm[3][3];
            mavlink_euler_to_dcm(roll, pitch, yaw, dcm);
            const float fx = -dcm[0][2] * thrust;
            const float fy = -dcm[1][2] * thrust;
            const float fz = -dcm[2][2] * thrust;
            uint16_t typeMask = (3<<10)|(7<<3)|(7<<0)|(1<<9); // select only FORCE control (disable everything else)
            mavlink_msg_set_position_target_local_ned_pack_chan(mavlink->getSystemId(),
                                                                mavlink->getComponentId(),
                                                                _vehicle->priorityLink()->mavlinkChannel(),
                                                                &message,
                                                                QGC::groundTimeUsecs(),
                                                                this->uasId,
                                                                0,
                                                                MAV_FRAME_LOCAL_NED,
                                                                typeMask,
                                                                0,
                                                                0,
                                                                0,
                                                                0,
                                                                0,
                                                                0,
                                                                fx,
                                                                fy,
                                                                fz,
                                                                0,
                                                                0);
        } else if (joystickMode == Vehicle::JoystickModeVelocity) {
            // Send the the local velocity setpoint (local pos sp external message)
            static float vx = 0;
            static float vy = 0;
            static float vz = 0;
            static float yawrate = 0;
            //XXX: find decent scaling
            vx -= pitch;
            vy += roll;
            vz -= 2.0f*(thrust-0.5);
            yawrate += yaw; //XXX: not sure what scale to apply here
            uint16_t typeMask = (1<<10)|(7<<6)|(7<<0); // select only VELOCITY control
            mavlink_msg_set_position_target_local_ned_pack_chan(mavlink->getSystemId(),
                                                                mavlink->getComponentId(),
                                                                _vehicle->priorityLink()->mavlinkChannel(),
                                                                &message,
                                                                QGC::groundTimeUsecs(),
                                                                this->uasId,
                                                                0,
                                                                MAV_FRAME_LOCAL_NED,
                                                                typeMask,
                                                                0,
                                                                0,
                                                                0,
                                                                vx,
                                                                vy,
                                                                vz,
                                                                0,
                                                                0,
                                                                0,
                                                                0,
                                                                yawrate);
        } else if (joystickMode == Vehicle::JoystickModeRC) {

            // Save the new manual control inputs
            manualRollAngle = roll;
            manualPitchAngle = pitch;
            manualYawAngle = yaw;
            manualThrust = thrust;
            manualButtons = buttons;

            // Store scaling values for all 3 axes
            const float axesScaling = 1.0 * 1000.0;

            // Calculate the new commands for roll, pitch, yaw, and thrust
            const float newRollCommand = roll * axesScaling;
            // negate pitch value because pitch is negative for pitching forward but mavlink message argument is positive for forward
            const float newPitchCommand = -pitch * axesScaling;
            const float newYawCommand = yaw * axesScaling;
            const float newThrustCommand = thrust * axesScaling;

            //qDebug() << newRollCommand << newPitchCommand << newYawCommand << newThrustCommand;

            // Send the MANUAL_COMMAND message
            mavlink_msg_manual_control_pack_chan(mavlink->getSystemId(),
                                                 mavlink->getComponentId(),
                                                 _vehicle->priorityLink()->mavlinkChannel(),
                                                 &message,
                                                 this->uasId,
                                                 newPitchCommand, newRollCommand, newThrustCommand, newYawCommand, buttons);
        }

        _vehicle->sendMessageOnLink(_vehicle->priorityLink(), message);
    }
}

#ifndef __mobile__
void UAS::setManual6DOFControlCommands(double x, double y, double z, double roll, double pitch, double yaw)
{
    if (!_vehicle) {
        return;
    }
    const uint8_t base_mode = _vehicle->baseMode();

   // If system has manual inputs enabled and is armed
    if(((base_mode & MAV_MODE_FLAG_DECODE_POSITION_MANUAL) && (base_mode & MAV_MODE_FLAG_DECODE_POSITION_SAFETY)) || (base_mode & MAV_MODE_FLAG_HIL_ENABLED))
    {
        mavlink_message_t message;
        float q[4];
        mavlink_euler_to_quaternion(roll, pitch, yaw, q);

        float yawrate = 0.0f;

        // Do not control rates and throttle
        quint8 mask = (1 << 0) | (1 << 1) | (1 << 2); // ignore rates
        mask |= (1 << 6); // ignore throttle
        mavlink_msg_set_attitude_target_pack_chan(mavlink->getSystemId(),
                                                  mavlink->getComponentId(),
                                                  _vehicle->priorityLink()->mavlinkChannel(),
                                                  &message,
                                                  QGC::groundTimeMilliseconds(), this->uasId, _vehicle->defaultComponentId(),
                                                  mask, q, 0, 0, 0, 0);
        _vehicle->sendMessageOnLink(_vehicle->priorityLink(), message);
        quint16 position_mask = (1 << 3) | (1 << 4) | (1 << 5) |
            (1 << 6) | (1 << 7) | (1 << 8);
        mavlink_msg_set_position_target_local_ned_pack_chan(mavlink->getSystemId(), mavlink->getComponentId(),
                                                            _vehicle->priorityLink()->mavlinkChannel(),
                                                            &message, QGC::groundTimeMilliseconds(), this->uasId, _vehicle->defaultComponentId(),
                                                            MAV_FRAME_LOCAL_NED, position_mask, x, y, z, 0, 0, 0, 0, 0, 0, yaw, yawrate);
        _vehicle->sendMessageOnLink(_vehicle->priorityLink(), message);
        qDebug() << __FILE__ << __LINE__ << ": SENT 6DOF CONTROL MESSAGES: x" << x << " y: " << y << " z: " << z << " roll: " << roll << " pitch: " << pitch << " yaw: " << yaw;
    }
    else
    {
        qDebug() << "3DMOUSE/MANUAL CONTROL: IGNORING COMMANDS: Set mode to MANUAL to send 3DMouse commands first";
    }
}
#endif

/**
* Order the robot to start receiver pairing
*/
void UAS::pairRX(int rxType, int rxSubType)
{
    if (_vehicle) {
        _vehicle->sendMavCommand(_vehicle->defaultComponentId(),    // target component
                                 MAV_CMD_START_RX_PAIR,             // command id
                                 true,                              // showError
                                 rxType,
                                 rxSubType);
    }
}
