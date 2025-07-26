
bool DummyRobot::MoveC(float x1, float y1, float z1, float x2, float y2, float z2, 
    float x3, float y3, float z3, float a, float b, float c)
{
    /**x1, y1, z1 Start*/
    /**x2, y2, z2 Middle*/
    /**x3, y3, z3 End*/

    /**The number of interpolation points can be 
     * adjusted according to actual needs*/
    const int16_t pN = 50;

    /**
     * Calculate parameters such as the center of the circle, 
     * radius, arc angle, etc Arc interpolation is performed to 
     * generate a series of points in sections Call MoveL or 
     * MoveJ for each interpolation point to implement 
     * trajectory tracking.
     */

    for (int16_t idx = 0; idx <= pN; ++idx) {
        float R = (float)idx / pN;
        /**Here a simple quadratic Bézier curve is used to approximate the arc 
         * (the arc interpolation formula can actually be used).*/
        float xt = (1.0f - R) * (1.0f - R) * x1 + 2 * (1.0f - R) * R * x2 + R * R * x3;
        float yt = (1.0f - R) * (1.0f - R) * y1 + 2 * (1.0f - R) * R * y2 + R * R * y3;
        float zt = (1.0f - R) * (1.0f - R) * z1 + 2 * (1.0f - R) * R * z2 + R * R * z3;
        /**Attitude linear interpolation*/
        float at = (1.0f - R) * a + R * a;
        float bt = (1.0f - R) * b + R * b;
        float ct = (1.0f - R) * c + R * c;
        /**Pointwise interpolation*/
        if (!MoveL(xt, yt, zt, at, bt, ct)) return false;
        MoveJoints(targetJoints);
        while (IsMoving()) osDelay(5);
    }
    return true;
}

uint32_t DummyRobot::CommandHandler::ParseCommand(const std::string &_cmd)
{
    uint8_t argNum;

    switch (context->commandMode)
    {
    case COMMAND_TARGET_POINT_SEQUENTIAL:
    case COMMAND_CONTINUES_TRAJECTORY:
        if (_cmd[0] == '*')
        {
            float x1, y1, z1, x2, y2, z2, x3, y3, z3, a, b, c;
            float speed;

            int argNum = sscanf(_cmd.c_str(), "*%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f",
                &x1, &y1, &z1, &x2, &y2, &z2, &x3, &y3, &z3, &a, &b, &c, &speed);
            if (argNum == 12) {
                context->MoveC(x1, y1, z1, x2, y2, z2, x3, y3, z3, a, b, c);
            }
            else if (argNum == 13)
            {
                context->SetJointSpeed(speed);
                context->MoveC(x1, y1, z1, x2, y2, z2, x3, y3, z3, a, b, c);
            }
            Respond(*usbStreamOutputPtr, "ok");
            Respond(*uart2StreamOutputPtr, "ok");
        }
        break;

    case COMMAND_TARGET_POINT_INTERRUPTABLE:
        if (_cmd[0] == '*')
        {
            float x1, y1, z1, x2, y2, z2, x3, y3, z3, a, b, c;
            float speed;

            int argNum = sscanf(_cmd.c_str(), "*%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f",
                &x1, &y1, &z1, &x2, &y2, &z2, &x3, &y3, &z3, &a, &b, &c, &speed);
            if (argNum == 12) {
                context->MoveC(x1, y1, z1, x2, y2, z2, x3, y3, z3, a, b, c);
            }
            else if (argNum == 13)
            {
                context->SetJointSpeed(speed);
                context->MoveC(x1, y1, z1, x2, y2, z2, x3, y3, z3, a, b, c);
            }
            Respond(*usbStreamOutputPtr, "ok");
            Respond(*uart2StreamOutputPtr, "ok");
        }
        break;

    case COMMAND_MOTOR_TUNING:
        break;
    }

    return osMessageQueueGetSpace(commandFifo);
}
