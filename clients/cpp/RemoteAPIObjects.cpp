namespace RemoteAPIObject
{
    sim::sim(RemoteAPIClient *client)
        : _client(client)
    {
    }

    // DEPRECATED START
    double sim::getJointMaxForce(int64_t jointHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(jointHandle);
        auto _ret = this->_client->call("sim.getJointMaxForce", _args);
        return _ret[0].as<double>();
    }
    void sim::setJointMaxForce(int64_t objectHandle, double forceOrTorque)
    {
        json _args(json_array_arg);
        _args.push_back(objectHandle);
        _args.push_back(forceOrTorque);
        auto _ret = this->_client->call("sim.setJointMaxForce", _args);
    }
    int64_t sim::createPureShape(int64_t primitiveType, int64_t options, std::vector<double> sizes, double mass, std::optional<std::vector<int64_t>> precision)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(primitiveType);
        _args.push_back(options);
        _args.push_back(sizes);
        _args.push_back(mass);
        if(precision)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*precision);
        }
        else _brk = true;
        auto _ret = this->_client->call("sim.createPureShape", _args);
        return _ret[0].as<int64_t>();
    }
    void sim::removeObject(int64_t objectHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(objectHandle);
        auto _ret = this->_client->call("sim.removeObject", _args);
    }
    std::tuple<std::vector<uint8_t>, std::vector<int64_t>> sim::getVisionSensorDepthBuffer(int64_t sensorHandle, std::optional<std::vector<int64_t>> pos, std::optional<std::vector<int64_t>> size)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(sensorHandle);
        if(pos)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*pos);
        }
        else _brk = true;
        if(size)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*size);
        }
        else _brk = true;
        auto _ret = this->_client->call("sim.getVisionSensorDepthBuffer", _args);
        return std::make_tuple(_ret[0].as<std::vector<uint8_t>>(), _ret[1].as<std::vector<int64_t>>());
    }
    std::tuple<std::vector<uint8_t>, std::vector<int64_t>> sim::getVisionSensorCharImage(int64_t sensorHandle, std::optional<std::vector<int64_t>> pos, std::optional<std::vector<int64_t>> size)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(sensorHandle);
        if(pos)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*pos);
        }
        else _brk = true;
        if(size)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*size);
        }
        else _brk = true;
        auto _ret = this->_client->call("sim.getVisionSensorCharImage", _args);
        return std::make_tuple(_ret[0].as<std::vector<uint8_t>>(), _ret[1].as<std::vector<int64_t>>());
    }
    void sim::setVisionSensorCharImage(int64_t sensorHandle, std::vector<uint8_t> image)
    {
        json _args(json_array_arg);
        _args.push_back(sensorHandle);
        _args.push_back(bin(image));
        auto _ret = this->_client->call("sim.setVisionSensorCharImage", _args);
    }
    // DEPRECATED END




    int64_t sim::addDrawingObject(int64_t objectType, double size, double duplicateTolerance, int64_t parentObjectHandle, int64_t maxItemCount, std::optional<std::vector<double>> ambient_diffuse, std::optional<std::vector<double>> reserved, std::optional<std::vector<double>> specular, std::optional<std::vector<double>> emission)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(objectType);
        _args.push_back(size);
        _args.push_back(duplicateTolerance);
        _args.push_back(parentObjectHandle);
        _args.push_back(maxItemCount);
        if(ambient_diffuse)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*ambient_diffuse);
        }
        else _brk = true;
        if(reserved)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*reserved);
        }
        else _brk = true;
        if(specular)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*specular);
        }
        else _brk = true;
        if(emission)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*emission);
        }
        else _brk = true;
        auto _ret = this->_client->call("sim.addDrawingObject", _args);
        return _ret[0].as<int64_t>();
    }

    int64_t sim::addDrawingObjectItem(int64_t drawingObjectHandle, std::vector<double> itemData)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(drawingObjectHandle);
        _args.push_back(itemData);
        auto _ret = this->_client->call("sim.addDrawingObjectItem", _args);
        return _ret[0].as<int64_t>();
    }

    void sim::addForce(int64_t shapeHandle, std::vector<double> position, std::vector<double> force)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(shapeHandle);
        _args.push_back(position);
        _args.push_back(force);
        auto _ret = this->_client->call("sim.addForce", _args);
    }

    void sim::addForceAndTorque(int64_t shapeHandle, std::optional<std::vector<double>> force, std::optional<std::vector<double>> torque)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(shapeHandle);
        if(force)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*force);
        }
        else _brk = true;
        if(torque)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*torque);
        }
        else _brk = true;
        auto _ret = this->_client->call("sim.addForceAndTorque", _args);
    }

    int64_t sim::addGraphCurve(int64_t graphHandle, std::string curveName, int64_t dim, std::vector<int64_t> streamIds, std::vector<double> defaultValues, std::string unitStr, std::optional<int64_t> options, std::optional<std::vector<double>> color, std::optional<int64_t> curveWidth)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(graphHandle);
        _args.push_back(curveName);
        _args.push_back(dim);
        _args.push_back(streamIds);
        _args.push_back(defaultValues);
        _args.push_back(unitStr);
        if(options)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*options);
        }
        else _brk = true;
        if(color)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*color);
        }
        else _brk = true;
        if(curveWidth)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*curveWidth);
        }
        else _brk = true;
        auto _ret = this->_client->call("sim.addGraphCurve", _args);
        return _ret[0].as<int64_t>();
    }

    int64_t sim::addGraphStream(int64_t graphHandle, std::string streamName, std::string unit, std::optional<int64_t> options, std::optional<std::vector<double>> color, std::optional<double> cyclicRange)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(graphHandle);
        _args.push_back(streamName);
        _args.push_back(unit);
        if(options)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*options);
        }
        else _brk = true;
        if(color)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*color);
        }
        else _brk = true;
        if(cyclicRange)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*cyclicRange);
        }
        else _brk = true;
        auto _ret = this->_client->call("sim.addGraphStream", _args);
        return _ret[0].as<int64_t>();
    }

    void sim::addItemToCollection(int64_t collectionHandle, int64_t what, int64_t objectHandle, int64_t options)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(collectionHandle);
        _args.push_back(what);
        _args.push_back(objectHandle);
        _args.push_back(options);
        auto _ret = this->_client->call("sim.addItemToCollection", _args);
    }

    void sim::addLog(int64_t verbosityLevel, std::string logMessage)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(verbosityLevel);
        _args.push_back(logMessage);
        auto _ret = this->_client->call("sim.addLog", _args);
    }

    int64_t sim::addParticleObject(int64_t objectType, double size, double density, std::vector<double> params, double lifeTime, int64_t maxItemCount, std::optional<std::vector<double>> ambient_diffuse, std::optional<std::vector<double>> reserved, std::optional<std::vector<double>> specular, std::optional<std::vector<double>> emission)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(objectType);
        _args.push_back(size);
        _args.push_back(density);
        _args.push_back(params);
        _args.push_back(lifeTime);
        _args.push_back(maxItemCount);
        if(ambient_diffuse)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*ambient_diffuse);
        }
        else _brk = true;
        if(reserved)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*reserved);
        }
        else _brk = true;
        if(specular)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*specular);
        }
        else _brk = true;
        if(emission)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*emission);
        }
        else _brk = true;
        auto _ret = this->_client->call("sim.addParticleObject", _args);
        return _ret[0].as<int64_t>();
    }

    void sim::addParticleObjectItem(int64_t particleObjectHandle, std::vector<double> itemData)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(particleObjectHandle);
        _args.push_back(itemData);
        auto _ret = this->_client->call("sim.addParticleObjectItem", _args);
    }

    int64_t sim::addScript(int64_t scriptType)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(scriptType);
        auto _ret = this->_client->call("sim.addScript", _args);
        return _ret[0].as<int64_t>();
    }

    int64_t sim::adjustView(int64_t viewHandleOrIndex, int64_t associatedViewableObjectHandle, int64_t options, std::optional<std::string> viewLabel)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(viewHandleOrIndex);
        _args.push_back(associatedViewableObjectHandle);
        _args.push_back(options);
        if(viewLabel)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*viewLabel);
        }
        else _brk = true;
        auto _ret = this->_client->call("sim.adjustView", _args);
        return _ret[0].as<int64_t>();
    }

    std::tuple<double, double, double> sim::alphaBetaGammaToYawPitchRoll(double alphaAngle, double betaAngle, double gammaAngle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(alphaAngle);
        _args.push_back(betaAngle);
        _args.push_back(gammaAngle);
        auto _ret = this->_client->call("sim.alphaBetaGammaToYawPitchRoll", _args);
        return std::make_tuple(_ret[0].as<double>(), _ret[1].as<double>(), _ret[2].as<double>());
    }

    int64_t sim::announceSceneContentChange()
    {
        bool _brk = false;
        json _args(json_array_arg);
        auto _ret = this->_client->call("sim.announceSceneContentChange", _args);
        return _ret[0].as<int64_t>();
    }

    void sim::associateScriptWithObject(int64_t scriptHandle, int64_t objectHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(scriptHandle);
        _args.push_back(objectHandle);
        auto _ret = this->_client->call("sim.associateScriptWithObject", _args);
    }

    int64_t sim::auxiliaryConsoleClose(int64_t consoleHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(consoleHandle);
        auto _ret = this->_client->call("sim.auxiliaryConsoleClose", _args);
        return _ret[0].as<int64_t>();
    }

    int64_t sim::auxiliaryConsoleOpen(std::string title, int64_t maxLines, int64_t mode, std::optional<std::vector<int64_t>> position, std::optional<std::vector<int64_t>> size, std::optional<std::vector<double>> textColor, std::optional<std::vector<double>> backgroundColor)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(title);
        _args.push_back(maxLines);
        _args.push_back(mode);
        if(position)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*position);
        }
        else _brk = true;
        if(size)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*size);
        }
        else _brk = true;
        if(textColor)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*textColor);
        }
        else _brk = true;
        if(backgroundColor)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*backgroundColor);
        }
        else _brk = true;
        auto _ret = this->_client->call("sim.auxiliaryConsoleOpen", _args);
        return _ret[0].as<int64_t>();
    }

    int64_t sim::auxiliaryConsolePrint(int64_t consoleHandle, std::string text)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(consoleHandle);
        _args.push_back(text);
        auto _ret = this->_client->call("sim.auxiliaryConsolePrint", _args);
        return _ret[0].as<int64_t>();
    }

    int64_t sim::auxiliaryConsoleShow(int64_t consoleHandle, bool showState)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(consoleHandle);
        _args.push_back(showState);
        auto _ret = this->_client->call("sim.auxiliaryConsoleShow", _args);
        return _ret[0].as<int64_t>();
    }

    void sim::broadcastMsg(json message, std::optional<int64_t> options)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(message);
        if(options)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*options);
        }
        else _brk = true;
        auto _ret = this->_client->call("sim.broadcastMsg", _args);
    }

    std::vector<double> sim::buildIdentityMatrix()
    {
        bool _brk = false;
        json _args(json_array_arg);
        auto _ret = this->_client->call("sim.buildIdentityMatrix", _args);
        return _ret[0].as<std::vector<double>>();
    }

    std::vector<double> sim::buildMatrix(std::vector<double> position, std::vector<double> eulerAngles)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(position);
        _args.push_back(eulerAngles);
        auto _ret = this->_client->call("sim.buildMatrix", _args);
        return _ret[0].as<std::vector<double>>();
    }

    std::vector<double> sim::buildMatrixQ(std::vector<double> position, std::vector<double> quaternion)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(position);
        _args.push_back(quaternion);
        auto _ret = this->_client->call("sim.buildMatrixQ", _args);
        return _ret[0].as<std::vector<double>>();
    }

    std::vector<double> sim::buildPose(std::vector<double> position, std::vector<double> eulerAnglesOrAxis, std::optional<int64_t> mode, std::optional<std::vector<double>> axis2)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(position);
        _args.push_back(eulerAnglesOrAxis);
        if(mode)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*mode);
        }
        else _brk = true;
        if(axis2)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*axis2);
        }
        else _brk = true;
        auto _ret = this->_client->call("sim.buildPose", _args);
        return _ret[0].as<std::vector<double>>();
    }

    json sim::callScriptFunction(std::string functionName, int64_t scriptHandle, std::optional<json> inArg)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(functionName);
        _args.push_back(scriptHandle);
        if(inArg)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*inArg);
        }
        else _brk = true;
        auto _ret = this->_client->call("sim.callScriptFunction", _args);
        return _ret[0].as<json>();
    }

    int64_t sim::cameraFitToView(int64_t viewHandleOrIndex, std::optional<std::vector<int64_t>> objectHandles, std::optional<int64_t> options, std::optional<double> scaling)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(viewHandleOrIndex);
        if(objectHandles)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*objectHandles);
        }
        else _brk = true;
        if(options)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*options);
        }
        else _brk = true;
        if(scaling)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*scaling);
        }
        else _brk = true;
        auto _ret = this->_client->call("sim.cameraFitToView", _args);
        return _ret[0].as<int64_t>();
    }

    std::vector<json> sim::changeEntityColor(int64_t entityHandle, std::vector<double> newColor, std::optional<int64_t> colorComponent)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(entityHandle);
        _args.push_back(newColor);
        if(colorComponent)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*colorComponent);
        }
        else _brk = true;
        auto _ret = this->_client->call("sim.changeEntityColor", _args);
        return _ret[0].as<std::vector<json>>();
    }

    std::tuple<int64_t, std::vector<int64_t>> sim::checkCollision(int64_t entity1Handle, int64_t entity2Handle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(entity1Handle);
        _args.push_back(entity2Handle);
        auto _ret = this->_client->call("sim.checkCollision", _args);
        return std::make_tuple(_ret[0].as<int64_t>(), _ret[1].as<std::vector<int64_t>>());
    }

    std::tuple<int64_t, std::vector<double>> sim::checkCollisionEx(int64_t entity1Handle, int64_t entity2Handle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(entity1Handle);
        _args.push_back(entity2Handle);
        auto _ret = this->_client->call("sim.checkCollisionEx", _args);
        return std::make_tuple(_ret[0].as<int64_t>(), _ret[1].as<std::vector<double>>());
    }

    std::tuple<int64_t, std::vector<double>, std::vector<int64_t>> sim::checkDistance(int64_t entity1Handle, int64_t entity2Handle, std::optional<double> threshold)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(entity1Handle);
        _args.push_back(entity2Handle);
        if(threshold)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*threshold);
        }
        else _brk = true;
        auto _ret = this->_client->call("sim.checkDistance", _args);
        return std::make_tuple(_ret[0].as<int64_t>(), _ret[1].as<std::vector<double>>(), _ret[2].as<std::vector<int64_t>>());
    }

    std::tuple<int64_t, int64_t, int64_t, int64_t> sim::checkOctreePointOccupancy(int64_t octreeHandle, int64_t options, std::vector<double> points)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(octreeHandle);
        _args.push_back(options);
        _args.push_back(points);
        auto _ret = this->_client->call("sim.checkOctreePointOccupancy", _args);
        return std::make_tuple(_ret[0].as<int64_t>(), _ret[1].as<int64_t>(), _ret[2].as<int64_t>(), _ret[3].as<int64_t>());
    }

    std::tuple<int64_t, double, std::vector<double>, int64_t, std::vector<double>> sim::checkProximitySensor(int64_t sensorHandle, int64_t entityHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(sensorHandle);
        _args.push_back(entityHandle);
        auto _ret = this->_client->call("sim.checkProximitySensor", _args);
        return std::make_tuple(_ret[0].as<int64_t>(), _ret[1].as<double>(), _ret[2].as<std::vector<double>>(), _ret[3].as<int64_t>(), _ret[4].as<std::vector<double>>());
    }

    std::tuple<int64_t, double, std::vector<double>, int64_t, std::vector<double>> sim::checkProximitySensorEx(int64_t sensorHandle, int64_t entityHandle, int64_t mode, double threshold, double maxAngle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(sensorHandle);
        _args.push_back(entityHandle);
        _args.push_back(mode);
        _args.push_back(threshold);
        _args.push_back(maxAngle);
        auto _ret = this->_client->call("sim.checkProximitySensorEx", _args);
        return std::make_tuple(_ret[0].as<int64_t>(), _ret[1].as<double>(), _ret[2].as<std::vector<double>>(), _ret[3].as<int64_t>(), _ret[4].as<std::vector<double>>());
    }

    std::tuple<int64_t, double, std::vector<double>, std::vector<double>> sim::checkProximitySensorEx2(int64_t sensorHandle, std::vector<double> vertices, int64_t itemType, int64_t itemCount, int64_t mode, double threshold, double maxAngle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(sensorHandle);
        _args.push_back(vertices);
        _args.push_back(itemType);
        _args.push_back(itemCount);
        _args.push_back(mode);
        _args.push_back(threshold);
        _args.push_back(maxAngle);
        auto _ret = this->_client->call("sim.checkProximitySensorEx2", _args);
        return std::make_tuple(_ret[0].as<int64_t>(), _ret[1].as<double>(), _ret[2].as<std::vector<double>>(), _ret[3].as<std::vector<double>>());
    }

    std::tuple<int64_t, std::vector<double>, std::vector<double>> sim::checkVisionSensor(int64_t sensorHandle, int64_t entityHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(sensorHandle);
        _args.push_back(entityHandle);
        auto _ret = this->_client->call("sim.checkVisionSensor", _args);
        return std::make_tuple(_ret[0].as<int64_t>(), _ret[1].as<std::vector<double>>(), _ret[2].as<std::vector<double>>());
    }

    std::vector<double> sim::checkVisionSensorEx(int64_t sensorHandle, int64_t entityHandle, bool returnImage)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(sensorHandle);
        _args.push_back(entityHandle);
        _args.push_back(returnImage);
        auto _ret = this->_client->call("sim.checkVisionSensorEx", _args);
        return _ret[0].as<std::vector<double>>();
    }

    void sim::clearDoubleSignal(std::string signalName)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(signalName);
        auto _ret = this->_client->call("clearDoubleSignal", _args);
    }

    void sim::clearFloatSignal(std::string signalName)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(signalName);
        auto _ret = this->_client->call("sim.clearFloatSignal", _args);
    }

    void sim::clearInt32Signal(std::string signalName)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(signalName);
        auto _ret = this->_client->call("sim.clearInt32Signal", _args);
    }

    void sim::clearStringSignal(std::string signalName)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(signalName);
        auto _ret = this->_client->call("clearStringSignal", _args);
    }

    int64_t sim::closeScene()
    {
        bool _brk = false;
        json _args(json_array_arg);
        auto _ret = this->_client->call("sim.closeScene", _args);
        return _ret[0].as<int64_t>();
    }

    std::vector<uint8_t> sim::combineRgbImages(std::vector<uint8_t> img1, std::vector<int64_t> img1Res, std::vector<uint8_t> img2, std::vector<int64_t> img2Res, int64_t operation)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(bin(img1));
        _args.push_back(img1Res);
        _args.push_back(bin(img2));
        _args.push_back(img2Res);
        _args.push_back(operation);
        auto _ret = this->_client->call("sim.combineRgbImages", _args);
        return _ret[0].as<std::vector<uint8_t>>();
    }

    int64_t sim::computeMassAndInertia(int64_t shapeHandle, double density)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(shapeHandle);
        _args.push_back(density);
        auto _ret = this->_client->call("sim.computeMassAndInertia", _args);
        return _ret[0].as<int64_t>();
    }

    int64_t sim::convexDecompose(int64_t shapeHandle, int64_t options, std::vector<int64_t> intParams, std::vector<double> floatParams)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(shapeHandle);
        _args.push_back(options);
        _args.push_back(intParams);
        _args.push_back(floatParams);
        auto _ret = this->_client->call("sim.convexDecompose", _args);
        return _ret[0].as<int64_t>();
    }

    std::vector<int64_t> sim::copyPasteObjects(std::vector<int64_t> objectHandles, std::optional<int64_t> options)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(objectHandles);
        if(options)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*options);
        }
        else _brk = true;
        auto _ret = this->_client->call("sim.copyPasteObjects", _args);
        return _ret[0].as<std::vector<int64_t>>();
    }

    std::vector<json> sim::copyTable(std::vector<json> original)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(original);
        auto _ret = this->_client->call("sim.copyTable", _args);
        return _ret[0].as<std::vector<json>>();
    }

    int64_t sim::createCollection(std::optional<int64_t> options)
    {
        bool _brk = false;
        json _args(json_array_arg);
        if(options)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*options);
        }
        else _brk = true;
        auto _ret = this->_client->call("sim.createCollection", _args);
        return _ret[0].as<int64_t>();
    }

    int64_t sim::createDummy(double size)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(size);
        auto _ret = this->_client->call("sim.createDummy", _args);
        return _ret[0].as<int64_t>();
    }

    int64_t sim::createForceSensor(int64_t options, std::vector<int64_t> intParams, std::vector<double> floatParams)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(options);
        _args.push_back(intParams);
        _args.push_back(floatParams);
        auto _ret = this->_client->call("sim.createForceSensor", _args);
        return _ret[0].as<int64_t>();
    }

    int64_t sim::createHeightfieldShape(int64_t options, double shadingAngle, int64_t xPointCount, int64_t yPointCount, double xSize, std::vector<double> heights)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(options);
        _args.push_back(shadingAngle);
        _args.push_back(xPointCount);
        _args.push_back(yPointCount);
        _args.push_back(xSize);
        _args.push_back(heights);
        auto _ret = this->_client->call("sim.createHeightfieldShape", _args);
        return _ret[0].as<int64_t>();
    }

    int64_t sim::createJoint(int64_t jointType, int64_t jointMode, int64_t options, std::optional<std::vector<double>> sizes)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(jointType);
        _args.push_back(jointMode);
        _args.push_back(options);
        if(sizes)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*sizes);
        }
        else _brk = true;
        auto _ret = this->_client->call("sim.createJoint", _args);
        return _ret[0].as<int64_t>();
    }

    int64_t sim::createMeshShape(int64_t options, double shadingAngle, std::vector<double> vertices, std::vector<int64_t> indices)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(options);
        _args.push_back(shadingAngle);
        _args.push_back(vertices);
        _args.push_back(indices);
        auto _ret = this->_client->call("sim.createMeshShape", _args);
        return _ret[0].as<int64_t>();
    }

    int64_t sim::createOctree(double voxelSize, int64_t options, double pointSize)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(voxelSize);
        _args.push_back(options);
        _args.push_back(pointSize);
        auto _ret = this->_client->call("sim.createOctree", _args);
        return _ret[0].as<int64_t>();
    }

    int64_t sim::createPath(std::vector<double> ctrlPts, std::optional<int64_t> options, std::optional<int64_t> subdiv, std::optional<double> smoothness, std::optional<int64_t> orientationMode, std::optional<std::vector<double>> upVector)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(ctrlPts);
        if(options)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*options);
        }
        else _brk = true;
        if(subdiv)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*subdiv);
        }
        else _brk = true;
        if(smoothness)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*smoothness);
        }
        else _brk = true;
        if(orientationMode)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*orientationMode);
        }
        else _brk = true;
        if(upVector)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*upVector);
        }
        else _brk = true;
        auto _ret = this->_client->call("sim.createPath", _args);
        return _ret[0].as<int64_t>();
    }

    int64_t sim::createPointCloud(double maxVoxelSize, int64_t maxPtCntPerVoxel, int64_t options, double pointSize)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(maxVoxelSize);
        _args.push_back(maxPtCntPerVoxel);
        _args.push_back(options);
        _args.push_back(pointSize);
        auto _ret = this->_client->call("sim.createPointCloud", _args);
        return _ret[0].as<int64_t>();
    }

    int64_t sim::createPrimitiveShape(int64_t primitiveType, std::vector<double> sizes, std::optional<int64_t> options)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(primitiveType);
        _args.push_back(sizes);
        if(options)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*options);
        }
        else _brk = true;
        auto _ret = this->_client->call("sim.createPrimitiveShape", _args);
        return _ret[0].as<int64_t>();
    }

    int64_t sim::createProximitySensor(int64_t sensorType, int64_t subType, int64_t options, std::vector<int64_t> intParams, std::vector<double> floatParams)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(sensorType);
        _args.push_back(subType);
        _args.push_back(options);
        _args.push_back(intParams);
        _args.push_back(floatParams);
        auto _ret = this->_client->call("sim.createProximitySensor", _args);
        return _ret[0].as<int64_t>();
    }

    std::tuple<int64_t, int64_t, std::vector<int64_t>> sim::createTexture(std::string fileName, int64_t options, std::optional<std::vector<double>> planeSizes, std::optional<std::vector<double>> scalingUV, std::optional<std::vector<double>> xy_g, std::optional<int64_t> fixedResolution, std::optional<std::vector<int64_t>> resolution)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(fileName);
        _args.push_back(options);
        if(planeSizes)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*planeSizes);
        }
        else _brk = true;
        if(scalingUV)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*scalingUV);
        }
        else _brk = true;
        if(xy_g)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*xy_g);
        }
        else _brk = true;
        if(fixedResolution)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*fixedResolution);
        }
        else _brk = true;
        if(resolution)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*resolution);
        }
        else _brk = true;
        auto _ret = this->_client->call("sim.createTexture", _args);
        return std::make_tuple(_ret[0].as<int64_t>(), _ret[1].as<int64_t>(), _ret[2].as<std::vector<int64_t>>());
    }

    int64_t sim::createVisionSensor(int64_t options, std::vector<int64_t> intParams, std::vector<double> floatParams)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(options);
        _args.push_back(intParams);
        _args.push_back(floatParams);
        auto _ret = this->_client->call("sim.createVisionSensor", _args);
        return _ret[0].as<int64_t>();
    }

    void sim::destroyCollection(int64_t collectionHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(collectionHandle);
        auto _ret = this->_client->call("sim.destroyCollection", _args);
    }

    void sim::destroyGraphCurve(int64_t graphHandle, int64_t curveId)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(graphHandle);
        _args.push_back(curveId);
        auto _ret = this->_client->call("sim.destroyGraphCurve", _args);
    }

    int64_t sim::duplicateGraphCurveToStatic(int64_t graphHandle, int64_t curveId, std::optional<std::string> curveName)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(graphHandle);
        _args.push_back(curveId);
        if(curveName)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*curveName);
        }
        else _brk = true;
        auto _ret = this->_client->call("sim.duplicateGraphCurveToStatic", _args);
        return _ret[0].as<int64_t>();
    }

    std::tuple<int64_t, json> sim::executeScriptString(std::string stringAtScriptName, int64_t scriptHandleOrType)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(stringAtScriptName);
        _args.push_back(scriptHandleOrType);
        auto _ret = this->_client->call("sim.executeScriptString", _args);
        return std::make_tuple(_ret[0].as<int64_t>(), _ret[1].as<json>());
    }

    void sim::exportMesh(int64_t fileformat, std::string pathAndFilename, int64_t options, double scalingFactor, std::vector<double> vertices, std::vector<int64_t> indices)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(fileformat);
        _args.push_back(pathAndFilename);
        _args.push_back(options);
        _args.push_back(scalingFactor);
        _args.push_back(vertices);
        _args.push_back(indices);
        auto _ret = this->_client->call("sim.exportMesh", _args);
    }

    int64_t sim::floatingViewAdd(double posX, double posY, double sizeX, double sizeY, int64_t options)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(posX);
        _args.push_back(posY);
        _args.push_back(sizeX);
        _args.push_back(sizeY);
        _args.push_back(options);
        auto _ret = this->_client->call("sim.floatingViewAdd", _args);
        return _ret[0].as<int64_t>();
    }

    int64_t sim::floatingViewRemove(int64_t floatingViewHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(floatingViewHandle);
        auto _ret = this->_client->call("sim.floatingViewRemove", _args);
        return _ret[0].as<int64_t>();
    }

    int64_t sim::generateShapeFromPath(std::vector<double> path, std::vector<double> section, std::optional<int64_t> options, std::optional<std::vector<double>> upVector)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(path);
        _args.push_back(section);
        if(options)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*options);
        }
        else _brk = true;
        if(upVector)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*upVector);
        }
        else _brk = true;
        auto _ret = this->_client->call("sim.generateShapeFromPath", _args);
        return _ret[0].as<int64_t>();
    }

    int64_t sim::generateTextShape(std::string txt, std::optional<std::vector<double>> color, std::optional<double> height, std::optional<bool> centered, std::optional<std::string> alphabetLocation)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(txt);
        if(color)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*color);
        }
        else _brk = true;
        if(height)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*height);
        }
        else _brk = true;
        if(centered)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*centered);
        }
        else _brk = true;
        if(alphabetLocation)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*alphabetLocation);
        }
        else _brk = true;
        auto _ret = this->_client->call("sim.generateTextShape", _args);
        return _ret[0].as<int64_t>();
    }

    std::tuple<std::vector<double>, std::vector<double>> sim::generateTimeOptimalTrajectory(std::vector<double> path, std::vector<double> pathLengths, std::vector<double> minMaxVel, std::vector<double> minMaxAccel, std::optional<int64_t> trajPtSamples, std::optional<std::string> boundaryCondition, std::optional<double> timeout)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(path);
        _args.push_back(pathLengths);
        _args.push_back(minMaxVel);
        _args.push_back(minMaxAccel);
        if(trajPtSamples)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*trajPtSamples);
        }
        else _brk = true;
        if(boundaryCondition)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*boundaryCondition);
        }
        else _brk = true;
        if(timeout)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*timeout);
        }
        else _brk = true;
        auto _ret = this->_client->call("sim.generateTimeOptimalTrajectory", _args);
        return std::make_tuple(_ret[0].as<std::vector<double>>(), _ret[1].as<std::vector<double>>());
    }

    std::vector<double> sim::getAlternateConfigs(std::vector<int64_t> jointHandles, std::vector<double> inputConfig, std::optional<int64_t> tipHandle, std::optional<std::vector<double>> lowLimits, std::optional<std::vector<double>> ranges)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(jointHandles);
        _args.push_back(inputConfig);
        if(tipHandle)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*tipHandle);
        }
        else _brk = true;
        if(lowLimits)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*lowLimits);
        }
        else _brk = true;
        if(ranges)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*ranges);
        }
        else _brk = true;
        auto _ret = this->_client->call("sim.getAlternateConfigs", _args);
        return _ret[0].as<std::vector<double>>();
    }

    std::vector<std::string> sim::getApiFunc(int64_t scriptHandleOrType, std::string apiWord)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(scriptHandleOrType);
        _args.push_back(apiWord);
        auto _ret = this->_client->call("sim.getApiFunc", _args);
        return _ret[0].as<std::vector<std::string>>();
    }

    std::string sim::getApiInfo(int64_t scriptHandleOrType, std::string apiWord)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(scriptHandleOrType);
        _args.push_back(apiWord);
        auto _ret = this->_client->call("sim.getApiInfo", _args);
        return _ret[0].as<std::string>();
    }

    std::vector<double> sim::getArrayParam(int64_t parameter)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(parameter);
        auto _ret = this->_client->call("sim.getArrayParam", _args);
        return _ret[0].as<std::vector<double>>();
    }

    bool sim::getBoolParam(int64_t parameter)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(parameter);
        auto _ret = this->_client->call("sim.getBoolParam", _args);
        return _ret[0].as<bool>();
    }

    double sim::getClosestPosOnPath(std::vector<double> path, std::vector<double> pathLengths, std::vector<double> absPt)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(path);
        _args.push_back(pathLengths);
        _args.push_back(absPt);
        auto _ret = this->_client->call("sim.getClosestPosOnPath", _args);
        return _ret[0].as<double>();
    }

    std::vector<int64_t> sim::getCollectionObjects(int64_t collectionHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(collectionHandle);
        auto _ret = this->_client->call("sim.getCollectionObjects", _args);
        return _ret[0].as<std::vector<int64_t>>();
    }

    double sim::getConfigDistance(std::vector<double> configA, std::vector<double> configB, std::optional<std::vector<double>> metric, std::optional<std::vector<int64_t>> types)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(configA);
        _args.push_back(configB);
        if(metric)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*metric);
        }
        else _brk = true;
        if(types)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*types);
        }
        else _brk = true;
        auto _ret = this->_client->call("sim.getConfigDistance", _args);
        return _ret[0].as<double>();
    }

    std::tuple<std::vector<int64_t>, std::vector<double>, std::vector<double>, std::vector<double>> sim::getContactInfo(int64_t dynamicPass, int64_t objectHandle, int64_t index)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(dynamicPass);
        _args.push_back(objectHandle);
        _args.push_back(index);
        auto _ret = this->_client->call("sim.getContactInfo", _args);
        return std::make_tuple(_ret[0].as<std::vector<int64_t>>(), _ret[1].as<std::vector<double>>(), _ret[2].as<std::vector<double>>(), _ret[3].as<std::vector<double>>());
    }

    std::tuple<std::vector<double>, std::vector<int64_t>> sim::getDecimatedMesh(std::vector<double> verticesIn, std::vector<int64_t> indicesIn, double decimationPercentage)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(verticesIn);
        _args.push_back(indicesIn);
        _args.push_back(decimationPercentage);
        auto _ret = this->_client->call("sim.getDecimatedMesh", _args);
        return std::make_tuple(_ret[0].as<std::vector<double>>(), _ret[1].as<std::vector<int64_t>>());
    }

    double sim::getDoubleSignal(std::string signalName)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(signalName);
        auto _ret = this->_client->call("sim.getDoubleSignal", _args);
        return _ret[0].as<double>();
    }

    bool sim::getEngineBoolParam(int64_t paramId, int64_t objectHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(paramId);
        _args.push_back(objectHandle);
        auto _ret = this->_client->call("sim.getEngineBoolParam", _args);
        return _ret[0].as<bool>();
    }

    double sim::getEngineFloatParam(int64_t paramId, int64_t objectHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(paramId);
        _args.push_back(objectHandle);
        auto _ret = this->_client->call("sim.getEngineFloatParam", _args);
        return _ret[0].as<double>();
    }

    int64_t sim::getEngineInt32Param(int64_t paramId, int64_t objectHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(paramId);
        _args.push_back(objectHandle);
        auto _ret = this->_client->call("sim.getEngineInt32Param", _args);
        return _ret[0].as<int64_t>();
    }

    std::vector<double> sim::getEulerAnglesFromMatrix(std::vector<double> matrix)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(matrix);
        auto _ret = this->_client->call("sim.getEulerAnglesFromMatrix", _args);
        return _ret[0].as<std::vector<double>>();
    }

    int64_t sim::getExplicitHandling(int64_t objectHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(objectHandle);
        auto _ret = this->_client->call("sim.getExplicitHandling", _args);
        return _ret[0].as<int64_t>();
    }

    std::string sim::getExtensionString(int64_t objectHandle, int64_t index, std::optional<std::string> key)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(objectHandle);
        _args.push_back(index);
        if(key)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*key);
        }
        else _brk = true;
        auto _ret = this->_client->call("sim.getExtensionString", _args);
        return _ret[0].as<std::string>();
    }

    double sim::getFloatParam(int64_t parameter)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(parameter);
        auto _ret = this->_client->call("sim.getFloatParam", _args);
        return _ret[0].as<double>();
    }

    double sim::getFloatSignal(std::string signalName)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(signalName);
        auto _ret = this->_client->call("sim.getFloatSignal", _args);
        return _ret[0].as<double>();
    }

    std::vector<json> sim::getGenesisEvents()
    {
        bool _brk = false;
        json _args(json_array_arg);
        auto _ret = this->_client->call("sim.getGenesisEvents", _args);
        return _ret[0].as<std::vector<json>>();
    }

    std::tuple<std::string, int64_t, std::vector<double>, std::vector<double>, std::vector<double>, std::vector<double>, int64_t, int64_t> sim::getGraphCurve(int64_t graphHandle, int64_t graphType, int64_t curveIndex)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(graphHandle);
        _args.push_back(graphType);
        _args.push_back(curveIndex);
        auto _ret = this->_client->call("sim.getGraphCurve", _args);
        return std::make_tuple(_ret[0].as<std::string>(), _ret[1].as<int64_t>(), _ret[2].as<std::vector<double>>(), _ret[3].as<std::vector<double>>(), _ret[4].as<std::vector<double>>(), _ret[5].as<std::vector<double>>(), _ret[6].as<int64_t>(), _ret[7].as<int64_t>());
    }

    std::tuple<int64_t, std::vector<double>, std::vector<double>, int64_t> sim::getGraphInfo(int64_t graphHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(graphHandle);
        auto _ret = this->_client->call("sim.getGraphInfo", _args);
        return std::make_tuple(_ret[0].as<int64_t>(), _ret[1].as<std::vector<double>>(), _ret[2].as<std::vector<double>>(), _ret[3].as<int64_t>());
    }

    int64_t sim::getInt32Param(int64_t parameter)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(parameter);
        auto _ret = this->_client->call("sim.getInt32Param", _args);
        return _ret[0].as<int64_t>();
    }

    int64_t sim::getInt32Signal(std::string signalName)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(signalName);
        auto _ret = this->_client->call("sim.getInt32Signal", _args);
        return _ret[0].as<int64_t>();
    }

    std::tuple<int64_t, double, double> sim::getJointDependency(int64_t jointHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(jointHandle);
        auto _ret = this->_client->call("sim.getJointDependency", _args);
        return std::make_tuple(_ret[0].as<int64_t>(), _ret[1].as<double>(), _ret[2].as<double>());
    }

    double sim::getJointForce(int64_t jointHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(jointHandle);
        auto _ret = this->_client->call("sim.getJointForce", _args);
        return _ret[0].as<double>();
    }

    std::tuple<bool, std::vector<double>> sim::getJointInterval(int64_t objectHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(objectHandle);
        auto _ret = this->_client->call("sim.getJointInterval", _args);
        return std::make_tuple(_ret[0].as<bool>(), _ret[1].as<std::vector<double>>());
    }

    std::tuple<int64_t, int64_t> sim::getJointMode(int64_t jointHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(jointHandle);
        auto _ret = this->_client->call("sim.getJointMode", _args);
        return std::make_tuple(_ret[0].as<int64_t>(), _ret[1].as<int64_t>());
    }

    double sim::getJointPosition(int64_t objectHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(objectHandle);
        auto _ret = this->_client->call("sim.getJointPosition", _args);
        return _ret[0].as<double>();
    }

    double sim::getJointTargetForce(int64_t jointHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(jointHandle);
        auto _ret = this->_client->call("sim.getJointTargetForce", _args);
        return _ret[0].as<double>();
    }

    double sim::getJointTargetPosition(int64_t objectHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(objectHandle);
        auto _ret = this->_client->call("sim.getJointTargetPosition", _args);
        return _ret[0].as<double>();
    }

    double sim::getJointTargetVelocity(int64_t objectHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(objectHandle);
        auto _ret = this->_client->call("sim.getJointTargetVelocity", _args);
        return _ret[0].as<double>();
    }

    int64_t sim::getJointType(int64_t objectHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(objectHandle);
        auto _ret = this->_client->call("sim.getJointType", _args);
        return _ret[0].as<int64_t>();
    }

    double sim::getJointVelocity(int64_t jointHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(jointHandle);
        auto _ret = this->_client->call("sim.getJointVelocity", _args);
        return _ret[0].as<double>();
    }

    std::tuple<int64_t, std::vector<double>, std::vector<double>, std::vector<double>> sim::getLightParameters(int64_t lightHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(lightHandle);
        auto _ret = this->_client->call("sim.getLightParameters", _args);
        return std::make_tuple(_ret[0].as<int64_t>(), _ret[1].as<std::vector<double>>(), _ret[2].as<std::vector<double>>(), _ret[3].as<std::vector<double>>());
    }

    int64_t sim::getLinkDummy(int64_t dummyHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(dummyHandle);
        auto _ret = this->_client->call("sim.getLinkDummy", _args);
        return _ret[0].as<int64_t>();
    }

    std::vector<std::string> sim::getMatchingPersistentDataTags(std::string pattern)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(pattern);
        auto _ret = this->_client->call("sim.getMatchingPersistentDataTags", _args);
        return _ret[0].as<std::vector<std::string>>();
    }

    int64_t sim::getModelProperty(int64_t objectHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(objectHandle);
        auto _ret = this->_client->call("sim.getModelProperty", _args);
        return _ret[0].as<int64_t>();
    }

    std::string sim::getModuleInfo(std::string moduleName, int64_t infoType)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(moduleName);
        _args.push_back(infoType);
        auto _ret = this->_client->call("sim.getModuleInfo", _args);
        return _ret[0].as<std::string>();
    }

    std::tuple<std::string, int64_t> sim::getModuleName(int64_t index)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(index);
        auto _ret = this->_client->call("sim.getModuleName", _args);
        return std::make_tuple(_ret[0].as<std::string>(), _ret[1].as<int64_t>());
    }

    std::vector<uint8_t> sim::getNamedStringParam(std::string paramName)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(paramName);
        auto _ret = this->_client->call("sim.getNamedStringParam", _args);
        return _ret[0].as<std::vector<uint8_t>>();
    }

    int64_t sim::getNavigationMode()
    {
        bool _brk = false;
        json _args(json_array_arg);
        auto _ret = this->_client->call("sim.getNavigationMode", _args);
        return _ret[0].as<int64_t>();
    }

    int64_t sim::getObject(std::string path, std::optional<json> options)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(path);
        if(options)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*options);
        }
        else _brk = true;
        auto _ret = this->_client->call("sim.getObject", _args);
        return _ret[0].as<int64_t>();
    }

    std::string sim::getObjectAlias(int64_t objectHandle, std::optional<int64_t> options)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(objectHandle);
        if(options)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*options);
        }
        else _brk = true;
        auto _ret = this->_client->call("sim.getObjectAlias", _args);
        return _ret[0].as<std::string>();
    }

    int64_t sim::getObjectChild(int64_t objectHandle, int64_t index)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(objectHandle);
        _args.push_back(index);
        auto _ret = this->_client->call("sim.getObjectChild", _args);
        return _ret[0].as<int64_t>();
    }

    std::vector<double> sim::getObjectChildPose(int64_t objectHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(objectHandle);
        auto _ret = this->_client->call("sim.getObjectChildPose", _args);
        return _ret[0].as<std::vector<double>>();
    }

    std::vector<double> sim::getObjectColor(int64_t objectHandle, int64_t index, int64_t colorComponent)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(objectHandle);
        _args.push_back(index);
        _args.push_back(colorComponent);
        auto _ret = this->_client->call("sim.getObjectColor", _args);
        return _ret[0].as<std::vector<double>>();
    }

    double sim::getObjectFloatParam(int64_t objectHandle, int64_t parameterID)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(objectHandle);
        _args.push_back(parameterID);
        auto _ret = this->_client->call("sim.getObjectFloatParam", _args);
        return _ret[0].as<double>();
    }

    void sim::getObjectFromUid(int64_t uid, std::optional<json> options)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(uid);
        if(options)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*options);
        }
        else _brk = true;
        auto _ret = this->_client->call("sim.getObjectFromUid", _args);
    }

    int64_t sim::getObjectInt32Param(int64_t objectHandle, int64_t parameterID)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(objectHandle);
        _args.push_back(parameterID);
        auto _ret = this->_client->call("sim.getObjectInt32Param", _args);
        return _ret[0].as<int64_t>();
    }

    std::vector<double> sim::getObjectMatrix(int64_t objectHandle, int64_t relativeToObjectHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(objectHandle);
        _args.push_back(relativeToObjectHandle);
        auto _ret = this->_client->call("sim.getObjectMatrix", _args);
        return _ret[0].as<std::vector<double>>();
    }

    std::vector<double> sim::getObjectOrientation(int64_t objectHandle, int64_t relativeToObjectHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(objectHandle);
        _args.push_back(relativeToObjectHandle);
        auto _ret = this->_client->call("sim.getObjectOrientation", _args);
        return _ret[0].as<std::vector<double>>();
    }

    int64_t sim::getObjectParent(int64_t objectHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(objectHandle);
        auto _ret = this->_client->call("sim.getObjectParent", _args);
        return _ret[0].as<int64_t>();
    }

    std::vector<double> sim::getObjectPose(int64_t objectHandle, int64_t relativeToObjectHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(objectHandle);
        _args.push_back(relativeToObjectHandle);
        auto _ret = this->_client->call("sim.getObjectPose", _args);
        return _ret[0].as<std::vector<double>>();
    }

    std::vector<double> sim::getObjectPosition(int64_t objectHandle, int64_t relativeToObjectHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(objectHandle);
        _args.push_back(relativeToObjectHandle);
        auto _ret = this->_client->call("sim.getObjectPosition", _args);
        return _ret[0].as<std::vector<double>>();
    }

    int64_t sim::getObjectProperty(int64_t objectHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(objectHandle);
        auto _ret = this->_client->call("sim.getObjectProperty", _args);
        return _ret[0].as<int64_t>();
    }

    std::vector<double> sim::getObjectQuaternion(int64_t objectHandle, int64_t relativeToObjectHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(objectHandle);
        _args.push_back(relativeToObjectHandle);
        auto _ret = this->_client->call("sim.getObjectQuaternion", _args);
        return _ret[0].as<std::vector<double>>();
    }

    std::vector<int64_t> sim::getObjectSelection()
    {
        bool _brk = false;
        json _args(json_array_arg);
        auto _ret = this->_client->call("sim.getObjectSelection", _args);
        return _ret[0].as<std::vector<int64_t>>();
    }

    double sim::getObjectSizeFactor(int64_t ObjectHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(ObjectHandle);
        auto _ret = this->_client->call("sim.getObjectSizeFactor", _args);
        return _ret[0].as<double>();
    }

    int64_t sim::getObjectSpecialProperty(int64_t objectHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(objectHandle);
        auto _ret = this->_client->call("sim.getObjectSpecialProperty", _args);
        return _ret[0].as<int64_t>();
    }

    std::vector<uint8_t> sim::getObjectStringParam(int64_t objectHandle, int64_t parameterID)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(objectHandle);
        _args.push_back(parameterID);
        auto _ret = this->_client->call("sim.getObjectStringParam", _args);
        return _ret[0].as<std::vector<uint8_t>>();
    }

    int64_t sim::getObjectType(int64_t objectHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(objectHandle);
        auto _ret = this->_client->call("sim.getObjectType", _args);
        return _ret[0].as<int64_t>();
    }

    int64_t sim::getObjectUid(int64_t objectHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(objectHandle);
        auto _ret = this->_client->call("sim.getObjectUid", _args);
        return _ret[0].as<int64_t>();
    }

    std::tuple<std::vector<double>, std::vector<double>> sim::getObjectVelocity(int64_t objectHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(objectHandle);
        auto _ret = this->_client->call("sim.getObjectVelocity", _args);
        return std::make_tuple(_ret[0].as<std::vector<double>>(), _ret[1].as<std::vector<double>>());
    }

    int64_t sim::getObjects(int64_t index, int64_t objectType)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(index);
        _args.push_back(objectType);
        auto _ret = this->_client->call("sim.getObjects", _args);
        return _ret[0].as<int64_t>();
    }

    std::vector<int64_t> sim::getObjectsInTree(int64_t treeBaseHandle, std::optional<int64_t> objectType, std::optional<int64_t> options)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(treeBaseHandle);
        if(objectType)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*objectType);
        }
        else _brk = true;
        if(options)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*options);
        }
        else _brk = true;
        auto _ret = this->_client->call("sim.getObjectsInTree", _args);
        return _ret[0].as<std::vector<int64_t>>();
    }

    std::vector<double> sim::getOctreeVoxels(int64_t octreeHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(octreeHandle);
        auto _ret = this->_client->call("sim.getOctreeVoxels", _args);
        return _ret[0].as<std::vector<double>>();
    }

    int64_t sim::getPage()
    {
        bool _brk = false;
        json _args(json_array_arg);
        auto _ret = this->_client->call("sim.getPage", _args);
        return _ret[0].as<int64_t>();
    }

    std::vector<double> sim::getPathInterpolatedConfig(std::vector<double> path, std::vector<double> pathLengths, double t, std::optional<json> method, std::optional<std::vector<int64_t>> types)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(path);
        _args.push_back(pathLengths);
        _args.push_back(t);
        if(method)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*method);
        }
        else _brk = true;
        if(types)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*types);
        }
        else _brk = true;
        auto _ret = this->_client->call("sim.getPathInterpolatedConfig", _args);
        return _ret[0].as<std::vector<double>>();
    }

    std::tuple<std::vector<double>, double> sim::getPathLengths(std::vector<double> path, int64_t dof, std::optional<std::string> distCallback)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(path);
        _args.push_back(dof);
        if(distCallback)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*distCallback);
        }
        else _brk = true;
        auto _ret = this->_client->call("sim.getPathLengths", _args);
        return std::make_tuple(_ret[0].as<std::vector<double>>(), _ret[1].as<double>());
    }

    std::vector<std::string> sim::getPersistentDataTags()
    {
        bool _brk = false;
        json _args(json_array_arg);
        auto _ret = this->_client->call("sim.getPersistentDataTags", _args);
        return _ret[0].as<std::vector<std::string>>();
    }

    std::tuple<double, int64_t, int64_t, double> sim::getPointCloudOptions(int64_t pointCloudHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(pointCloudHandle);
        auto _ret = this->_client->call("sim.getPointCloudOptions", _args);
        return std::make_tuple(_ret[0].as<double>(), _ret[1].as<int64_t>(), _ret[2].as<int64_t>(), _ret[3].as<double>());
    }

    std::vector<double> sim::getPointCloudPoints(int64_t pointCloudHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(pointCloudHandle);
        auto _ret = this->_client->call("sim.getPointCloudPoints", _args);
        return _ret[0].as<std::vector<double>>();
    }

    std::tuple<std::vector<double>, std::vector<int64_t>> sim::getQHull(std::vector<double> verticesIn)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(verticesIn);
        auto _ret = this->_client->call("sim.getQHull", _args);
        return std::make_tuple(_ret[0].as<std::vector<double>>(), _ret[1].as<std::vector<int64_t>>());
    }

    std::vector<double> sim::getQuaternionFromMatrix(std::vector<double> matrix)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(matrix);
        auto _ret = this->_client->call("sim.getQuaternionFromMatrix", _args);
        return _ret[0].as<std::vector<double>>();
    }

    double sim::getRandom(std::optional<int64_t> seed)
    {
        bool _brk = false;
        json _args(json_array_arg);
        if(seed)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*seed);
        }
        else _brk = true;
        auto _ret = this->_client->call("sim.getRandom", _args);
        return _ret[0].as<double>();
    }

    int64_t sim::getRealTimeSimulation()
    {
        bool _brk = false;
        json _args(json_array_arg);
        auto _ret = this->_client->call("sim.getRealTimeSimulation", _args);
        return _ret[0].as<int64_t>();
    }

    std::vector<int64_t> sim::getReferencedHandles(int64_t objectHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(objectHandle);
        auto _ret = this->_client->call("sim.getReferencedHandles", _args);
        return _ret[0].as<std::vector<int64_t>>();
    }

    std::tuple<std::vector<double>, double> sim::getRotationAxis(std::vector<double> matrixStart, std::vector<double> matrixGoal)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(matrixStart);
        _args.push_back(matrixGoal);
        auto _ret = this->_client->call("sim.getRotationAxis", _args);
        return std::make_tuple(_ret[0].as<std::vector<double>>(), _ret[1].as<double>());
    }

    std::tuple<std::vector<uint8_t>, std::vector<int64_t>> sim::getScaledImage(std::vector<uint8_t> imageIn, std::vector<int64_t> resolutionIn, std::vector<int64_t> desiredResolutionOut, int64_t options)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(bin(imageIn));
        _args.push_back(resolutionIn);
        _args.push_back(desiredResolutionOut);
        _args.push_back(options);
        auto _ret = this->_client->call("sim.getScaledImage", _args);
        return std::make_tuple(_ret[0].as<std::vector<uint8_t>>(), _ret[1].as<std::vector<int64_t>>());
    }

    int64_t sim::getScript(int64_t scriptType, std::optional<int64_t> objectHandle, std::optional<std::string> scriptName)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(scriptType);
        if(objectHandle)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*objectHandle);
        }
        else _brk = true;
        if(scriptName)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*scriptName);
        }
        else _brk = true;
        auto _ret = this->_client->call("sim.getScript", _args);
        return _ret[0].as<int64_t>();
    }

    int64_t sim::getScriptInt32Param(int64_t scriptHandle, int64_t parameterID)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(scriptHandle);
        _args.push_back(parameterID);
        auto _ret = this->_client->call("sim.getScriptInt32Param", _args);
        return _ret[0].as<int64_t>();
    }

    std::vector<uint8_t> sim::getScriptStringParam(int64_t scriptHandle, int64_t parameterID)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(scriptHandle);
        _args.push_back(parameterID);
        auto _ret = this->_client->call("sim.getScriptStringParam", _args);
        return _ret[0].as<std::vector<uint8_t>>();
    }

    std::vector<double> sim::getShapeBB(int64_t shapeHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(shapeHandle);
        auto _ret = this->_client->call("sim.getShapeBB", _args);
        return _ret[0].as<std::vector<double>>();
    }

    std::tuple<int64_t, std::vector<double>> sim::getShapeColor(int64_t shapeHandle, std::string colorName, int64_t colorComponent)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(shapeHandle);
        _args.push_back(colorName);
        _args.push_back(colorComponent);
        auto _ret = this->_client->call("sim.getShapeColor", _args);
        return std::make_tuple(_ret[0].as<int64_t>(), _ret[1].as<std::vector<double>>());
    }

    std::tuple<int64_t, int64_t, std::vector<double>> sim::getShapeGeomInfo(int64_t shapeHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(shapeHandle);
        auto _ret = this->_client->call("sim.getShapeGeomInfo", _args);
        return std::make_tuple(_ret[0].as<int64_t>(), _ret[1].as<int64_t>(), _ret[2].as<std::vector<double>>());
    }

    std::tuple<std::vector<double>, std::vector<double>> sim::getShapeInertia(int64_t shapeHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(shapeHandle);
        auto _ret = this->_client->call("sim.getShapeInertia", _args);
        return std::make_tuple(_ret[0].as<std::vector<double>>(), _ret[1].as<std::vector<double>>());
    }

    double sim::getShapeMass(int64_t shapeHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(shapeHandle);
        auto _ret = this->_client->call("sim.getShapeMassAndInertia", _args);
        return _ret[0].as<double>();
    }

    std::tuple<std::vector<double>, std::vector<int64_t>, std::vector<double>> sim::getShapeMesh(int64_t shapeHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(shapeHandle);
        auto _ret = this->_client->call("sim.getShapeMesh", _args);
        return std::make_tuple(_ret[0].as<std::vector<double>>(), _ret[1].as<std::vector<int64_t>>(), _ret[2].as<std::vector<double>>());
    }

    int64_t sim::getShapeTextureId(int64_t shapeHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(shapeHandle);
        auto _ret = this->_client->call("sim.getShapeTextureId", _args);
        return _ret[0].as<int64_t>();
    }

    json sim::getShapeViz(int64_t shapeHandle, int64_t itemIndex)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(shapeHandle);
        _args.push_back(itemIndex);
        auto _ret = this->_client->call("sim.getShapeViz", _args);
        return _ret[0].as<json>();
    }

    std::string sim::getSignalName(int64_t signalIndex, int64_t signalType)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(signalIndex);
        _args.push_back(signalType);
        auto _ret = this->_client->call("sim.getSignalName", _args);
        return _ret[0].as<std::string>();
    }

    int64_t sim::getSimulationState()
    {
        bool _brk = false;
        json _args(json_array_arg);
        auto _ret = this->_client->call("sim.getSimulationState", _args);
        return _ret[0].as<int64_t>();
    }

    double sim::getSimulationTime()
    {
        bool _brk = false;
        json _args(json_array_arg);
        auto _ret = this->_client->call("sim.getSimulationTime", _args);
        return _ret[0].as<double>();
    }

    double sim::getSimulationTimeStep()
    {
        bool _brk = false;
        json _args(json_array_arg);
        auto _ret = this->_client->call("sim.getSimulationTimeStep", _args);
        return _ret[0].as<double>();
    }

    std::tuple<int64_t, std::vector<int64_t>, std::vector<int64_t>> sim::getSimulatorMessage()
    {
        bool _brk = false;
        json _args(json_array_arg);
        auto _ret = this->_client->call("sim.getSimulatorMessage", _args);
        return std::make_tuple(_ret[0].as<int64_t>(), _ret[1].as<std::vector<int64_t>>(), _ret[2].as<std::vector<int64_t>>());
    }

    std::string sim::getStackTraceback(std::optional<int64_t> scriptHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        if(scriptHandle)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*scriptHandle);
        }
        else _brk = true;
        auto _ret = this->_client->call("sim.getStackTraceback", _args);
        return _ret[0].as<std::string>();
    }

    std::string sim::getStringParam(int64_t parameter)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(parameter);
        auto _ret = this->_client->call("sim.getStringParam", _args);
        return _ret[0].as<std::string>();
    }

    std::vector<uint8_t> sim::getStringSignal(std::string signalName)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(signalName);
        auto _ret = this->_client->call("sim.getStringSignal", _args);
        return _ret[0].as<std::vector<uint8_t>>();
    }

    double sim::getSystemTime()
    {
        bool _brk = false;
        json _args(json_array_arg);
        auto _ret = this->_client->call("sim.getSystemTime", _args);
        return _ret[0].as<double>();
    }

    std::tuple<int64_t, std::vector<int64_t>> sim::getTextureId(std::string textureName)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(textureName);
        auto _ret = this->_client->call("sim.getTextureId", _args);
        return std::make_tuple(_ret[0].as<int64_t>(), _ret[1].as<std::vector<int64_t>>());
    }

    bool sim::getThreadAutomaticSwitch()
    {
        bool _brk = false;
        json _args(json_array_arg);
        auto _ret = this->_client->call("sim.getThreadAutomaticSwitch", _args);
        return _ret[0].as<bool>();
    }

    bool sim::getThreadExistRequest()
    {
        bool _brk = false;
        json _args(json_array_arg);
        auto _ret = this->_client->call("sim.getThreadExistRequest", _args);
        return _ret[0].as<bool>();
    }

    int64_t sim::getThreadId()
    {
        bool _brk = false;
        json _args(json_array_arg);
        auto _ret = this->_client->call("sim.getThreadId", _args);
        return _ret[0].as<int64_t>();
    }

    bool sim::getThreadSwitchAllowed()
    {
        bool _brk = false;
        json _args(json_array_arg);
        auto _ret = this->_client->call("sim.getThreadSwitchAllowed", _args);
        return _ret[0].as<bool>();
    }

    int64_t sim::getThreadSwitchTiming()
    {
        bool _brk = false;
        json _args(json_array_arg);
        auto _ret = this->_client->call("sim.getThreadSwitchTiming", _args);
        return _ret[0].as<int64_t>();
    }

    std::vector<std::string> sim::getUserVariables()
    {
        bool _brk = false;
        json _args(json_array_arg);
        auto _ret = this->_client->call("sim.getUserVariables", _args);
        return _ret[0].as<std::vector<std::string>>();
    }

    std::tuple<std::vector<double>, std::vector<double>> sim::getVelocity(int64_t shapeHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(shapeHandle);
        auto _ret = this->_client->call("sim.getVelocity", _args);
        return std::make_tuple(_ret[0].as<std::vector<double>>(), _ret[1].as<std::vector<double>>());
    }

    std::tuple<std::vector<uint8_t>, std::vector<int64_t>> sim::getVisionSensorDepth(int64_t sensorHandle, std::optional<int64_t> options, std::optional<std::vector<int64_t>> pos, std::optional<std::vector<int64_t>> size)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(sensorHandle);
        if(options)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*options);
        }
        else _brk = true;
        if(pos)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*pos);
        }
        else _brk = true;
        if(size)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*size);
        }
        else _brk = true;
        auto _ret = this->_client->call("sim.getVisionSensorDepth", _args);
        return std::make_tuple(_ret[0].as<std::vector<uint8_t>>(), _ret[1].as<std::vector<int64_t>>());
    }

    std::tuple<std::vector<uint8_t>, std::vector<int64_t>> sim::getVisionSensorImg(int64_t sensorHandle, std::optional<int64_t> options, std::optional<double> rgbaCutOff, std::optional<std::vector<int64_t>> pos, std::optional<std::vector<int64_t>> size)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(sensorHandle);
        if(options)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*options);
        }
        else _brk = true;
        if(rgbaCutOff)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*rgbaCutOff);
        }
        else _brk = true;
        if(pos)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*pos);
        }
        else _brk = true;
        if(size)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*size);
        }
        else _brk = true;
        auto _ret = this->_client->call("sim.getVisionSensorImg", _args);
        return std::make_tuple(_ret[0].as<std::vector<uint8_t>>(), _ret[1].as<std::vector<int64_t>>());
    }

    int64_t sim::groupShapes(std::vector<int64_t> shapeHandles, std::optional<bool> merge)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(shapeHandles);
        if(merge)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*merge);
        }
        else _brk = true;
        auto _ret = this->_client->call("sim.groupShapes", _args);
        return _ret[0].as<int64_t>();
    }

    int64_t sim::handleAddOnScripts(int64_t callType)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(callType);
        auto _ret = this->_client->call("sim.handleAddOnScripts", _args);
        return _ret[0].as<int64_t>();
    }

    int64_t sim::handleChildScripts(int64_t callType)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(callType);
        auto _ret = this->_client->call("sim.handleChildScripts", _args);
        return _ret[0].as<int64_t>();
    }

    int64_t sim::handleCustomizationScripts(int64_t callType)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(callType);
        auto _ret = this->_client->call("sim.handleCustomizationScripts", _args);
        return _ret[0].as<int64_t>();
    }

    int64_t sim::handleDynamics(double deltaTime)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(deltaTime);
        auto _ret = this->_client->call("sim.handleDynamics", _args);
        return _ret[0].as<int64_t>();
    }

    void sim::handleGraph(int64_t objectHandle, double simulationTime)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(objectHandle);
        _args.push_back(simulationTime);
        auto _ret = this->_client->call("sim.handleGraph", _args);
    }

    void sim::handleJointMotion()
    {
        bool _brk = false;
        json _args(json_array_arg);
        auto _ret = this->_client->call("sim.handleJointMotion", _args);
    }

    std::tuple<int64_t, double, std::vector<double>, int64_t, std::vector<double>> sim::handleProximitySensor(int64_t sensorHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(sensorHandle);
        auto _ret = this->_client->call("sim.handleProximitySensor", _args);
        return std::make_tuple(_ret[0].as<int64_t>(), _ret[1].as<double>(), _ret[2].as<std::vector<double>>(), _ret[3].as<int64_t>(), _ret[4].as<std::vector<double>>());
    }

    void sim::handleSandboxScript(int64_t callType)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(callType);
        auto _ret = this->_client->call("sim.handleSandboxScript", _args);
    }

    void sim::handleSensingStart()
    {
        bool _brk = false;
        json _args(json_array_arg);
        auto _ret = this->_client->call("sim.handleSensingStart", _args);
    }

    void sim::handleSimulationStart()
    {
        bool _brk = false;
        json _args(json_array_arg);
        auto _ret = this->_client->call("sim.handleSimulationStart", _args);
    }

    std::tuple<int64_t, std::vector<double>, std::vector<double>> sim::handleVisionSensor(int64_t sensorHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(sensorHandle);
        auto _ret = this->_client->call("sim.handleVisionSensor", _args);
        return std::make_tuple(_ret[0].as<int64_t>(), _ret[1].as<std::vector<double>>(), _ret[2].as<std::vector<double>>());
    }

    std::tuple<std::vector<double>, std::vector<int64_t>> sim::importMesh(int64_t fileformat, std::string pathAndFilename, int64_t options, double identicalVerticeTolerance, double scalingFactor)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(fileformat);
        _args.push_back(pathAndFilename);
        _args.push_back(options);
        _args.push_back(identicalVerticeTolerance);
        _args.push_back(scalingFactor);
        auto _ret = this->_client->call("sim.importMesh", _args);
        return std::make_tuple(_ret[0].as<std::vector<double>>(), _ret[1].as<std::vector<int64_t>>());
    }

    int64_t sim::importShape(int64_t fileformat, std::string pathAndFilename, int64_t options, double identicalVerticeTolerance, double scalingFactor)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(fileformat);
        _args.push_back(pathAndFilename);
        _args.push_back(options);
        _args.push_back(identicalVerticeTolerance);
        _args.push_back(scalingFactor);
        auto _ret = this->_client->call("sim.importShape", _args);
        return _ret[0].as<int64_t>();
    }

    bool sim::initScript(int64_t scriptHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(scriptHandle);
        auto _ret = this->_client->call("sim.initScript", _args);
        return _ret[0].as<bool>();
    }

    int64_t sim::insertObjectIntoOctree(int64_t octreeHandle, int64_t objectHandle, int64_t options, std::optional<std::vector<double>> color, std::optional<int64_t> tag)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(octreeHandle);
        _args.push_back(objectHandle);
        _args.push_back(options);
        if(color)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*color);
        }
        else _brk = true;
        if(tag)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*tag);
        }
        else _brk = true;
        auto _ret = this->_client->call("sim.insertObjectIntoOctree", _args);
        return _ret[0].as<int64_t>();
    }

    int64_t sim::insertObjectIntoPointCloud(int64_t pointCloudHandle, int64_t objectHandle, int64_t options, double gridSize, std::optional<std::vector<double>> color, std::optional<double> duplicateTolerance)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(pointCloudHandle);
        _args.push_back(objectHandle);
        _args.push_back(options);
        _args.push_back(gridSize);
        if(color)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*color);
        }
        else _brk = true;
        if(duplicateTolerance)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*duplicateTolerance);
        }
        else _brk = true;
        auto _ret = this->_client->call("sim.insertObjectIntoPointCloud", _args);
        return _ret[0].as<int64_t>();
    }

    int64_t sim::insertPointsIntoPointCloud(int64_t pointCloudHandle, int64_t options, std::vector<double> points, std::optional<std::vector<double>> color, std::optional<double> duplicateTolerance)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(pointCloudHandle);
        _args.push_back(options);
        _args.push_back(points);
        if(color)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*color);
        }
        else _brk = true;
        if(duplicateTolerance)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*duplicateTolerance);
        }
        else _brk = true;
        auto _ret = this->_client->call("sim.insertPointsIntoPointCloud", _args);
        return _ret[0].as<int64_t>();
    }

    int64_t sim::insertVoxelsIntoOctree(int64_t octreeHandle, int64_t options, std::vector<double> points, std::optional<std::vector<double>> color, std::optional<std::vector<int64_t>> tag)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(octreeHandle);
        _args.push_back(options);
        _args.push_back(points);
        if(color)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*color);
        }
        else _brk = true;
        if(tag)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*tag);
        }
        else _brk = true;
        auto _ret = this->_client->call("sim.insertVoxelsIntoOctree", _args);
        return _ret[0].as<int64_t>();
    }

    std::vector<double> sim::interpolateMatrices(std::vector<double> matrixIn1, std::vector<double> matrixIn2, double interpolFactor)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(matrixIn1);
        _args.push_back(matrixIn2);
        _args.push_back(interpolFactor);
        auto _ret = this->_client->call("sim.interpolateMatrices", _args);
        return _ret[0].as<std::vector<double>>();
    }

    int64_t sim::intersectPointsWithPointCloud(int64_t pointCloudHandle, int64_t options, std::vector<double> points, double tolerance)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(pointCloudHandle);
        _args.push_back(options);
        _args.push_back(points);
        _args.push_back(tolerance);
        auto _ret = this->_client->call("sim.intersectPointsWithPointCloud", _args);
        return _ret[0].as<int64_t>();
    }

    void sim::invertMatrix(std::vector<double> matrix)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(matrix);
        auto _ret = this->_client->call("sim.invertMatrix", _args);
    }

    int64_t sim::isDeprecated(std::string funcOrConst)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(funcOrConst);
        auto _ret = this->_client->call("sim.isDeprecated", _args);
        return _ret[0].as<int64_t>();
    }

    bool sim::isDynamicallyEnabled(int64_t objectHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(objectHandle);
        auto _ret = this->_client->call("sim.isDynamicallyEnabled", _args);
        return _ret[0].as<bool>();
    }

    bool sim::isHandle(int64_t objectHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(objectHandle);
        auto _ret = this->_client->call("sim.isHandle", _args);
        return _ret[0].as<bool>();
    }

    void sim::launchExecutable(std::string filename, std::optional<std::string> parameters, std::optional<int64_t> showStatus)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(filename);
        if(parameters)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*parameters);
        }
        else _brk = true;
        if(showStatus)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*showStatus);
        }
        else _brk = true;
        auto _ret = this->_client->call("sim.launchExecutable", _args);
    }

    std::tuple<std::vector<uint8_t>, std::vector<int64_t>> sim::loadImage(int64_t options, std::string filename)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(options);
        _args.push_back(filename);
        auto _ret = this->_client->call("sim.loadImage", _args);
        return std::make_tuple(_ret[0].as<std::vector<uint8_t>>(), _ret[1].as<std::vector<int64_t>>());
    }

    int64_t sim::loadModel(std::string filename)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(filename);
        auto _ret = this->_client->call("sim.loadModel", _args);
        return _ret[0].as<int64_t>();
    }

    int64_t sim::loadModule(std::string filenameAndPath, std::string pluginName)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(filenameAndPath);
        _args.push_back(pluginName);
        auto _ret = this->_client->call("sim.loadModule", _args);
        return _ret[0].as<int64_t>();
    }

    void sim::loadScene(std::string filename)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(filename);
        auto _ret = this->_client->call("sim.loadScene", _args);
    }

    int64_t sim::moduleEntry(int64_t handle, std::optional<std::string> label, std::optional<int64_t> state)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(handle);
        if(label)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*label);
        }
        else _brk = true;
        if(state)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*state);
        }
        else _brk = true;
        auto _ret = this->_client->call("sim.moduleEntry", _args);
        return _ret[0].as<int64_t>();
    }

    std::tuple<std::vector<double>, std::vector<double>, std::vector<double>, double> sim::moveToConfig(int64_t flags, std::vector<double> currentPos, std::vector<double> currentVel, std::vector<double> currentAccel, std::vector<double> maxVel, std::vector<double> maxAccel, std::vector<double> maxJerk, std::vector<double> targetPos, std::vector<double> targetVel, std::string callback, std::optional<json> auxData, std::optional<std::vector<bool>> cyclicJoints, std::optional<double> timeStep)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(flags);
        _args.push_back(currentPos);
        _args.push_back(currentVel);
        _args.push_back(currentAccel);
        _args.push_back(maxVel);
        _args.push_back(maxAccel);
        _args.push_back(maxJerk);
        _args.push_back(targetPos);
        _args.push_back(targetVel);
        _args.push_back(callback);
        if(auxData)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*auxData);
        }
        else _brk = true;
        if(cyclicJoints)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*cyclicJoints);
        }
        else _brk = true;
        if(timeStep)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*timeStep);
        }
        else _brk = true;
        auto _ret = this->_client->call("sim.moveToConfig", _args);
        return std::make_tuple(_ret[0].as<std::vector<double>>(), _ret[1].as<std::vector<double>>(), _ret[2].as<std::vector<double>>(), _ret[3].as<double>());
    }

    std::tuple<std::vector<double>, double> sim::moveToPose(int64_t flags, std::vector<double> currentPose, std::vector<double> maxVel, std::vector<double> maxAccel, std::vector<double> maxJerk, std::vector<double> targetPose, std::string callback, std::optional<json> auxData, std::optional<std::vector<double>> metric, std::optional<double> timeStep)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(flags);
        _args.push_back(currentPose);
        _args.push_back(maxVel);
        _args.push_back(maxAccel);
        _args.push_back(maxJerk);
        _args.push_back(targetPose);
        _args.push_back(callback);
        if(auxData)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*auxData);
        }
        else _brk = true;
        if(metric)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*metric);
        }
        else _brk = true;
        if(timeStep)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*timeStep);
        }
        else _brk = true;
        auto _ret = this->_client->call("sim.moveToPose", _args);
        return std::make_tuple(_ret[0].as<std::vector<double>>(), _ret[1].as<double>());
    }

    std::vector<double> sim::multiplyMatrices(std::vector<double> matrixIn1, std::vector<double> matrixIn2)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(matrixIn1);
        _args.push_back(matrixIn2);
        auto _ret = this->_client->call("sim.multiplyMatrices", _args);
        return _ret[0].as<std::vector<double>>();
    }

    std::vector<double> sim::multiplyVector(std::vector<double> matrix, std::vector<double> inVectors)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(matrix);
        _args.push_back(inVectors);
        auto _ret = this->_client->call("sim.multiplyVector", _args);
        return _ret[0].as<std::vector<double>>();
    }

    std::vector<uint8_t> sim::packDoubleTable(std::vector<double> doubleNumbers, std::optional<int64_t> startDoubleIndex, std::optional<int64_t> doubleCount)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(doubleNumbers);
        if(startDoubleIndex)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*startDoubleIndex);
        }
        else _brk = true;
        if(doubleCount)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*doubleCount);
        }
        else _brk = true;
        auto _ret = this->_client->call("sim.packDoubleTable", _args);
        return _ret[0].as<std::vector<uint8_t>>();
    }

    std::vector<uint8_t> sim::packFloatTable(std::vector<double> floatNumbers, std::optional<int64_t> startFloatIndex, std::optional<int64_t> floatCount)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(floatNumbers);
        if(startFloatIndex)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*startFloatIndex);
        }
        else _brk = true;
        if(floatCount)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*floatCount);
        }
        else _brk = true;
        auto _ret = this->_client->call("sim.packFloatTable", _args);
        return _ret[0].as<std::vector<uint8_t>>();
    }

    std::vector<uint8_t> sim::packInt32Table(std::vector<int64_t> int32Numbers, std::optional<int64_t> startInt32Index, std::optional<int64_t> int32Count)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(int32Numbers);
        if(startInt32Index)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*startInt32Index);
        }
        else _brk = true;
        if(int32Count)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*int32Count);
        }
        else _brk = true;
        auto _ret = this->_client->call("sim.packInt32Table", _args);
        return _ret[0].as<std::vector<uint8_t>>();
    }

    std::vector<uint8_t> sim::packTable(std::vector<json> aTable, std::optional<int64_t> scheme)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(aTable);
        if(scheme)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*scheme);
        }
        else _brk = true;
        auto _ret = this->_client->call("sim.packTable", _args);
        return _ret[0].as<std::vector<uint8_t>>();
    }

    std::vector<uint8_t> sim::packUInt16Table(std::vector<int64_t> uint16Numbers, std::optional<int64_t> startUint16Index, std::optional<int64_t> uint16Count)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(uint16Numbers);
        if(startUint16Index)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*startUint16Index);
        }
        else _brk = true;
        if(uint16Count)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*uint16Count);
        }
        else _brk = true;
        auto _ret = this->_client->call("sim.packUInt16Table", _args);
        return _ret[0].as<std::vector<uint8_t>>();
    }

    std::vector<uint8_t> sim::packUInt32Table(std::vector<int64_t> uint32Numbers, std::optional<int64_t> startUInt32Index, std::optional<int64_t> uint32Count)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(uint32Numbers);
        if(startUInt32Index)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*startUInt32Index);
        }
        else _brk = true;
        if(uint32Count)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*uint32Count);
        }
        else _brk = true;
        auto _ret = this->_client->call("sim.packUInt32Table", _args);
        return _ret[0].as<std::vector<uint8_t>>();
    }

    std::vector<uint8_t> sim::packUInt8Table(std::vector<int64_t> uint8Numbers, std::optional<int64_t> startUint8Index, std::optional<int64_t> uint8count)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(uint8Numbers);
        if(startUint8Index)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*startUint8Index);
        }
        else _brk = true;
        if(uint8count)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*uint8count);
        }
        else _brk = true;
        auto _ret = this->_client->call("sim.packUInt8Table", _args);
        return _ret[0].as<std::vector<uint8_t>>();
    }

    int64_t sim::pauseSimulation()
    {
        bool _brk = false;
        json _args(json_array_arg);
        auto _ret = this->_client->call("sim.pauseSimulation", _args);
        return _ret[0].as<int64_t>();
    }

    std::vector<uint8_t> sim::persistentDataRead(std::string dataTag)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(dataTag);
        auto _ret = this->_client->call("sim.persistentDataRead", _args);
        return _ret[0].as<std::vector<uint8_t>>();
    }

    void sim::persistentDataWrite(std::string dataTag, std::vector<uint8_t> dataValue, std::optional<int64_t> options)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(dataTag);
        _args.push_back(bin(dataValue));
        if(options)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*options);
        }
        else _brk = true;
        auto _ret = this->_client->call("sim.persistentDataWrite", _args);
    }

    void sim::pushUserEvent(std::string event, int64_t handle, int64_t uid, json eventData, std::optional<int64_t> options)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(event);
        _args.push_back(handle);
        _args.push_back(uid);
        _args.push_back(eventData);
        if(options)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*options);
        }
        else _brk = true;
        auto _ret = this->_client->call("sim.pushUserEvent", _args);
    }

    void sim::quitSimulator()
    {
        bool _brk = false;
        json _args(json_array_arg);
        auto _ret = this->_client->call("sim.quitSimulator", _args);
    }

    std::vector<uint8_t> sim::readCustomDataBlock(int64_t objectHandle, std::string tagName)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(objectHandle);
        _args.push_back(tagName);
        auto _ret = this->_client->call("sim.readCustomDataBlock", _args);
        return _ret[0].as<std::vector<uint8_t>>();
    }

    std::vector<std::string> sim::readCustomDataBlockTags(int64_t objectHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(objectHandle);
        auto _ret = this->_client->call("sim.readCustomDataBlockTags", _args);
        return _ret[0].as<std::vector<std::string>>();
    }

    json sim::readCustomTableData(int64_t objectHandle, std::string tagName)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(objectHandle);
        _args.push_back(tagName);
        auto _ret = this->_client->call("sim.readCustomTableData", _args);
        return _ret[0].as<json>();
    }

    std::tuple<int64_t, std::vector<double>, std::vector<double>> sim::readForceSensor(int64_t objectHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(objectHandle);
        auto _ret = this->_client->call("sim.readForceSensor", _args);
        return std::make_tuple(_ret[0].as<int64_t>(), _ret[1].as<std::vector<double>>(), _ret[2].as<std::vector<double>>());
    }

    std::tuple<int64_t, double, std::vector<double>, int64_t, std::vector<double>> sim::readProximitySensor(int64_t sensorHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(sensorHandle);
        auto _ret = this->_client->call("sim.readProximitySensor", _args);
        return std::make_tuple(_ret[0].as<int64_t>(), _ret[1].as<double>(), _ret[2].as<std::vector<double>>(), _ret[3].as<int64_t>(), _ret[4].as<std::vector<double>>());
    }

    std::vector<uint8_t> sim::readTexture(int64_t textureId, int64_t options, std::optional<int64_t> posX, std::optional<int64_t> posY, std::optional<int64_t> sizeX, std::optional<int64_t> sizeY)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(textureId);
        _args.push_back(options);
        if(posX)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*posX);
        }
        else _brk = true;
        if(posY)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*posY);
        }
        else _brk = true;
        if(sizeX)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*sizeX);
        }
        else _brk = true;
        if(sizeY)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*sizeY);
        }
        else _brk = true;
        auto _ret = this->_client->call("sim.readTexture", _args);
        return _ret[0].as<std::vector<uint8_t>>();
    }

    std::tuple<int64_t, std::vector<double>, std::vector<double>> sim::readVisionSensor(int64_t sensorHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(sensorHandle);
        auto _ret = this->_client->call("sim.readVisionSensor", _args);
        return std::make_tuple(_ret[0].as<int64_t>(), _ret[1].as<std::vector<double>>(), _ret[2].as<std::vector<double>>());
    }

    int64_t sim::refreshDialogs(int64_t refreshDegree)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(refreshDegree);
        auto _ret = this->_client->call("sim.refreshDialogs", _args);
        return _ret[0].as<int64_t>();
    }

    int64_t sim::registerScriptFuncHook(std::string funcToHook, std::string userFunc, bool execBefore)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(funcToHook);
        _args.push_back(userFunc);
        _args.push_back(execBefore);
        auto _ret = this->_client->call("sim.registerScriptFuncHook", _args);
        return _ret[0].as<int64_t>();
    }

    int64_t sim::registerScriptFunction(std::string funcNameAtPluginName, std::string callTips)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(funcNameAtPluginName);
        _args.push_back(callTips);
        auto _ret = this->_client->call("sim.registerScriptFunction", _args);
        return _ret[0].as<int64_t>();
    }

    int64_t sim::registerScriptVariable(std::string varNameAtPluginName)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(varNameAtPluginName);
        auto _ret = this->_client->call("sim.registerScriptVariable", _args);
        return _ret[0].as<int64_t>();
    }

    void sim::removeDrawingObject(int64_t drawingObjectHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(drawingObjectHandle);
        auto _ret = this->_client->call("sim.removeDrawingObject", _args);
    }

    int64_t sim::removeModel(int64_t objectHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(objectHandle);
        auto _ret = this->_client->call("sim.removeModel", _args);
        return _ret[0].as<int64_t>();
    }

    void sim::removeObjects(std::vector<int64_t> objectHandles)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(objectHandles);
        auto _ret = this->_client->call("sim.removeObjects", _args);
    }

    void sim::removeParticleObject(int64_t particleObjectHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(particleObjectHandle);
        auto _ret = this->_client->call("sim.removeParticleObject", _args);
    }

    int64_t sim::removePointsFromPointCloud(int64_t pointCloudHandle, int64_t options, std::vector<double> points, double tolerance)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(pointCloudHandle);
        _args.push_back(options);
        _args.push_back(points);
        _args.push_back(tolerance);
        auto _ret = this->_client->call("sim.removePointsFromPointCloud", _args);
        return _ret[0].as<int64_t>();
    }

    void sim::removeScript(int64_t scriptHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(scriptHandle);
        auto _ret = this->_client->call("sim.removeScript", _args);
    }

    int64_t sim::removeVoxelsFromOctree(int64_t octreeHandle, int64_t options, std::vector<double> points)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(octreeHandle);
        _args.push_back(options);
        _args.push_back(points);
        auto _ret = this->_client->call("sim.removeVoxelsFromOctree", _args);
        return _ret[0].as<int64_t>();
    }

    int64_t sim::reorientShapeBoundingBox(int64_t shapeHandle, int64_t relativeToHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(shapeHandle);
        _args.push_back(relativeToHandle);
        auto _ret = this->_client->call("sim.reorientShapeBoundingBox", _args);
        return _ret[0].as<int64_t>();
    }

    std::vector<double> sim::resamplePath(std::vector<double> path, std::vector<double> pathLengths, int64_t finalConfigCnt, std::optional<json> method, std::optional<std::vector<int64_t>> types)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(path);
        _args.push_back(pathLengths);
        _args.push_back(finalConfigCnt);
        if(method)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*method);
        }
        else _brk = true;
        if(types)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*types);
        }
        else _brk = true;
        auto _ret = this->_client->call("sim.resamplePath", _args);
        return _ret[0].as<std::vector<double>>();
    }

    void sim::resetDynamicObject(int64_t objectHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(objectHandle);
        auto _ret = this->_client->call("sim.resetDynamicObject", _args);
    }

    void sim::resetGraph(int64_t objectHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(objectHandle);
        auto _ret = this->_client->call("sim.resetGraph", _args);
    }

    void sim::resetProximitySensor(int64_t objectHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(objectHandle);
        auto _ret = this->_client->call("sim.resetProximitySensor", _args);
    }

    void sim::resetVisionSensor(int64_t sensorHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(sensorHandle);
        auto _ret = this->_client->call("sim.resetVisionSensor", _args);
    }

    void sim::restoreEntityColor(std::vector<json> originalColorData)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(originalColorData);
        auto _ret = this->_client->call("sim.restoreEntityColor", _args);
    }

    std::vector<double> sim::rotateAroundAxis(std::vector<double> matrixIn, std::vector<double> axis, std::vector<double> axisPos, double angle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(matrixIn);
        _args.push_back(axis);
        _args.push_back(axisPos);
        _args.push_back(angle);
        auto _ret = this->_client->call("sim.rotateAroundAxis", _args);
        return _ret[0].as<std::vector<double>>();
    }

    int64_t sim::ruckigPos(int64_t dofs, double baseCycleTime, int64_t flags, std::vector<double> currentPosVelAccel, std::vector<double> maxVelAccelJerk, std::vector<int64_t> selection, std::vector<double> targetPosVel)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(dofs);
        _args.push_back(baseCycleTime);
        _args.push_back(flags);
        _args.push_back(currentPosVelAccel);
        _args.push_back(maxVelAccelJerk);
        _args.push_back(selection);
        _args.push_back(targetPosVel);
        auto _ret = this->_client->call("sim.ruckigPos", _args);
        return _ret[0].as<int64_t>();
    }

    void sim::ruckigRemove(int64_t handle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(handle);
        auto _ret = this->_client->call("sim.ruckigRemove", _args);
    }

    std::tuple<int64_t, std::vector<double>, double> sim::ruckigStep(int64_t handle, double cycleTime)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(handle);
        _args.push_back(cycleTime);
        auto _ret = this->_client->call("sim.ruckigStep", _args);
        return std::make_tuple(_ret[0].as<int64_t>(), _ret[1].as<std::vector<double>>(), _ret[2].as<double>());
    }

    int64_t sim::ruckigVel(int64_t dofs, double baseCycleTime, int64_t flags, std::vector<double> currentPosVelAccel, std::vector<double> maxAccelJerk, std::vector<int64_t> selection, std::vector<double> targetVel)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(dofs);
        _args.push_back(baseCycleTime);
        _args.push_back(flags);
        _args.push_back(currentPosVelAccel);
        _args.push_back(maxAccelJerk);
        _args.push_back(selection);
        _args.push_back(targetVel);
        auto _ret = this->_client->call("sim.ruckigVel", _args);
        return _ret[0].as<int64_t>();
    }

    std::vector<uint8_t> sim::saveImage(std::vector<uint8_t> image, std::vector<int64_t> resolution, int64_t options, std::string filename, int64_t quality)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(bin(image));
        _args.push_back(resolution);
        _args.push_back(options);
        _args.push_back(filename);
        _args.push_back(quality);
        auto _ret = this->_client->call("sim.saveImage", _args);
        return _ret[0].as<std::vector<uint8_t>>();
    }

    void sim::saveModel(int64_t modelBaseHandle, std::string filename)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(modelBaseHandle);
        _args.push_back(filename);
        auto _ret = this->_client->call("sim.saveModel", _args);
    }

    void sim::saveScene(std::string filename)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(filename);
        auto _ret = this->_client->call("sim.saveScene", _args);
    }

    void sim::scaleObject(int64_t objectHandle, double xScale, double yScale, double zScale, std::optional<int64_t> options)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(objectHandle);
        _args.push_back(xScale);
        _args.push_back(yScale);
        _args.push_back(zScale);
        if(options)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*options);
        }
        else _brk = true;
        auto _ret = this->_client->call("sim.scaleObject", _args);
    }

    void sim::scaleObjects(std::vector<int64_t> objectHandles, double scalingFactor, bool scalePositionsToo)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(objectHandles);
        _args.push_back(scalingFactor);
        _args.push_back(scalePositionsToo);
        auto _ret = this->_client->call("sim.scaleObjects", _args);
    }

    int64_t sim::serialCheck(int64_t portHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(portHandle);
        auto _ret = this->_client->call("sim.serialCheck", _args);
        return _ret[0].as<int64_t>();
    }

    void sim::serialClose(int64_t portHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(portHandle);
        auto _ret = this->_client->call("sim.serialClose", _args);
    }

    int64_t sim::serialOpen(std::string portString, int64_t baudrate)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(portString);
        _args.push_back(baudrate);
        auto _ret = this->_client->call("sim.serialOpen", _args);
        return _ret[0].as<int64_t>();
    }

    std::vector<uint8_t> sim::serialRead(int64_t portHandle, int64_t dataLengthToRead, bool blockingOperation, std::optional<std::vector<uint8_t>> closingString, std::optional<double> timeout)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(portHandle);
        _args.push_back(dataLengthToRead);
        _args.push_back(blockingOperation);
        if(closingString)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(bin(*closingString));
        }
        else _brk = true;
        if(timeout)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*timeout);
        }
        else _brk = true;
        auto _ret = this->_client->call("sim.serialRead", _args);
        return _ret[0].as<std::vector<uint8_t>>();
    }

    int64_t sim::serialSend(int64_t portHandle, std::vector<uint8_t> data)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(portHandle);
        _args.push_back(bin(data));
        auto _ret = this->_client->call("sim.serialSend", _args);
        return _ret[0].as<int64_t>();
    }

    void sim::setArrayParam(int64_t parameter, std::vector<double> arrayOfValues)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(parameter);
        _args.push_back(arrayOfValues);
        auto _ret = this->_client->call("sim.setArrayParam", _args);
    }

    void sim::setBoolParam(int64_t parameter, bool boolState)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(parameter);
        _args.push_back(boolState);
        auto _ret = this->_client->call("sim.setBoolParam", _args);
    }

    void sim::setDoubleSignal(std::string signalName, double signalValue)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(signalName);
        _args.push_back(signalValue);
        auto _ret = this->_client->call("sim.setDoubleSignal", _args);
    }

    void sim::setEngineBoolParam(int64_t paramId, int64_t objectHandle, bool boolParam)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(paramId);
        _args.push_back(objectHandle);
        _args.push_back(boolParam);
        auto _ret = this->_client->call("sim.setEngineBoolParam", _args);
    }

    void sim::setEngineFloatParam(int64_t paramId, int64_t objectHandle, double floatParam)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(paramId);
        _args.push_back(objectHandle);
        _args.push_back(floatParam);
        auto _ret = this->_client->call("sim.setEngineFloatParam", _args);
    }

    void sim::setEngineInt32Param(int64_t paramId, int64_t objectHandle, int64_t int32Param)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(paramId);
        _args.push_back(objectHandle);
        _args.push_back(int32Param);
        auto _ret = this->_client->call("sim.setEngineInt32Param", _args);
    }

    void sim::setExplicitHandling(int64_t objectHandle, int64_t explicitHandlingFlags)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(objectHandle);
        _args.push_back(explicitHandlingFlags);
        auto _ret = this->_client->call("sim.setExplicitHandling", _args);
    }

    void sim::setFloatParam(int64_t parameter, double floatState)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(parameter);
        _args.push_back(floatState);
        auto _ret = this->_client->call("sim.setFloatParam", _args);
    }

    void sim::setFloatSignal(std::string signalName, double signalValue)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(signalName);
        _args.push_back(signalValue);
        auto _ret = this->_client->call("sim.setFloatSignal", _args);
    }

    void sim::setGraphStreamTransformation(int64_t graphHandle, int64_t streamId, int64_t trType, std::optional<double> mult, std::optional<double> off, std::optional<int64_t> movAvgPeriod)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(graphHandle);
        _args.push_back(streamId);
        _args.push_back(trType);
        if(mult)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*mult);
        }
        else _brk = true;
        if(off)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*off);
        }
        else _brk = true;
        if(movAvgPeriod)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*movAvgPeriod);
        }
        else _brk = true;
        auto _ret = this->_client->call("sim.setGraphStreamTransformation", _args);
    }

    void sim::setGraphStreamValue(int64_t graphHandle, int64_t streamId, double value)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(graphHandle);
        _args.push_back(streamId);
        _args.push_back(value);
        auto _ret = this->_client->call("sim.setGraphStreamValue", _args);
    }

    void sim::setInt32Param(int64_t parameter, int64_t intState)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(parameter);
        _args.push_back(intState);
        auto _ret = this->_client->call("sim.setInt32Param", _args);
    }

    void sim::setInt32Signal(std::string signalName, int64_t signalValue)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(signalName);
        _args.push_back(signalValue);
        auto _ret = this->_client->call("sim.setInt32Signal", _args);
    }

    void sim::setJointDependency(int64_t jointHandle, int64_t masterJointHandle, double offset, double multCoeff)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(jointHandle);
        _args.push_back(masterJointHandle);
        _args.push_back(offset);
        _args.push_back(multCoeff);
        auto _ret = this->_client->call("sim.setJointDependency", _args);
    }

    void sim::setJointInterval(int64_t objectHandle, bool cyclic, std::vector<double> interval)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(objectHandle);
        _args.push_back(cyclic);
        _args.push_back(interval);
        auto _ret = this->_client->call("sim.setJointInterval", _args);
    }

    void sim::setJointMode(int64_t jointHandle, int64_t jointMode, int64_t options)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(jointHandle);
        _args.push_back(jointMode);
        _args.push_back(options);
        auto _ret = this->_client->call("sim.setJointMode", _args);
    }

    void sim::setJointPosition(int64_t objectHandle, double position)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(objectHandle);
        _args.push_back(position);
        auto _ret = this->_client->call("sim.setJointPosition", _args);
    }

    void sim::setJointTargetForce(int64_t objectHandle, double forceOrTorque, std::optional<bool> signedValue)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(objectHandle);
        _args.push_back(forceOrTorque);
        if(signedValue)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*signedValue);
        }
        else _brk = true;
        auto _ret = this->_client->call("sim.setJointTargetForce", _args);
    }

    void sim::setJointTargetPosition(int64_t objectHandle, double targetPosition, std::optional<std::vector<double>> maxVelAccelJerk)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(objectHandle);
        _args.push_back(targetPosition);
        if(maxVelAccelJerk)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*maxVelAccelJerk);
        }
        else _brk = true;
        auto _ret = this->_client->call("sim.setJointTargetPosition", _args);
    }

    void sim::setJointTargetVelocity(int64_t objectHandle, double targetVelocity, std::optional<std::vector<double>> maxAccelJerk, std::optional<double> initVelocity)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(objectHandle);
        _args.push_back(targetVelocity);
        if(maxAccelJerk)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*maxAccelJerk);
        }
        else _brk = true;
        if(initVelocity)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*initVelocity);
        }
        else _brk = true;
        auto _ret = this->_client->call("sim.setJointTargetVelocity", _args);
    }

    void sim::setLightParameters(int64_t lightHandle, int64_t state, std::vector<double> reserved, std::vector<double> diffusePart, std::vector<double> specularPart)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(lightHandle);
        _args.push_back(state);
        _args.push_back(reserved);
        _args.push_back(diffusePart);
        _args.push_back(specularPart);
        auto _ret = this->_client->call("sim.setLightParameters", _args);
    }

    void sim::setLinkDummy(int64_t dummyHandle, int64_t linkDummyHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(dummyHandle);
        _args.push_back(linkDummyHandle);
        auto _ret = this->_client->call("sim.setLinkDummy", _args);
    }

    void sim::setModelProperty(int64_t objectHandle, int64_t property)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(objectHandle);
        _args.push_back(property);
        auto _ret = this->_client->call("sim.setModelProperty", _args);
    }

    void sim::setModuleInfo(std::string moduleName, int64_t infoType, std::string info)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(moduleName);
        _args.push_back(infoType);
        _args.push_back(info);
        auto _ret = this->_client->call("sim.setModuleInfo", _args);
    }

    void sim::setNamedStringParam(std::string paramName, std::vector<uint8_t> stringParam)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(paramName);
        _args.push_back(bin(stringParam));
        auto _ret = this->_client->call("sim.setNamedStringParam", _args);
    }

    void sim::setNavigationMode(int64_t navigationMode)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(navigationMode);
        auto _ret = this->_client->call("sim.setNavigationMode", _args);
    }

    void sim::setObjectAlias(int64_t objectHandle, std::string objectAlias)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(objectHandle);
        _args.push_back(objectAlias);
        auto _ret = this->_client->call("sim.setObjectAlias", _args);
    }

    void sim::setObjectChildPose(int64_t objectHandle, std::vector<double> pose)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(objectHandle);
        _args.push_back(pose);
        auto _ret = this->_client->call("sim.setObjectChildPose", _args);
    }

    bool sim::setObjectColor(int64_t objectHandle, int64_t index, int64_t colorComponent, std::vector<double> rgbData)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(objectHandle);
        _args.push_back(index);
        _args.push_back(colorComponent);
        _args.push_back(rgbData);
        auto _ret = this->_client->call("sim.setObjectColor", _args);
        return _ret[0].as<bool>();
    }

    void sim::setObjectFloatParam(int64_t objectHandle, int64_t parameterID, double parameter)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(objectHandle);
        _args.push_back(parameterID);
        _args.push_back(parameter);
        auto _ret = this->_client->call("sim.setObjectFloatParam", _args);
    }

    void sim::setObjectInt32Param(int64_t objectHandle, int64_t parameterID, int64_t parameter)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(objectHandle);
        _args.push_back(parameterID);
        _args.push_back(parameter);
        auto _ret = this->_client->call("sim.setObjectInt32Param", _args);
    }

    void sim::setObjectMatrix(int64_t objectHandle, int64_t relativeToObjectHandle, std::vector<double> matrix)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(objectHandle);
        _args.push_back(relativeToObjectHandle);
        _args.push_back(matrix);
        auto _ret = this->_client->call("sim.setObjectMatrix", _args);
    }

    void sim::setObjectOrientation(int64_t objectHandle, int64_t relativeToObjectHandle, std::vector<double> eulerAngles)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(objectHandle);
        _args.push_back(relativeToObjectHandle);
        _args.push_back(eulerAngles);
        auto _ret = this->_client->call("sim.setObjectOrientation", _args);
    }

    void sim::setObjectParent(int64_t objectHandle, int64_t parentObjectHandle, std::optional<bool> keepInPlace)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(objectHandle);
        _args.push_back(parentObjectHandle);
        if(keepInPlace)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*keepInPlace);
        }
        else _brk = true;
        auto _ret = this->_client->call("sim.setObjectParent", _args);
    }

    void sim::setObjectPose(int64_t objectHandle, int64_t relativeToObjectHandle, std::vector<double> pose)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(objectHandle);
        _args.push_back(relativeToObjectHandle);
        _args.push_back(pose);
        auto _ret = this->_client->call("sim.setObjectPose", _args);
    }

    void sim::setObjectPosition(int64_t objectHandle, int64_t relativeToObjectHandle, std::vector<double> position)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(objectHandle);
        _args.push_back(relativeToObjectHandle);
        _args.push_back(position);
        auto _ret = this->_client->call("sim.setObjectPosition", _args);
    }

    void sim::setObjectProperty(int64_t objectHandle, int64_t property)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(objectHandle);
        _args.push_back(property);
        auto _ret = this->_client->call("sim.setObjectProperty", _args);
    }

    void sim::setObjectQuaternion(int64_t objectHandle, int64_t relativeToObjectHandle, std::vector<double> quaternion)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(objectHandle);
        _args.push_back(relativeToObjectHandle);
        _args.push_back(quaternion);
        auto _ret = this->_client->call("sim.setObjectQuaternion", _args);
    }

    void sim::setObjectSelection(std::vector<double> objectHandles)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(objectHandles);
        auto _ret = this->_client->call("sim.setObjectSelection", _args);
    }

    void sim::setObjectSpecialProperty(int64_t objectHandle, int64_t property)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(objectHandle);
        _args.push_back(property);
        auto _ret = this->_client->call("sim.setObjectSpecialProperty", _args);
    }

    void sim::setObjectStringParam(int64_t objectHandle, int64_t parameterID, std::vector<uint8_t> parameter)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(objectHandle);
        _args.push_back(parameterID);
        _args.push_back(bin(parameter));
        auto _ret = this->_client->call("sim.setObjectStringParam", _args);
    }

    void sim::setPage(int64_t pageIndex)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(pageIndex);
        auto _ret = this->_client->call("sim.setPage", _args);
    }

    void sim::setPointCloudOptions(int64_t pointCloudHandle, double maxVoxelSize, int64_t maxPtCntPerVoxel, int64_t options, double pointSize)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(pointCloudHandle);
        _args.push_back(maxVoxelSize);
        _args.push_back(maxPtCntPerVoxel);
        _args.push_back(options);
        _args.push_back(pointSize);
        auto _ret = this->_client->call("sim.setPointCloudOptions", _args);
    }

    void sim::setReferencedHandles(int64_t objectHandle, std::vector<int64_t> referencedHandles)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(objectHandle);
        _args.push_back(referencedHandles);
        auto _ret = this->_client->call("sim.setReferencedHandles", _args);
    }

    void sim::setScriptInt32Param(int64_t scriptHandle, int64_t parameterID, int64_t parameter)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(scriptHandle);
        _args.push_back(parameterID);
        _args.push_back(parameter);
        auto _ret = this->_client->call("sim.setScriptInt32Param", _args);
    }

    void sim::setScriptStringParam(int64_t scriptHandle, int64_t parameterID, std::vector<uint8_t> parameter)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(scriptHandle);
        _args.push_back(parameterID);
        _args.push_back(bin(parameter));
        auto _ret = this->_client->call("sim.setScriptStringParam", _args);
    }

    void sim::setShapeBB(int64_t shapeHandle, std::vector<double> size)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(shapeHandle);
        _args.push_back(size);
        auto _ret = this->_client->call("sim.setShapeBB", _args);
    }

    void sim::setShapeColor(int64_t shapeHandle, std::string colorName, int64_t colorComponent, std::vector<double> rgbData)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(shapeHandle);
        _args.push_back(colorName);
        _args.push_back(colorComponent);
        _args.push_back(rgbData);
        auto _ret = this->_client->call("sim.setShapeColor", _args);
    }

    void sim::setShapeInertia(int64_t shapeHandle, std::vector<double> inertiaMatrix, std::vector<double> transformationMatrix)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(shapeHandle);
        _args.push_back(inertiaMatrix);
        _args.push_back(transformationMatrix);
        auto _ret = this->_client->call("sim.setShapeInertia", _args);
    }

    void sim::setShapeMass(int64_t shapeHandle, double mass)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(shapeHandle);
        _args.push_back(mass);
        auto _ret = this->_client->call("sim.setShapeMass", _args);
    }

    void sim::setShapeMaterial(int64_t shapeHandle, int64_t materialIdOrShapeHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(shapeHandle);
        _args.push_back(materialIdOrShapeHandle);
        auto _ret = this->_client->call("sim.setShapeMaterial", _args);
    }

    void sim::setShapeTexture(int64_t shapeHandle, int64_t textureId, int64_t mappingMode, int64_t options, std::vector<double> uvScaling, std::optional<std::vector<double>> position, std::optional<std::vector<double>> orientation)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(shapeHandle);
        _args.push_back(textureId);
        _args.push_back(mappingMode);
        _args.push_back(options);
        _args.push_back(uvScaling);
        if(position)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*position);
        }
        else _brk = true;
        if(orientation)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*orientation);
        }
        else _brk = true;
        auto _ret = this->_client->call("sim.setShapeTexture", _args);
    }

    void sim::setStringParam(int64_t parameter, std::string stringState)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(parameter);
        _args.push_back(stringState);
        auto _ret = this->_client->call("sim.setStringParam", _args);
    }

    void sim::setStringSignal(std::string signalName, std::vector<uint8_t> signalValue)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(signalName);
        _args.push_back(bin(signalValue));
        auto _ret = this->_client->call("sim.setStringSignal", _args);
    }

    int64_t sim::setThreadAutomaticSwitch(bool automaticSwitch)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(automaticSwitch);
        auto _ret = this->_client->call("sim.setThreadAutomaticSwitch", _args);
        return _ret[0].as<int64_t>();
    }

    int64_t sim::setThreadSwitchAllowed(bool allowed)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(allowed);
        auto _ret = this->_client->call("sim.setThreadSwitchAllowed", _args);
        return _ret[0].as<int64_t>();
    }

    void sim::setThreadSwitchTiming(int64_t dtInMs)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(dtInMs);
        auto _ret = this->_client->call("sim.setThreadSwitchTiming", _args);
    }

    void sim::setVisionSensorImg(int64_t sensorHandle, std::vector<uint8_t> image, std::optional<int64_t> options, std::optional<std::vector<int64_t>> pos, std::optional<std::vector<int64_t>> size)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(sensorHandle);
        _args.push_back(bin(image));
        if(options)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*options);
        }
        else _brk = true;
        if(pos)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*pos);
        }
        else _brk = true;
        if(size)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*size);
        }
        else _brk = true;
        auto _ret = this->_client->call("sim.setVisionSensorImg", _args);
    }

    int64_t sim::startSimulation()
    {
        bool _brk = false;
        json _args(json_array_arg);
        auto _ret = this->_client->call("sim.startSimulation", _args);
        return _ret[0].as<int64_t>();
    }

    int64_t sim::stopSimulation()
    {
        bool _brk = false;
        json _args(json_array_arg);
        auto _ret = this->_client->call("sim.stopSimulation", _args);
        return _ret[0].as<int64_t>();
    }

    int64_t sim::subtractObjectFromOctree(int64_t octreeHandle, int64_t objectHandle, int64_t options)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(octreeHandle);
        _args.push_back(objectHandle);
        _args.push_back(options);
        auto _ret = this->_client->call("sim.subtractObjectFromOctree", _args);
        return _ret[0].as<int64_t>();
    }

    int64_t sim::subtractObjectFromPointCloud(int64_t pointCloudHandle, int64_t objectHandle, int64_t options, double tolerance)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(pointCloudHandle);
        _args.push_back(objectHandle);
        _args.push_back(options);
        _args.push_back(tolerance);
        auto _ret = this->_client->call("sim.subtractObjectFromPointCloud", _args);
        return _ret[0].as<int64_t>();
    }

    void sim::switchThread()
    {
        bool _brk = false;
        json _args(json_array_arg);
        auto _ret = this->_client->call("sim.switchThread", _args);
    }

    std::tuple<std::string, std::vector<int64_t>, std::vector<int64_t>> sim::textEditorClose(int64_t handle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(handle);
        auto _ret = this->_client->call("sim.textEditorClose", _args);
        return std::make_tuple(_ret[0].as<std::string>(), _ret[1].as<std::vector<int64_t>>(), _ret[2].as<std::vector<int64_t>>());
    }

    std::tuple<std::string, std::vector<int64_t>, std::vector<int64_t>, bool> sim::textEditorGetInfo(int64_t handle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(handle);
        auto _ret = this->_client->call("sim.textEditorGetInfo", _args);
        return std::make_tuple(_ret[0].as<std::string>(), _ret[1].as<std::vector<int64_t>>(), _ret[2].as<std::vector<int64_t>>(), _ret[3].as<bool>());
    }

    int64_t sim::textEditorOpen(std::string initText, std::string properties)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(initText);
        _args.push_back(properties);
        auto _ret = this->_client->call("sim.textEditorOpen", _args);
        return _ret[0].as<int64_t>();
    }

    void sim::textEditorShow(int64_t handle, bool showState)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(handle);
        _args.push_back(showState);
        auto _ret = this->_client->call("sim.textEditorShow", _args);
    }

    std::vector<uint8_t> sim::transformBuffer(std::vector<uint8_t> inBuffer, int64_t inFormat, double multiplier, double offset, int64_t outFormat)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(bin(inBuffer));
        _args.push_back(inFormat);
        _args.push_back(multiplier);
        _args.push_back(offset);
        _args.push_back(outFormat);
        auto _ret = this->_client->call("sim.transformBuffer", _args);
        return _ret[0].as<std::vector<uint8_t>>();
    }

    void sim::transformImage(std::vector<uint8_t> image, std::vector<int64_t> resolution, int64_t options)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(bin(image));
        _args.push_back(resolution);
        _args.push_back(options);
        auto _ret = this->_client->call("sim.transformImage", _args);
    }

    std::vector<int64_t> sim::ungroupShape(int64_t shapeHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(shapeHandle);
        auto _ret = this->_client->call("sim.ungroupShape", _args);
        return _ret[0].as<std::vector<int64_t>>();
    }

    int64_t sim::unloadModule(int64_t pluginHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(pluginHandle);
        auto _ret = this->_client->call("sim.unloadModule", _args);
        return _ret[0].as<int64_t>();
    }

    std::vector<double> sim::unpackDoubleTable(std::vector<uint8_t> data, std::optional<int64_t> startDoubleIndex, std::optional<int64_t> doubleCount, std::optional<int64_t> additionalByteOffset)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(bin(data));
        if(startDoubleIndex)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*startDoubleIndex);
        }
        else _brk = true;
        if(doubleCount)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*doubleCount);
        }
        else _brk = true;
        if(additionalByteOffset)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*additionalByteOffset);
        }
        else _brk = true;
        auto _ret = this->_client->call("sim.unpackDoubleTable", _args);
        return _ret[0].as<std::vector<double>>();
    }

    std::vector<double> sim::unpackFloatTable(std::vector<uint8_t> data, std::optional<int64_t> startFloatIndex, std::optional<int64_t> floatCount, std::optional<int64_t> additionalByteOffset)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(bin(data));
        if(startFloatIndex)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*startFloatIndex);
        }
        else _brk = true;
        if(floatCount)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*floatCount);
        }
        else _brk = true;
        if(additionalByteOffset)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*additionalByteOffset);
        }
        else _brk = true;
        auto _ret = this->_client->call("sim.unpackFloatTable", _args);
        return _ret[0].as<std::vector<double>>();
    }

    std::vector<int64_t> sim::unpackInt32Table(std::vector<uint8_t> data, std::optional<int64_t> startInt32Index, std::optional<int64_t> int32Count, std::optional<int64_t> additionalByteOffset)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(bin(data));
        if(startInt32Index)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*startInt32Index);
        }
        else _brk = true;
        if(int32Count)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*int32Count);
        }
        else _brk = true;
        if(additionalByteOffset)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*additionalByteOffset);
        }
        else _brk = true;
        auto _ret = this->_client->call("sim.unpackInt32Table", _args);
        return _ret[0].as<std::vector<int64_t>>();
    }

    json sim::unpackTable(std::vector<uint8_t> buffer)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(bin(buffer));
        auto _ret = this->_client->call("sim.unpackTable", _args);
        return _ret[0].as<json>();
    }

    std::vector<int64_t> sim::unpackUInt16Table(std::vector<uint8_t> data, std::optional<int64_t> startUint16Index, std::optional<int64_t> uint16Count, std::optional<int64_t> additionalByteOffset)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(bin(data));
        if(startUint16Index)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*startUint16Index);
        }
        else _brk = true;
        if(uint16Count)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*uint16Count);
        }
        else _brk = true;
        if(additionalByteOffset)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*additionalByteOffset);
        }
        else _brk = true;
        auto _ret = this->_client->call("sim.unpackUInt16Table", _args);
        return _ret[0].as<std::vector<int64_t>>();
    }

    std::vector<int64_t> sim::unpackUInt32Table(std::vector<uint8_t> data, std::optional<int64_t> startUint32Index, std::optional<int64_t> uint32Count, std::optional<int64_t> additionalByteOffset)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(bin(data));
        if(startUint32Index)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*startUint32Index);
        }
        else _brk = true;
        if(uint32Count)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*uint32Count);
        }
        else _brk = true;
        if(additionalByteOffset)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*additionalByteOffset);
        }
        else _brk = true;
        auto _ret = this->_client->call("sim.unpackUInt32Table", _args);
        return _ret[0].as<std::vector<int64_t>>();
    }

    std::vector<int64_t> sim::unpackUInt8Table(std::vector<uint8_t> data, std::optional<int64_t> startUint8Index, std::optional<int64_t> uint8count)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(bin(data));
        if(startUint8Index)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*startUint8Index);
        }
        else _brk = true;
        if(uint8count)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*uint8count);
        }
        else _brk = true;
        auto _ret = this->_client->call("sim.unpackUInt8Table", _args);
        return _ret[0].as<std::vector<int64_t>>();
    }

    double sim::wait(double dt, std::optional<bool> simulationTime)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(dt);
        if(simulationTime)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*simulationTime);
        }
        else _brk = true;
        auto _ret = this->_client->call("sim.wait", _args);
        return _ret[0].as<double>();
    }

    json sim::waitForSignal(std::string sigName)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(sigName);
        auto _ret = this->_client->call("sim.waitForSignal", _args);
        return _ret[0].as<json>();
    }

    void sim::writeCustomDataBlock(int64_t objectHandle, std::string tagName, std::vector<uint8_t> data)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(objectHandle);
        _args.push_back(tagName);
        _args.push_back(bin(data));
        auto _ret = this->_client->call("sim.writeCustomDataBlock", _args);
    }

    void sim::writeCustomTableData(int64_t objectHandle, std::string tagName, std::vector<json> data)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(objectHandle);
        _args.push_back(tagName);
        _args.push_back(data);
        auto _ret = this->_client->call("sim.writeCustomTableData", _args);
    }

    void sim::writeTexture(int64_t textureId, int64_t options, std::vector<uint8_t> textureData, std::optional<int64_t> posX, std::optional<int64_t> posY, std::optional<int64_t> sizeX, std::optional<int64_t> sizeY, std::optional<double> interpol)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(textureId);
        _args.push_back(options);
        _args.push_back(bin(textureData));
        if(posX)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*posX);
        }
        else _brk = true;
        if(posY)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*posY);
        }
        else _brk = true;
        if(sizeX)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*sizeX);
        }
        else _brk = true;
        if(sizeY)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*sizeY);
        }
        else _brk = true;
        if(interpol)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*interpol);
        }
        else _brk = true;
        auto _ret = this->_client->call("sim.writeTexture", _args);
    }

    std::tuple<double, double, double> sim::yawPitchRollToAlphaBetaGamma(double yawAngle, double pitchAngle, double rollAngle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(yawAngle);
        _args.push_back(pitchAngle);
        _args.push_back(rollAngle);
        auto _ret = this->_client->call("sim.yawPitchRollToAlphaBetaGamma", _args);
        return std::make_tuple(_ret[0].as<double>(), _ret[1].as<double>(), _ret[2].as<double>());
    }

    simIK::simIK(RemoteAPIClient *client)
        : _client(client)
    {
    }

    int64_t simIK::addIkElement(int64_t environmentHandle, int64_t ikGroupHandle, int64_t tipDummyHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(environmentHandle);
        _args.push_back(ikGroupHandle);
        _args.push_back(tipDummyHandle);
        auto _ret = this->_client->call("simIK.addIkElement", _args);
        return _ret[0].as<int64_t>();
    }

    std::tuple<int64_t, json> simIK::addIkElementFromScene(int64_t environmentHandle, int64_t ikGroup, int64_t baseHandle, int64_t tipHandle, int64_t targetHandle, int64_t constraints)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(environmentHandle);
        _args.push_back(ikGroup);
        _args.push_back(baseHandle);
        _args.push_back(tipHandle);
        _args.push_back(targetHandle);
        _args.push_back(constraints);
        auto _ret = this->_client->call("simIK.addIkElementFromScene", _args);
        return std::make_tuple(_ret[0].as<int64_t>(), _ret[1].as<json>());
    }

    int64_t simIK::applyIkEnvironmentToScene(int64_t environmentHandle, int64_t ikGroup, std::optional<bool> applyOnlyWhenSuccessful)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(environmentHandle);
        _args.push_back(ikGroup);
        if(applyOnlyWhenSuccessful)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*applyOnlyWhenSuccessful);
        }
        else _brk = true;
        auto _ret = this->_client->call("simIK.applyIkEnvironmentToScene", _args);
        return _ret[0].as<int64_t>();
    }

    void simIK::applySceneToIkEnvironment(int64_t environmentHandle, int64_t ikGroup)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(environmentHandle);
        _args.push_back(ikGroup);
        auto _ret = this->_client->call("simIK.applySceneToIkEnvironment", _args);
    }

    bool simIK::computeJacobian(int64_t environmentHandle, int64_t ikGroupHandle, int64_t options)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(environmentHandle);
        _args.push_back(ikGroupHandle);
        _args.push_back(options);
        auto _ret = this->_client->call("simIK.computeJacobian", _args);
        return _ret[0].as<bool>();
    }

    int64_t simIK::createDummy(int64_t environmentHandle, std::optional<std::string> dummyName)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(environmentHandle);
        if(dummyName)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*dummyName);
        }
        else _brk = true;
        auto _ret = this->_client->call("simIK.createDummy", _args);
        return _ret[0].as<int64_t>();
    }

    int64_t simIK::createEnvironment()
    {
        bool _brk = false;
        json _args(json_array_arg);
        auto _ret = this->_client->call("simIK.createEnvironment", _args);
        return _ret[0].as<int64_t>();
    }

    int64_t simIK::createIkGroup(int64_t environmentHandle, std::optional<std::string> ikGroupName)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(environmentHandle);
        if(ikGroupName)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*ikGroupName);
        }
        else _brk = true;
        auto _ret = this->_client->call("simIK.createIkGroup", _args);
        return _ret[0].as<int64_t>();
    }

    int64_t simIK::createJoint(int64_t environmentHandle, int64_t jointType, std::optional<std::string> jointName)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(environmentHandle);
        _args.push_back(jointType);
        if(jointName)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*jointName);
        }
        else _brk = true;
        auto _ret = this->_client->call("simIK.createJoint", _args);
        return _ret[0].as<int64_t>();
    }

    bool simIK::doesIkGroupExist(int64_t environmentHandle, std::string ikGroupName)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(environmentHandle);
        _args.push_back(ikGroupName);
        auto _ret = this->_client->call("simIK.doesIkGroupExist", _args);
        return _ret[0].as<bool>();
    }

    bool simIK::doesObjectExist(int64_t environmentHandle, std::string objectName)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(environmentHandle);
        _args.push_back(objectName);
        auto _ret = this->_client->call("simIK.doesObjectExist", _args);
        return _ret[0].as<bool>();
    }

    int64_t simIK::duplicateEnvironment(int64_t environmentHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(environmentHandle);
        auto _ret = this->_client->call("simIK.duplicateEnvironment", _args);
        return _ret[0].as<int64_t>();
    }

    void simIK::eraseEnvironment(int64_t environmentHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(environmentHandle);
        auto _ret = this->_client->call("simIK.eraseEnvironment", _args);
    }

    void simIK::eraseObject(int64_t environmentHandle, int64_t objectHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(environmentHandle);
        _args.push_back(objectHandle);
        auto _ret = this->_client->call("simIK.eraseObject", _args);
    }

    std::vector<double> simIK::findConfig(int64_t environmentHandle, int64_t ikGroupHandle, std::vector<int64_t> jointHandles, std::optional<double> thresholdDist, std::optional<double> maxTime, std::optional<std::vector<double>> metric, std::optional<std::string> validationCallback, std::optional<json> auxData)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(environmentHandle);
        _args.push_back(ikGroupHandle);
        _args.push_back(jointHandles);
        if(thresholdDist)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*thresholdDist);
        }
        else _brk = true;
        if(maxTime)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*maxTime);
        }
        else _brk = true;
        if(metric)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*metric);
        }
        else _brk = true;
        if(validationCallback)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*validationCallback);
        }
        else _brk = true;
        if(auxData)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*auxData);
        }
        else _brk = true;
        auto _ret = this->_client->call("simIK.findConfig", _args);
        return _ret[0].as<std::vector<double>>();
    }

    std::vector<double> simIK::generatePath(int64_t environmentHandle, int64_t ikGroupHandle, std::vector<int64_t> jointHandles, int64_t tipHandle, int64_t pathPointCount, std::optional<std::string> validationCallback, std::optional<json> auxData)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(environmentHandle);
        _args.push_back(ikGroupHandle);
        _args.push_back(jointHandles);
        _args.push_back(tipHandle);
        _args.push_back(pathPointCount);
        if(validationCallback)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*validationCallback);
        }
        else _brk = true;
        if(auxData)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*auxData);
        }
        else _brk = true;
        auto _ret = this->_client->call("simIK.generatePath", _args);
        return _ret[0].as<std::vector<double>>();
    }

    std::vector<double> simIK::getAlternateConfigs(int64_t environmentHandle, std::vector<int64_t> jointHandles, std::optional<std::vector<double>> lowLimits, std::optional<std::vector<double>> ranges)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(environmentHandle);
        _args.push_back(jointHandles);
        if(lowLimits)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*lowLimits);
        }
        else _brk = true;
        if(ranges)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*ranges);
        }
        else _brk = true;
        auto _ret = this->_client->call("simIK.getAlternateConfigs", _args);
        return _ret[0].as<std::vector<double>>();
    }

    std::tuple<int64_t, int64_t> simIK::getIkElementBase(int64_t environmentHandle, int64_t ikGroupHandle, int64_t elementHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(environmentHandle);
        _args.push_back(ikGroupHandle);
        _args.push_back(elementHandle);
        auto _ret = this->_client->call("simIK.getIkElementBase", _args);
        return std::make_tuple(_ret[0].as<int64_t>(), _ret[1].as<int64_t>());
    }

    int64_t simIK::getIkElementConstraints(int64_t environmentHandle, int64_t ikGroupHandle, int64_t elementHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(environmentHandle);
        _args.push_back(ikGroupHandle);
        _args.push_back(elementHandle);
        auto _ret = this->_client->call("simIK.getIkElementConstraints", _args);
        return _ret[0].as<int64_t>();
    }

    int64_t simIK::getIkElementFlags(int64_t environmentHandle, int64_t ikGroupHandle, int64_t elementHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(environmentHandle);
        _args.push_back(ikGroupHandle);
        _args.push_back(elementHandle);
        auto _ret = this->_client->call("simIK.getIkElementFlags", _args);
        return _ret[0].as<int64_t>();
    }

    std::vector<double> simIK::getIkElementPrecision(int64_t environmentHandle, int64_t ikGroupHandle, int64_t elementHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(environmentHandle);
        _args.push_back(ikGroupHandle);
        _args.push_back(elementHandle);
        auto _ret = this->_client->call("simIK.getIkElementPrecision", _args);
        return _ret[0].as<std::vector<double>>();
    }

    std::vector<double> simIK::getIkElementWeights(int64_t environmentHandle, int64_t ikGroupHandle, int64_t elementHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(environmentHandle);
        _args.push_back(ikGroupHandle);
        _args.push_back(elementHandle);
        auto _ret = this->_client->call("simIK.getIkElementWeights", _args);
        return _ret[0].as<std::vector<double>>();
    }

    std::tuple<int64_t, double, int64_t> simIK::getIkGroupCalculation(int64_t environmentHandle, int64_t ikGroupHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(environmentHandle);
        _args.push_back(ikGroupHandle);
        auto _ret = this->_client->call("simIK.getIkGroupCalculation", _args);
        return std::make_tuple(_ret[0].as<int64_t>(), _ret[1].as<double>(), _ret[2].as<int64_t>());
    }

    int64_t simIK::getIkGroupFlags(int64_t environmentHandle, int64_t ikGroupHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(environmentHandle);
        _args.push_back(ikGroupHandle);
        auto _ret = this->_client->call("simIK.getIkGroupFlags", _args);
        return _ret[0].as<int64_t>();
    }

    int64_t simIK::getIkGroupHandle(int64_t environmentHandle, std::string ikGroupName)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(environmentHandle);
        _args.push_back(ikGroupName);
        auto _ret = this->_client->call("simIK.getIkGroupHandle", _args);
        return _ret[0].as<int64_t>();
    }

    std::tuple<std::vector<double>, std::vector<int64_t>> simIK::getJacobian(int64_t environmentHandle, int64_t ikGroupHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(environmentHandle);
        _args.push_back(ikGroupHandle);
        auto _ret = this->_client->call("simIK.getJacobian", _args);
        return std::make_tuple(_ret[0].as<std::vector<double>>(), _ret[1].as<std::vector<int64_t>>());
    }

    std::tuple<int64_t, double, double> simIK::getJointDependency(int64_t environmentHandle, int64_t jointHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(environmentHandle);
        _args.push_back(jointHandle);
        auto _ret = this->_client->call("simIK.getJointDependency", _args);
        return std::make_tuple(_ret[0].as<int64_t>(), _ret[1].as<double>(), _ret[2].as<double>());
    }

    double simIK::getJointIkWeight(int64_t environmentHandle, int64_t jointHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(environmentHandle);
        _args.push_back(jointHandle);
        auto _ret = this->_client->call("simIK.getJointIkWeight", _args);
        return _ret[0].as<double>();
    }

    std::tuple<bool, std::vector<double>> simIK::getJointInterval(int64_t environmentHandle, int64_t jointHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(environmentHandle);
        _args.push_back(jointHandle);
        auto _ret = this->_client->call("simIK.getJointInterval", _args);
        return std::make_tuple(_ret[0].as<bool>(), _ret[1].as<std::vector<double>>());
    }

    std::vector<double> simIK::getJointMatrix(int64_t environmentHandle, int64_t jointHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(environmentHandle);
        _args.push_back(jointHandle);
        auto _ret = this->_client->call("simIK.getJointMatrix", _args);
        return _ret[0].as<std::vector<double>>();
    }

    double simIK::getJointMaxStepSize(int64_t environmentHandle, int64_t jointHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(environmentHandle);
        _args.push_back(jointHandle);
        auto _ret = this->_client->call("simIK.getJointMaxStepSize", _args);
        return _ret[0].as<double>();
    }

    int64_t simIK::getJointMode(int64_t environmentHandle, int64_t jointHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(environmentHandle);
        _args.push_back(jointHandle);
        auto _ret = this->_client->call("simIK.getJointMode", _args);
        return _ret[0].as<int64_t>();
    }

    double simIK::getJointPosition(int64_t environmentHandle, int64_t jointHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(environmentHandle);
        _args.push_back(jointHandle);
        auto _ret = this->_client->call("simIK.getJointPosition", _args);
        return _ret[0].as<double>();
    }

    double simIK::getJointScrewPitch(int64_t environmentHandle, int64_t jointHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(environmentHandle);
        _args.push_back(jointHandle);
        auto _ret = this->_client->call("simIK.getJointScrewPitch", _args);
        return _ret[0].as<double>();
    }

    std::tuple<std::vector<double>, std::vector<double>, std::vector<double>> simIK::getJointTransformation(int64_t environmentHandle, int64_t jointHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(environmentHandle);
        _args.push_back(jointHandle);
        auto _ret = this->_client->call("simIK.getJointTransformation", _args);
        return std::make_tuple(_ret[0].as<std::vector<double>>(), _ret[1].as<std::vector<double>>(), _ret[2].as<std::vector<double>>());
    }

    int64_t simIK::getJointType(int64_t environmentHandle, int64_t jointHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(environmentHandle);
        _args.push_back(jointHandle);
        auto _ret = this->_client->call("simIK.getJointType", _args);
        return _ret[0].as<int64_t>();
    }

    int64_t simIK::getLinkedDummy(int64_t environmentHandle, int64_t dummyHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(environmentHandle);
        _args.push_back(dummyHandle);
        auto _ret = this->_client->call("simIK.getLinkedDummy", _args);
        return _ret[0].as<int64_t>();
    }

    double simIK::getManipulability(int64_t environmentHandle, int64_t ikGroupHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(environmentHandle);
        _args.push_back(ikGroupHandle);
        auto _ret = this->_client->call("simIK.getManipulability", _args);
        return _ret[0].as<double>();
    }

    int64_t simIK::getObjectHandle(int64_t environmentHandle, std::string objectName)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(environmentHandle);
        _args.push_back(objectName);
        auto _ret = this->_client->call("simIK.getObjectHandle", _args);
        return _ret[0].as<int64_t>();
    }

    std::vector<double> simIK::getObjectMatrix(int64_t environmentHandle, int64_t objectHandle, int64_t relativeToObjectHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(environmentHandle);
        _args.push_back(objectHandle);
        _args.push_back(relativeToObjectHandle);
        auto _ret = this->_client->call("simIK.getObjectMatrix", _args);
        return _ret[0].as<std::vector<double>>();
    }

    int64_t simIK::getObjectParent(int64_t environmentHandle, int64_t objectHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(environmentHandle);
        _args.push_back(objectHandle);
        auto _ret = this->_client->call("simIK.getObjectParent", _args);
        return _ret[0].as<int64_t>();
    }

    std::vector<double> simIK::getObjectPose(int64_t environmentHandle, int64_t objectHandle, int64_t relativeToObjectHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(environmentHandle);
        _args.push_back(objectHandle);
        _args.push_back(relativeToObjectHandle);
        auto _ret = this->_client->call("simIK.getObjectPose", _args);
        return _ret[0].as<std::vector<double>>();
    }

    std::tuple<std::vector<double>, std::vector<double>, std::vector<double>> simIK::getObjectTransformation(int64_t environmentHandle, int64_t objectHandle, int64_t relativeToObjectHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(environmentHandle);
        _args.push_back(objectHandle);
        _args.push_back(relativeToObjectHandle);
        auto _ret = this->_client->call("simIK.getObjectTransformation", _args);
        return std::make_tuple(_ret[0].as<std::vector<double>>(), _ret[1].as<std::vector<double>>(), _ret[2].as<std::vector<double>>());
    }

    std::tuple<int64_t, std::string, bool, int64_t> simIK::getObjects(int64_t environmentHandle, int64_t index)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(environmentHandle);
        _args.push_back(index);
        auto _ret = this->_client->call("simIK.getObjects", _args);
        return std::make_tuple(_ret[0].as<int64_t>(), _ret[1].as<std::string>(), _ret[2].as<bool>(), _ret[3].as<int64_t>());
    }

    int64_t simIK::handleIkGroup(int64_t environmentHandle, std::optional<int64_t> ikGroupHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(environmentHandle);
        if(ikGroupHandle)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*ikGroupHandle);
        }
        else _brk = true;
        auto _ret = this->_client->call("simIK.handleIkGroup", _args);
        return _ret[0].as<int64_t>();
    }

    void simIK::load(int64_t environmentHandle, std::string data)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(environmentHandle);
        _args.push_back(data);
        auto _ret = this->_client->call("simIK.load", _args);
    }

    std::string simIK::save(int64_t environmentHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(environmentHandle);
        auto _ret = this->_client->call("simIK.save", _args);
        return _ret[0].as<std::string>();
    }

    void simIK::setIkElementBase(int64_t environmentHandle, int64_t ikGroupHandle, int64_t elementHandle, int64_t baseHandle, std::optional<int64_t> constraintsBaseHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(environmentHandle);
        _args.push_back(ikGroupHandle);
        _args.push_back(elementHandle);
        _args.push_back(baseHandle);
        if(constraintsBaseHandle)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*constraintsBaseHandle);
        }
        else _brk = true;
        auto _ret = this->_client->call("simIK.setIkElementBase", _args);
    }

    void simIK::setIkElementConstraints(int64_t environmentHandle, int64_t ikGroupHandle, int64_t elementHandle, int64_t constraints)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(environmentHandle);
        _args.push_back(ikGroupHandle);
        _args.push_back(elementHandle);
        _args.push_back(constraints);
        auto _ret = this->_client->call("simIK.setIkElementConstraints", _args);
    }

    void simIK::setIkElementFlags(int64_t environmentHandle, int64_t ikGroupHandle, int64_t elementHandle, int64_t flags)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(environmentHandle);
        _args.push_back(ikGroupHandle);
        _args.push_back(elementHandle);
        _args.push_back(flags);
        auto _ret = this->_client->call("simIK.setIkElementFlags", _args);
    }

    void simIK::setIkElementPrecision(int64_t environmentHandle, int64_t ikGroupHandle, int64_t elementHandle, std::vector<double> precision)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(environmentHandle);
        _args.push_back(ikGroupHandle);
        _args.push_back(elementHandle);
        _args.push_back(precision);
        auto _ret = this->_client->call("simIK.setIkElementPrecision", _args);
    }

    void simIK::setIkElementWeights(int64_t environmentHandle, int64_t ikGroupHandle, int64_t elementHandle, std::vector<double> weights)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(environmentHandle);
        _args.push_back(ikGroupHandle);
        _args.push_back(elementHandle);
        _args.push_back(weights);
        auto _ret = this->_client->call("simIK.setIkElementWeights", _args);
    }

    void simIK::setIkGroupCalculation(int64_t environmentHandle, int64_t ikGroupHandle, int64_t method, double damping, int64_t maxIterations)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(environmentHandle);
        _args.push_back(ikGroupHandle);
        _args.push_back(method);
        _args.push_back(damping);
        _args.push_back(maxIterations);
        auto _ret = this->_client->call("simIK.setIkGroupCalculation", _args);
    }

    void simIK::setIkGroupFlags(int64_t environmentHandle, int64_t ikGroupHandle, int64_t flags)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(environmentHandle);
        _args.push_back(ikGroupHandle);
        _args.push_back(flags);
        auto _ret = this->_client->call("simIK.setIkGroupFlags", _args);
    }

    void simIK::setJointDependency(int64_t environmentHandle, int64_t jointHandle, int64_t depJointHandle, std::optional<double> offset, std::optional<double> mult)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(environmentHandle);
        _args.push_back(jointHandle);
        _args.push_back(depJointHandle);
        if(offset)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*offset);
        }
        else _brk = true;
        if(mult)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*mult);
        }
        else _brk = true;
        auto _ret = this->_client->call("simIK.setJointDependency", _args);
    }

    void simIK::setJointIkWeight(int64_t environmentHandle, int64_t jointHandle, double weight)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(environmentHandle);
        _args.push_back(jointHandle);
        _args.push_back(weight);
        auto _ret = this->_client->call("simIK.setJointIkWeight", _args);
    }

    void simIK::setJointInterval(int64_t environmentHandle, int64_t jointHandle, bool cyclic, std::optional<std::vector<double>> interval)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(environmentHandle);
        _args.push_back(jointHandle);
        _args.push_back(cyclic);
        if(interval)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*interval);
        }
        else _brk = true;
        auto _ret = this->_client->call("simIK.setJointInterval", _args);
    }

    void simIK::setJointMaxStepSize(int64_t environmentHandle, int64_t jointHandle, double stepSize)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(environmentHandle);
        _args.push_back(jointHandle);
        _args.push_back(stepSize);
        auto _ret = this->_client->call("simIK.setJointMaxStepSize", _args);
    }

    void simIK::setJointMode(int64_t environmentHandle, int64_t jointHandle, int64_t jointMode)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(environmentHandle);
        _args.push_back(jointHandle);
        _args.push_back(jointMode);
        auto _ret = this->_client->call("simIK.setJointMode", _args);
    }

    void simIK::setJointPosition(int64_t environmentHandle, int64_t jointHandle, double position)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(environmentHandle);
        _args.push_back(jointHandle);
        _args.push_back(position);
        auto _ret = this->_client->call("simIK.setJointPosition", _args);
    }

    void simIK::setJointScrewPitch(int64_t environmentHandle, int64_t jointHandle, double pitch)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(environmentHandle);
        _args.push_back(jointHandle);
        _args.push_back(pitch);
        auto _ret = this->_client->call("simIK.setJointScrewPitch", _args);
    }

    void simIK::setLinkedDummy(int64_t environmentHandle, int64_t dummyHandle, int64_t linkedDummyHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(environmentHandle);
        _args.push_back(dummyHandle);
        _args.push_back(linkedDummyHandle);
        auto _ret = this->_client->call("simIK.setLinkedDummy", _args);
    }

    void simIK::setObjectMatrix(int64_t environmentHandle, int64_t objectHandle, int64_t relativeToObjectHandle, std::vector<double> matrix)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(environmentHandle);
        _args.push_back(objectHandle);
        _args.push_back(relativeToObjectHandle);
        _args.push_back(matrix);
        auto _ret = this->_client->call("simIK.setObjectMatrix", _args);
    }

    void simIK::setObjectParent(int64_t environmentHandle, int64_t objectHandle, int64_t parentObjectHandle, std::optional<bool> keepInPlace)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(environmentHandle);
        _args.push_back(objectHandle);
        _args.push_back(parentObjectHandle);
        if(keepInPlace)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*keepInPlace);
        }
        else _brk = true;
        auto _ret = this->_client->call("simIK.setObjectParent", _args);
    }

    void simIK::setObjectPose(int64_t environmentHandle, int64_t objectHandle, int64_t relativeToObjectHandle, std::vector<double> pose)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(environmentHandle);
        _args.push_back(objectHandle);
        _args.push_back(relativeToObjectHandle);
        _args.push_back(pose);
        auto _ret = this->_client->call("simIK.setObjectPose", _args);
    }

    void simIK::setObjectTransformation(int64_t environmentHandle, int64_t objectHandle, int64_t relativeToObjectHandle, std::vector<double> position, std::vector<double> eulerOrQuaternion)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(environmentHandle);
        _args.push_back(objectHandle);
        _args.push_back(relativeToObjectHandle);
        _args.push_back(position);
        _args.push_back(eulerOrQuaternion);
        auto _ret = this->_client->call("simIK.setObjectTransformation", _args);
    }

    void simIK::setSphericalJointMatrix(int64_t environmentHandle, int64_t jointHandle, std::vector<double> matrix)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(environmentHandle);
        _args.push_back(jointHandle);
        _args.push_back(matrix);
        auto _ret = this->_client->call("simIK.setSphericalJointMatrix", _args);
    }

    void simIK::setSphericalJointRotation(int64_t environmentHandle, int64_t jointHandle, std::vector<double> eulerOrQuaternion)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(environmentHandle);
        _args.push_back(jointHandle);
        _args.push_back(eulerOrQuaternion);
        auto _ret = this->_client->call("simIK.setSphericalJointRotation", _args);
    }

};

RemoteAPIObjects::RemoteAPIObjects(RemoteAPIClient *client)
{
    this->client = client;
}

RemoteAPIObject::sim RemoteAPIObjects::sim()
{
    return RemoteAPIObject::sim(this->client);
}

RemoteAPIObject::simIK RemoteAPIObjects::simIK()
{
    return RemoteAPIObject::simIK(this->client);
}

