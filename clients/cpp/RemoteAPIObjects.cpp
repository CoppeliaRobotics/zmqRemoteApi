namespace RemoteAPIObject
{
    sim::sim(RemoteAPIClient *client)
        : _client(client)
    {
        _client->require("sim");
    }

#include "sim-deprecated.cpp"
#include "sim-special.cpp"

    void sim::acquireLock()
    {
        bool _brk = false;
        json _args(json_array_arg);
        auto _ret = this->_client->call("sim.acquireLock", _args);
    }

    int64_t sim::addDrawingObject(int64_t objectType, double size, double duplicateTolerance, int64_t parentObjectHandle, int64_t maxItemCount, std::optional<std::vector<double>> color)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(objectType);
        _args.push_back(size);
        _args.push_back(duplicateTolerance);
        _args.push_back(parentObjectHandle);
        _args.push_back(maxItemCount);
        if(color)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*color);
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

    int64_t sim::addParticleObject(int64_t objectType, double size, double density, std::vector<double> params, double lifeTime, int64_t maxItemCount, std::optional<std::vector<double>> color)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(objectType);
        _args.push_back(size);
        _args.push_back(density);
        _args.push_back(params);
        _args.push_back(lifeTime);
        _args.push_back(maxItemCount);
        if(color)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*color);
        }
        else _brk = true;
        auto _ret = this->_client->call("sim.addParticleObject", _args);
        return _ret[0].as<int64_t>();
    }

    void sim::addParticleObjectItem(int64_t objectHandle, std::vector<double> itemData)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(objectHandle);
        _args.push_back(itemData);
        auto _ret = this->_client->call("sim.addParticleObjectItem", _args);
    }

    void sim::addReferencedHandle(int64_t objectHandle, int64_t referencedHandle, std::optional<std::string> tag, std::optional<json> opts)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(objectHandle);
        _args.push_back(referencedHandle);
        if(tag)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*tag);
        }
        else _brk = true;
        if(opts)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*opts);
        }
        else _brk = true;
        auto _ret = this->_client->call("sim.addReferencedHandle", _args);
    }

    int64_t sim::adjustView(int64_t viewHandleOrIndex, int64_t objectHandle, int64_t options, std::optional<std::string> viewLabel)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(viewHandleOrIndex);
        _args.push_back(objectHandle);
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

    int64_t sim::alignShapeBB(int64_t shapeHandle, std::vector<double> pose)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(shapeHandle);
        _args.push_back(pose);
        auto _ret = this->_client->call("sim.alignShapeBB", _args);
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

    bool sim::cancelScheduledExecution(int64_t id)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(id);
        auto _ret = this->_client->call("sim.cancelScheduledExecution", _args);
        return _ret[0].as<bool>();
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

    void sim::clearBufferSignal(std::string signalName)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(signalName);
        auto _ret = this->_client->call("sim.clearBufferSignal", _args);
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
        auto _ret = this->_client->call("sim.clearStringSignal", _args);
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

    int64_t sim::createScript(int64_t scriptType, std::string scriptString, std::optional<int64_t> options, std::optional<std::string> lang)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(scriptType);
        _args.push_back(scriptString);
        if(options)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*options);
        }
        else _brk = true;
        if(lang)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*lang);
        }
        else _brk = true;
        auto _ret = this->_client->call("sim.createScript", _args);
        return _ret[0].as<int64_t>();
    }

    int64_t sim::createShape(int64_t options, double shadingAngle, std::vector<double> vertices, std::vector<int64_t> indices, std::vector<double> normals, std::vector<double> textureCoordinates, std::vector<uint8_t> texture, std::vector<int64_t> textureResolution)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(options);
        _args.push_back(shadingAngle);
        _args.push_back(vertices);
        _args.push_back(indices);
        _args.push_back(normals);
        _args.push_back(textureCoordinates);
        _args.push_back(bin(texture));
        _args.push_back(textureResolution);
        auto _ret = this->_client->call("sim.createShape", _args);
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

    std::tuple<int64_t, json> sim::executeScriptString(std::string stringToExecute, int64_t scriptHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(stringToExecute);
        _args.push_back(scriptHandle);
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

    std::vector<std::string> sim::getApiFunc(int64_t scriptHandle, std::string apiWord)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(scriptHandle);
        _args.push_back(apiWord);
        auto _ret = this->_client->call("sim.getApiFunc", _args);
        return _ret[0].as<std::vector<std::string>>();
    }

    std::string sim::getApiInfo(int64_t scriptHandle, std::string apiWord)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(scriptHandle);
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

    double sim::getAutoYieldDelay()
    {
        bool _brk = false;
        json _args(json_array_arg);
        auto _ret = this->_client->call("sim.getAutoYieldDelay", _args);
        return _ret[0].as<double>();
    }

    bool sim::getBoolParam(int64_t parameter)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(parameter);
        auto _ret = this->_client->call("sim.getBoolParam", _args);
        return _ret[0].as<bool>();
    }

    bool sim::getBoolProperty(int64_t target, std::string pName, std::optional<json> options)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(target);
        _args.push_back(pName);
        if(options)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*options);
        }
        else _brk = true;
        auto _ret = this->_client->call("sim.getBoolProperty", _args);
        return _ret[0].as<bool>();
    }

    std::vector<uint8_t> sim::getBufferProperty(int64_t target, std::string pName, std::optional<json> options)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(target);
        _args.push_back(pName);
        if(options)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*options);
        }
        else _brk = true;
        auto _ret = this->_client->call("sim.getBufferProperty", _args);
        return _ret[0].as<std::vector<uint8_t>>();
    }

    std::vector<uint8_t> sim::getBufferSignal(std::string signalName)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(signalName);
        auto _ret = this->_client->call("sim.getBufferSignal", _args);
        return _ret[0].as<std::vector<uint8_t>>();
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

    std::vector<double> sim::getColorProperty(int64_t target, std::string pName, std::optional<json> options)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(target);
        _args.push_back(pName);
        if(options)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*options);
        }
        else _brk = true;
        auto _ret = this->_client->call("sim.getColorProperty", _args);
        return _ret[0].as<std::vector<double>>();
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

    std::vector<double> sim::getFloatArrayProperty(int64_t target, std::string pName, std::optional<json> options)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(target);
        _args.push_back(pName);
        if(options)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*options);
        }
        else _brk = true;
        auto _ret = this->_client->call("sim.getFloatArrayProperty", _args);
        return _ret[0].as<std::vector<double>>();
    }

    double sim::getFloatParam(int64_t parameter)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(parameter);
        auto _ret = this->_client->call("sim.getFloatParam", _args);
        return _ret[0].as<double>();
    }

    double sim::getFloatProperty(int64_t target, std::string pName, std::optional<json> options)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(target);
        _args.push_back(pName);
        if(options)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*options);
        }
        else _brk = true;
        auto _ret = this->_client->call("sim.getFloatProperty", _args);
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

    std::tuple<int64_t, std::vector<double>, std::vector<double>> sim::getGraphInfo(int64_t graphHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(graphHandle);
        auto _ret = this->_client->call("sim.getGraphInfo", _args);
        return std::make_tuple(_ret[0].as<int64_t>(), _ret[1].as<std::vector<double>>(), _ret[2].as<std::vector<double>>());
    }

    int64_t sim::getInt32Param(int64_t parameter)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(parameter);
        auto _ret = this->_client->call("sim.getInt32Param", _args);
        return _ret[0].as<int64_t>();
    }

    std::vector<int64_t> sim::getIntArray2Property(int64_t target, std::string pName, std::optional<json> options)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(target);
        _args.push_back(pName);
        if(options)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*options);
        }
        else _brk = true;
        auto _ret = this->_client->call("sim.getIntArray2Property", _args);
        return _ret[0].as<std::vector<int64_t>>();
    }

    std::vector<int64_t> sim::getIntArrayProperty(int64_t target, std::string pName, std::optional<json> options)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(target);
        _args.push_back(pName);
        if(options)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*options);
        }
        else _brk = true;
        auto _ret = this->_client->call("sim.getIntArrayProperty", _args);
        return _ret[0].as<std::vector<int64_t>>();
    }

    int64_t sim::getIntProperty(int64_t target, std::string pName, std::optional<json> options)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(target);
        _args.push_back(pName);
        if(options)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*options);
        }
        else _brk = true;
        auto _ret = this->_client->call("sim.getIntProperty", _args);
        return _ret[0].as<int64_t>();
    }

    int64_t sim::getIsRealTimeSimulation()
    {
        bool _brk = false;
        json _args(json_array_arg);
        auto _ret = this->_client->call("sim.getIsRealTimeSimulation", _args);
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

    std::string sim::getLastInfo()
    {
        bool _brk = false;
        json _args(json_array_arg);
        auto _ret = this->_client->call("sim.getLastInfo", _args);
        return _ret[0].as<std::string>();
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

    int64_t sim::getLongProperty(int64_t target, std::string pName, std::optional<json> options)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(target);
        _args.push_back(pName);
        if(options)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*options);
        }
        else _brk = true;
        auto _ret = this->_client->call("sim.getLongProperty", _args);
        return _ret[0].as<int64_t>();
    }

    std::vector<double> sim::getMatrixInverse(std::vector<double> matrix)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(matrix);
        auto _ret = this->_client->call("sim.getMatrixInverse", _args);
        return _ret[0].as<std::vector<double>>();
    }

    int64_t sim::getModelProperty(int64_t objectHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(objectHandle);
        auto _ret = this->_client->call("sim.getModelProperty", _args);
        return _ret[0].as<int64_t>();
    }

    bool sim::getNamedBoolParam(std::string name)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(name);
        auto _ret = this->_client->call("sim.getNamedBoolParam", _args);
        return _ret[0].as<bool>();
    }

    double sim::getNamedFloatParam(std::string name)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(name);
        auto _ret = this->_client->call("sim.getNamedFloatParam", _args);
        return _ret[0].as<double>();
    }

    int64_t sim::getNamedInt32Param(std::string name)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(name);
        auto _ret = this->_client->call("sim.getNamedInt32Param", _args);
        return _ret[0].as<int64_t>();
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

    std::string sim::getObjectAliasRelative(int64_t handle, int64_t baseHandle, std::optional<int64_t> options)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(handle);
        _args.push_back(baseHandle);
        if(options)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*options);
        }
        else _brk = true;
        auto _ret = this->_client->call("sim.getObjectAliasRelative", _args);
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

    std::vector<double> sim::getObjectFloatArrayParam(int64_t objectHandle, int64_t parameterID)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(objectHandle);
        _args.push_back(parameterID);
        auto _ret = this->_client->call("sim.getObjectFloatArrayParam", _args);
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

    std::tuple<int64_t, int64_t> sim::getObjectHierarchyOrder(int64_t objectHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(objectHandle);
        auto _ret = this->_client->call("sim.getObjectHierarchyOrder", _args);
        return std::make_tuple(_ret[0].as<int64_t>(), _ret[1].as<int64_t>());
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

    std::vector<double> sim::getObjectMatrix(int64_t objectHandle, std::optional<int64_t> relativeToObjectHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(objectHandle);
        if(relativeToObjectHandle)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*relativeToObjectHandle);
        }
        else _brk = true;
        auto _ret = this->_client->call("sim.getObjectMatrix", _args);
        return _ret[0].as<std::vector<double>>();
    }

    std::vector<double> sim::getObjectOrientation(int64_t objectHandle, std::optional<int64_t> relativeToObjectHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(objectHandle);
        if(relativeToObjectHandle)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*relativeToObjectHandle);
        }
        else _brk = true;
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

    std::vector<double> sim::getObjectPose(int64_t objectHandle, std::optional<int64_t> relativeToObjectHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(objectHandle);
        if(relativeToObjectHandle)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*relativeToObjectHandle);
        }
        else _brk = true;
        auto _ret = this->_client->call("sim.getObjectPose", _args);
        return _ret[0].as<std::vector<double>>();
    }

    std::vector<double> sim::getObjectPosition(int64_t objectHandle, std::optional<int64_t> relativeToObjectHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(objectHandle);
        if(relativeToObjectHandle)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*relativeToObjectHandle);
        }
        else _brk = true;
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

    std::vector<double> sim::getObjectQuaternion(int64_t objectHandle, std::optional<int64_t> relativeToObjectHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(objectHandle);
        if(relativeToObjectHandle)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*relativeToObjectHandle);
        }
        else _brk = true;
        auto _ret = this->_client->call("sim.getObjectQuaternion", _args);
        return _ret[0].as<std::vector<double>>();
    }

    std::vector<int64_t> sim::getObjectSel()
    {
        bool _brk = false;
        json _args(json_array_arg);
        auto _ret = this->_client->call("sim.getObjectSel", _args);
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

    std::string sim::getPluginInfo(std::string pluginName, int64_t infoType)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(pluginName);
        _args.push_back(infoType);
        auto _ret = this->_client->call("sim.getPluginInfo", _args);
        return _ret[0].as<std::string>();
    }

    std::string sim::getPluginName(int64_t index)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(index);
        auto _ret = this->_client->call("sim.getPluginName", _args);
        return _ret[0].as<std::string>();
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

    std::vector<double> sim::getPoseInverse(std::vector<double> pose)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(pose);
        auto _ret = this->_client->call("sim.getPoseInverse", _args);
        return _ret[0].as<std::vector<double>>();
    }

    std::vector<double> sim::getPoseProperty(int64_t target, std::string pName, std::optional<json> options)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(target);
        _args.push_back(pName);
        if(options)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*options);
        }
        else _brk = true;
        auto _ret = this->_client->call("sim.getPoseProperty", _args);
        return _ret[0].as<std::vector<double>>();
    }

    json sim::getProperties(int64_t target, std::optional<json> opts)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(target);
        if(opts)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*opts);
        }
        else _brk = true;
        auto _ret = this->_client->call("sim.getProperties", _args);
        return _ret[0].as<json>();
    }

    json sim::getPropertiesInfos(int64_t target, std::optional<json> opts)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(target);
        if(opts)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*opts);
        }
        else _brk = true;
        auto _ret = this->_client->call("sim.getPropertiesInfos", _args);
        return _ret[0].as<json>();
    }

    json sim::getProperty(int64_t target, std::string pName, std::optional<json> options)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(target);
        _args.push_back(pName);
        if(options)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*options);
        }
        else _brk = true;
        auto _ret = this->_client->call("sim.getProperty", _args);
        return _ret[0].as<json>();
    }

    std::tuple<int64_t, int64_t, std::string> sim::getPropertyInfo(int64_t target, std::string pName, std::optional<json> options)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(target);
        _args.push_back(pName);
        if(options)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*options);
        }
        else _brk = true;
        auto _ret = this->_client->call("sim.getPropertyInfo", _args);
        return std::make_tuple(_ret[0].as<int64_t>(), _ret[1].as<int64_t>(), _ret[2].as<std::string>());
    }

    std::tuple<std::string, std::string> sim::getPropertyName(int64_t target, int64_t index, std::optional<json> options)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(target);
        _args.push_back(index);
        if(options)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*options);
        }
        else _brk = true;
        auto _ret = this->_client->call("sim.getPropertyName", _args);
        return std::make_tuple(_ret[0].as<std::string>(), _ret[1].as<std::string>());
    }

    std::string sim::getPropertyTypeString(int64_t pType)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(pType);
        auto _ret = this->_client->call("sim.getPropertyTypeString", _args);
        return _ret[0].as<std::string>();
    }

    std::vector<double> sim::getQuaternionProperty(int64_t target, std::string pName, std::optional<json> options)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(target);
        _args.push_back(pName);
        if(options)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*options);
        }
        else _brk = true;
        auto _ret = this->_client->call("sim.getQuaternionProperty", _args);
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

    bool sim::getRealTimeSimulation()
    {
        bool _brk = false;
        json _args(json_array_arg);
        auto _ret = this->_client->call("sim.getRealTimeSimulation", _args);
        return _ret[0].as<bool>();
    }

    int64_t sim::getReferencedHandle(int64_t objectHandle, std::optional<std::string> tag)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(objectHandle);
        if(tag)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*tag);
        }
        else _brk = true;
        auto _ret = this->_client->call("sim.getReferencedHandle", _args);
        return _ret[0].as<int64_t>();
    }

    std::vector<int64_t> sim::getReferencedHandles(int64_t objectHandle, std::optional<std::string> tag)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(objectHandle);
        if(tag)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*tag);
        }
        else _brk = true;
        auto _ret = this->_client->call("sim.getReferencedHandles", _args);
        return _ret[0].as<std::vector<int64_t>>();
    }

    std::vector<std::string> sim::getReferencedHandlesTags(int64_t objectHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(objectHandle);
        auto _ret = this->_client->call("sim.getReferencedHandlesTags", _args);
        return _ret[0].as<std::vector<std::string>>();
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

    int64_t sim::getScript(int64_t scriptType, std::optional<std::string> scriptName)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(scriptType);
        if(scriptName)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*scriptName);
        }
        else _brk = true;
        auto _ret = this->_client->call("sim.getScript", _args);
        return _ret[0].as<int64_t>();
    }

    json sim::getScriptFunctions(int64_t scriptHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(scriptHandle);
        auto _ret = this->_client->call("sim.getScriptFunctions", _args);
        return _ret[0].as<json>();
    }

    bool sim::getSettingBool(std::string key)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(key);
        auto _ret = this->_client->call("sim.getSettingBool", _args);
        return _ret[0].as<bool>();
    }

    double sim::getSettingFloat(std::string key)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(key);
        auto _ret = this->_client->call("sim.getSettingFloat", _args);
        return _ret[0].as<double>();
    }

    int64_t sim::getSettingInt32(std::string key)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(key);
        auto _ret = this->_client->call("sim.getSettingInt32", _args);
        return _ret[0].as<int64_t>();
    }

    std::string sim::getSettingString(std::string key)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(key);
        auto _ret = this->_client->call("sim.getSettingString", _args);
        return _ret[0].as<std::string>();
    }

    json sim::getShapeAppearance(int64_t handle, std::optional<json> opts)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(handle);
        if(opts)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*opts);
        }
        else _brk = true;
        auto _ret = this->_client->call("sim.getShapeAppearance", _args);
        return _ret[0].as<json>();
    }

    std::tuple<std::vector<double>, std::vector<double>> sim::getShapeBB(int64_t shapeHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(shapeHandle);
        auto _ret = this->_client->call("sim.getShapeBB", _args);
        return std::make_tuple(_ret[0].as<std::vector<double>>(), _ret[1].as<std::vector<double>>());
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
        auto _ret = this->_client->call("sim.getShapeMass", _args);
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

    bool sim::getSimulationStopping()
    {
        bool _brk = false;
        json _args(json_array_arg);
        auto _ret = this->_client->call("sim.getSimulationStopping", _args);
        return _ret[0].as<bool>();
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

    std::string sim::getStringProperty(int64_t target, std::string pName, std::optional<json> options)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(target);
        _args.push_back(pName);
        if(options)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*options);
        }
        else _brk = true;
        auto _ret = this->_client->call("sim.getStringProperty", _args);
        return _ret[0].as<std::string>();
    }

    double sim::getSystemTime()
    {
        bool _brk = false;
        json _args(json_array_arg);
        auto _ret = this->_client->call("sim.getSystemTime", _args);
        return _ret[0].as<double>();
    }

    json sim::getTableProperty(int64_t target, std::string pName, std::optional<json> options)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(target);
        _args.push_back(pName);
        if(options)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*options);
        }
        else _brk = true;
        auto _ret = this->_client->call("sim.getTableProperty", _args);
        return _ret[0].as<json>();
    }

    std::tuple<int64_t, std::vector<int64_t>> sim::getTextureId(std::string textureName)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(textureName);
        auto _ret = this->_client->call("sim.getTextureId", _args);
        return std::make_tuple(_ret[0].as<int64_t>(), _ret[1].as<std::vector<int64_t>>());
    }

    int64_t sim::getThreadId()
    {
        bool _brk = false;
        json _args(json_array_arg);
        auto _ret = this->_client->call("sim.getThreadId", _args);
        return _ret[0].as<int64_t>();
    }

    std::vector<std::string> sim::getUserVariables()
    {
        bool _brk = false;
        json _args(json_array_arg);
        auto _ret = this->_client->call("sim.getUserVariables", _args);
        return _ret[0].as<std::vector<std::string>>();
    }

    std::vector<double> sim::getVector2Property(int64_t target, std::string pName, std::optional<json> options)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(target);
        _args.push_back(pName);
        if(options)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*options);
        }
        else _brk = true;
        auto _ret = this->_client->call("sim.getVector2Property", _args);
        return _ret[0].as<std::vector<double>>();
    }

    std::vector<double> sim::getVector3Property(int64_t target, std::string pName, std::optional<json> options)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(target);
        _args.push_back(pName);
        if(options)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*options);
        }
        else _brk = true;
        auto _ret = this->_client->call("sim.getVector3Property", _args);
        return _ret[0].as<std::vector<double>>();
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

    void sim::getVisionSensorRes(int64_t sensorHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(sensorHandle);
        auto _ret = this->_client->call("sim.getVisionSensorRes", _args);
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

    int64_t sim::handleDynamics(double deltaTime)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(deltaTime);
        auto _ret = this->_client->call("sim.handleDynamics", _args);
        return _ret[0].as<int64_t>();
    }

    int64_t sim::handleEmbeddedScripts(int64_t callType)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(callType);
        auto _ret = this->_client->call("sim.handleEmbeddedScripts", _args);
        return _ret[0].as<int64_t>();
    }

    void sim::handleExtCalls()
    {
        bool _brk = false;
        json _args(json_array_arg);
        auto _ret = this->_client->call("sim.handleExtCalls", _args);
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

    int64_t sim::handleSimulationScripts(int64_t callType)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(callType);
        auto _ret = this->_client->call("sim.handleSimulationScripts", _args);
        return _ret[0].as<int64_t>();
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

    void sim::initScript(int64_t scriptHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(scriptHandle);
        auto _ret = this->_client->call("sim.initScript", _args);
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

    std::vector<double> sim::interpolatePoses(std::vector<double> poseIn1, std::vector<double> poseIn2, double interpolFactor)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(poseIn1);
        _args.push_back(poseIn2);
        _args.push_back(interpolFactor);
        auto _ret = this->_client->call("sim.interpolatePoses", _args);
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

    void sim::loadScene(std::string filename)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(filename);
        auto _ret = this->_client->call("sim.loadScene", _args);
    }

    std::vector<double> sim::matrixToPose(std::vector<double> matrix)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(matrix);
        auto _ret = this->_client->call("sim.matrixToPose", _args);
        return _ret[0].as<std::vector<double>>();
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

    json sim::moveToConfig(json params)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(params);
        auto _ret = this->_client->call("sim.moveToConfig", _args);
        return _ret[0].as<json>();
    }

    void sim::moveToConfig_cleanup(json motionObject)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(motionObject);
        auto _ret = this->_client->call("sim.moveToConfig_cleanup", _args);
    }

    json sim::moveToConfig_init(json params)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(params);
        auto _ret = this->_client->call("sim.moveToConfig_init", _args);
        return _ret[0].as<json>();
    }

    std::tuple<int64_t, json> sim::moveToConfig_step(json motionObject)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(motionObject);
        auto _ret = this->_client->call("sim.moveToConfig_step", _args);
        return std::make_tuple(_ret[0].as<int64_t>(), _ret[1].as<json>());
    }

    json sim::moveToPose(json params)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(params);
        auto _ret = this->_client->call("sim.moveToPose", _args);
        return _ret[0].as<json>();
    }

    void sim::moveToPose_cleanup(json motionObject)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(motionObject);
        auto _ret = this->_client->call("sim.moveToPose_cleanup", _args);
    }

    json sim::moveToPose_init(json params)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(params);
        auto _ret = this->_client->call("sim.moveToPose_init", _args);
        return _ret[0].as<json>();
    }

    std::tuple<int64_t, json> sim::moveToPose_step(json motionObject)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(motionObject);
        auto _ret = this->_client->call("sim.moveToPose_step", _args);
        return std::make_tuple(_ret[0].as<int64_t>(), _ret[1].as<json>());
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

    std::vector<double> sim::multiplyPoses(std::vector<double> poseIn1, std::vector<double> poseIn2)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(poseIn1);
        _args.push_back(poseIn2);
        auto _ret = this->_client->call("sim.multiplyPoses", _args);
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

    void sim::pauseSimulation()
    {
        bool _brk = false;
        json _args(json_array_arg);
        auto _ret = this->_client->call("sim.pauseSimulation", _args);
    }

    std::vector<double> sim::poseToMatrix(std::vector<double> pose)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(pose);
        auto _ret = this->_client->call("sim.poseToMatrix", _args);
        return _ret[0].as<std::vector<double>>();
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

    std::vector<uint8_t> sim::readCustomBufferData(int64_t objectHandle, std::string tagName)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(objectHandle);
        _args.push_back(tagName);
        auto _ret = this->_client->call("sim.readCustomBufferData", _args);
        return _ret[0].as<std::vector<uint8_t>>();
    }

    std::vector<std::string> sim::readCustomDataTags(int64_t objectHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(objectHandle);
        auto _ret = this->_client->call("sim.readCustomDataTags", _args);
        return _ret[0].as<std::vector<std::string>>();
    }

    std::string sim::readCustomStringData(int64_t objectHandle, std::string tagName)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(objectHandle);
        _args.push_back(tagName);
        auto _ret = this->_client->call("sim.readCustomStringData", _args);
        return _ret[0].as<std::string>();
    }

    json sim::readCustomTableData(int64_t handle, std::string tagName, std::optional<json> options)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(handle);
        _args.push_back(tagName);
        if(options)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*options);
        }
        else _brk = true;
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

    void sim::releaseLock()
    {
        bool _brk = false;
        json _args(json_array_arg);
        auto _ret = this->_client->call("sim.releaseLock", _args);
    }

    int64_t sim::relocateShapeFrame(int64_t shapeHandle, std::vector<double> pose)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(shapeHandle);
        _args.push_back(pose);
        auto _ret = this->_client->call("sim.relocateShapeFrame", _args);
        return _ret[0].as<int64_t>();
    }

    void sim::removeDrawingObject(int64_t drawingObjectHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(drawingObjectHandle);
        auto _ret = this->_client->call("sim.removeDrawingObject", _args);
    }

    int64_t sim::removeModel(int64_t objectHandle, std::optional<bool> delayedRemoval)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(objectHandle);
        if(delayedRemoval)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*delayedRemoval);
        }
        else _brk = true;
        auto _ret = this->_client->call("sim.removeModel", _args);
        return _ret[0].as<int64_t>();
    }

    void sim::removeObjects(std::vector<int64_t> objectHandles, std::optional<bool> delayedRemoval)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(objectHandles);
        if(delayedRemoval)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*delayedRemoval);
        }
        else _brk = true;
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

    void sim::removeProperty(int64_t target, std::string pName, std::optional<json> options)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(target);
        _args.push_back(pName);
        if(options)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*options);
        }
        else _brk = true;
        auto _ret = this->_client->call("sim.removeProperty", _args);
    }

    void sim::removeReferencedObjects(int64_t objectHandle, std::optional<std::string> tag)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(objectHandle);
        if(tag)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*tag);
        }
        else _brk = true;
        auto _ret = this->_client->call("sim.removeReferencedObjects", _args);
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

    int64_t sim::scheduleExecution(std::string f, std::vector<json> args, double timePoint, std::optional<bool> simTime)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(f);
        _args.push_back(args);
        _args.push_back(timePoint);
        if(simTime)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*simTime);
        }
        else _brk = true;
        auto _ret = this->_client->call("sim.scheduleExecution", _args);
        return _ret[0].as<int64_t>();
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

    void sim::setAutoYieldDelay(double dt)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(dt);
        auto _ret = this->_client->call("sim.setAutoYieldDelay", _args);
    }

    void sim::setBoolParam(int64_t parameter, bool boolState)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(parameter);
        _args.push_back(boolState);
        auto _ret = this->_client->call("sim.setBoolParam", _args);
    }

    void sim::setBoolProperty(int64_t target, std::string pName, bool pValue, std::optional<json> options)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(target);
        _args.push_back(pName);
        _args.push_back(pValue);
        if(options)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*options);
        }
        else _brk = true;
        auto _ret = this->_client->call("sim.setBoolProperty", _args);
    }

    void sim::setBufferProperty(int64_t target, std::string pName, std::vector<uint8_t> pValue, std::optional<json> options)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(target);
        _args.push_back(pName);
        _args.push_back(bin(pValue));
        if(options)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*options);
        }
        else _brk = true;
        auto _ret = this->_client->call("sim.setBufferProperty", _args);
    }

    void sim::setBufferSignal(std::string signalName, std::vector<uint8_t> signalValue)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(signalName);
        _args.push_back(bin(signalValue));
        auto _ret = this->_client->call("sim.setBufferSignal", _args);
    }

    void sim::setColorProperty(int64_t target, std::string pName, std::vector<double> pValue, std::optional<json> options)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(target);
        _args.push_back(pName);
        _args.push_back(pValue);
        if(options)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*options);
        }
        else _brk = true;
        auto _ret = this->_client->call("sim.setColorProperty", _args);
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

    void sim::setEventFilters(std::optional<json> filters)
    {
        bool _brk = false;
        json _args(json_array_arg);
        if(filters)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*filters);
        }
        else _brk = true;
        auto _ret = this->_client->call("sim.setEventFilters", _args);
    }

    void sim::setExplicitHandling(int64_t objectHandle, int64_t explicitHandlingFlags)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(objectHandle);
        _args.push_back(explicitHandlingFlags);
        auto _ret = this->_client->call("sim.setExplicitHandling", _args);
    }

    void sim::setFloatArrayProperty(int64_t target, std::string pName, std::vector<double> pValue, std::optional<json> options)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(target);
        _args.push_back(pName);
        _args.push_back(pValue);
        if(options)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*options);
        }
        else _brk = true;
        auto _ret = this->_client->call("sim.setFloatArrayProperty", _args);
    }

    void sim::setFloatParam(int64_t parameter, double floatState)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(parameter);
        _args.push_back(floatState);
        auto _ret = this->_client->call("sim.setFloatParam", _args);
    }

    void sim::setFloatProperty(int64_t target, std::string pName, double pValue, std::optional<json> options)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(target);
        _args.push_back(pName);
        _args.push_back(pValue);
        if(options)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*options);
        }
        else _brk = true;
        auto _ret = this->_client->call("sim.setFloatProperty", _args);
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

    void sim::setIntArray2Property(int64_t target, std::string pName, std::vector<int64_t> pValue, std::optional<json> options)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(target);
        _args.push_back(pName);
        _args.push_back(pValue);
        if(options)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*options);
        }
        else _brk = true;
        auto _ret = this->_client->call("sim.setIntArray2Property", _args);
    }

    void sim::setIntArrayProperty(int64_t target, std::string pName, std::vector<int64_t> pValue, std::optional<json> options)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(target);
        _args.push_back(pName);
        _args.push_back(pValue);
        if(options)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*options);
        }
        else _brk = true;
        auto _ret = this->_client->call("sim.setIntArrayProperty", _args);
    }

    void sim::setIntProperty(int64_t target, std::string pName, int64_t pValue, std::optional<json> options)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(target);
        _args.push_back(pName);
        _args.push_back(pValue);
        if(options)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*options);
        }
        else _brk = true;
        auto _ret = this->_client->call("sim.setIntProperty", _args);
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

    void sim::setJointTargetPosition(int64_t objectHandle, double targetPosition, std::optional<std::vector<double>> motionParams)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(objectHandle);
        _args.push_back(targetPosition);
        if(motionParams)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*motionParams);
        }
        else _brk = true;
        auto _ret = this->_client->call("sim.setJointTargetPosition", _args);
    }

    void sim::setJointTargetVelocity(int64_t objectHandle, double targetVelocity, std::optional<std::vector<double>> motionParams)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(objectHandle);
        _args.push_back(targetVelocity);
        if(motionParams)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*motionParams);
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

    void sim::setLongProperty(int64_t target, std::string pName, int64_t pValue, std::optional<json> options)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(target);
        _args.push_back(pName);
        _args.push_back(pValue);
        if(options)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*options);
        }
        else _brk = true;
        auto _ret = this->_client->call("sim.setLongProperty", _args);
    }

    void sim::setModelProperty(int64_t objectHandle, int64_t property)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(objectHandle);
        _args.push_back(property);
        auto _ret = this->_client->call("sim.setModelProperty", _args);
    }

    void sim::setNamedBoolParam(std::string name, bool value)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(name);
        _args.push_back(value);
        auto _ret = this->_client->call("sim.setNamedBoolParam", _args);
    }

    void sim::setNamedFloatParam(std::string name, double value)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(name);
        _args.push_back(value);
        auto _ret = this->_client->call("sim.setNamedFloatParam", _args);
    }

    void sim::setNamedInt32Param(std::string name, int64_t value)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(name);
        _args.push_back(value);
        auto _ret = this->_client->call("sim.setNamedInt32Param", _args);
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

    void sim::setObjectFloatArrayParam(int64_t objectHandle, int64_t parameterID, std::vector<double> params)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(objectHandle);
        _args.push_back(parameterID);
        _args.push_back(params);
        auto _ret = this->_client->call("sim.setObjectFloatArrayParam", _args);
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

    void sim::setObjectHierarchyOrder(int64_t objectHandle, int64_t order)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(objectHandle);
        _args.push_back(order);
        auto _ret = this->_client->call("sim.setObjectHierarchyOrder", _args);
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

    void sim::setObjectMatrix(int64_t objectHandle, std::vector<double> matrix, std::optional<int64_t> relativeToObjectHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(objectHandle);
        _args.push_back(matrix);
        if(relativeToObjectHandle)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*relativeToObjectHandle);
        }
        else _brk = true;
        auto _ret = this->_client->call("sim.setObjectMatrix", _args);
    }

    void sim::setObjectOrientation(int64_t objectHandle, std::vector<double> eulerAngles, std::optional<int64_t> relativeToObjectHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(objectHandle);
        _args.push_back(eulerAngles);
        if(relativeToObjectHandle)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*relativeToObjectHandle);
        }
        else _brk = true;
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

    void sim::setObjectPose(int64_t objectHandle, std::vector<double> pose, std::optional<int64_t> relativeToObjectHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(objectHandle);
        _args.push_back(pose);
        if(relativeToObjectHandle)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*relativeToObjectHandle);
        }
        else _brk = true;
        auto _ret = this->_client->call("sim.setObjectPose", _args);
    }

    void sim::setObjectPosition(int64_t objectHandle, std::vector<double> position, std::optional<int64_t> relativeToObjectHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(objectHandle);
        _args.push_back(position);
        if(relativeToObjectHandle)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*relativeToObjectHandle);
        }
        else _brk = true;
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

    void sim::setObjectQuaternion(int64_t objectHandle, std::vector<double> quaternion, std::optional<int64_t> relativeToObjectHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(objectHandle);
        _args.push_back(quaternion);
        if(relativeToObjectHandle)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*relativeToObjectHandle);
        }
        else _brk = true;
        auto _ret = this->_client->call("sim.setObjectQuaternion", _args);
    }

    void sim::setObjectSel(std::vector<int64_t> objectHandles)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(objectHandles);
        auto _ret = this->_client->call("sim.setObjectSel", _args);
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

    void sim::setPluginInfo(std::string pluginName, int64_t infoType, std::string info)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(pluginName);
        _args.push_back(infoType);
        _args.push_back(info);
        auto _ret = this->_client->call("sim.setPluginInfo", _args);
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

    void sim::setPoseProperty(int64_t target, std::string pName, std::vector<double> pValue, std::optional<json> options)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(target);
        _args.push_back(pName);
        _args.push_back(pValue);
        if(options)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*options);
        }
        else _brk = true;
        auto _ret = this->_client->call("sim.setPoseProperty", _args);
    }

    void sim::setProperties(int64_t target, json props)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(target);
        _args.push_back(props);
        auto _ret = this->_client->call("sim.setProperties", _args);
    }

    void sim::setProperty(int64_t target, std::string pName, json pValue, std::optional<int64_t> pType)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(target);
        _args.push_back(pName);
        _args.push_back(pValue);
        if(pType)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*pType);
        }
        else _brk = true;
        auto _ret = this->_client->call("sim.setProperty", _args);
    }

    void sim::setQuaternionProperty(int64_t target, std::string pName, std::vector<double> pValue, std::optional<json> options)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(target);
        _args.push_back(pName);
        _args.push_back(pValue);
        if(options)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*options);
        }
        else _brk = true;
        auto _ret = this->_client->call("sim.setQuaternionProperty", _args);
    }

    void sim::setReferencedHandles(int64_t objectHandle, std::vector<int64_t> referencedHandles, std::optional<std::string> tag)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(objectHandle);
        _args.push_back(referencedHandles);
        if(tag)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*tag);
        }
        else _brk = true;
        auto _ret = this->_client->call("sim.setReferencedHandles", _args);
    }

    int64_t sim::setShapeAppearance(int64_t handle, json savedData, std::optional<json> opts)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(handle);
        _args.push_back(savedData);
        if(opts)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*opts);
        }
        else _brk = true;
        auto _ret = this->_client->call("sim.setShapeAppearance", _args);
        return _ret[0].as<int64_t>();
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

    void sim::setShapeInertia(int64_t shapeHandle, std::vector<double> inertiaMatrix, std::vector<double> comMatrix)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(shapeHandle);
        _args.push_back(inertiaMatrix);
        _args.push_back(comMatrix);
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

    int64_t sim::setStepping(bool enabled)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(enabled);
        auto _ret = this->_client->call("sim.setStepping", _args);
        return _ret[0].as<int64_t>();
    }

    void sim::setStringParam(int64_t parameter, std::string stringState)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(parameter);
        _args.push_back(stringState);
        auto _ret = this->_client->call("sim.setStringParam", _args);
    }

    void sim::setStringProperty(int64_t target, std::string pName, std::string pValue, std::optional<json> options)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(target);
        _args.push_back(pName);
        _args.push_back(pValue);
        if(options)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*options);
        }
        else _brk = true;
        auto _ret = this->_client->call("sim.setStringProperty", _args);
    }

    void sim::setStringSignal(std::string signalName, std::string signalValue)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(signalName);
        _args.push_back(signalValue);
        auto _ret = this->_client->call("sim.setStringSignal", _args);
    }

    void sim::setTableProperty(int64_t target, std::string pName, json pValue, std::optional<json> options)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(target);
        _args.push_back(pName);
        _args.push_back(pValue);
        if(options)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*options);
        }
        else _brk = true;
        auto _ret = this->_client->call("sim.setTableProperty", _args);
    }

    void sim::setVector2Property(int64_t target, std::string pName, std::vector<double> pValue, std::optional<json> options)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(target);
        _args.push_back(pName);
        _args.push_back(pValue);
        if(options)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*options);
        }
        else _brk = true;
        auto _ret = this->_client->call("sim.setVector2Property", _args);
    }

    void sim::setVector3Property(int64_t target, std::string pName, std::vector<double> pValue, std::optional<json> options)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(target);
        _args.push_back(pName);
        _args.push_back(pValue);
        if(options)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*options);
        }
        else _brk = true;
        auto _ret = this->_client->call("sim.setVector3Property", _args);
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

    void sim::startSimulation()
    {
        bool _brk = false;
        json _args(json_array_arg);
        auto _ret = this->_client->call("sim.startSimulation", _args);
    }

    void sim::step()
    {
        bool _brk = false;
        json _args(json_array_arg);
        auto _ret = this->_client->call("sim.step", _args);
    }

    void sim::stopSimulation(std::optional<bool> wait)
    {
        bool _brk = false;
        json _args(json_array_arg);
        if(wait)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*wait);
        }
        else _brk = true;
        auto _ret = this->_client->call("sim.stopSimulation", _args);
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

    void sim::systemSemaphore(std::string key, bool acquire)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(key);
        _args.push_back(acquire);
        auto _ret = this->_client->call("sim.systemSemaphore", _args);
    }

    int64_t sim::testCB(int64_t a, std::string cb, int64_t b)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(a);
        _args.push_back(cb);
        _args.push_back(b);
        auto _ret = this->_client->call("sim.testCB", _args);
        return _ret[0].as<int64_t>();
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

    void sim::visitTree(int64_t rootHandle, std::string visitorFunc, std::optional<json> options)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(rootHandle);
        _args.push_back(visitorFunc);
        if(options)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*options);
        }
        else _brk = true;
        auto _ret = this->_client->call("sim.visitTree", _args);
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

    json sim::waitForSignal(int64_t target, std::string sigName)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(target);
        _args.push_back(sigName);
        auto _ret = this->_client->call("sim.waitForSignal", _args);
        return _ret[0].as<json>();
    }

    void sim::writeCustomBufferData(int64_t objectHandle, std::string tagName, std::vector<uint8_t> data)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(objectHandle);
        _args.push_back(tagName);
        _args.push_back(bin(data));
        auto _ret = this->_client->call("sim.writeCustomBufferData", _args);
    }

    void sim::writeCustomStringData(int64_t objectHandle, std::string tagName, std::string data)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(objectHandle);
        _args.push_back(tagName);
        _args.push_back(data);
        auto _ret = this->_client->call("sim.writeCustomStringData", _args);
    }

    void sim::writeCustomTableData(int64_t handle, std::string tagName, json theTable, std::optional<json> options)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(handle);
        _args.push_back(tagName);
        _args.push_back(theTable);
        if(options)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*options);
        }
        else _brk = true;
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

    void sim::yield()
    {
        bool _brk = false;
        json _args(json_array_arg);
        auto _ret = this->_client->call("sim.yield", _args);
    }

    simAssimp::simAssimp(RemoteAPIClient *client)
        : _client(client)
    {
        _client->require("simAssimp");
    }

    void simAssimp::exportMeshes(json allVertices, json allIndices, std::string filename, std::string formatId, std::optional<double> scaling, std::optional<int64_t> upVector, std::optional<int64_t> options)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(allVertices);
        _args.push_back(allIndices);
        _args.push_back(filename);
        _args.push_back(formatId);
        if(scaling)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*scaling);
        }
        else _brk = true;
        if(upVector)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*upVector);
        }
        else _brk = true;
        if(options)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*options);
        }
        else _brk = true;
        auto _ret = this->_client->call("simAssimp.exportMeshes", _args);
    }

    void simAssimp::exportShapes(std::vector<int64_t> shapeHandles, std::string filename, std::string formatId, std::optional<double> scaling, std::optional<int64_t> upVector, std::optional<int64_t> options)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(shapeHandles);
        _args.push_back(filename);
        _args.push_back(formatId);
        if(scaling)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*scaling);
        }
        else _brk = true;
        if(upVector)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*upVector);
        }
        else _brk = true;
        if(options)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*options);
        }
        else _brk = true;
        auto _ret = this->_client->call("simAssimp.exportShapes", _args);
    }

    void simAssimp::exportShapesDlg(std::string filename, std::vector<int64_t> shapeHandles)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(filename);
        _args.push_back(shapeHandles);
        auto _ret = this->_client->call("simAssimp.exportShapesDlg", _args);
    }

    std::tuple<std::string, std::string, std::string> simAssimp::getExportFormat(int64_t index)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(index);
        auto _ret = this->_client->call("simAssimp.getExportFormat", _args);
        return std::make_tuple(_ret[0].as<std::string>(), _ret[1].as<std::string>(), _ret[2].as<std::string>());
    }

    std::tuple<std::string, std::string> simAssimp::getImportFormat(int64_t index)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(index);
        auto _ret = this->_client->call("simAssimp.getImportFormat", _args);
        return std::make_tuple(_ret[0].as<std::string>(), _ret[1].as<std::string>());
    }

    std::tuple<json, json> simAssimp::importMeshes(std::string filenames, std::optional<double> scaling, std::optional<int64_t> upVector, std::optional<int64_t> options)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(filenames);
        if(scaling)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*scaling);
        }
        else _brk = true;
        if(upVector)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*upVector);
        }
        else _brk = true;
        if(options)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*options);
        }
        else _brk = true;
        auto _ret = this->_client->call("simAssimp.importMeshes", _args);
        return std::make_tuple(_ret[0].as<json>(), _ret[1].as<json>());
    }

    std::vector<int64_t> simAssimp::importShapes(std::string filenames, std::optional<int64_t> maxTextureSize, std::optional<double> scaling, std::optional<int64_t> upVector, std::optional<int64_t> options)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(filenames);
        if(maxTextureSize)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*maxTextureSize);
        }
        else _brk = true;
        if(scaling)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*scaling);
        }
        else _brk = true;
        if(upVector)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*upVector);
        }
        else _brk = true;
        if(options)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*options);
        }
        else _brk = true;
        auto _ret = this->_client->call("simAssimp.importShapes", _args);
        return _ret[0].as<std::vector<int64_t>>();
    }

    std::vector<int64_t> simAssimp::importShapesDlg(std::string filename)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(filename);
        auto _ret = this->_client->call("simAssimp.importShapesDlg", _args);
        return _ret[0].as<std::vector<int64_t>>();
    }

    simBubble::simBubble(RemoteAPIClient *client)
        : _client(client)
    {
        _client->require("simBubble");
    }

    int64_t simBubble::create(std::vector<int64_t> motorJointHandles, int64_t sensorHandle, std::vector<double> backRelativeVelocities)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(motorJointHandles);
        _args.push_back(sensorHandle);
        _args.push_back(backRelativeVelocities);
        auto _ret = this->_client->call("simBubble.create", _args);
        return _ret[0].as<int64_t>();
    }

    bool simBubble::destroy(int64_t bubbleRobHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(bubbleRobHandle);
        auto _ret = this->_client->call("simBubble.destroy", _args);
        return _ret[0].as<bool>();
    }

    bool simBubble::start(int64_t bubbleRobHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(bubbleRobHandle);
        auto _ret = this->_client->call("simBubble.start", _args);
        return _ret[0].as<bool>();
    }

    bool simBubble::stop(int64_t bubbleRobHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(bubbleRobHandle);
        auto _ret = this->_client->call("simBubble.stop", _args);
        return _ret[0].as<bool>();
    }

    simCHAI3D::simCHAI3D(RemoteAPIClient *client)
        : _client(client)
    {
        _client->require("simCHAI3D");
    }

    int64_t simCHAI3D::addConstraintPlane(int64_t deviceIndex, std::vector<double> position, std::vector<double> normal, double Kp, double Kv, double Fmax)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(deviceIndex);
        _args.push_back(position);
        _args.push_back(normal);
        _args.push_back(Kp);
        _args.push_back(Kv);
        _args.push_back(Fmax);
        auto _ret = this->_client->call("simCHAI3D.addConstraintPlane", _args);
        return _ret[0].as<int64_t>();
    }

    int64_t simCHAI3D::addConstraintPoint(int64_t deviceIndex, std::vector<double> position, double Kp, double Kv, double Fmax)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(deviceIndex);
        _args.push_back(position);
        _args.push_back(Kp);
        _args.push_back(Kv);
        _args.push_back(Fmax);
        auto _ret = this->_client->call("simCHAI3D.addConstraintPoint", _args);
        return _ret[0].as<int64_t>();
    }

    int64_t simCHAI3D::addConstraintSegment(int64_t deviceIndex, std::vector<double> point, std::vector<double> segment, double Kp, double Kv, double Fmax)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(deviceIndex);
        _args.push_back(point);
        _args.push_back(segment);
        _args.push_back(Kp);
        _args.push_back(Kv);
        _args.push_back(Fmax);
        auto _ret = this->_client->call("simCHAI3D.addConstraintSegment", _args);
        return _ret[0].as<int64_t>();
    }

    int64_t simCHAI3D::addShape(std::vector<double> vertices, std::vector<int64_t> indices, std::vector<double> position, std::vector<double> orientation, double stiffnessFactor)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(vertices);
        _args.push_back(indices);
        _args.push_back(position);
        _args.push_back(orientation);
        _args.push_back(stiffnessFactor);
        auto _ret = this->_client->call("simCHAI3D.addShape", _args);
        return _ret[0].as<int64_t>();
    }

    int64_t simCHAI3D::readButtons(int64_t deviceIndex)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(deviceIndex);
        auto _ret = this->_client->call("simCHAI3D.readButtons", _args);
        return _ret[0].as<int64_t>();
    }

    std::vector<double> simCHAI3D::readForce(int64_t deviceIndex)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(deviceIndex);
        auto _ret = this->_client->call("simCHAI3D.readForce", _args);
        return _ret[0].as<std::vector<double>>();
    }

    std::vector<double> simCHAI3D::readPosition(int64_t deviceIndex)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(deviceIndex);
        auto _ret = this->_client->call("simCHAI3D.readPosition", _args);
        return _ret[0].as<std::vector<double>>();
    }

    void simCHAI3D::removeObject(int64_t objectID)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(objectID);
        auto _ret = this->_client->call("simCHAI3D.removeObject", _args);
    }

    void simCHAI3D::reset()
    {
        bool _brk = false;
        json _args(json_array_arg);
        auto _ret = this->_client->call("simCHAI3D.reset", _args);
    }

    int64_t simCHAI3D::start(int64_t deviceIndex, double toolRadius, double workspaceRadius)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(deviceIndex);
        _args.push_back(toolRadius);
        _args.push_back(workspaceRadius);
        auto _ret = this->_client->call("simCHAI3D.start", _args);
        return _ret[0].as<int64_t>();
    }

    void simCHAI3D::updateConstraint(int64_t objectID, std::vector<double> positionA, std::vector<double> positionB, double Kp, double Kv, double Fmax)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(objectID);
        _args.push_back(positionA);
        _args.push_back(positionB);
        _args.push_back(Kp);
        _args.push_back(Kv);
        _args.push_back(Fmax);
        auto _ret = this->_client->call("simCHAI3D.updateConstraint", _args);
    }

    void simCHAI3D::updateShape(int64_t objectID, std::vector<double> position, std::vector<double> orientation, double stiffnessFactor)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(objectID);
        _args.push_back(position);
        _args.push_back(orientation);
        _args.push_back(stiffnessFactor);
        auto _ret = this->_client->call("simCHAI3D.updateShape", _args);
    }

    simCam::simCam(RemoteAPIClient *client)
        : _client(client)
    {
        _client->require("simCam");
    }

    int64_t simCam::grab(int64_t deviceIndex, int64_t visionSensorHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(deviceIndex);
        _args.push_back(visionSensorHandle);
        auto _ret = this->_client->call("simCam.grab", _args);
        return _ret[0].as<int64_t>();
    }

    std::string simCam::info(int64_t deviceIndex)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(deviceIndex);
        auto _ret = this->_client->call("simCam.info", _args);
        return _ret[0].as<std::string>();
    }

    std::tuple<int64_t, int64_t, int64_t> simCam::start(int64_t deviceIndex, int64_t resX, int64_t resY)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(deviceIndex);
        _args.push_back(resX);
        _args.push_back(resY);
        auto _ret = this->_client->call("simCam.start", _args);
        return std::make_tuple(_ret[0].as<int64_t>(), _ret[1].as<int64_t>(), _ret[2].as<int64_t>());
    }

    int64_t simCam::stop(int64_t deviceIndex)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(deviceIndex);
        auto _ret = this->_client->call("simCam.stop", _args);
        return _ret[0].as<int64_t>();
    }

    simConvex::simConvex(RemoteAPIClient *client)
        : _client(client)
    {
        _client->require("simConvex");
    }

    std::vector<int64_t> simConvex::hacd(int64_t shapeHandle, std::optional<json> params)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(shapeHandle);
        if(params)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*params);
        }
        else _brk = true;
        auto _ret = this->_client->call("simConvex.hacd", _args);
        return _ret[0].as<std::vector<int64_t>>();
    }

    int64_t simConvex::hull(std::vector<int64_t> objectHandles, std::optional<double> growth)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(objectHandles);
        if(growth)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*growth);
        }
        else _brk = true;
        auto _ret = this->_client->call("simConvex.hull", _args);
        return _ret[0].as<int64_t>();
    }

    std::tuple<std::vector<double>, std::vector<int64_t>> simConvex::qhull(std::vector<double> points, std::optional<double> growth)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(points);
        if(growth)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*growth);
        }
        else _brk = true;
        auto _ret = this->_client->call("simConvex.qhull", _args);
        return std::make_tuple(_ret[0].as<std::vector<double>>(), _ret[1].as<std::vector<int64_t>>());
    }

    std::vector<int64_t> simConvex::vhacd(int64_t shapeHandle, std::optional<json> params)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(shapeHandle);
        if(params)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*params);
        }
        else _brk = true;
        auto _ret = this->_client->call("simConvex.vhacd", _args);
        return _ret[0].as<std::vector<int64_t>>();
    }

    simGLTF::simGLTF(RemoteAPIClient *client)
        : _client(client)
    {
        _client->require("simGLTF");
    }

    int64_t simGLTF::animationFrameCount()
    {
        bool _brk = false;
        json _args(json_array_arg);
        auto _ret = this->_client->call("simGLTF.animationFrameCount", _args);
        return _ret[0].as<int64_t>();
    }

    void simGLTF::clear()
    {
        bool _brk = false;
        json _args(json_array_arg);
        auto _ret = this->_client->call("simGLTF.clear", _args);
    }

    void simGLTF::exportAllObjects()
    {
        bool _brk = false;
        json _args(json_array_arg);
        auto _ret = this->_client->call("simGLTF.exportAllObjects", _args);
    }

    void simGLTF::exportAnimation()
    {
        bool _brk = false;
        json _args(json_array_arg);
        auto _ret = this->_client->call("simGLTF.exportAnimation", _args);
    }

    int64_t simGLTF::exportObject(int64_t objectHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(objectHandle);
        auto _ret = this->_client->call("simGLTF.exportObject", _args);
        return _ret[0].as<int64_t>();
    }

    void simGLTF::exportObjects(std::vector<int64_t> objectHandles)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(objectHandles);
        auto _ret = this->_client->call("simGLTF.exportObjects", _args);
    }

    void simGLTF::exportSelectedObjects()
    {
        bool _brk = false;
        json _args(json_array_arg);
        auto _ret = this->_client->call("simGLTF.exportSelectedObjects", _args);
    }

    int64_t simGLTF::exportShape(int64_t shapeHandle, std::optional<int64_t> parentHandle, std::optional<int64_t> parentNodeIndex)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(shapeHandle);
        if(parentHandle)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*parentHandle);
        }
        else _brk = true;
        if(parentNodeIndex)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*parentNodeIndex);
        }
        else _brk = true;
        auto _ret = this->_client->call("simGLTF.exportShape", _args);
        return _ret[0].as<int64_t>();
    }

    std::tuple<int64_t, std::string> simGLTF::getExportTextureFormat()
    {
        bool _brk = false;
        json _args(json_array_arg);
        auto _ret = this->_client->call("simGLTF.getExportTextureFormat", _args);
        return std::make_tuple(_ret[0].as<int64_t>(), _ret[1].as<std::string>());
    }

    std::tuple<bool, std::string, std::string> simGLTF::loadASCII(std::string filepath)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(filepath);
        auto _ret = this->_client->call("simGLTF.loadASCII", _args);
        return std::make_tuple(_ret[0].as<bool>(), _ret[1].as<std::string>(), _ret[2].as<std::string>());
    }

    std::tuple<bool, std::string, std::string> simGLTF::loadBinary(std::string filepath)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(filepath);
        auto _ret = this->_client->call("simGLTF.loadBinary", _args);
        return std::make_tuple(_ret[0].as<bool>(), _ret[1].as<std::string>(), _ret[2].as<std::string>());
    }

    void simGLTF::recordAnimation(bool enable)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(enable);
        auto _ret = this->_client->call("simGLTF.recordAnimation", _args);
    }

    bool simGLTF::saveASCII(std::string filepath)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(filepath);
        auto _ret = this->_client->call("simGLTF.saveASCII", _args);
        return _ret[0].as<bool>();
    }

    bool simGLTF::saveBinary(std::string filepath)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(filepath);
        auto _ret = this->_client->call("simGLTF.saveBinary", _args);
        return _ret[0].as<bool>();
    }

    std::string simGLTF::serialize()
    {
        bool _brk = false;
        json _args(json_array_arg);
        auto _ret = this->_client->call("simGLTF.serialize", _args);
        return _ret[0].as<std::string>();
    }

    void simGLTF::setExportTextureFormat(int64_t textureFormat)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(textureFormat);
        auto _ret = this->_client->call("simGLTF.setExportTextureFormat", _args);
    }

    simGeom::simGeom(RemoteAPIClient *client)
        : _client(client)
    {
        _client->require("simGeom");
    }

    int64_t simGeom::copyMesh(int64_t meshHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(meshHandle);
        auto _ret = this->_client->call("simGeom.copyMesh", _args);
        return _ret[0].as<int64_t>();
    }

    int64_t simGeom::copyOctree(int64_t octreeHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(octreeHandle);
        auto _ret = this->_client->call("simGeom.copyOctree", _args);
        return _ret[0].as<int64_t>();
    }

    int64_t simGeom::copyPtcloud(int64_t ptcloudHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(ptcloudHandle);
        auto _ret = this->_client->call("simGeom.copyPtcloud", _args);
        return _ret[0].as<int64_t>();
    }

    int64_t simGeom::createMesh(std::vector<double> vertices, std::vector<int64_t> indices, std::optional<std::vector<double>> meshOriginPos, std::optional<std::vector<double>> meshOriginQuaternion, std::optional<double> maxTriangleEdgeLength, std::optional<int64_t> maxTriangleCountInLeafObb)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(vertices);
        _args.push_back(indices);
        if(meshOriginPos)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*meshOriginPos);
        }
        else _brk = true;
        if(meshOriginQuaternion)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*meshOriginQuaternion);
        }
        else _brk = true;
        if(maxTriangleEdgeLength)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*maxTriangleEdgeLength);
        }
        else _brk = true;
        if(maxTriangleCountInLeafObb)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*maxTriangleCountInLeafObb);
        }
        else _brk = true;
        auto _ret = this->_client->call("simGeom.createMesh", _args);
        return _ret[0].as<int64_t>();
    }

    int64_t simGeom::createMeshFromSerializationData(std::string data)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(data);
        auto _ret = this->_client->call("simGeom.createMeshFromSerializationData", _args);
        return _ret[0].as<int64_t>();
    }

    int64_t simGeom::createOctreeFromColorPoints(std::vector<double> points, std::optional<std::vector<double>> octreeOriginPos, std::optional<std::vector<double>> octreeOriginQuaternion, std::optional<double> maxCellSize, std::optional<std::vector<double>> colors, std::optional<std::vector<int64_t>> userData)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(points);
        if(octreeOriginPos)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*octreeOriginPos);
        }
        else _brk = true;
        if(octreeOriginQuaternion)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*octreeOriginQuaternion);
        }
        else _brk = true;
        if(maxCellSize)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*maxCellSize);
        }
        else _brk = true;
        if(colors)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*colors);
        }
        else _brk = true;
        if(userData)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*userData);
        }
        else _brk = true;
        auto _ret = this->_client->call("simGeom.createOctreeFromColorPoints", _args);
        return _ret[0].as<int64_t>();
    }

    int64_t simGeom::createOctreeFromMesh(int64_t meshHandle, std::vector<double> meshPos, std::vector<double> meshQuaternion, std::optional<std::vector<double>> octreeOriginPos, std::optional<std::vector<double>> octreeOriginQuaternion, std::optional<double> maxCellSize, std::optional<std::vector<int64_t>> pointColor, std::optional<int64_t> userData)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(meshHandle);
        _args.push_back(meshPos);
        _args.push_back(meshQuaternion);
        if(octreeOriginPos)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*octreeOriginPos);
        }
        else _brk = true;
        if(octreeOriginQuaternion)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*octreeOriginQuaternion);
        }
        else _brk = true;
        if(maxCellSize)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*maxCellSize);
        }
        else _brk = true;
        if(pointColor)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*pointColor);
        }
        else _brk = true;
        if(userData)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*userData);
        }
        else _brk = true;
        auto _ret = this->_client->call("simGeom.createOctreeFromMesh", _args);
        return _ret[0].as<int64_t>();
    }

    int64_t simGeom::createOctreeFromOctree(int64_t octreeHandle, std::vector<double> octreePos, std::vector<double> octreeQuaternion, std::optional<std::vector<double>> newOctreeOriginPos, std::optional<std::vector<double>> newOctreeOriginQuaternion, std::optional<double> maxCellSize, std::optional<std::vector<int64_t>> pointColor, std::optional<int64_t> userData)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(octreeHandle);
        _args.push_back(octreePos);
        _args.push_back(octreeQuaternion);
        if(newOctreeOriginPos)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*newOctreeOriginPos);
        }
        else _brk = true;
        if(newOctreeOriginQuaternion)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*newOctreeOriginQuaternion);
        }
        else _brk = true;
        if(maxCellSize)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*maxCellSize);
        }
        else _brk = true;
        if(pointColor)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*pointColor);
        }
        else _brk = true;
        if(userData)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*userData);
        }
        else _brk = true;
        auto _ret = this->_client->call("simGeom.createOctreeFromOctree", _args);
        return _ret[0].as<int64_t>();
    }

    int64_t simGeom::createOctreeFromPoints(std::vector<double> points, std::optional<std::vector<double>> octreeOriginPos, std::optional<std::vector<double>> octreeOriginQuaternion, std::optional<double> maxCellSize, std::optional<std::vector<int64_t>> pointColor, std::optional<int64_t> userData)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(points);
        if(octreeOriginPos)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*octreeOriginPos);
        }
        else _brk = true;
        if(octreeOriginQuaternion)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*octreeOriginQuaternion);
        }
        else _brk = true;
        if(maxCellSize)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*maxCellSize);
        }
        else _brk = true;
        if(pointColor)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*pointColor);
        }
        else _brk = true;
        if(userData)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*userData);
        }
        else _brk = true;
        auto _ret = this->_client->call("simGeom.createOctreeFromPoints", _args);
        return _ret[0].as<int64_t>();
    }

    int64_t simGeom::createOctreeFromSerializationData(std::string data)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(data);
        auto _ret = this->_client->call("simGeom.createOctreeFromSerializationData", _args);
        return _ret[0].as<int64_t>();
    }

    int64_t simGeom::createPtcloudFromColorPoints(std::vector<double> points, std::optional<std::vector<double>> octreeOriginPos, std::optional<std::vector<double>> octreeOriginQuaternion, std::optional<double> maxCellSize, std::optional<int64_t> maxPtsInCell, std::optional<std::vector<double>> colors, std::optional<double> proximityTolerance)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(points);
        if(octreeOriginPos)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*octreeOriginPos);
        }
        else _brk = true;
        if(octreeOriginQuaternion)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*octreeOriginQuaternion);
        }
        else _brk = true;
        if(maxCellSize)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*maxCellSize);
        }
        else _brk = true;
        if(maxPtsInCell)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*maxPtsInCell);
        }
        else _brk = true;
        if(colors)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*colors);
        }
        else _brk = true;
        if(proximityTolerance)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*proximityTolerance);
        }
        else _brk = true;
        auto _ret = this->_client->call("simGeom.createPtcloudFromColorPoints", _args);
        return _ret[0].as<int64_t>();
    }

    int64_t simGeom::createPtcloudFromPoints(std::vector<double> points, std::optional<std::vector<double>> octreeOriginPos, std::optional<std::vector<double>> octreeOriginQuaternion, std::optional<double> maxCellSize, std::optional<int64_t> maxPtsInCell, std::optional<std::vector<int64_t>> pointColor, std::optional<double> proximityTolerance)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(points);
        if(octreeOriginPos)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*octreeOriginPos);
        }
        else _brk = true;
        if(octreeOriginQuaternion)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*octreeOriginQuaternion);
        }
        else _brk = true;
        if(maxCellSize)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*maxCellSize);
        }
        else _brk = true;
        if(maxPtsInCell)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*maxPtsInCell);
        }
        else _brk = true;
        if(pointColor)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*pointColor);
        }
        else _brk = true;
        if(proximityTolerance)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*proximityTolerance);
        }
        else _brk = true;
        auto _ret = this->_client->call("simGeom.createPtcloudFromPoints", _args);
        return _ret[0].as<int64_t>();
    }

    int64_t simGeom::createPtcloudFromSerializationData(std::string data)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(data);
        auto _ret = this->_client->call("simGeom.createPtcloudFromSerializationData", _args);
        return _ret[0].as<int64_t>();
    }

    void simGeom::destroyMesh(int64_t meshHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(meshHandle);
        auto _ret = this->_client->call("simGeom.destroyMesh", _args);
    }

    void simGeom::destroyOctree(int64_t octreeHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(octreeHandle);
        auto _ret = this->_client->call("simGeom.destroyOctree", _args);
    }

    void simGeom::destroyPtcloud(int64_t ptcloudHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(ptcloudHandle);
        auto _ret = this->_client->call("simGeom.destroyPtcloud", _args);
    }

    std::tuple<double, std::vector<double>, std::vector<double>> simGeom::getBoxBoxDistance(std::vector<double> box1Pos, std::vector<double> box1Quaternion, std::vector<double> box1HalfSize, std::vector<double> box2Pos, std::vector<double> box2Quaternion, std::vector<double> box2HalfSize, bool boxesAreSolid)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(box1Pos);
        _args.push_back(box1Quaternion);
        _args.push_back(box1HalfSize);
        _args.push_back(box2Pos);
        _args.push_back(box2Quaternion);
        _args.push_back(box2HalfSize);
        _args.push_back(boxesAreSolid);
        auto _ret = this->_client->call("simGeom.getBoxBoxDistance", _args);
        return std::make_tuple(_ret[0].as<double>(), _ret[1].as<std::vector<double>>(), _ret[2].as<std::vector<double>>());
    }

    std::tuple<double, std::vector<double>> simGeom::getBoxPointDistance(std::vector<double> boxPos, std::vector<double> boxQuaternion, std::vector<double> boxHalfSize, bool boxIsSolid, std::vector<double> point)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(boxPos);
        _args.push_back(boxQuaternion);
        _args.push_back(boxHalfSize);
        _args.push_back(boxIsSolid);
        _args.push_back(point);
        auto _ret = this->_client->call("simGeom.getBoxPointDistance", _args);
        return std::make_tuple(_ret[0].as<double>(), _ret[1].as<std::vector<double>>());
    }

    std::tuple<double, std::vector<double>, std::vector<double>> simGeom::getBoxSegmentDistance(std::vector<double> boxPos, std::vector<double> boxQuaternion, std::vector<double> boxHalfSize, bool boxIsSolid, std::vector<double> segmentPt1, std::vector<double> segmentPt2, std::optional<bool> altRoutine)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(boxPos);
        _args.push_back(boxQuaternion);
        _args.push_back(boxHalfSize);
        _args.push_back(boxIsSolid);
        _args.push_back(segmentPt1);
        _args.push_back(segmentPt2);
        if(altRoutine)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*altRoutine);
        }
        else _brk = true;
        auto _ret = this->_client->call("simGeom.getBoxSegmentDistance", _args);
        return std::make_tuple(_ret[0].as<double>(), _ret[1].as<std::vector<double>>(), _ret[2].as<std::vector<double>>());
    }

    std::tuple<double, std::vector<double>, std::vector<double>> simGeom::getBoxTriangleDistance(std::vector<double> boxPos, std::vector<double> boxQuaternion, std::vector<double> boxHalfSize, bool boxIsSolid, std::vector<double> triPt1, std::vector<double> triPt2, std::vector<double> triPt3, std::optional<bool> altRoutine)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(boxPos);
        _args.push_back(boxQuaternion);
        _args.push_back(boxHalfSize);
        _args.push_back(boxIsSolid);
        _args.push_back(triPt1);
        _args.push_back(triPt2);
        _args.push_back(triPt3);
        if(altRoutine)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*altRoutine);
        }
        else _brk = true;
        auto _ret = this->_client->call("simGeom.getBoxTriangleDistance", _args);
        return std::make_tuple(_ret[0].as<double>(), _ret[1].as<std::vector<double>>(), _ret[2].as<std::vector<double>>());
    }

    std::tuple<bool, std::vector<int64_t>, std::vector<double>> simGeom::getMeshMeshCollision(int64_t mesh1Handle, std::vector<double> mesh1Pos, std::vector<double> mesh1Quaternion, int64_t mesh2Handle, std::vector<double> mesh2Pos, std::vector<double> mesh2Quaternion, std::optional<std::vector<int64_t>> cache, std::optional<bool> returnIntersections)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(mesh1Handle);
        _args.push_back(mesh1Pos);
        _args.push_back(mesh1Quaternion);
        _args.push_back(mesh2Handle);
        _args.push_back(mesh2Pos);
        _args.push_back(mesh2Quaternion);
        if(cache)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*cache);
        }
        else _brk = true;
        if(returnIntersections)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*returnIntersections);
        }
        else _brk = true;
        auto _ret = this->_client->call("simGeom.getMeshMeshCollision", _args);
        return std::make_tuple(_ret[0].as<bool>(), _ret[1].as<std::vector<int64_t>>(), _ret[2].as<std::vector<double>>());
    }

    std::tuple<double, std::vector<double>, std::vector<double>, std::vector<int64_t>> simGeom::getMeshMeshDistance(int64_t mesh1Handle, std::vector<double> mesh1Pos, std::vector<double> mesh1Quaternion, int64_t mesh2Handle, std::vector<double> mesh2Pos, std::vector<double> mesh2Quaternion, std::optional<double> distanceThreshold, std::optional<std::vector<int64_t>> cache)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(mesh1Handle);
        _args.push_back(mesh1Pos);
        _args.push_back(mesh1Quaternion);
        _args.push_back(mesh2Handle);
        _args.push_back(mesh2Pos);
        _args.push_back(mesh2Quaternion);
        if(distanceThreshold)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*distanceThreshold);
        }
        else _brk = true;
        if(cache)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*cache);
        }
        else _brk = true;
        auto _ret = this->_client->call("simGeom.getMeshMeshDistance", _args);
        return std::make_tuple(_ret[0].as<double>(), _ret[1].as<std::vector<double>>(), _ret[2].as<std::vector<double>>(), _ret[3].as<std::vector<int64_t>>());
    }

    std::tuple<bool, std::vector<int64_t>> simGeom::getMeshOctreeCollision(int64_t meshHandle, std::vector<double> meshPos, std::vector<double> meshQuaternion, int64_t octreeHandle, std::vector<double> octreePos, std::vector<double> octreeQuaternion, std::optional<std::vector<int64_t>> cache)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(meshHandle);
        _args.push_back(meshPos);
        _args.push_back(meshQuaternion);
        _args.push_back(octreeHandle);
        _args.push_back(octreePos);
        _args.push_back(octreeQuaternion);
        if(cache)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*cache);
        }
        else _brk = true;
        auto _ret = this->_client->call("simGeom.getMeshOctreeCollision", _args);
        return std::make_tuple(_ret[0].as<bool>(), _ret[1].as<std::vector<int64_t>>());
    }

    std::tuple<double, std::vector<double>, std::vector<double>, std::vector<int64_t>> simGeom::getMeshOctreeDistance(int64_t meshHandle, std::vector<double> meshPos, std::vector<double> meshQuaternion, int64_t octreeHandle, std::vector<double> octreePos, std::vector<double> octreeQuaternion, std::optional<double> distanceThreshold, std::optional<std::vector<int64_t>> cache)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(meshHandle);
        _args.push_back(meshPos);
        _args.push_back(meshQuaternion);
        _args.push_back(octreeHandle);
        _args.push_back(octreePos);
        _args.push_back(octreeQuaternion);
        if(distanceThreshold)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*distanceThreshold);
        }
        else _brk = true;
        if(cache)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*cache);
        }
        else _brk = true;
        auto _ret = this->_client->call("simGeom.getMeshOctreeDistance", _args);
        return std::make_tuple(_ret[0].as<double>(), _ret[1].as<std::vector<double>>(), _ret[2].as<std::vector<double>>(), _ret[3].as<std::vector<int64_t>>());
    }

    std::tuple<double, std::vector<double>, int64_t> simGeom::getMeshPointDistance(int64_t meshHandle, std::vector<double> meshPos, std::vector<double> meshQuaternion, std::vector<double> point, std::optional<double> distanceThreshold, std::optional<int64_t> cache)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(meshHandle);
        _args.push_back(meshPos);
        _args.push_back(meshQuaternion);
        _args.push_back(point);
        if(distanceThreshold)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*distanceThreshold);
        }
        else _brk = true;
        if(cache)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*cache);
        }
        else _brk = true;
        auto _ret = this->_client->call("simGeom.getMeshPointDistance", _args);
        return std::make_tuple(_ret[0].as<double>(), _ret[1].as<std::vector<double>>(), _ret[2].as<int64_t>());
    }

    std::tuple<double, std::vector<double>, std::vector<double>, std::vector<int64_t>> simGeom::getMeshPtcloudDistance(int64_t meshHandle, std::vector<double> meshPos, std::vector<double> meshQuaternion, int64_t ptcloudHandle, std::vector<double> ptcloudPos, std::vector<double> ptcloudQuaternion, std::optional<double> distanceThreshold, std::optional<std::vector<int64_t>> cache)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(meshHandle);
        _args.push_back(meshPos);
        _args.push_back(meshQuaternion);
        _args.push_back(ptcloudHandle);
        _args.push_back(ptcloudPos);
        _args.push_back(ptcloudQuaternion);
        if(distanceThreshold)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*distanceThreshold);
        }
        else _brk = true;
        if(cache)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*cache);
        }
        else _brk = true;
        auto _ret = this->_client->call("simGeom.getMeshPtcloudDistance", _args);
        return std::make_tuple(_ret[0].as<double>(), _ret[1].as<std::vector<double>>(), _ret[2].as<std::vector<double>>(), _ret[3].as<std::vector<int64_t>>());
    }

    std::tuple<bool, int64_t, std::vector<double>> simGeom::getMeshSegmentCollision(int64_t meshHandle, std::vector<double> meshPos, std::vector<double> meshQuaternion, std::vector<double> segmentPt1, std::vector<double> segmentPt2, std::optional<int64_t> cache, std::optional<bool> returnIntersections)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(meshHandle);
        _args.push_back(meshPos);
        _args.push_back(meshQuaternion);
        _args.push_back(segmentPt1);
        _args.push_back(segmentPt2);
        if(cache)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*cache);
        }
        else _brk = true;
        if(returnIntersections)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*returnIntersections);
        }
        else _brk = true;
        auto _ret = this->_client->call("simGeom.getMeshSegmentCollision", _args);
        return std::make_tuple(_ret[0].as<bool>(), _ret[1].as<int64_t>(), _ret[2].as<std::vector<double>>());
    }

    std::tuple<double, std::vector<double>, std::vector<double>, int64_t> simGeom::getMeshSegmentDistance(int64_t meshHandle, std::vector<double> meshPos, std::vector<double> meshQuaternion, std::vector<double> segmentPt1, std::vector<double> segmentPt2, std::optional<double> distanceThreshold, std::optional<int64_t> cache)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(meshHandle);
        _args.push_back(meshPos);
        _args.push_back(meshQuaternion);
        _args.push_back(segmentPt1);
        _args.push_back(segmentPt2);
        if(distanceThreshold)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*distanceThreshold);
        }
        else _brk = true;
        if(cache)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*cache);
        }
        else _brk = true;
        auto _ret = this->_client->call("simGeom.getMeshSegmentDistance", _args);
        return std::make_tuple(_ret[0].as<double>(), _ret[1].as<std::vector<double>>(), _ret[2].as<std::vector<double>>(), _ret[3].as<int64_t>());
    }

    std::string simGeom::getMeshSerializationData(int64_t meshHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(meshHandle);
        auto _ret = this->_client->call("simGeom.getMeshSerializationData", _args);
        return _ret[0].as<std::string>();
    }

    std::tuple<bool, int64_t, std::vector<double>> simGeom::getMeshTriangleCollision(int64_t meshHandle, std::vector<double> meshPos, std::vector<double> meshQuaternion, std::vector<double> triPt1, std::vector<double> triPt2, std::vector<double> triPt3, std::optional<int64_t> cache, std::optional<bool> returnIntersections)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(meshHandle);
        _args.push_back(meshPos);
        _args.push_back(meshQuaternion);
        _args.push_back(triPt1);
        _args.push_back(triPt2);
        _args.push_back(triPt3);
        if(cache)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*cache);
        }
        else _brk = true;
        if(returnIntersections)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*returnIntersections);
        }
        else _brk = true;
        auto _ret = this->_client->call("simGeom.getMeshTriangleCollision", _args);
        return std::make_tuple(_ret[0].as<bool>(), _ret[1].as<int64_t>(), _ret[2].as<std::vector<double>>());
    }

    std::tuple<double, std::vector<double>, std::vector<double>, int64_t> simGeom::getMeshTriangleDistance(int64_t meshHandle, std::vector<double> meshPos, std::vector<double> meshQuaternion, std::vector<double> triPt1, std::vector<double> triPt2, std::vector<double> triPt3, std::optional<double> distanceThreshold, std::optional<int64_t> cache)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(meshHandle);
        _args.push_back(meshPos);
        _args.push_back(meshQuaternion);
        _args.push_back(triPt1);
        _args.push_back(triPt2);
        _args.push_back(triPt3);
        if(distanceThreshold)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*distanceThreshold);
        }
        else _brk = true;
        if(cache)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*cache);
        }
        else _brk = true;
        auto _ret = this->_client->call("simGeom.getMeshTriangleDistance", _args);
        return std::make_tuple(_ret[0].as<double>(), _ret[1].as<std::vector<double>>(), _ret[2].as<std::vector<double>>(), _ret[3].as<int64_t>());
    }

    std::tuple<bool, std::vector<int64_t>> simGeom::getOctreeOctreeCollision(int64_t octree1Handle, std::vector<double> octree1Pos, std::vector<double> octree1Quaternion, int64_t octree2Handle, std::vector<double> octree2Pos, std::vector<double> octree2Quaternion, std::optional<std::vector<int64_t>> cache)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(octree1Handle);
        _args.push_back(octree1Pos);
        _args.push_back(octree1Quaternion);
        _args.push_back(octree2Handle);
        _args.push_back(octree2Pos);
        _args.push_back(octree2Quaternion);
        if(cache)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*cache);
        }
        else _brk = true;
        auto _ret = this->_client->call("simGeom.getOctreeOctreeCollision", _args);
        return std::make_tuple(_ret[0].as<bool>(), _ret[1].as<std::vector<int64_t>>());
    }

    std::tuple<double, std::vector<double>, std::vector<double>, std::vector<int64_t>> simGeom::getOctreeOctreeDistance(int64_t octree1Handle, std::vector<double> octree1Pos, std::vector<double> octree1Quaternion, int64_t octree2Handle, std::vector<double> octree2Pos, std::vector<double> octree2Quaternion, std::optional<double> distanceThreshold, std::optional<std::vector<int64_t>> cache)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(octree1Handle);
        _args.push_back(octree1Pos);
        _args.push_back(octree1Quaternion);
        _args.push_back(octree2Handle);
        _args.push_back(octree2Pos);
        _args.push_back(octree2Quaternion);
        if(distanceThreshold)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*distanceThreshold);
        }
        else _brk = true;
        if(cache)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*cache);
        }
        else _brk = true;
        auto _ret = this->_client->call("simGeom.getOctreeOctreeDistance", _args);
        return std::make_tuple(_ret[0].as<double>(), _ret[1].as<std::vector<double>>(), _ret[2].as<std::vector<double>>(), _ret[3].as<std::vector<int64_t>>());
    }

    std::tuple<bool, int64_t> simGeom::getOctreePointCollision(int64_t octreeHandle, std::vector<double> octreePos, std::vector<double> octreeQuaternion, std::vector<double> point, std::optional<int64_t> cache)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(octreeHandle);
        _args.push_back(octreePos);
        _args.push_back(octreeQuaternion);
        _args.push_back(point);
        if(cache)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*cache);
        }
        else _brk = true;
        auto _ret = this->_client->call("simGeom.getOctreePointCollision", _args);
        return std::make_tuple(_ret[0].as<bool>(), _ret[1].as<int64_t>());
    }

    std::tuple<double, std::vector<double>, int64_t> simGeom::getOctreePointDistance(int64_t octreeHandle, std::vector<double> octreePos, std::vector<double> octreeQuaternion, std::vector<double> point, std::optional<double> distanceThreshold, std::optional<int64_t> cache)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(octreeHandle);
        _args.push_back(octreePos);
        _args.push_back(octreeQuaternion);
        _args.push_back(point);
        if(distanceThreshold)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*distanceThreshold);
        }
        else _brk = true;
        if(cache)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*cache);
        }
        else _brk = true;
        auto _ret = this->_client->call("simGeom.getOctreePointDistance", _args);
        return std::make_tuple(_ret[0].as<double>(), _ret[1].as<std::vector<double>>(), _ret[2].as<int64_t>());
    }

    std::tuple<bool, std::vector<int64_t>> simGeom::getOctreePtcloudCollision(int64_t octreeHandle, std::vector<double> octreePos, std::vector<double> octreeQuaternion, int64_t ptcloudHandle, std::vector<double> ptcloudPos, std::vector<double> ptcloudQuaternion, std::optional<std::vector<int64_t>> cache)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(octreeHandle);
        _args.push_back(octreePos);
        _args.push_back(octreeQuaternion);
        _args.push_back(ptcloudHandle);
        _args.push_back(ptcloudPos);
        _args.push_back(ptcloudQuaternion);
        if(cache)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*cache);
        }
        else _brk = true;
        auto _ret = this->_client->call("simGeom.getOctreePtcloudCollision", _args);
        return std::make_tuple(_ret[0].as<bool>(), _ret[1].as<std::vector<int64_t>>());
    }

    std::tuple<double, std::vector<double>, std::vector<double>, std::vector<int64_t>> simGeom::getOctreePtcloudDistance(int64_t octreeHandle, std::vector<double> octreePos, std::vector<double> octreeQuaternion, int64_t ptcloudHandle, std::vector<double> ptcloudPos, std::vector<double> ptcloudQuaternion, std::optional<double> distanceThreshold, std::optional<std::vector<int64_t>> cache)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(octreeHandle);
        _args.push_back(octreePos);
        _args.push_back(octreeQuaternion);
        _args.push_back(ptcloudHandle);
        _args.push_back(ptcloudPos);
        _args.push_back(ptcloudQuaternion);
        if(distanceThreshold)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*distanceThreshold);
        }
        else _brk = true;
        if(cache)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*cache);
        }
        else _brk = true;
        auto _ret = this->_client->call("simGeom.getOctreePtcloudDistance", _args);
        return std::make_tuple(_ret[0].as<double>(), _ret[1].as<std::vector<double>>(), _ret[2].as<std::vector<double>>(), _ret[3].as<std::vector<int64_t>>());
    }

    std::tuple<bool, int64_t> simGeom::getOctreeSegmentCollision(int64_t octreeHandle, std::vector<double> octreePos, std::vector<double> octreeQuaternion, std::vector<double> segPt1, std::vector<double> segPt2, std::optional<int64_t> cache)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(octreeHandle);
        _args.push_back(octreePos);
        _args.push_back(octreeQuaternion);
        _args.push_back(segPt1);
        _args.push_back(segPt2);
        if(cache)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*cache);
        }
        else _brk = true;
        auto _ret = this->_client->call("simGeom.getOctreeSegmentCollision", _args);
        return std::make_tuple(_ret[0].as<bool>(), _ret[1].as<int64_t>());
    }

    std::tuple<double, std::vector<double>, std::vector<double>, int64_t> simGeom::getOctreeSegmentDistance(int64_t octreeHandle, std::vector<double> octreePos, std::vector<double> octreeQuaternion, std::vector<double> segPt1, std::vector<double> segPt2, std::optional<double> distanceThreshold, std::optional<int64_t> cache)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(octreeHandle);
        _args.push_back(octreePos);
        _args.push_back(octreeQuaternion);
        _args.push_back(segPt1);
        _args.push_back(segPt2);
        if(distanceThreshold)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*distanceThreshold);
        }
        else _brk = true;
        if(cache)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*cache);
        }
        else _brk = true;
        auto _ret = this->_client->call("simGeom.getOctreeSegmentDistance", _args);
        return std::make_tuple(_ret[0].as<double>(), _ret[1].as<std::vector<double>>(), _ret[2].as<std::vector<double>>(), _ret[3].as<int64_t>());
    }

    std::string simGeom::getOctreeSerializationData(int64_t octreeHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(octreeHandle);
        auto _ret = this->_client->call("simGeom.getOctreeSerializationData", _args);
        return _ret[0].as<std::string>();
    }

    std::tuple<bool, int64_t> simGeom::getOctreeTriangleCollision(int64_t octreeHandle, std::vector<double> octreePos, std::vector<double> octreeQuaternion, std::vector<double> triPt1, std::vector<double> triPt2, std::vector<double> triPt3, std::optional<int64_t> cache)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(octreeHandle);
        _args.push_back(octreePos);
        _args.push_back(octreeQuaternion);
        _args.push_back(triPt1);
        _args.push_back(triPt2);
        _args.push_back(triPt3);
        if(cache)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*cache);
        }
        else _brk = true;
        auto _ret = this->_client->call("simGeom.getOctreeTriangleCollision", _args);
        return std::make_tuple(_ret[0].as<bool>(), _ret[1].as<int64_t>());
    }

    std::tuple<double, std::vector<double>, std::vector<double>, int64_t> simGeom::getOctreeTriangleDistance(int64_t octreeHandle, std::vector<double> octreePos, std::vector<double> octreeQuaternion, std::vector<double> triPt1, std::vector<double> triPt2, std::vector<double> triPt3, std::optional<double> distanceThreshold, std::optional<int64_t> cache)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(octreeHandle);
        _args.push_back(octreePos);
        _args.push_back(octreeQuaternion);
        _args.push_back(triPt1);
        _args.push_back(triPt2);
        _args.push_back(triPt3);
        if(distanceThreshold)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*distanceThreshold);
        }
        else _brk = true;
        if(cache)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*cache);
        }
        else _brk = true;
        auto _ret = this->_client->call("simGeom.getOctreeTriangleDistance", _args);
        return std::make_tuple(_ret[0].as<double>(), _ret[1].as<std::vector<double>>(), _ret[2].as<std::vector<double>>(), _ret[3].as<int64_t>());
    }

    std::tuple<std::vector<double>, std::vector<double>, std::vector<int64_t>> simGeom::getOctreeVoxels(int64_t octreeHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(octreeHandle);
        auto _ret = this->_client->call("simGeom.getOctreeVoxels", _args);
        return std::make_tuple(_ret[0].as<std::vector<double>>(), _ret[1].as<std::vector<double>>(), _ret[2].as<std::vector<int64_t>>());
    }

    std::tuple<double, std::vector<double>, int64_t> simGeom::getPtcloudPointDistance(int64_t ptcloudHandle, std::vector<double> ptcloudPos, std::vector<double> ptcloudQuaternion, std::vector<double> point, std::optional<double> distanceThreshold, std::optional<int64_t> cache)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(ptcloudHandle);
        _args.push_back(ptcloudPos);
        _args.push_back(ptcloudQuaternion);
        _args.push_back(point);
        if(distanceThreshold)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*distanceThreshold);
        }
        else _brk = true;
        if(cache)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*cache);
        }
        else _brk = true;
        auto _ret = this->_client->call("simGeom.getPtcloudPointDistance", _args);
        return std::make_tuple(_ret[0].as<double>(), _ret[1].as<std::vector<double>>(), _ret[2].as<int64_t>());
    }

    std::tuple<std::vector<double>, std::vector<double>> simGeom::getPtcloudPoints(int64_t ptcloudHandle, std::optional<double> subsetProportion)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(ptcloudHandle);
        if(subsetProportion)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*subsetProportion);
        }
        else _brk = true;
        auto _ret = this->_client->call("simGeom.getPtcloudPoints", _args);
        return std::make_tuple(_ret[0].as<std::vector<double>>(), _ret[1].as<std::vector<double>>());
    }

    std::tuple<double, std::vector<double>, std::vector<double>, std::vector<int64_t>> simGeom::getPtcloudPtcloudDistance(int64_t ptcloud1Handle, std::vector<double> ptcloud1Pos, std::vector<double> ptcloud1Quaternion, int64_t ptcloud2Handle, std::vector<double> ptcloud2Pos, std::vector<double> ptcloud2Quaternion, std::optional<double> distanceThreshold, std::optional<std::vector<int64_t>> cache)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(ptcloud1Handle);
        _args.push_back(ptcloud1Pos);
        _args.push_back(ptcloud1Quaternion);
        _args.push_back(ptcloud2Handle);
        _args.push_back(ptcloud2Pos);
        _args.push_back(ptcloud2Quaternion);
        if(distanceThreshold)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*distanceThreshold);
        }
        else _brk = true;
        if(cache)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*cache);
        }
        else _brk = true;
        auto _ret = this->_client->call("simGeom.getPtcloudPtcloudDistance", _args);
        return std::make_tuple(_ret[0].as<double>(), _ret[1].as<std::vector<double>>(), _ret[2].as<std::vector<double>>(), _ret[3].as<std::vector<int64_t>>());
    }

    std::tuple<double, std::vector<double>, std::vector<double>, int64_t> simGeom::getPtcloudSegmentDistance(int64_t ptcloudHandle, std::vector<double> ptcloudPos, std::vector<double> ptcloudQuaternion, std::vector<double> segPt1, std::vector<double> segPt2, std::optional<double> distanceThreshold, std::optional<int64_t> cache)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(ptcloudHandle);
        _args.push_back(ptcloudPos);
        _args.push_back(ptcloudQuaternion);
        _args.push_back(segPt1);
        _args.push_back(segPt2);
        if(distanceThreshold)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*distanceThreshold);
        }
        else _brk = true;
        if(cache)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*cache);
        }
        else _brk = true;
        auto _ret = this->_client->call("simGeom.getPtcloudSegmentDistance", _args);
        return std::make_tuple(_ret[0].as<double>(), _ret[1].as<std::vector<double>>(), _ret[2].as<std::vector<double>>(), _ret[3].as<int64_t>());
    }

    std::string simGeom::getPtcloudSerializationData(int64_t octreeHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(octreeHandle);
        auto _ret = this->_client->call("simGeom.getPtcloudSerializationData", _args);
        return _ret[0].as<std::string>();
    }

    std::tuple<double, std::vector<double>, std::vector<double>, int64_t> simGeom::getPtcloudTriangleDistance(int64_t ptcloudHandle, std::vector<double> ptcloudPos, std::vector<double> ptcloudQuaternion, std::vector<double> triPt1, std::vector<double> triPt2, std::vector<double> triPt3, std::optional<double> distanceThreshold, std::optional<int64_t> cache)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(ptcloudHandle);
        _args.push_back(ptcloudPos);
        _args.push_back(ptcloudQuaternion);
        _args.push_back(triPt1);
        _args.push_back(triPt2);
        _args.push_back(triPt3);
        if(distanceThreshold)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*distanceThreshold);
        }
        else _brk = true;
        if(cache)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*cache);
        }
        else _brk = true;
        auto _ret = this->_client->call("simGeom.getPtcloudTriangleDistance", _args);
        return std::make_tuple(_ret[0].as<double>(), _ret[1].as<std::vector<double>>(), _ret[2].as<std::vector<double>>(), _ret[3].as<int64_t>());
    }

    std::tuple<double, std::vector<double>> simGeom::getSegmentPointDistance(std::vector<double> segmentPt1, std::vector<double> segmentPt2, std::vector<double> point)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(segmentPt1);
        _args.push_back(segmentPt2);
        _args.push_back(point);
        auto _ret = this->_client->call("simGeom.getSegmentPointDistance", _args);
        return std::make_tuple(_ret[0].as<double>(), _ret[1].as<std::vector<double>>());
    }

    std::tuple<double, std::vector<double>, std::vector<double>> simGeom::getSegmentSegmentDistance(std::vector<double> segment1Pt1, std::vector<double> segment1Pt2, std::vector<double> segment2Pt1, std::vector<double> segment2Pt2)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(segment1Pt1);
        _args.push_back(segment1Pt2);
        _args.push_back(segment2Pt1);
        _args.push_back(segment2Pt2);
        auto _ret = this->_client->call("simGeom.getSegmentSegmentDistance", _args);
        return std::make_tuple(_ret[0].as<double>(), _ret[1].as<std::vector<double>>(), _ret[2].as<std::vector<double>>());
    }

    std::vector<double> simGeom::getTransformedPoints(std::vector<double> points, std::vector<double> position, std::vector<double> quaternion)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(points);
        _args.push_back(position);
        _args.push_back(quaternion);
        auto _ret = this->_client->call("simGeom.getTransformedPoints", _args);
        return _ret[0].as<std::vector<double>>();
    }

    std::tuple<double, std::vector<double>> simGeom::getTrianglePointDistance(std::vector<double> triPt1, std::vector<double> triPt2, std::vector<double> triPt3, std::vector<double> point)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(triPt1);
        _args.push_back(triPt2);
        _args.push_back(triPt3);
        _args.push_back(point);
        auto _ret = this->_client->call("simGeom.getTrianglePointDistance", _args);
        return std::make_tuple(_ret[0].as<double>(), _ret[1].as<std::vector<double>>());
    }

    std::tuple<double, std::vector<double>, std::vector<double>> simGeom::getTriangleSegmentDistance(std::vector<double> triPt1, std::vector<double> triPt2, std::vector<double> triPt3, std::vector<double> segmentPt1, std::vector<double> segmentPt2)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(triPt1);
        _args.push_back(triPt2);
        _args.push_back(triPt3);
        _args.push_back(segmentPt1);
        _args.push_back(segmentPt2);
        auto _ret = this->_client->call("simGeom.getTriangleSegmentDistance", _args);
        return std::make_tuple(_ret[0].as<double>(), _ret[1].as<std::vector<double>>(), _ret[2].as<std::vector<double>>());
    }

    std::tuple<double, std::vector<double>, std::vector<double>> simGeom::getTriangleTriangleDistance(std::vector<double> tri1Pt1, std::vector<double> tri1Pt2, std::vector<double> tri1Pt3, std::vector<double> tri2Pt1, std::vector<double> tri2Pt2, std::vector<double> tri2Pt3)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(tri1Pt1);
        _args.push_back(tri1Pt2);
        _args.push_back(tri1Pt3);
        _args.push_back(tri2Pt1);
        _args.push_back(tri2Pt2);
        _args.push_back(tri2Pt3);
        auto _ret = this->_client->call("simGeom.getTriangleTriangleDistance", _args);
        return std::make_tuple(_ret[0].as<double>(), _ret[1].as<std::vector<double>>(), _ret[2].as<std::vector<double>>());
    }

    void simGeom::scaleMesh(int64_t meshHandle, double scaleFactor)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(meshHandle);
        _args.push_back(scaleFactor);
        auto _ret = this->_client->call("simGeom.scaleMesh", _args);
    }

    void simGeom::scaleOctree(int64_t octreeHandle, double scaleFactor)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(octreeHandle);
        _args.push_back(scaleFactor);
        auto _ret = this->_client->call("simGeom.scaleOctree", _args);
    }

    void simGeom::scalePtcloud(int64_t ptcloudHandle, double scaleFactor)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(ptcloudHandle);
        _args.push_back(scaleFactor);
        auto _ret = this->_client->call("simGeom.scalePtcloud", _args);
    }

    simICP::simICP(RemoteAPIClient *client)
        : _client(client)
    {
        _client->require("simICP");
    }

    std::vector<double> simICP::match(int64_t model_handle, int64_t template_handle, std::optional<double> outlier_treshold)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(model_handle);
        _args.push_back(template_handle);
        if(outlier_treshold)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*outlier_treshold);
        }
        else _brk = true;
        auto _ret = this->_client->call("simICP.match", _args);
        return _ret[0].as<std::vector<double>>();
    }

    std::vector<double> simICP::matchToShape(int64_t model_handle, int64_t template_handle, std::optional<double> outlier_treshold)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(model_handle);
        _args.push_back(template_handle);
        if(outlier_treshold)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*outlier_treshold);
        }
        else _brk = true;
        auto _ret = this->_client->call("simICP.matchToShape", _args);
        return _ret[0].as<std::vector<double>>();
    }

    simIK::simIK(RemoteAPIClient *client)
        : _client(client)
    {
        _client->require("simIK");
    }

    int64_t simIK::addElement(int64_t environmentHandle, int64_t ikGroupHandle, int64_t tipDummyHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(environmentHandle);
        _args.push_back(ikGroupHandle);
        _args.push_back(tipDummyHandle);
        auto _ret = this->_client->call("simIK.addElement", _args);
        return _ret[0].as<int64_t>();
    }

    std::tuple<int64_t, json, json> simIK::addElementFromScene(int64_t environmentHandle, int64_t ikGroup, int64_t baseHandle, int64_t tipHandle, int64_t targetHandle, int64_t constraints)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(environmentHandle);
        _args.push_back(ikGroup);
        _args.push_back(baseHandle);
        _args.push_back(tipHandle);
        _args.push_back(targetHandle);
        _args.push_back(constraints);
        auto _ret = this->_client->call("simIK.addElementFromScene", _args);
        return std::make_tuple(_ret[0].as<int64_t>(), _ret[1].as<json>(), _ret[2].as<json>());
    }

    std::tuple<std::vector<double>, std::vector<double>> simIK::computeGroupJacobian(int64_t environmentHandle, int64_t ikGroupHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(environmentHandle);
        _args.push_back(ikGroupHandle);
        auto _ret = this->_client->call("simIK.computeGroupJacobian", _args);
        return std::make_tuple(_ret[0].as<std::vector<double>>(), _ret[1].as<std::vector<double>>());
    }

    std::tuple<std::vector<double>, std::vector<double>> simIK::computeJacobian(int64_t environmentHandle, int64_t baseObject, int64_t lastJoint, int64_t constraints, std::vector<double> tipMatrix, std::optional<std::vector<double>> targetMatrix, std::optional<std::vector<double>> constrBaseMatrix)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(environmentHandle);
        _args.push_back(baseObject);
        _args.push_back(lastJoint);
        _args.push_back(constraints);
        _args.push_back(tipMatrix);
        if(targetMatrix)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*targetMatrix);
        }
        else _brk = true;
        if(constrBaseMatrix)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*constrBaseMatrix);
        }
        else _brk = true;
        auto _ret = this->_client->call("simIK.computeJacobian", _args);
        return std::make_tuple(_ret[0].as<std::vector<double>>(), _ret[1].as<std::vector<double>>());
    }

    int64_t simIK::createDebugOverlay(int64_t environmentHandle, int64_t tipHandle, std::optional<int64_t> baseHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(environmentHandle);
        _args.push_back(tipHandle);
        if(baseHandle)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*baseHandle);
        }
        else _brk = true;
        auto _ret = this->_client->call("simIK.createDebugOverlay", _args);
        return _ret[0].as<int64_t>();
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

    int64_t simIK::createEnvironment(std::optional<int64_t> flags)
    {
        bool _brk = false;
        json _args(json_array_arg);
        if(flags)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*flags);
        }
        else _brk = true;
        auto _ret = this->_client->call("simIK.createEnvironment", _args);
        return _ret[0].as<int64_t>();
    }

    int64_t simIK::createGroup(int64_t environmentHandle, std::optional<std::string> ikGroupName)
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
        auto _ret = this->_client->call("simIK.createGroup", _args);
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

    bool simIK::doesGroupExist(int64_t environmentHandle, std::string ikGroupName)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(environmentHandle);
        _args.push_back(ikGroupName);
        auto _ret = this->_client->call("simIK.doesGroupExist", _args);
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

    void simIK::eraseDebugOverlay(int64_t debugObject)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(debugObject);
        auto _ret = this->_client->call("simIK.eraseDebugOverlay", _args);
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

    std::vector<json> simIK::findConfigs(int64_t envHandle, int64_t ikGroupHandle, std::vector<int64_t> jointHandles, std::optional<json> params, std::optional<std::vector<json>> configs)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(envHandle);
        _args.push_back(ikGroupHandle);
        _args.push_back(jointHandles);
        if(params)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*params);
        }
        else _brk = true;
        if(configs)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*configs);
        }
        else _brk = true;
        auto _ret = this->_client->call("simIK.findConfigs", _args);
        return _ret[0].as<std::vector<json>>();
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

    std::tuple<int64_t, int64_t> simIK::getElementBase(int64_t environmentHandle, int64_t ikGroupHandle, int64_t elementHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(environmentHandle);
        _args.push_back(ikGroupHandle);
        _args.push_back(elementHandle);
        auto _ret = this->_client->call("simIK.getElementBase", _args);
        return std::make_tuple(_ret[0].as<int64_t>(), _ret[1].as<int64_t>());
    }

    int64_t simIK::getElementConstraints(int64_t environmentHandle, int64_t ikGroupHandle, int64_t elementHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(environmentHandle);
        _args.push_back(ikGroupHandle);
        _args.push_back(elementHandle);
        auto _ret = this->_client->call("simIK.getElementConstraints", _args);
        return _ret[0].as<int64_t>();
    }

    int64_t simIK::getElementFlags(int64_t environmentHandle, int64_t ikGroupHandle, int64_t elementHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(environmentHandle);
        _args.push_back(ikGroupHandle);
        _args.push_back(elementHandle);
        auto _ret = this->_client->call("simIK.getElementFlags", _args);
        return _ret[0].as<int64_t>();
    }

    std::vector<double> simIK::getElementPrecision(int64_t environmentHandle, int64_t ikGroupHandle, int64_t elementHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(environmentHandle);
        _args.push_back(ikGroupHandle);
        _args.push_back(elementHandle);
        auto _ret = this->_client->call("simIK.getElementPrecision", _args);
        return _ret[0].as<std::vector<double>>();
    }

    std::vector<double> simIK::getElementWeights(int64_t environmentHandle, int64_t ikGroupHandle, int64_t elementHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(environmentHandle);
        _args.push_back(ikGroupHandle);
        _args.push_back(elementHandle);
        auto _ret = this->_client->call("simIK.getElementWeights", _args);
        return _ret[0].as<std::vector<double>>();
    }

    std::string simIK::getFailureDescription(int64_t reason)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(reason);
        auto _ret = this->_client->call("simIK.getFailureDescription", _args);
        return _ret[0].as<std::string>();
    }

    std::tuple<int64_t, double, int64_t> simIK::getGroupCalculation(int64_t environmentHandle, int64_t ikGroupHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(environmentHandle);
        _args.push_back(ikGroupHandle);
        auto _ret = this->_client->call("simIK.getGroupCalculation", _args);
        return std::make_tuple(_ret[0].as<int64_t>(), _ret[1].as<double>(), _ret[2].as<int64_t>());
    }

    int64_t simIK::getGroupFlags(int64_t environmentHandle, int64_t ikGroupHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(environmentHandle);
        _args.push_back(ikGroupHandle);
        auto _ret = this->_client->call("simIK.getGroupFlags", _args);
        return _ret[0].as<int64_t>();
    }

    int64_t simIK::getGroupHandle(int64_t environmentHandle, std::string ikGroupName)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(environmentHandle);
        _args.push_back(ikGroupName);
        auto _ret = this->_client->call("simIK.getGroupHandle", _args);
        return _ret[0].as<int64_t>();
    }

    std::tuple<std::vector<int64_t>, std::vector<double>> simIK::getGroupJointLimitHits(int64_t environmentHandle, int64_t ikGroupHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(environmentHandle);
        _args.push_back(ikGroupHandle);
        auto _ret = this->_client->call("simIK.getGroupJointLimitHits", _args);
        return std::make_tuple(_ret[0].as<std::vector<int64_t>>(), _ret[1].as<std::vector<double>>());
    }

    std::vector<int64_t> simIK::getGroupJoints(int64_t environmentHandle, int64_t ikGroupHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(environmentHandle);
        _args.push_back(ikGroupHandle);
        auto _ret = this->_client->call("simIK.getGroupJoints", _args);
        return _ret[0].as<std::vector<int64_t>>();
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

    std::tuple<bool, std::vector<double>> simIK::getJointInterval(int64_t environmentHandle, int64_t jointHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(environmentHandle);
        _args.push_back(jointHandle);
        auto _ret = this->_client->call("simIK.getJointInterval", _args);
        return std::make_tuple(_ret[0].as<bool>(), _ret[1].as<std::vector<double>>());
    }

    double simIK::getJointLimitMargin(int64_t environmentHandle, int64_t jointHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(environmentHandle);
        _args.push_back(jointHandle);
        auto _ret = this->_client->call("simIK.getJointLimitMargin", _args);
        return _ret[0].as<double>();
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

    double simIK::getJointScrewLead(int64_t environmentHandle, int64_t jointHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(environmentHandle);
        _args.push_back(jointHandle);
        auto _ret = this->_client->call("simIK.getJointScrewLead", _args);
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

    double simIK::getJointWeight(int64_t environmentHandle, int64_t jointHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(environmentHandle);
        _args.push_back(jointHandle);
        auto _ret = this->_client->call("simIK.getJointWeight", _args);
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

    std::vector<double> simIK::getObjectMatrix(int64_t environmentHandle, int64_t objectHandle, std::optional<int64_t> relativeToObjectHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(environmentHandle);
        _args.push_back(objectHandle);
        if(relativeToObjectHandle)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*relativeToObjectHandle);
        }
        else _brk = true;
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

    std::vector<double> simIK::getObjectPose(int64_t environmentHandle, int64_t objectHandle, std::optional<int64_t> relativeToObjectHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(environmentHandle);
        _args.push_back(objectHandle);
        if(relativeToObjectHandle)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*relativeToObjectHandle);
        }
        else _brk = true;
        auto _ret = this->_client->call("simIK.getObjectPose", _args);
        return _ret[0].as<std::vector<double>>();
    }

    std::tuple<std::vector<double>, std::vector<double>, std::vector<double>> simIK::getObjectTransformation(int64_t environmentHandle, int64_t objectHandle, std::optional<int64_t> relativeToObjectHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(environmentHandle);
        _args.push_back(objectHandle);
        if(relativeToObjectHandle)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*relativeToObjectHandle);
        }
        else _brk = true;
        auto _ret = this->_client->call("simIK.getObjectTransformation", _args);
        return std::make_tuple(_ret[0].as<std::vector<double>>(), _ret[1].as<std::vector<double>>(), _ret[2].as<std::vector<double>>());
    }

    int64_t simIK::getObjectType(int64_t environmentHandle, int64_t objectHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(environmentHandle);
        _args.push_back(objectHandle);
        auto _ret = this->_client->call("simIK.getObjectType", _args);
        return _ret[0].as<int64_t>();
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

    int64_t simIK::getTargetDummy(int64_t environmentHandle, int64_t dummyHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(environmentHandle);
        _args.push_back(dummyHandle);
        auto _ret = this->_client->call("simIK.getTargetDummy", _args);
        return _ret[0].as<int64_t>();
    }

    std::tuple<int64_t, int64_t, std::vector<double>> simIK::handleGroup(int64_t environmentHandle, int64_t ikGroup, std::optional<json> options)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(environmentHandle);
        _args.push_back(ikGroup);
        if(options)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*options);
        }
        else _brk = true;
        auto _ret = this->_client->call("simIK.handleGroup", _args);
        return std::make_tuple(_ret[0].as<int64_t>(), _ret[1].as<int64_t>(), _ret[2].as<std::vector<double>>());
    }

    std::tuple<int64_t, int64_t, std::vector<double>> simIK::handleGroups(int64_t environmentHandle, std::vector<int64_t> ikGroups, std::optional<json> options)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(environmentHandle);
        _args.push_back(ikGroups);
        if(options)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*options);
        }
        else _brk = true;
        auto _ret = this->_client->call("simIK.handleGroups", _args);
        return std::make_tuple(_ret[0].as<int64_t>(), _ret[1].as<int64_t>(), _ret[2].as<std::vector<double>>());
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

    void simIK::setElementBase(int64_t environmentHandle, int64_t ikGroupHandle, int64_t elementHandle, int64_t baseHandle, std::optional<int64_t> constraintsBaseHandle)
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
        auto _ret = this->_client->call("simIK.setElementBase", _args);
    }

    void simIK::setElementConstraints(int64_t environmentHandle, int64_t ikGroupHandle, int64_t elementHandle, int64_t constraints)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(environmentHandle);
        _args.push_back(ikGroupHandle);
        _args.push_back(elementHandle);
        _args.push_back(constraints);
        auto _ret = this->_client->call("simIK.setElementConstraints", _args);
    }

    void simIK::setElementFlags(int64_t environmentHandle, int64_t ikGroupHandle, int64_t elementHandle, int64_t flags)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(environmentHandle);
        _args.push_back(ikGroupHandle);
        _args.push_back(elementHandle);
        _args.push_back(flags);
        auto _ret = this->_client->call("simIK.setElementFlags", _args);
    }

    void simIK::setElementPrecision(int64_t environmentHandle, int64_t ikGroupHandle, int64_t elementHandle, std::vector<double> precision)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(environmentHandle);
        _args.push_back(ikGroupHandle);
        _args.push_back(elementHandle);
        _args.push_back(precision);
        auto _ret = this->_client->call("simIK.setElementPrecision", _args);
    }

    void simIK::setElementWeights(int64_t environmentHandle, int64_t ikGroupHandle, int64_t elementHandle, std::vector<double> weights)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(environmentHandle);
        _args.push_back(ikGroupHandle);
        _args.push_back(elementHandle);
        _args.push_back(weights);
        auto _ret = this->_client->call("simIK.setElementWeights", _args);
    }

    void simIK::setGroupCalculation(int64_t environmentHandle, int64_t ikGroupHandle, int64_t method, double damping, int64_t maxIterations)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(environmentHandle);
        _args.push_back(ikGroupHandle);
        _args.push_back(method);
        _args.push_back(damping);
        _args.push_back(maxIterations);
        auto _ret = this->_client->call("simIK.setGroupCalculation", _args);
    }

    void simIK::setGroupFlags(int64_t environmentHandle, int64_t ikGroupHandle, int64_t flags)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(environmentHandle);
        _args.push_back(ikGroupHandle);
        _args.push_back(flags);
        auto _ret = this->_client->call("simIK.setGroupFlags", _args);
    }

    void simIK::setJointDependency(int64_t environmentHandle, int64_t jointHandle, int64_t masterJointHandle, std::optional<double> offset, std::optional<double> mult, std::optional<std::string> callback)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(environmentHandle);
        _args.push_back(jointHandle);
        _args.push_back(masterJointHandle);
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
        if(callback)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*callback);
        }
        else _brk = true;
        auto _ret = this->_client->call("simIK.setJointDependency", _args);
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

    void simIK::setJointLimitMargin(int64_t environmentHandle, int64_t jointHandle, double margin)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(environmentHandle);
        _args.push_back(jointHandle);
        _args.push_back(margin);
        auto _ret = this->_client->call("simIK.setJointLimitMargin", _args);
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

    void simIK::setJointScrewLead(int64_t environmentHandle, int64_t jointHandle, double lead)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(environmentHandle);
        _args.push_back(jointHandle);
        _args.push_back(lead);
        auto _ret = this->_client->call("simIK.setJointScrewLead", _args);
    }

    void simIK::setJointWeight(int64_t environmentHandle, int64_t jointHandle, double weight)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(environmentHandle);
        _args.push_back(jointHandle);
        _args.push_back(weight);
        auto _ret = this->_client->call("simIK.setJointWeight", _args);
    }

    void simIK::setObjectMatrix(int64_t environmentHandle, int64_t objectHandle, std::vector<double> matrix, std::optional<int64_t> relativeToObjectHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(environmentHandle);
        _args.push_back(objectHandle);
        _args.push_back(matrix);
        if(relativeToObjectHandle)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*relativeToObjectHandle);
        }
        else _brk = true;
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

    void simIK::setObjectPose(int64_t environmentHandle, int64_t objectHandle, std::vector<double> pose, std::optional<int64_t> relativeToObjectHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(environmentHandle);
        _args.push_back(objectHandle);
        _args.push_back(pose);
        if(relativeToObjectHandle)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*relativeToObjectHandle);
        }
        else _brk = true;
        auto _ret = this->_client->call("simIK.setObjectPose", _args);
    }

    void simIK::setObjectTransformation(int64_t environmentHandle, int64_t objectHandle, std::vector<double> position, std::vector<double> eulerOrQuaternion, std::optional<int64_t> relativeToObjectHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(environmentHandle);
        _args.push_back(objectHandle);
        _args.push_back(position);
        _args.push_back(eulerOrQuaternion);
        if(relativeToObjectHandle)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*relativeToObjectHandle);
        }
        else _brk = true;
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

    void simIK::setTargetDummy(int64_t environmentHandle, int64_t dummyHandle, int64_t targetDummyHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(environmentHandle);
        _args.push_back(dummyHandle);
        _args.push_back(targetDummyHandle);
        auto _ret = this->_client->call("simIK.setTargetDummy", _args);
    }

    void simIK::syncFromSim(int64_t environmentHandle, std::vector<int64_t> ikGroups)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(environmentHandle);
        _args.push_back(ikGroups);
        auto _ret = this->_client->call("simIK.syncFromSim", _args);
    }

    void simIK::syncToSim(int64_t environmentHandle, std::vector<int64_t> ikGroups)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(environmentHandle);
        _args.push_back(ikGroups);
        auto _ret = this->_client->call("simIK.syncToSim", _args);
    }

    simLDraw::simLDraw(RemoteAPIClient *client)
        : _client(client)
    {
        _client->require("simLDraw");
    }

    std::vector<int64_t> simLDraw::import(std::string filePath)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(filePath);
        auto _ret = this->_client->call("simLDraw.import", _args);
        return _ret[0].as<std::vector<int64_t>>();
    }

    simLuaCmd::simLuaCmd(RemoteAPIClient *client)
        : _client(client)
    {
        _client->require("simLuaCmd");
    }

    void simLuaCmd::clearHistory()
    {
        bool _brk = false;
        json _args(json_array_arg);
        auto _ret = this->_client->call("simLuaCmd.clearHistory", _args);
    }

    void simLuaCmd::setExecWrapper(std::string wrapperFunc)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(wrapperFunc);
        auto _ret = this->_client->call("simLuaCmd.setExecWrapper", _args);
    }

    void simLuaCmd::setVisible(bool b)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(b);
        auto _ret = this->_client->call("simLuaCmd.setVisible", _args);
    }

    simMTB::simMTB(RemoteAPIClient *client)
        : _client(client)
    {
        _client->require("simMTB");
    }

    bool simMTB::connectInput(int64_t inputMtbServerHandle, int64_t inputBitNumber, int64_t outputMtbServerHandle, int64_t outputBitNumber, int64_t connectionType)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(inputMtbServerHandle);
        _args.push_back(inputBitNumber);
        _args.push_back(outputMtbServerHandle);
        _args.push_back(outputBitNumber);
        _args.push_back(connectionType);
        auto _ret = this->_client->call("simMTB.connectInput", _args);
        return _ret[0].as<bool>();
    }

    bool simMTB::disconnectInput(int64_t inputMtbServerHandle, int64_t inputBitNumber)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(inputMtbServerHandle);
        _args.push_back(inputBitNumber);
        auto _ret = this->_client->call("simMTB.disconnectInput", _args);
        return _ret[0].as<bool>();
    }

    std::vector<int64_t> simMTB::getInput(int64_t mtbServerHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(mtbServerHandle);
        auto _ret = this->_client->call("simMTB.getInput", _args);
        return _ret[0].as<std::vector<int64_t>>();
    }

    std::vector<double> simMTB::getJoints(int64_t mtbServerHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(mtbServerHandle);
        auto _ret = this->_client->call("simMTB.getJoints", _args);
        return _ret[0].as<std::vector<double>>();
    }

    std::vector<int64_t> simMTB::getOutput(int64_t mtbServerHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(mtbServerHandle);
        auto _ret = this->_client->call("simMTB.getOutput", _args);
        return _ret[0].as<std::vector<int64_t>>();
    }

    bool simMTB::setInput(int64_t mtbServerHandle, std::vector<int64_t> inputValues)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(mtbServerHandle);
        _args.push_back(inputValues);
        auto _ret = this->_client->call("simMTB.setInput", _args);
        return _ret[0].as<bool>();
    }

    std::tuple<int64_t, std::string> simMTB::startServer(std::string mtbServerExecutable, int64_t portNumber, std::vector<uint8_t> program, std::vector<double> jointPositions, std::vector<double> velocities)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(mtbServerExecutable);
        _args.push_back(portNumber);
        _args.push_back(bin(program));
        _args.push_back(jointPositions);
        _args.push_back(velocities);
        auto _ret = this->_client->call("simMTB.startServer", _args);
        return std::make_tuple(_ret[0].as<int64_t>(), _ret[1].as<std::string>());
    }

    std::tuple<int64_t, std::string> simMTB::step(int64_t mtbServerHandle, double timeStep)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(mtbServerHandle);
        _args.push_back(timeStep);
        auto _ret = this->_client->call("simMTB.step", _args);
        return std::make_tuple(_ret[0].as<int64_t>(), _ret[1].as<std::string>());
    }

    bool simMTB::stopServer(int64_t mtbServerHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(mtbServerHandle);
        auto _ret = this->_client->call("simMTB.stopServer", _args);
        return _ret[0].as<bool>();
    }

    simMujoco::simMujoco(RemoteAPIClient *client)
        : _client(client)
    {
        _client->require("simMujoco");
    }

    int64_t simMujoco::composite(std::string xml, json info)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(xml);
        _args.push_back(info);
        auto _ret = this->_client->call("simMujoco.composite", _args);
        return _ret[0].as<int64_t>();
    }

    int64_t simMujoco::flexcomp(std::string xml, json info)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(xml);
        _args.push_back(info);
        auto _ret = this->_client->call("simMujoco.flexcomp", _args);
        return _ret[0].as<int64_t>();
    }

    json simMujoco::getCompositeInfo(int64_t injectionId, int64_t what)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(injectionId);
        _args.push_back(what);
        auto _ret = this->_client->call("simMujoco.getCompositeInfo", _args);
        return _ret[0].as<json>();
    }

    json simMujoco::getFlexcompInfo(int64_t injectionId, int64_t what)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(injectionId);
        _args.push_back(what);
        auto _ret = this->_client->call("simMujoco.getFlexcompInfo", _args);
        return _ret[0].as<json>();
    }

    std::string simMujoco::getInfo(std::string what)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(what);
        auto _ret = this->_client->call("simMujoco.getInfo", _args);
        return _ret[0].as<std::string>();
    }

    int64_t simMujoco::injectXML(std::string xml, std::string element, json info)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(xml);
        _args.push_back(element);
        _args.push_back(info);
        auto _ret = this->_client->call("simMujoco.injectXML", _args);
        return _ret[0].as<int64_t>();
    }

    void simMujoco::removeXML(int64_t injectionId)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(injectionId);
        auto _ret = this->_client->call("simMujoco.removeXML", _args);
    }

    simOpenMesh::simOpenMesh(RemoteAPIClient *client)
        : _client(client)
    {
        _client->require("simOpenMesh");
    }

    int64_t simOpenMesh::decimate(int64_t inputShape, std::optional<json> params)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(inputShape);
        if(params)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*params);
        }
        else _brk = true;
        auto _ret = this->_client->call("simOpenMesh.decimate", _args);
        return _ret[0].as<int64_t>();
    }

    simPython::simPython(RemoteAPIClient *client)
        : _client(client)
    {
        _client->require("simPython");
    }

    json simPython::call(std::string scriptStateHandle, std::string func, json args)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(scriptStateHandle);
        _args.push_back(func);
        _args.push_back(args);
        auto _ret = this->_client->call("simPython.call", _args);
        return _ret[0].as<json>();
    }

    std::string simPython::create()
    {
        bool _brk = false;
        json _args(json_array_arg);
        auto _ret = this->_client->call("simPython.create", _args);
        return _ret[0].as<std::string>();
    }

    void simPython::destroy(std::string scriptStateHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(scriptStateHandle);
        auto _ret = this->_client->call("simPython.destroy", _args);
    }

    std::vector<int64_t> simPython::getVersion()
    {
        bool _brk = false;
        json _args(json_array_arg);
        auto _ret = this->_client->call("simPython.getVersion", _args);
        return _ret[0].as<std::vector<int64_t>>();
    }

    json simPython::run(std::string scriptStateHandle, std::string code)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(scriptStateHandle);
        _args.push_back(code);
        auto _ret = this->_client->call("simPython.run", _args);
        return _ret[0].as<json>();
    }

    simROS2::simROS2(RemoteAPIClient *client)
        : _client(client)
    {
        _client->require("simROS2");
    }

    void simROS2::actionClientTreatUInt8ArrayAsString(std::string actionClientHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(actionClientHandle);
        auto _ret = this->_client->call("simROS2.actionClientTreatUInt8ArrayAsString", _args);
    }

    void simROS2::actionServerActionAbort(std::string actionServerHandle, std::string goalUUID, json result)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(actionServerHandle);
        _args.push_back(goalUUID);
        _args.push_back(result);
        auto _ret = this->_client->call("simROS2.actionServerActionAbort", _args);
    }

    void simROS2::actionServerActionCanceled(std::string actionServerHandle, std::string goalUUID, json result)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(actionServerHandle);
        _args.push_back(goalUUID);
        _args.push_back(result);
        auto _ret = this->_client->call("simROS2.actionServerActionCanceled", _args);
    }

    void simROS2::actionServerActionExecute(std::string actionServerHandle, std::string goalUUID)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(actionServerHandle);
        _args.push_back(goalUUID);
        auto _ret = this->_client->call("simROS2.actionServerActionExecute", _args);
    }

    bool simROS2::actionServerActionIsActive(std::string actionServerHandle, std::string goalUUID)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(actionServerHandle);
        _args.push_back(goalUUID);
        auto _ret = this->_client->call("simROS2.actionServerActionIsActive", _args);
        return _ret[0].as<bool>();
    }

    bool simROS2::actionServerActionIsCanceling(std::string actionServerHandle, std::string goalUUID)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(actionServerHandle);
        _args.push_back(goalUUID);
        auto _ret = this->_client->call("simROS2.actionServerActionIsCanceling", _args);
        return _ret[0].as<bool>();
    }

    bool simROS2::actionServerActionIsExecuting(std::string actionServerHandle, std::string goalUUID)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(actionServerHandle);
        _args.push_back(goalUUID);
        auto _ret = this->_client->call("simROS2.actionServerActionIsExecuting", _args);
        return _ret[0].as<bool>();
    }

    void simROS2::actionServerActionSucceed(std::string actionServerHandle, std::string goalUUID, json result)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(actionServerHandle);
        _args.push_back(goalUUID);
        _args.push_back(result);
        auto _ret = this->_client->call("simROS2.actionServerActionSucceed", _args);
    }

    void simROS2::actionServerPublishFeedback(std::string actionServerHandle, std::string goalUUID, json feedback)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(actionServerHandle);
        _args.push_back(goalUUID);
        _args.push_back(feedback);
        auto _ret = this->_client->call("simROS2.actionServerPublishFeedback", _args);
    }

    void simROS2::actionServerTreatUInt8ArrayAsString(std::string actionServerHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(actionServerHandle);
        auto _ret = this->_client->call("simROS2.actionServerTreatUInt8ArrayAsString", _args);
    }

    json simROS2::call(std::string clientHandle, json request)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(clientHandle);
        _args.push_back(request);
        auto _ret = this->_client->call("simROS2.call", _args);
        return _ret[0].as<json>();
    }

    bool simROS2::cancelLastGoal(std::string actionClientHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(actionClientHandle);
        auto _ret = this->_client->call("simROS2.cancelLastGoal", _args);
        return _ret[0].as<bool>();
    }

    void simROS2::clientTreatUInt8ArrayAsString(std::string clientHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(clientHandle);
        auto _ret = this->_client->call("simROS2.clientTreatUInt8ArrayAsString", _args);
    }

    std::string simROS2::createActionClient(std::string actionName, std::string actionType, std::string goalResponseCallback, std::string feedbackCallback, std::string resultCallback)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(actionName);
        _args.push_back(actionType);
        _args.push_back(goalResponseCallback);
        _args.push_back(feedbackCallback);
        _args.push_back(resultCallback);
        auto _ret = this->_client->call("simROS2.createActionClient", _args);
        return _ret[0].as<std::string>();
    }

    std::string simROS2::createActionServer(std::string actionName, std::string actionType, std::string handleGoalCallback, std::string handleCancelCallback, std::string handleAcceptedCallback)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(actionName);
        _args.push_back(actionType);
        _args.push_back(handleGoalCallback);
        _args.push_back(handleCancelCallback);
        _args.push_back(handleAcceptedCallback);
        auto _ret = this->_client->call("simROS2.createActionServer", _args);
        return _ret[0].as<std::string>();
    }

    std::string simROS2::createClient(std::string serviceName, std::string serviceType)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(serviceName);
        _args.push_back(serviceType);
        auto _ret = this->_client->call("simROS2.createClient", _args);
        return _ret[0].as<std::string>();
    }

    json simROS2::createInterface(std::string type)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(type);
        auto _ret = this->_client->call("simROS2.createInterface", _args);
        return _ret[0].as<json>();
    }

    std::string simROS2::createPublisher(std::string topicName, std::string topicType, std::optional<int64_t> unused, std::optional<bool> unused2, std::optional<json> qos)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(topicName);
        _args.push_back(topicType);
        if(unused)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*unused);
        }
        else _brk = true;
        if(unused2)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*unused2);
        }
        else _brk = true;
        if(qos)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*qos);
        }
        else _brk = true;
        auto _ret = this->_client->call("simROS2.createPublisher", _args);
        return _ret[0].as<std::string>();
    }

    std::string simROS2::createService(std::string serviceName, std::string serviceType, std::string serviceCallback)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(serviceName);
        _args.push_back(serviceType);
        _args.push_back(serviceCallback);
        auto _ret = this->_client->call("simROS2.createService", _args);
        return _ret[0].as<std::string>();
    }

    std::string simROS2::createSubscription(std::string topicName, std::string topicType, std::string topicCallback, std::optional<int64_t> unused, std::optional<json> qos)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(topicName);
        _args.push_back(topicType);
        _args.push_back(topicCallback);
        if(unused)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*unused);
        }
        else _brk = true;
        if(qos)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*qos);
        }
        else _brk = true;
        auto _ret = this->_client->call("simROS2.createSubscription", _args);
        return _ret[0].as<std::string>();
    }

    void simROS2::deleteParam(std::string name)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(name);
        auto _ret = this->_client->call("simROS2.deleteParam", _args);
    }

    json simROS2::getInterfaceConstants(std::string type)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(type);
        auto _ret = this->_client->call("simROS2.getInterfaceConstants", _args);
        return _ret[0].as<json>();
    }

    std::tuple<bool, bool> simROS2::getParamBool(std::string name, std::optional<bool> defaultValue)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(name);
        if(defaultValue)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*defaultValue);
        }
        else _brk = true;
        auto _ret = this->_client->call("simROS2.getParamBool", _args);
        return std::make_tuple(_ret[0].as<bool>(), _ret[1].as<bool>());
    }

    std::tuple<bool, double> simROS2::getParamDouble(std::string name, std::optional<double> defaultValue)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(name);
        if(defaultValue)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*defaultValue);
        }
        else _brk = true;
        auto _ret = this->_client->call("simROS2.getParamDouble", _args);
        return std::make_tuple(_ret[0].as<bool>(), _ret[1].as<double>());
    }

    std::tuple<bool, int64_t> simROS2::getParamInt(std::string name, std::optional<int64_t> defaultValue)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(name);
        if(defaultValue)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*defaultValue);
        }
        else _brk = true;
        auto _ret = this->_client->call("simROS2.getParamInt", _args);
        return std::make_tuple(_ret[0].as<bool>(), _ret[1].as<int64_t>());
    }

    std::tuple<bool, std::string> simROS2::getParamString(std::string name, std::optional<std::string> defaultValue)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(name);
        if(defaultValue)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*defaultValue);
        }
        else _brk = true;
        auto _ret = this->_client->call("simROS2.getParamString", _args);
        return std::make_tuple(_ret[0].as<bool>(), _ret[1].as<std::string>());
    }

    void simROS2::getSimulationTime()
    {
        bool _brk = false;
        json _args(json_array_arg);
        auto _ret = this->_client->call("simROS2.getSimulationTime", _args);
    }

    void simROS2::getSystemTime()
    {
        bool _brk = false;
        json _args(json_array_arg);
        auto _ret = this->_client->call("simROS2.getSystemTime", _args);
    }

    json simROS2::getTime(std::optional<int64_t> clock_type)
    {
        bool _brk = false;
        json _args(json_array_arg);
        if(clock_type)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*clock_type);
        }
        else _brk = true;
        auto _ret = this->_client->call("simROS2.getTime", _args);
        return _ret[0].as<json>();
    }

    bool simROS2::hasParam(std::string name)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(name);
        auto _ret = this->_client->call("simROS2.hasParam", _args);
        return _ret[0].as<bool>();
    }

    std::string simROS2::imageTransportCreatePublisher(std::string topicName, std::optional<int64_t> queueSize)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(topicName);
        if(queueSize)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*queueSize);
        }
        else _brk = true;
        auto _ret = this->_client->call("simROS2.imageTransportCreatePublisher", _args);
        return _ret[0].as<std::string>();
    }

    std::string simROS2::imageTransportCreateSubscription(std::string topicName, std::string topicCallback, std::optional<int64_t> queueSize)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(topicName);
        _args.push_back(topicCallback);
        if(queueSize)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*queueSize);
        }
        else _brk = true;
        auto _ret = this->_client->call("simROS2.imageTransportCreateSubscription", _args);
        return _ret[0].as<std::string>();
    }

    void simROS2::imageTransportPublish(std::string publisherHandle, std::vector<uint8_t> data, int64_t width, int64_t height, std::string frame_id)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(publisherHandle);
        _args.push_back(bin(data));
        _args.push_back(width);
        _args.push_back(height);
        _args.push_back(frame_id);
        auto _ret = this->_client->call("simROS2.imageTransportPublish", _args);
    }

    void simROS2::imageTransportShutdownPublisher(std::string publisherHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(publisherHandle);
        auto _ret = this->_client->call("simROS2.imageTransportShutdownPublisher", _args);
    }

    void simROS2::imageTransportShutdownSubscription(std::string subscriptionHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(subscriptionHandle);
        auto _ret = this->_client->call("simROS2.imageTransportShutdownSubscription", _args);
    }

    void simROS2::importInterface(std::string name)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(name);
        auto _ret = this->_client->call("simROS2.importInterface", _args);
    }

    void simROS2::publish(std::string publisherHandle, json message)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(publisherHandle);
        _args.push_back(message);
        auto _ret = this->_client->call("simROS2.publish", _args);
    }

    void simROS2::publisherTreatUInt8ArrayAsString(std::string publisherHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(publisherHandle);
        auto _ret = this->_client->call("simROS2.publisherTreatUInt8ArrayAsString", _args);
    }

    bool simROS2::sendGoal(std::string actionClientHandle, json goal)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(actionClientHandle);
        _args.push_back(goal);
        auto _ret = this->_client->call("simROS2.sendGoal", _args);
        return _ret[0].as<bool>();
    }

    void simROS2::sendTransform(json transform)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(transform);
        auto _ret = this->_client->call("simROS2.sendTransform", _args);
    }

    void simROS2::sendTransforms(json transforms)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(transforms);
        auto _ret = this->_client->call("simROS2.sendTransforms", _args);
    }

    void simROS2::serviceTreatUInt8ArrayAsString(std::string serviceHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(serviceHandle);
        auto _ret = this->_client->call("simROS2.serviceTreatUInt8ArrayAsString", _args);
    }

    void simROS2::setParamBool(std::string name, bool value)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(name);
        _args.push_back(value);
        auto _ret = this->_client->call("simROS2.setParamBool", _args);
    }

    void simROS2::setParamDouble(std::string name, double value)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(name);
        _args.push_back(value);
        auto _ret = this->_client->call("simROS2.setParamDouble", _args);
    }

    void simROS2::setParamInt(std::string name, int64_t value)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(name);
        _args.push_back(value);
        auto _ret = this->_client->call("simROS2.setParamInt", _args);
    }

    void simROS2::setParamString(std::string name, std::string value)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(name);
        _args.push_back(value);
        auto _ret = this->_client->call("simROS2.setParamString", _args);
    }

    void simROS2::shutdownActionClient(std::string actionClientHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(actionClientHandle);
        auto _ret = this->_client->call("simROS2.shutdownActionClient", _args);
    }

    void simROS2::shutdownActionServer(std::string actionServerHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(actionServerHandle);
        auto _ret = this->_client->call("simROS2.shutdownActionServer", _args);
    }

    void simROS2::shutdownClient(std::string clientHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(clientHandle);
        auto _ret = this->_client->call("simROS2.shutdownClient", _args);
    }

    void simROS2::shutdownPublisher(std::string publisherHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(publisherHandle);
        auto _ret = this->_client->call("simROS2.shutdownPublisher", _args);
    }

    void simROS2::shutdownService(std::string serviceHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(serviceHandle);
        auto _ret = this->_client->call("simROS2.shutdownService", _args);
    }

    void simROS2::shutdownSubscription(std::string subscriptionHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(subscriptionHandle);
        auto _ret = this->_client->call("simROS2.shutdownSubscription", _args);
    }

    void simROS2::spinSome()
    {
        bool _brk = false;
        json _args(json_array_arg);
        auto _ret = this->_client->call("simROS2.spinSome", _args);
    }

    void simROS2::subscriptionTreatUInt8ArrayAsString(std::string subscriptionHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(subscriptionHandle);
        auto _ret = this->_client->call("simROS2.subscriptionTreatUInt8ArrayAsString", _args);
    }

    std::vector<std::string> simROS2::supportedInterfaces()
    {
        bool _brk = false;
        json _args(json_array_arg);
        auto _ret = this->_client->call("simROS2.supportedInterfaces", _args);
        return _ret[0].as<std::vector<std::string>>();
    }

    void simROS2::timeFromFloat(double t)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(t);
        auto _ret = this->_client->call("simROS2.timeFromFloat", _args);
    }

    void simROS2::timeToFloat(json t)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(t);
        auto _ret = this->_client->call("simROS2.timeToFloat", _args);
    }

    bool simROS2::waitForService(std::string clientHandle, double timeout)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(clientHandle);
        _args.push_back(timeout);
        auto _ret = this->_client->call("simROS2.waitForService", _args);
        return _ret[0].as<bool>();
    }

    simRRS1::simRRS1(RemoteAPIClient *client)
        : _client(client)
    {
        _client->require("simRRS1");
    }

    int64_t simRRS1::CANCEL_EVENT(std::vector<uint8_t> rcsHandle, int64_t eventId)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(bin(rcsHandle));
        _args.push_back(eventId);
        auto _ret = this->_client->call("simRRS1.CANCEL_EVENT", _args);
        return _ret[0].as<int64_t>();
    }

    int64_t simRRS1::CANCEL_FLYBY_CRITERIA(std::vector<uint8_t> rcsHandle, int64_t paramNumber)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(bin(rcsHandle));
        _args.push_back(paramNumber);
        auto _ret = this->_client->call("simRRS1.CANCEL_FLYBY_CRITERIA", _args);
        return _ret[0].as<int64_t>();
    }

    int64_t simRRS1::CANCEL_MOTION(std::vector<uint8_t> rcsHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(bin(rcsHandle));
        auto _ret = this->_client->call("simRRS1.CANCEL_MOTION", _args);
        return _ret[0].as<int64_t>();
    }

    int64_t simRRS1::CONTINUE_MOTION(std::vector<uint8_t> rcsHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(bin(rcsHandle));
        auto _ret = this->_client->call("simRRS1.CONTINUE_MOTION", _args);
        return _ret[0].as<int64_t>();
    }

    std::tuple<int64_t, std::vector<uint8_t>, std::string> simRRS1::CONTROLLER_POSITION_TO_MATRIX(std::vector<uint8_t> rcsHandle, std::string contrPos)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(bin(rcsHandle));
        _args.push_back(contrPos);
        auto _ret = this->_client->call("simRRS1.CONTROLLER_POSITION_TO_MATRIX", _args);
        return std::make_tuple(_ret[0].as<int64_t>(), _ret[1].as<std::vector<uint8_t>>(), _ret[2].as<std::string>());
    }

    int64_t simRRS1::DEBUG(std::vector<uint8_t> rcsHandle, std::vector<uint8_t> debugFlags, int64_t opcodeSelect, std::string logFileName)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(bin(rcsHandle));
        _args.push_back(bin(debugFlags));
        _args.push_back(opcodeSelect);
        _args.push_back(logFileName);
        auto _ret = this->_client->call("simRRS1.DEBUG", _args);
        return _ret[0].as<int64_t>();
    }

    int64_t simRRS1::DEFINE_EVENT(std::vector<uint8_t> rcsHandle, int64_t eventId, int64_t targetId, double resolution, int64_t typeOfEvent, std::vector<double> eventSpec)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(bin(rcsHandle));
        _args.push_back(eventId);
        _args.push_back(targetId);
        _args.push_back(resolution);
        _args.push_back(typeOfEvent);
        _args.push_back(eventSpec);
        auto _ret = this->_client->call("simRRS1.DEFINE_EVENT", _args);
        return _ret[0].as<int64_t>();
    }

    std::tuple<int64_t, std::string> simRRS1::EXTENDED_SERVICE(std::vector<uint8_t> rcsHandle, std::string inData)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(bin(rcsHandle));
        _args.push_back(inData);
        auto _ret = this->_client->call("simRRS1.EXTENDED_SERVICE", _args);
        return std::make_tuple(_ret[0].as<int64_t>(), _ret[1].as<std::string>());
    }

    std::tuple<int64_t, std::string, int64_t, std::string, std::vector<uint8_t>, std::vector<uint8_t>> simRRS1::GET_CELL_FRAME(std::vector<uint8_t> rcsHandle, int64_t storage, int64_t firstNext, std::string frameId)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(bin(rcsHandle));
        _args.push_back(storage);
        _args.push_back(firstNext);
        _args.push_back(frameId);
        auto _ret = this->_client->call("simRRS1.GET_CELL_FRAME", _args);
        return std::make_tuple(_ret[0].as<int64_t>(), _ret[1].as<std::string>(), _ret[2].as<int64_t>(), _ret[3].as<std::string>(), _ret[4].as<std::vector<uint8_t>>(), _ret[5].as<std::vector<uint8_t>>());
    }

    std::tuple<int64_t, int64_t> simRRS1::GET_CURRENT_TARGETID(std::vector<uint8_t> rcsHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(bin(rcsHandle));
        auto _ret = this->_client->call("simRRS1.GET_CURRENT_TARGETID", _args);
        return std::make_tuple(_ret[0].as<int64_t>(), _ret[1].as<int64_t>());
    }

    std::tuple<int64_t, int64_t, double> simRRS1::GET_EVENT(std::vector<uint8_t> rcsHandle, int64_t eventNumber)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(bin(rcsHandle));
        _args.push_back(eventNumber);
        auto _ret = this->_client->call("simRRS1.GET_EVENT", _args);
        return std::make_tuple(_ret[0].as<int64_t>(), _ret[1].as<int64_t>(), _ret[2].as<double>());
    }

    std::tuple<int64_t, std::vector<uint8_t>, std::vector<uint8_t>, std::string, std::vector<uint8_t>, int64_t> simRRS1::GET_FORWARD_KINEMATIC(std::vector<uint8_t> rcsHandle, std::vector<uint8_t> jointPos)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(bin(rcsHandle));
        _args.push_back(bin(jointPos));
        auto _ret = this->_client->call("simRRS1.GET_FORWARD_KINEMATIC", _args);
        return std::make_tuple(_ret[0].as<int64_t>(), _ret[1].as<std::vector<uint8_t>>(), _ret[2].as<std::vector<uint8_t>>(), _ret[3].as<std::string>(), _ret[4].as<std::vector<uint8_t>>(), _ret[5].as<int64_t>());
    }

    std::tuple<int64_t, std::vector<uint8_t>> simRRS1::GET_HOME_JOINT_POSITION(std::vector<uint8_t> rcsHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(bin(rcsHandle));
        auto _ret = this->_client->call("simRRS1.GET_HOME_JOINT_POSITION", _args);
        return std::make_tuple(_ret[0].as<int64_t>(), _ret[1].as<std::vector<uint8_t>>());
    }

    std::tuple<int64_t, std::vector<uint8_t>, std::vector<uint8_t>, int64_t> simRRS1::GET_INVERSE_KINEMATIC(std::vector<uint8_t> rcsHandle, std::vector<uint8_t> cartPos, std::vector<uint8_t> jointPos, std::string configuration, std::vector<uint8_t> outputFormat)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(bin(rcsHandle));
        _args.push_back(bin(cartPos));
        _args.push_back(bin(jointPos));
        _args.push_back(configuration);
        _args.push_back(bin(outputFormat));
        auto _ret = this->_client->call("simRRS1.GET_INVERSE_KINEMATIC", _args);
        return std::make_tuple(_ret[0].as<int64_t>(), _ret[1].as<std::vector<uint8_t>>(), _ret[2].as<std::vector<uint8_t>>(), _ret[3].as<int64_t>());
    }

    std::tuple<int64_t, int64_t, std::string> simRRS1::GET_MESSAGE(std::vector<uint8_t> rcsHandle, int64_t messageNumber)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(bin(rcsHandle));
        _args.push_back(messageNumber);
        auto _ret = this->_client->call("simRRS1.GET_MESSAGE", _args);
        return std::make_tuple(_ret[0].as<int64_t>(), _ret[1].as<int64_t>(), _ret[2].as<std::string>());
    }

    std::tuple<int64_t, std::vector<uint8_t>, std::vector<uint8_t>, std::string, double, std::vector<uint8_t>, int64_t, int64_t> simRRS1::GET_NEXT_STEP(std::vector<uint8_t> rcsHandle, std::vector<uint8_t> outputFormat)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(bin(rcsHandle));
        _args.push_back(bin(outputFormat));
        auto _ret = this->_client->call("simRRS1.GET_NEXT_STEP", _args);
        return std::make_tuple(_ret[0].as<int64_t>(), _ret[1].as<std::vector<uint8_t>>(), _ret[2].as<std::vector<uint8_t>>(), _ret[3].as<std::string>(), _ret[4].as<double>(), _ret[5].as<std::vector<uint8_t>>(), _ret[6].as<int64_t>(), _ret[7].as<int64_t>());
    }

    std::tuple<int64_t, std::string, std::string, int64_t> simRRS1::GET_RCS_DATA(std::vector<uint8_t> rcsHandle, int64_t storage, int64_t firstNext, std::string paramId)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(bin(rcsHandle));
        _args.push_back(storage);
        _args.push_back(firstNext);
        _args.push_back(paramId);
        auto _ret = this->_client->call("simRRS1.GET_RCS_DATA", _args);
        return std::make_tuple(_ret[0].as<int64_t>(), _ret[1].as<std::string>(), _ret[2].as<std::string>(), _ret[3].as<int64_t>());
    }

    std::tuple<int64_t, std::string, std::string, std::string> simRRS1::GET_ROBOT_STAMP(std::vector<uint8_t> rcsHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(bin(rcsHandle));
        auto _ret = this->_client->call("simRRS1.GET_ROBOT_STAMP", _args);
        return std::make_tuple(_ret[0].as<int64_t>(), _ret[1].as<std::string>(), _ret[2].as<std::string>(), _ret[3].as<std::string>());
    }

    std::tuple<int64_t, std::vector<uint8_t>, int64_t, int64_t, int64_t> simRRS1::INITIALIZE(int64_t robotNumber, std::string robotPathName, std::string modulePathName, std::string manipulatorType, int64_t CarrrsVersion, int64_t debug)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(robotNumber);
        _args.push_back(robotPathName);
        _args.push_back(modulePathName);
        _args.push_back(manipulatorType);
        _args.push_back(CarrrsVersion);
        _args.push_back(debug);
        auto _ret = this->_client->call("simRRS1.INITIALIZE", _args);
        return std::make_tuple(_ret[0].as<int64_t>(), _ret[1].as<std::vector<uint8_t>>(), _ret[2].as<int64_t>(), _ret[3].as<int64_t>(), _ret[4].as<int64_t>());
    }

    std::tuple<int64_t, int64_t> simRRS1::LOAD_RCS_DATA(std::vector<uint8_t> rcsHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(bin(rcsHandle));
        auto _ret = this->_client->call("simRRS1.LOAD_RCS_DATA", _args);
        return std::make_tuple(_ret[0].as<int64_t>(), _ret[1].as<int64_t>());
    }

    std::tuple<int64_t, std::string> simRRS1::MATRIX_TO_CONTROLLER_POSITION(std::vector<uint8_t> rcsHandle, std::vector<uint8_t> cartPos, std::string configuration)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(bin(rcsHandle));
        _args.push_back(bin(cartPos));
        _args.push_back(configuration);
        auto _ret = this->_client->call("simRRS1.MATRIX_TO_CONTROLLER_POSITION", _args);
        return std::make_tuple(_ret[0].as<int64_t>(), _ret[1].as<std::string>());
    }

    int64_t simRRS1::MODIFY_CELL_FRAME(std::vector<uint8_t> rcsHandle, int64_t storage, std::string frameId, std::vector<uint8_t> frameData)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(bin(rcsHandle));
        _args.push_back(storage);
        _args.push_back(frameId);
        _args.push_back(bin(frameData));
        auto _ret = this->_client->call("simRRS1.MODIFY_CELL_FRAME", _args);
        return _ret[0].as<int64_t>();
    }

    int64_t simRRS1::MODIFY_RCS_DATA(std::vector<uint8_t> rcsHandle, int64_t storage, std::string paramId, std::string paramContents)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(bin(rcsHandle));
        _args.push_back(storage);
        _args.push_back(paramId);
        _args.push_back(paramContents);
        auto _ret = this->_client->call("simRRS1.MODIFY_RCS_DATA", _args);
        return _ret[0].as<int64_t>();
    }

    std::tuple<int64_t, int64_t> simRRS1::RESET(std::vector<uint8_t> rcsHandle, int64_t resetLevel)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(bin(rcsHandle));
        _args.push_back(resetLevel);
        auto _ret = this->_client->call("simRRS1.RESET", _args);
        return std::make_tuple(_ret[0].as<int64_t>(), _ret[1].as<int64_t>());
    }

    int64_t simRRS1::REVERSE_MOTION(std::vector<uint8_t> rcsHandle, double distance)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(bin(rcsHandle));
        _args.push_back(distance);
        auto _ret = this->_client->call("simRRS1.REVERSE_MOTION", _args);
        return _ret[0].as<int64_t>();
    }

    int64_t simRRS1::SAVE_RCS_DATA(std::vector<uint8_t> rcsHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(bin(rcsHandle));
        auto _ret = this->_client->call("simRRS1.SAVE_RCS_DATA", _args);
        return _ret[0].as<int64_t>();
    }

    int64_t simRRS1::SELECT_DOMINANT_INTERPOLATION(std::vector<uint8_t> rcsHandle, int64_t dominantIntType, int64_t dominantIntParam)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(bin(rcsHandle));
        _args.push_back(dominantIntType);
        _args.push_back(dominantIntParam);
        auto _ret = this->_client->call("simRRS1.SELECT_DOMINANT_INTERPOLATION", _args);
        return _ret[0].as<int64_t>();
    }

    int64_t simRRS1::SELECT_FLYBY_CRITERIA(std::vector<uint8_t> rcsHandle, int64_t paramNumber)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(bin(rcsHandle));
        _args.push_back(paramNumber);
        auto _ret = this->_client->call("simRRS1.SELECT_FLYBY_CRITERIA", _args);
        return _ret[0].as<int64_t>();
    }

    int64_t simRRS1::SELECT_FLYBY_MODE(std::vector<uint8_t> rcsHandle, int64_t flyByOn)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(bin(rcsHandle));
        _args.push_back(flyByOn);
        auto _ret = this->_client->call("simRRS1.SELECT_FLYBY_MODE", _args);
        return _ret[0].as<int64_t>();
    }

    int64_t simRRS1::SELECT_MOTION_TYPE(std::vector<uint8_t> rcsHandle, int64_t motionType)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(bin(rcsHandle));
        _args.push_back(motionType);
        auto _ret = this->_client->call("simRRS1.SELECT_MOTION_TYPE", _args);
        return _ret[0].as<int64_t>();
    }

    int64_t simRRS1::SELECT_ORIENTATION_INTERPOLATION_MODE(std::vector<uint8_t> rcsHandle, int64_t interpolationMode, int64_t oriConst)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(bin(rcsHandle));
        _args.push_back(interpolationMode);
        _args.push_back(oriConst);
        auto _ret = this->_client->call("simRRS1.SELECT_ORIENTATION_INTERPOLATION_MODE", _args);
        return _ret[0].as<int64_t>();
    }

    int64_t simRRS1::SELECT_POINT_ACCURACY(std::vector<uint8_t> rcsHandle, int64_t accuracyType)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(bin(rcsHandle));
        _args.push_back(accuracyType);
        auto _ret = this->_client->call("simRRS1.SELECT_POINT_ACCURACY", _args);
        return _ret[0].as<int64_t>();
    }

    int64_t simRRS1::SELECT_TARGET_TYPE(std::vector<uint8_t> rcsHandle, int64_t targetType, std::vector<uint8_t> cartPos, std::vector<uint8_t> jointPos, std::string configuration)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(bin(rcsHandle));
        _args.push_back(targetType);
        _args.push_back(bin(cartPos));
        _args.push_back(bin(jointPos));
        _args.push_back(configuration);
        auto _ret = this->_client->call("simRRS1.SELECT_TARGET_TYPE", _args);
        return _ret[0].as<int64_t>();
    }

    int64_t simRRS1::SELECT_TIME_COMPENSATION(std::vector<uint8_t> rcsHandle, std::vector<uint8_t> compensation)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(bin(rcsHandle));
        _args.push_back(bin(compensation));
        auto _ret = this->_client->call("simRRS1.SELECT_TIME_COMPENSATION", _args);
        return _ret[0].as<int64_t>();
    }

    int64_t simRRS1::SELECT_TRACKING(std::vector<uint8_t> rcsHandle, std::vector<uint8_t> conveyorFlags)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(bin(rcsHandle));
        _args.push_back(bin(conveyorFlags));
        auto _ret = this->_client->call("simRRS1.SELECT_TRACKING", _args);
        return _ret[0].as<int64_t>();
    }

    int64_t simRRS1::SELECT_TRAJECTORY_MODE(std::vector<uint8_t> rcsHandle, int64_t trajectoryOn)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(bin(rcsHandle));
        _args.push_back(trajectoryOn);
        auto _ret = this->_client->call("simRRS1.SELECT_TRAJECTORY_MODE", _args);
        return _ret[0].as<int64_t>();
    }

    int64_t simRRS1::SELECT_WEAVING_GROUP(std::vector<uint8_t> rcsHandle, int64_t groupNo, int64_t groupOn)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(bin(rcsHandle));
        _args.push_back(groupNo);
        _args.push_back(groupOn);
        auto _ret = this->_client->call("simRRS1.SELECT_WEAVING_GROUP", _args);
        return _ret[0].as<int64_t>();
    }

    int64_t simRRS1::SELECT_WEAVING_MODE(std::vector<uint8_t> rcsHandle, int64_t weavingMode)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(bin(rcsHandle));
        _args.push_back(weavingMode);
        auto _ret = this->_client->call("simRRS1.SELECT_WEAVING_MODE", _args);
        return _ret[0].as<int64_t>();
    }

    int64_t simRRS1::SELECT_WORK_FRAMES(std::vector<uint8_t> rcsHandle, std::string toolId, std::string objectId)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(bin(rcsHandle));
        _args.push_back(toolId);
        _args.push_back(objectId);
        auto _ret = this->_client->call("simRRS1.SELECT_WORK_FRAMES", _args);
        return _ret[0].as<int64_t>();
    }

    int64_t simRRS1::SET_ADVANCE_MOTION(std::vector<uint8_t> rcsHandle, int64_t numberOfMotion)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(bin(rcsHandle));
        _args.push_back(numberOfMotion);
        auto _ret = this->_client->call("simRRS1.SET_ADVANCE_MOTION", _args);
        return _ret[0].as<int64_t>();
    }

    int64_t simRRS1::SET_CARTESIAN_ORIENTATION_ACCELERATION(std::vector<uint8_t> rcsHandle, int64_t rotationNo, double accelValue, int64_t accelType)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(bin(rcsHandle));
        _args.push_back(rotationNo);
        _args.push_back(accelValue);
        _args.push_back(accelType);
        auto _ret = this->_client->call("simRRS1.SET_CARTESIAN_ORIENTATION_ACCELERATION", _args);
        return _ret[0].as<int64_t>();
    }

    int64_t simRRS1::SET_CARTESIAN_ORIENTATION_SPEED(std::vector<uint8_t> rcsHandle, int64_t rotationNo, double speedValue)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(bin(rcsHandle));
        _args.push_back(rotationNo);
        _args.push_back(speedValue);
        auto _ret = this->_client->call("simRRS1.SET_CARTESIAN_ORIENTATION_SPEED", _args);
        return _ret[0].as<int64_t>();
    }

    int64_t simRRS1::SET_CARTESIAN_POSITION_ACCELERATION(std::vector<uint8_t> rcsHandle, double accelValue, int64_t accelType)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(bin(rcsHandle));
        _args.push_back(accelValue);
        _args.push_back(accelType);
        auto _ret = this->_client->call("simRRS1.SET_CARTESIAN_POSITION_ACCELERATION", _args);
        return _ret[0].as<int64_t>();
    }

    int64_t simRRS1::SET_CARTESIAN_POSITION_SPEED(std::vector<uint8_t> rcsHandle, double speedValue)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(bin(rcsHandle));
        _args.push_back(speedValue);
        auto _ret = this->_client->call("simRRS1.SET_CARTESIAN_POSITION_SPEED", _args);
        return _ret[0].as<int64_t>();
    }

    int64_t simRRS1::SET_CONFIGURATION_CONTROL(std::vector<uint8_t> rcsHandle, std::string paramId, std::string paramContents)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(bin(rcsHandle));
        _args.push_back(paramId);
        _args.push_back(paramContents);
        auto _ret = this->_client->call("simRRS1.SET_CONFIGURATION_CONTROL", _args);
        return _ret[0].as<int64_t>();
    }

    int64_t simRRS1::SET_CONVEYOR_POSITION(std::vector<uint8_t> rcsHandle, std::vector<uint8_t> inputFormat, std::vector<uint8_t> conveyorFlags, std::vector<double> conveyorPos)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(bin(rcsHandle));
        _args.push_back(bin(inputFormat));
        _args.push_back(bin(conveyorFlags));
        _args.push_back(conveyorPos);
        auto _ret = this->_client->call("simRRS1.SET_CONVEYOR_POSITION", _args);
        return _ret[0].as<int64_t>();
    }

    int64_t simRRS1::SET_FLYBY_CRITERIA_PARAMETER(std::vector<uint8_t> rcsHandle, int64_t paramNumber, int64_t jointNr, double paramValue)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(bin(rcsHandle));
        _args.push_back(paramNumber);
        _args.push_back(jointNr);
        _args.push_back(paramValue);
        auto _ret = this->_client->call("simRRS1.SET_FLYBY_CRITERIA_PARAMETER", _args);
        return _ret[0].as<int64_t>();
    }

    std::tuple<int64_t, std::vector<uint8_t>> simRRS1::SET_INITIAL_POSITION(std::vector<uint8_t> rcsHandle, std::vector<uint8_t> cartPos, std::vector<uint8_t> jointPos, std::string configuration)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(bin(rcsHandle));
        _args.push_back(bin(cartPos));
        _args.push_back(bin(jointPos));
        _args.push_back(configuration);
        auto _ret = this->_client->call("simRRS1.SET_INITIAL_POSITION", _args);
        return std::make_tuple(_ret[0].as<int64_t>(), _ret[1].as<std::vector<uint8_t>>());
    }

    int64_t simRRS1::SET_INTERPOLATION_TIME(std::vector<uint8_t> rcsHandle, double interpolationTime)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(bin(rcsHandle));
        _args.push_back(interpolationTime);
        auto _ret = this->_client->call("simRRS1.SET_INTERPOLATION_TIME", _args);
        return _ret[0].as<int64_t>();
    }

    int64_t simRRS1::SET_JOINT_ACCELERATIONS(std::vector<uint8_t> rcsHandle, int64_t allJointFlags, std::vector<uint8_t> jointFlags, std::vector<double> accelPercent, int64_t accelType)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(bin(rcsHandle));
        _args.push_back(allJointFlags);
        _args.push_back(bin(jointFlags));
        _args.push_back(accelPercent);
        _args.push_back(accelType);
        auto _ret = this->_client->call("simRRS1.SET_JOINT_ACCELERATIONS", _args);
        return _ret[0].as<int64_t>();
    }

    int64_t simRRS1::SET_JOINT_JERKS(std::vector<uint8_t> rcsHandle, int64_t allJointFlags, std::vector<uint8_t> jointFlags, std::vector<double> jerkPercent, int64_t jerkType)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(bin(rcsHandle));
        _args.push_back(allJointFlags);
        _args.push_back(bin(jointFlags));
        _args.push_back(jerkPercent);
        _args.push_back(jerkType);
        auto _ret = this->_client->call("simRRS1.SET_JOINT_JERKS", _args);
        return _ret[0].as<int64_t>();
    }

    int64_t simRRS1::SET_JOINT_SPEEDS(std::vector<uint8_t> rcsHandle, int64_t allJointFlags, std::vector<uint8_t> jointFlags, std::vector<double> speedPercent)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(bin(rcsHandle));
        _args.push_back(allJointFlags);
        _args.push_back(bin(jointFlags));
        _args.push_back(speedPercent);
        auto _ret = this->_client->call("simRRS1.SET_JOINT_SPEEDS", _args);
        return _ret[0].as<int64_t>();
    }

    int64_t simRRS1::SET_MOTION_FILTER(std::vector<uint8_t> rcsHandle, int64_t filterFactor)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(bin(rcsHandle));
        _args.push_back(filterFactor);
        auto _ret = this->_client->call("simRRS1.SET_MOTION_FILTER", _args);
        return _ret[0].as<int64_t>();
    }

    int64_t simRRS1::SET_MOTION_TIME(std::vector<uint8_t> rcsHandle, double timeValue)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(bin(rcsHandle));
        _args.push_back(timeValue);
        auto _ret = this->_client->call("simRRS1.SET_MOTION_TIME", _args);
        return _ret[0].as<int64_t>();
    }

    int64_t simRRS1::SET_NEXT_TARGET(std::vector<uint8_t> rcsHandle, int64_t targetId, int64_t targetParam, std::vector<uint8_t> cartPos, std::vector<uint8_t> jointPos, std::string configuration, double targetParamValue)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(bin(rcsHandle));
        _args.push_back(targetId);
        _args.push_back(targetParam);
        _args.push_back(bin(cartPos));
        _args.push_back(bin(jointPos));
        _args.push_back(configuration);
        _args.push_back(targetParamValue);
        auto _ret = this->_client->call("simRRS1.SET_NEXT_TARGET", _args);
        return _ret[0].as<int64_t>();
    }

    int64_t simRRS1::SET_OVERRIDE_ACCELERATION(std::vector<uint8_t> rcsHandle, double correctionValue, int64_t accelType, int64_t correctionType)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(bin(rcsHandle));
        _args.push_back(correctionValue);
        _args.push_back(accelType);
        _args.push_back(correctionType);
        auto _ret = this->_client->call("simRRS1.SET_OVERRIDE_ACCELERATION", _args);
        return _ret[0].as<int64_t>();
    }

    int64_t simRRS1::SET_OVERRIDE_POSITION(std::vector<uint8_t> rcsHandle, std::vector<uint8_t> posOffset)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(bin(rcsHandle));
        _args.push_back(bin(posOffset));
        auto _ret = this->_client->call("simRRS1.SET_OVERRIDE_POSITION", _args);
        return _ret[0].as<int64_t>();
    }

    int64_t simRRS1::SET_OVERRIDE_SPEED(std::vector<uint8_t> rcsHandle, double correctionValue, int64_t correctionType)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(bin(rcsHandle));
        _args.push_back(correctionValue);
        _args.push_back(correctionType);
        auto _ret = this->_client->call("simRRS1.SET_OVERRIDE_SPEED", _args);
        return _ret[0].as<int64_t>();
    }

    int64_t simRRS1::SET_PAYLOAD_PARAMETER(std::vector<uint8_t> rcsHandle, int64_t storage, std::string frameId, int64_t paramNumber, double paramValue)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(bin(rcsHandle));
        _args.push_back(storage);
        _args.push_back(frameId);
        _args.push_back(paramNumber);
        _args.push_back(paramValue);
        auto _ret = this->_client->call("simRRS1.SET_PAYLOAD_PARAMETER", _args);
        return _ret[0].as<int64_t>();
    }

    int64_t simRRS1::SET_POINT_ACCURACY_PARAMETER(std::vector<uint8_t> rcsHandle, int64_t accuracyType, double accuracyValue)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(bin(rcsHandle));
        _args.push_back(accuracyType);
        _args.push_back(accuracyValue);
        auto _ret = this->_client->call("simRRS1.SET_POINT_ACCURACY_PARAMETER", _args);
        return _ret[0].as<int64_t>();
    }

    int64_t simRRS1::SET_REST_PARAMETER(std::vector<uint8_t> rcsHandle, int64_t paramNumber, double paramValue)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(bin(rcsHandle));
        _args.push_back(paramNumber);
        _args.push_back(paramValue);
        auto _ret = this->_client->call("simRRS1.SET_REST_PARAMETER", _args);
        return _ret[0].as<int64_t>();
    }

    int64_t simRRS1::SET_WEAVING_GROUP_PARAMETER(std::vector<uint8_t> rcsHandle, int64_t groupNo, int64_t paramNo, double paramValue)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(bin(rcsHandle));
        _args.push_back(groupNo);
        _args.push_back(paramNo);
        _args.push_back(paramValue);
        auto _ret = this->_client->call("simRRS1.SET_WEAVING_GROUP_PARAMETER", _args);
        return _ret[0].as<int64_t>();
    }

    int64_t simRRS1::STOP_MOTION(std::vector<uint8_t> rcsHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(bin(rcsHandle));
        auto _ret = this->_client->call("simRRS1.STOP_MOTION", _args);
        return _ret[0].as<int64_t>();
    }

    int64_t simRRS1::TERMINATE(std::vector<uint8_t> rcsHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(bin(rcsHandle));
        auto _ret = this->_client->call("simRRS1.TERMINATE", _args);
        return _ret[0].as<int64_t>();
    }

    bool simRRS1::selectRcsServer(int64_t rcsServerHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(rcsServerHandle);
        auto _ret = this->_client->call("simRRS1.selectRcsServer", _args);
        return _ret[0].as<bool>();
    }

    int64_t simRRS1::startRcsServer(std::string rcsLibraryFilename, std::string rcsLibraryFunctionName, int64_t portNumber)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(rcsLibraryFilename);
        _args.push_back(rcsLibraryFunctionName);
        _args.push_back(portNumber);
        auto _ret = this->_client->call("simRRS1.startRcsServer", _args);
        return _ret[0].as<int64_t>();
    }

    bool simRRS1::stopRcsServer(int64_t rcsServerHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(rcsServerHandle);
        auto _ret = this->_client->call("simRRS1.stopRcsServer", _args);
        return _ret[0].as<bool>();
    }

    simSDF::simSDF(RemoteAPIClient *client)
        : _client(client)
    {
        _client->require("simSDF");
    }

    void simSDF::dump(std::string fileName)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(fileName);
        auto _ret = this->_client->call("simSDF.dump", _args);
    }

    void simSDF::import(std::string fileName, std::optional<json> options)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(fileName);
        if(options)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*options);
        }
        else _brk = true;
        auto _ret = this->_client->call("simSDF.import", _args);
    }

    simSkeleton::simSkeleton(RemoteAPIClient *client)
        : _client(client)
    {
        _client->require("simSkeleton");
    }

    std::tuple<std::string, json> simSkeleton::getData(std::string inputString, json inputMap)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(inputString);
        _args.push_back(inputMap);
        auto _ret = this->_client->call("simSkeleton.getData", _args);
        return std::make_tuple(_ret[0].as<std::string>(), _ret[1].as<json>());
    }

    simSurfRec::simSurfRec(RemoteAPIClient *client)
        : _client(client)
    {
        _client->require("simSurfRec");
    }

    int64_t simSurfRec::reconstruct_scale_space(int64_t pointCloudHandle, std::optional<int64_t> iterations, std::optional<int64_t> neighbors, std::optional<int64_t> samples, std::optional<double> squared_radius)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(pointCloudHandle);
        if(iterations)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*iterations);
        }
        else _brk = true;
        if(neighbors)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*neighbors);
        }
        else _brk = true;
        if(samples)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*samples);
        }
        else _brk = true;
        if(squared_radius)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*squared_radius);
        }
        else _brk = true;
        auto _ret = this->_client->call("simSurfRec.reconstruct_scale_space", _args);
        return _ret[0].as<int64_t>();
    }

    simVision::simVision(RemoteAPIClient *client)
        : _client(client)
    {
        _client->require("simVision");
    }

    void simVision::addBuffer1ToWorkImg(int64_t visionSensorHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(visionSensorHandle);
        auto _ret = this->_client->call("simVision.addBuffer1ToWorkImg", _args);
    }

    void simVision::addWorkImgToBuffer1(int64_t visionSensorHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(visionSensorHandle);
        auto _ret = this->_client->call("simVision.addWorkImgToBuffer1", _args);
    }

    std::tuple<bool, std::vector<uint8_t>> simVision::binaryWorkImg(int64_t visionSensorHandle, double threshold, double oneProportion, double oneTol, double xCenter, double xCenterTol, double yCenter, double yCenterTol, double orient, double orientTol, double roundness, bool enableTrigger, std::optional<std::vector<double>> overlayColor)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(visionSensorHandle);
        _args.push_back(threshold);
        _args.push_back(oneProportion);
        _args.push_back(oneTol);
        _args.push_back(xCenter);
        _args.push_back(xCenterTol);
        _args.push_back(yCenter);
        _args.push_back(yCenterTol);
        _args.push_back(orient);
        _args.push_back(orientTol);
        _args.push_back(roundness);
        _args.push_back(enableTrigger);
        if(overlayColor)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*overlayColor);
        }
        else _brk = true;
        auto _ret = this->_client->call("simVision.binaryWorkImg", _args);
        return std::make_tuple(_ret[0].as<bool>(), _ret[1].as<std::vector<uint8_t>>());
    }

    std::tuple<bool, std::vector<uint8_t>> simVision::blobDetectionOnWorkImg(int64_t visionSensorHandle, double threshold, double minBlobSize, bool modifyWorkImage, std::optional<std::vector<double>> overlayColor)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(visionSensorHandle);
        _args.push_back(threshold);
        _args.push_back(minBlobSize);
        _args.push_back(modifyWorkImage);
        if(overlayColor)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*overlayColor);
        }
        else _brk = true;
        auto _ret = this->_client->call("simVision.blobDetectionOnWorkImg", _args);
        return std::make_tuple(_ret[0].as<bool>(), _ret[1].as<std::vector<uint8_t>>());
    }

    void simVision::buffer1ToWorkImg(int64_t visionSensorHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(visionSensorHandle);
        auto _ret = this->_client->call("simVision.buffer1ToWorkImg", _args);
    }

    void simVision::buffer2ToWorkImg(int64_t visionSensorHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(visionSensorHandle);
        auto _ret = this->_client->call("simVision.buffer2ToWorkImg", _args);
    }

    std::tuple<bool, std::vector<uint8_t>> simVision::changedPixelsOnWorkImg(int64_t visionSensorHandle, double threshold)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(visionSensorHandle);
        _args.push_back(threshold);
        auto _ret = this->_client->call("simVision.changedPixelsOnWorkImg", _args);
        return std::make_tuple(_ret[0].as<bool>(), _ret[1].as<std::vector<uint8_t>>());
    }

    void simVision::circularCutWorkImg(int64_t visionSensorHandle, double radius, bool copyToBuffer1)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(visionSensorHandle);
        _args.push_back(radius);
        _args.push_back(copyToBuffer1);
        auto _ret = this->_client->call("simVision.circularCutWorkImg", _args);
    }

    void simVision::colorSegmentationOnWorkImg(int64_t visionSensorHandle, double maxColorColorDistance)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(visionSensorHandle);
        _args.push_back(maxColorColorDistance);
        auto _ret = this->_client->call("simVision.colorSegmentationOnWorkImg", _args);
    }

    std::tuple<bool, std::vector<uint8_t>, std::vector<uint8_t>> simVision::coordinatesFromWorkImg(int64_t visionSensorHandle, std::vector<int64_t> xyPointCount, bool evenlySpacedInAngularSpace, std::optional<bool> returnColorData)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(visionSensorHandle);
        _args.push_back(xyPointCount);
        _args.push_back(evenlySpacedInAngularSpace);
        if(returnColorData)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*returnColorData);
        }
        else _brk = true;
        auto _ret = this->_client->call("simVision.coordinatesFromWorkImg", _args);
        return std::make_tuple(_ret[0].as<bool>(), _ret[1].as<std::vector<uint8_t>>(), _ret[2].as<std::vector<uint8_t>>());
    }

    int64_t simVision::createVelodyneHDL64E(std::vector<int64_t> visionSensorHandles, double frequency, std::optional<int64_t> options, std::optional<int64_t> pointSize, std::optional<std::vector<double>> coloring_closeFarDist, std::optional<double> displayScalingFactor)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(visionSensorHandles);
        _args.push_back(frequency);
        if(options)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*options);
        }
        else _brk = true;
        if(pointSize)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*pointSize);
        }
        else _brk = true;
        if(coloring_closeFarDist)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*coloring_closeFarDist);
        }
        else _brk = true;
        if(displayScalingFactor)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*displayScalingFactor);
        }
        else _brk = true;
        auto _ret = this->_client->call("simVision.createVelodyneHDL64E", _args);
        return _ret[0].as<int64_t>();
    }

    int64_t simVision::createVelodyneVPL16(std::vector<int64_t> visionSensorHandles, double frequency, std::optional<int64_t> options, std::optional<int64_t> pointSize, std::optional<std::vector<double>> coloring_closeFarDist, std::optional<double> displayScalingFactor)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(visionSensorHandles);
        _args.push_back(frequency);
        if(options)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*options);
        }
        else _brk = true;
        if(pointSize)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*pointSize);
        }
        else _brk = true;
        if(coloring_closeFarDist)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*coloring_closeFarDist);
        }
        else _brk = true;
        if(displayScalingFactor)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*displayScalingFactor);
        }
        else _brk = true;
        auto _ret = this->_client->call("simVision.createVelodyneVPL16", _args);
        return _ret[0].as<int64_t>();
    }

    int64_t simVision::destroyVelodyneHDL64E(int64_t velodyneHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(velodyneHandle);
        auto _ret = this->_client->call("simVision.destroyVelodyneHDL64E", _args);
        return _ret[0].as<int64_t>();
    }

    int64_t simVision::destroyVelodyneVPL16(int64_t velodyneHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(velodyneHandle);
        auto _ret = this->_client->call("simVision.destroyVelodyneVPL16", _args);
        return _ret[0].as<int64_t>();
    }

    void simVision::distort(int64_t visionSensorHandle, std::optional<std::vector<int64_t>> pixelMap, std::optional<std::vector<double>> depthScalings)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(visionSensorHandle);
        if(pixelMap)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*pixelMap);
        }
        else _brk = true;
        if(depthScalings)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*depthScalings);
        }
        else _brk = true;
        auto _ret = this->_client->call("simVision.distort", _args);
    }

    void simVision::edgeDetectionOnWorkImg(int64_t visionSensorHandle, double threshold)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(visionSensorHandle);
        _args.push_back(threshold);
        auto _ret = this->_client->call("simVision.edgeDetectionOnWorkImg", _args);
    }

    int64_t simVision::handleAnaglyphStereo(int64_t passiveVisionSensorHandle, std::vector<int64_t> activeVisionSensorHandles, std::optional<std::vector<double>> leftAndRightColors)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(passiveVisionSensorHandle);
        _args.push_back(activeVisionSensorHandles);
        if(leftAndRightColors)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*leftAndRightColors);
        }
        else _brk = true;
        auto _ret = this->_client->call("simVision.handleAnaglyphStereo", _args);
        return _ret[0].as<int64_t>();
    }

    int64_t simVision::handleSpherical(int64_t passiveVisionSensorHandleForRGB, std::vector<int64_t> activeVisionSensorHandles, double horizontalAngle, double verticalAngle, std::optional<int64_t> passiveVisionSensorHandleForDepth)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(passiveVisionSensorHandleForRGB);
        _args.push_back(activeVisionSensorHandles);
        _args.push_back(horizontalAngle);
        _args.push_back(verticalAngle);
        if(passiveVisionSensorHandleForDepth)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*passiveVisionSensorHandleForDepth);
        }
        else _brk = true;
        auto _ret = this->_client->call("simVision.handleSpherical", _args);
        return _ret[0].as<int64_t>();
    }

    std::tuple<std::vector<uint8_t>, std::vector<uint8_t>> simVision::handleVelodyneHDL64E(int64_t velodyneHandle, double dt)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(velodyneHandle);
        _args.push_back(dt);
        auto _ret = this->_client->call("simVision.handleVelodyneHDL64E", _args);
        return std::make_tuple(_ret[0].as<std::vector<uint8_t>>(), _ret[1].as<std::vector<uint8_t>>());
    }

    std::tuple<std::vector<uint8_t>, std::vector<uint8_t>> simVision::handleVelodyneVPL16(int64_t velodyneHandle, double dt)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(velodyneHandle);
        _args.push_back(dt);
        auto _ret = this->_client->call("simVision.handleVelodyneVPL16", _args);
        return std::make_tuple(_ret[0].as<std::vector<uint8_t>>(), _ret[1].as<std::vector<uint8_t>>());
    }

    void simVision::horizontalFlipWorkImg(int64_t visionSensorHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(visionSensorHandle);
        auto _ret = this->_client->call("simVision.horizontalFlipWorkImg", _args);
    }

    void simVision::intensityScaleOnWorkImg(int64_t visionSensorHandle, double start, double end, bool greyScale)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(visionSensorHandle);
        _args.push_back(start);
        _args.push_back(end);
        _args.push_back(greyScale);
        auto _ret = this->_client->call("simVision.intensityScaleOnWorkImg", _args);
    }

    void simVision::matrix3x3OnWorkImg(int64_t visionSensorHandle, int64_t passes, double multiplier, std::optional<std::vector<double>> matrix)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(visionSensorHandle);
        _args.push_back(passes);
        _args.push_back(multiplier);
        if(matrix)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*matrix);
        }
        else _brk = true;
        auto _ret = this->_client->call("simVision.matrix3x3OnWorkImg", _args);
    }

    void simVision::matrix5x5OnWorkImg(int64_t visionSensorHandle, int64_t passes, double multiplier, std::optional<std::vector<double>> matrix)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(visionSensorHandle);
        _args.push_back(passes);
        _args.push_back(multiplier);
        if(matrix)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*matrix);
        }
        else _brk = true;
        auto _ret = this->_client->call("simVision.matrix5x5OnWorkImg", _args);
    }

    void simVision::multiplyWorkImgWithBuffer1(int64_t visionSensorHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(visionSensorHandle);
        auto _ret = this->_client->call("simVision.multiplyWorkImgWithBuffer1", _args);
    }

    void simVision::normalizeWorkImg(int64_t visionSensorHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(visionSensorHandle);
        auto _ret = this->_client->call("simVision.normalizeWorkImg", _args);
    }

    void simVision::rectangularCutWorkImg(int64_t visionSensorHandle, std::vector<double> sizes, bool copyToBuffer1)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(visionSensorHandle);
        _args.push_back(sizes);
        _args.push_back(copyToBuffer1);
        auto _ret = this->_client->call("simVision.rectangularCutWorkImg", _args);
    }

    void simVision::resizeWorkImg(int64_t visionSensorHandle, std::vector<double> scaling)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(visionSensorHandle);
        _args.push_back(scaling);
        auto _ret = this->_client->call("simVision.resizeWorkImg", _args);
    }

    void simVision::rotateWorkImg(int64_t visionSensorHandle, double rotationAngle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(visionSensorHandle);
        _args.push_back(rotationAngle);
        auto _ret = this->_client->call("simVision.rotateWorkImg", _args);
    }

    void simVision::scaleAndOffsetWorkImg(int64_t visionSensorHandle, std::vector<double> preOffset, std::vector<double> scaling, std::vector<double> postOffset, bool rgb)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(visionSensorHandle);
        _args.push_back(preOffset);
        _args.push_back(scaling);
        _args.push_back(postOffset);
        _args.push_back(rgb);
        auto _ret = this->_client->call("simVision.scaleAndOffsetWorkImg", _args);
    }

    void simVision::selectiveColorOnWorkImg(int64_t visionSensorHandle, std::vector<double> color, std::vector<double> colorTolerance, bool rgb, bool keep, bool removedPartToBuffer1)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(visionSensorHandle);
        _args.push_back(color);
        _args.push_back(colorTolerance);
        _args.push_back(rgb);
        _args.push_back(keep);
        _args.push_back(removedPartToBuffer1);
        auto _ret = this->_client->call("simVision.selectiveColorOnWorkImg", _args);
    }

    void simVision::sensorDepthMapToWorkImg(int64_t visionSensorHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(visionSensorHandle);
        auto _ret = this->_client->call("simVision.sensorDepthMapToWorkImg", _args);
    }

    void simVision::sensorImgToWorkImg(int64_t visionSensorHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(visionSensorHandle);
        auto _ret = this->_client->call("simVision.sensorImgToWorkImg", _args);
    }

    void simVision::sharpenWorkImg(int64_t visionSensorHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(visionSensorHandle);
        auto _ret = this->_client->call("simVision.sharpenWorkImg", _args);
    }

    void simVision::shiftWorkImg(int64_t visionSensorHandle, std::vector<double> shift, bool wrapAround)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(visionSensorHandle);
        _args.push_back(shift);
        _args.push_back(wrapAround);
        auto _ret = this->_client->call("simVision.shiftWorkImg", _args);
    }

    void simVision::subtractBuffer1FromWorkImg(int64_t visionSensorHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(visionSensorHandle);
        auto _ret = this->_client->call("simVision.subtractBuffer1FromWorkImg", _args);
    }

    void simVision::subtractWorkImgFromBuffer1(int64_t visionSensorHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(visionSensorHandle);
        auto _ret = this->_client->call("simVision.subtractWorkImgFromBuffer1", _args);
    }

    void simVision::swapBuffers(int64_t visionSensorHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(visionSensorHandle);
        auto _ret = this->_client->call("simVision.swapBuffers", _args);
    }

    void simVision::swapWorkImgWithBuffer1(int64_t visionSensorHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(visionSensorHandle);
        auto _ret = this->_client->call("simVision.swapWorkImgWithBuffer1", _args);
    }

    void simVision::uniformImgToWorkImg(int64_t visionSensorHandle, std::vector<double> color)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(visionSensorHandle);
        _args.push_back(color);
        auto _ret = this->_client->call("simVision.uniformImgToWorkImg", _args);
    }

    std::tuple<bool, std::vector<uint8_t>, std::vector<uint8_t>> simVision::velodyneDataFromWorkImg(int64_t visionSensorHandle, std::vector<int64_t> xyPointCount, double vAngle, std::optional<bool> returnColorData)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(visionSensorHandle);
        _args.push_back(xyPointCount);
        _args.push_back(vAngle);
        if(returnColorData)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*returnColorData);
        }
        else _brk = true;
        auto _ret = this->_client->call("simVision.velodyneDataFromWorkImg", _args);
        return std::make_tuple(_ret[0].as<bool>(), _ret[1].as<std::vector<uint8_t>>(), _ret[2].as<std::vector<uint8_t>>());
    }

    void simVision::verticalFlipWorkImg(int64_t visionSensorHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(visionSensorHandle);
        auto _ret = this->_client->call("simVision.verticalFlipWorkImg", _args);
    }

    void simVision::workImgToBuffer1(int64_t visionSensorHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(visionSensorHandle);
        auto _ret = this->_client->call("simVision.workImgToBuffer1", _args);
    }

    void simVision::workImgToBuffer2(int64_t visionSensorHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(visionSensorHandle);
        auto _ret = this->_client->call("simVision.workImgToBuffer2", _args);
    }

    void simVision::workImgToSensorDepthMap(int64_t visionSensorHandle, std::optional<bool> removeBuffer)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(visionSensorHandle);
        if(removeBuffer)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*removeBuffer);
        }
        else _brk = true;
        auto _ret = this->_client->call("simVision.workImgToSensorDepthMap", _args);
    }

    void simVision::workImgToSensorImg(int64_t visionSensorHandle, std::optional<bool> removeBuffer)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(visionSensorHandle);
        if(removeBuffer)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*removeBuffer);
        }
        else _brk = true;
        auto _ret = this->_client->call("simVision.workImgToSensorImg", _args);
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

RemoteAPIObject::simAssimp RemoteAPIObjects::simAssimp()
{
    return RemoteAPIObject::simAssimp(this->client);
}

RemoteAPIObject::simBubble RemoteAPIObjects::simBubble()
{
    return RemoteAPIObject::simBubble(this->client);
}

RemoteAPIObject::simCHAI3D RemoteAPIObjects::simCHAI3D()
{
    return RemoteAPIObject::simCHAI3D(this->client);
}

RemoteAPIObject::simCam RemoteAPIObjects::simCam()
{
    return RemoteAPIObject::simCam(this->client);
}

RemoteAPIObject::simConvex RemoteAPIObjects::simConvex()
{
    return RemoteAPIObject::simConvex(this->client);
}

RemoteAPIObject::simGLTF RemoteAPIObjects::simGLTF()
{
    return RemoteAPIObject::simGLTF(this->client);
}

RemoteAPIObject::simGeom RemoteAPIObjects::simGeom()
{
    return RemoteAPIObject::simGeom(this->client);
}

RemoteAPIObject::simICP RemoteAPIObjects::simICP()
{
    return RemoteAPIObject::simICP(this->client);
}

RemoteAPIObject::simIK RemoteAPIObjects::simIK()
{
    return RemoteAPIObject::simIK(this->client);
}

RemoteAPIObject::simLDraw RemoteAPIObjects::simLDraw()
{
    return RemoteAPIObject::simLDraw(this->client);
}

RemoteAPIObject::simLuaCmd RemoteAPIObjects::simLuaCmd()
{
    return RemoteAPIObject::simLuaCmd(this->client);
}

RemoteAPIObject::simMTB RemoteAPIObjects::simMTB()
{
    return RemoteAPIObject::simMTB(this->client);
}

RemoteAPIObject::simMujoco RemoteAPIObjects::simMujoco()
{
    return RemoteAPIObject::simMujoco(this->client);
}

RemoteAPIObject::simOpenMesh RemoteAPIObjects::simOpenMesh()
{
    return RemoteAPIObject::simOpenMesh(this->client);
}

RemoteAPIObject::simPython RemoteAPIObjects::simPython()
{
    return RemoteAPIObject::simPython(this->client);
}

RemoteAPIObject::simROS2 RemoteAPIObjects::simROS2()
{
    return RemoteAPIObject::simROS2(this->client);
}

RemoteAPIObject::simRRS1 RemoteAPIObjects::simRRS1()
{
    return RemoteAPIObject::simRRS1(this->client);
}

RemoteAPIObject::simSDF RemoteAPIObjects::simSDF()
{
    return RemoteAPIObject::simSDF(this->client);
}

RemoteAPIObject::simSkeleton RemoteAPIObjects::simSkeleton()
{
    return RemoteAPIObject::simSkeleton(this->client);
}

RemoteAPIObject::simSurfRec RemoteAPIObjects::simSurfRec()
{
    return RemoteAPIObject::simSurfRec(this->client);
}

RemoteAPIObject::simVision RemoteAPIObjects::simVision()
{
    return RemoteAPIObject::simVision(this->client);
}

