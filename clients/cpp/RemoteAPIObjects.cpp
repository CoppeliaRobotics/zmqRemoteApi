namespace RemoteAPIObject
{
    sim::sim(RemoteAPIClient *client)
        : _client(client)
    {
        _client->require("sim");
    }

#include "sim-deprecated.cpp"
#include "sim-special.cpp"

    json sim::Object(int64_t handle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(handle);
        auto _ret = this->_client->call("sim.Object", _args);
        return _ret[0].as<json>();
    }

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

    json sim::convertPropertyValue(json value, int64_t fromType, int64_t toType)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(value);
        _args.push_back(fromType);
        _args.push_back(toType);
        auto _ret = this->_client->call("sim.convertPropertyValue", _args);
        return _ret[0].as<json>();
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

    std::tuple<bool, json> sim::executeLuaCode(std::string theCode)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(theCode);
        auto _ret = this->_client->call("sim.executeLuaCode", _args);
        return std::make_tuple(_ret[0].as<bool>(), _ret[1].as<json>());
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

    void sim::fastIdleLoop(bool enable)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(enable);
        auto _ret = this->_client->call("sim.fastIdleLoop", _args);
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

    std::vector<std::string> sim::getLoadedPlugins()
    {
        bool _brk = false;
        json _args(json_array_arg);
        auto _ret = this->_client->call("sim.getLoadedPlugins", _args);
        return _ret[0].as<std::vector<std::string>>();
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

    std::vector<std::string> sim::getMatchingPersistentDataTags(std::string pattern)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(pattern);
        auto _ret = this->_client->call("sim.getMatchingPersistentDataTags", _args);
        return _ret[0].as<std::vector<std::string>>();
    }

    std::vector<double> sim::getMatrixInverse(std::vector<double> matrix)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(matrix);
        auto _ret = this->_client->call("sim.getMatrixInverse", _args);
        return _ret[0].as<std::vector<double>>();
    }

    std::vector<double> sim::getModelBB(int64_t handle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(handle);
        auto _ret = this->_client->call("sim.getModelBB", _args);
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

    int64_t sim::getObjectHandle(std::string path, std::optional<json> options)
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
        auto _ret = this->_client->call("sim.getObjectHandle", _args);
        return _ret[0].as<int64_t>();
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

    std::vector<int64_t> sim::getObjectsWithTag(std::string tagName, std::optional<bool> justModels)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(tagName);
        if(justModels)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*justModels);
        }
        else _brk = true;
        auto _ret = this->_client->call("sim.getObjectsWithTag", _args);
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

    std::vector<double> sim::getQuaternionInverse(std::vector<double> quat)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(quat);
        auto _ret = this->_client->call("sim.getQuaternionInverse", _args);
        return _ret[0].as<std::vector<double>>();
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

    bool sim::isPluginLoaded(std::string name)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(name);
        auto _ret = this->_client->call("sim.isPluginLoaded", _args);
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

    int64_t sim::loadPlugin(std::string name)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(name);
        auto _ret = this->_client->call("sim.loadPlugin", _args);
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

    std::tuple<std::vector<uint8_t>, std::string> sim::readCustomDataBlockEx(int64_t handle, std::string tag, std::optional<json> options)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(handle);
        _args.push_back(tag);
        if(options)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*options);
        }
        else _brk = true;
        auto _ret = this->_client->call("sim.readCustomDataBlockEx", _args);
        return std::make_tuple(_ret[0].as<std::vector<uint8_t>>(), _ret[1].as<std::string>());
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

    void sim::setJointMode(int64_t jointHandle, int64_t jointMode)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(jointHandle);
        _args.push_back(jointMode);
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

    std::vector<uint8_t> sim::transformImage(std::vector<uint8_t> image, std::vector<int64_t> resolution, int64_t options)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(bin(image));
        _args.push_back(resolution);
        _args.push_back(options);
        auto _ret = this->_client->call("sim.transformImage", _args);
        return _ret[0].as<std::vector<uint8_t>>();
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

    void sim::writeCustomDataBlockEx(int64_t handle, std::string tag, std::vector<uint8_t> data, std::optional<json> options)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(handle);
        _args.push_back(tag);
        _args.push_back(bin(data));
        if(options)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*options);
        }
        else _brk = true;
        auto _ret = this->_client->call("sim.writeCustomDataBlockEx", _args);
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

