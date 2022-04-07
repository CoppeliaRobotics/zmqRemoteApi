namespace RemoteAPIObject
{
    sim::sim(RemoteAPIClient *client)
        : _client(client)
    {
    }

    int sim::addDrawingObject(int objectType, float size, float duplicateTolerance, int parentObjectHandle, int maxItemCount, std::optional<std::vector<float>> ambient_diffuse, std::optional<std::vector<float>> reserved, std::optional<std::vector<float>> specular, std::optional<std::vector<float>> emission)
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
        return _ret[0].as<int>();
    }

    int sim::addDrawingObjectItem(int drawingObjectHandle, std::vector<float> itemData)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(drawingObjectHandle);
        _args.push_back(itemData);
        auto _ret = this->_client->call("sim.addDrawingObjectItem", _args);
        return _ret[0].as<int>();
    }

    void sim::addForce(int shapeHandle, std::vector<float> position, std::vector<float> force)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(shapeHandle);
        _args.push_back(position);
        _args.push_back(force);
        auto _ret = this->_client->call("sim.addForce", _args);
    }

    void sim::addForceAndTorque(int shapeHandle, std::optional<std::vector<float>> force, std::optional<std::vector<float>> torque)
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

    int sim::addGraphCurve(int graphHandle, std::string curveName, int dim, std::vector<int> streamIds, std::vector<float> defaultValues, std::string unitStr, std::optional<int> options, std::optional<std::vector<float>> color, std::optional<int> curveWidth)
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
        return _ret[0].as<int>();
    }

    int sim::addGraphStream(int graphHandle, std::string streamName, std::string unit, std::optional<int> options, std::optional<std::vector<float>> color, std::optional<float> cyclicRange)
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
        return _ret[0].as<int>();
    }

    void sim::addItemToCollection(int collectionHandle, int what, int objectHandle, int options)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(collectionHandle);
        _args.push_back(what);
        _args.push_back(objectHandle);
        _args.push_back(options);
        auto _ret = this->_client->call("sim.addItemToCollection", _args);
    }

    void sim::addLog(int verbosityLevel, std::string logMessage)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(verbosityLevel);
        _args.push_back(logMessage);
        auto _ret = this->_client->call("sim.addLog", _args);
    }

    int sim::addParticleObject(int objectType, float size, float density, std::vector<float> params, float lifeTime, int maxItemCount, std::optional<std::vector<float>> ambient_diffuse, std::optional<std::vector<float>> reserved, std::optional<std::vector<float>> specular, std::optional<std::vector<float>> emission)
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
        return _ret[0].as<int>();
    }

    void sim::addParticleObjectItem(int particleObjectHandle, std::vector<float> itemData)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(particleObjectHandle);
        _args.push_back(itemData);
        auto _ret = this->_client->call("sim.addParticleObjectItem", _args);
    }

    int sim::addScript(int scriptType)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(scriptType);
        auto _ret = this->_client->call("sim.addScript", _args);
        return _ret[0].as<int>();
    }

    int sim::adjustView(int viewHandleOrIndex, int associatedViewableObjectHandle, int options, std::optional<std::string> viewLabel)
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
        return _ret[0].as<int>();
    }

    std::tuple<float, float, float> sim::alphaBetaGammaToYawPitchRoll(float alphaAngle, float betaAngle, float gammaAngle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(alphaAngle);
        _args.push_back(betaAngle);
        _args.push_back(gammaAngle);
        auto _ret = this->_client->call("sim.alphaBetaGammaToYawPitchRoll", _args);
        return std::make_tuple(_ret[0].as<float>(), _ret[1].as<float>(), _ret[2].as<float>());
    }

    int sim::announceSceneContentChange()
    {
        bool _brk = false;
        json _args(json_array_arg);
        auto _ret = this->_client->call("sim.announceSceneContentChange", _args);
        return _ret[0].as<int>();
    }

    void sim::associateScriptWithObject(int scriptHandle, int objectHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(scriptHandle);
        _args.push_back(objectHandle);
        auto _ret = this->_client->call("sim.associateScriptWithObject", _args);
    }

    int sim::auxiliaryConsoleClose(int consoleHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(consoleHandle);
        auto _ret = this->_client->call("sim.auxiliaryConsoleClose", _args);
        return _ret[0].as<int>();
    }

    int sim::auxiliaryConsoleOpen(std::string title, int maxLines, int mode, std::optional<std::vector<int>> position, std::optional<std::vector<int>> size, std::optional<std::vector<float>> textColor, std::optional<std::vector<float>> backgroundColor)
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
        return _ret[0].as<int>();
    }

    int sim::auxiliaryConsolePrint(int consoleHandle, std::string text)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(consoleHandle);
        _args.push_back(text);
        auto _ret = this->_client->call("sim.auxiliaryConsolePrint", _args);
        return _ret[0].as<int>();
    }

    int sim::auxiliaryConsoleShow(int consoleHandle, bool showState)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(consoleHandle);
        _args.push_back(showState);
        auto _ret = this->_client->call("sim.auxiliaryConsoleShow", _args);
        return _ret[0].as<int>();
    }

    std::vector<float> sim::buildIdentityMatrix()
    {
        bool _brk = false;
        json _args(json_array_arg);
        auto _ret = this->_client->call("sim.buildIdentityMatrix", _args);
        return _ret[0].as<std::vector<float>>();
    }

    std::vector<float> sim::buildMatrix(std::vector<float> position, std::vector<float> eulerAngles)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(position);
        _args.push_back(eulerAngles);
        auto _ret = this->_client->call("sim.buildMatrix", _args);
        return _ret[0].as<std::vector<float>>();
    }

    std::vector<float> sim::buildMatrixQ(std::vector<float> position, std::vector<float> quaternion)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(position);
        _args.push_back(quaternion);
        auto _ret = this->_client->call("sim.buildMatrixQ", _args);
        return _ret[0].as<std::vector<float>>();
    }

    std::vector<float> sim::buildPose(std::vector<float> position, std::vector<float> eulerAnglesOrAxis, std::optional<int> mode, std::optional<std::vector<float>> axis2)
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
        return _ret[0].as<std::vector<float>>();
    }

    int sim::cameraFitToView(int viewHandleOrIndex, std::optional<std::vector<int>> objectHandles, std::optional<int> options, std::optional<float> scaling)
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
        return _ret[0].as<int>();
    }

    std::vector<json> sim::changeEntityColor(int entityHandle, std::vector<float> newColor, std::optional<int> colorComponent)
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

    std::tuple<int, std::vector<int>> sim::checkCollision(int entity1Handle, int entity2Handle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(entity1Handle);
        _args.push_back(entity2Handle);
        auto _ret = this->_client->call("sim.checkCollision", _args);
        return std::make_tuple(_ret[0].as<int>(), _ret[1].as<std::vector<int>>());
    }

    std::tuple<int, std::vector<float>> sim::checkCollisionEx(int entity1Handle, int entity2Handle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(entity1Handle);
        _args.push_back(entity2Handle);
        auto _ret = this->_client->call("sim.checkCollisionEx", _args);
        return std::make_tuple(_ret[0].as<int>(), _ret[1].as<std::vector<float>>());
    }

    std::tuple<int, std::vector<float>, std::vector<int>> sim::checkDistance(int entity1Handle, int entity2Handle, std::optional<float> threshold)
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
        return std::make_tuple(_ret[0].as<int>(), _ret[1].as<std::vector<float>>(), _ret[2].as<std::vector<int>>());
    }

    std::tuple<int, int, int, int> sim::checkOctreePointOccupancy(int octreeHandle, int options, std::vector<float> points)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(octreeHandle);
        _args.push_back(options);
        _args.push_back(points);
        auto _ret = this->_client->call("sim.checkOctreePointOccupancy", _args);
        return std::make_tuple(_ret[0].as<int>(), _ret[1].as<int>(), _ret[2].as<int>(), _ret[3].as<int>());
    }

    std::tuple<int, float, std::vector<float>, int, std::vector<float>> sim::checkProximitySensor(int sensorHandle, int entityHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(sensorHandle);
        _args.push_back(entityHandle);
        auto _ret = this->_client->call("sim.checkProximitySensor", _args);
        return std::make_tuple(_ret[0].as<int>(), _ret[1].as<float>(), _ret[2].as<std::vector<float>>(), _ret[3].as<int>(), _ret[4].as<std::vector<float>>());
    }

    std::tuple<int, float, std::vector<float>, int, std::vector<float>> sim::checkProximitySensorEx(int sensorHandle, int entityHandle, int mode, float threshold, float maxAngle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(sensorHandle);
        _args.push_back(entityHandle);
        _args.push_back(mode);
        _args.push_back(threshold);
        _args.push_back(maxAngle);
        auto _ret = this->_client->call("sim.checkProximitySensorEx", _args);
        return std::make_tuple(_ret[0].as<int>(), _ret[1].as<float>(), _ret[2].as<std::vector<float>>(), _ret[3].as<int>(), _ret[4].as<std::vector<float>>());
    }

    std::tuple<int, float, std::vector<float>, std::vector<float>> sim::checkProximitySensorEx2(int sensorHandle, std::vector<float> vertices, int itemType, int itemCount, int mode, float threshold, float maxAngle)
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
        return std::make_tuple(_ret[0].as<int>(), _ret[1].as<float>(), _ret[2].as<std::vector<float>>(), _ret[3].as<std::vector<float>>());
    }

    std::vector<float> sim::checkVisionSensorEx(int sensorHandle, int entityHandle, bool returnImage)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(sensorHandle);
        _args.push_back(entityHandle);
        _args.push_back(returnImage);
        auto _ret = this->_client->call("sim.checkVisionSensorEx", _args);
        return _ret[0].as<std::vector<float>>();
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

    int sim::closeScene()
    {
        bool _brk = false;
        json _args(json_array_arg);
        auto _ret = this->_client->call("sim.closeScene", _args);
        return _ret[0].as<int>();
    }

    std::vector<uint8_t> sim::combineRgbImages(std::vector<uint8_t> img1, std::vector<int> img1Res, std::vector<uint8_t> img2, std::vector<int> img2Res, int operation)
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

    int sim::computeMassAndInertia(int shapeHandle, float density)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(shapeHandle);
        _args.push_back(density);
        auto _ret = this->_client->call("sim.computeMassAndInertia", _args);
        return _ret[0].as<int>();
    }

    int sim::convexDecompose(int shapeHandle, int options, std::vector<int> intParams, std::vector<float> floatParams)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(shapeHandle);
        _args.push_back(options);
        _args.push_back(intParams);
        _args.push_back(floatParams);
        auto _ret = this->_client->call("sim.convexDecompose", _args);
        return _ret[0].as<int>();
    }

    std::vector<int> sim::copyPasteObjects(std::vector<int> objectHandles, int options)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(objectHandles);
        _args.push_back(options);
        auto _ret = this->_client->call("sim.copyPasteObjects", _args);
        return _ret[0].as<std::vector<int>>();
    }

    std::vector<json> sim::copyTable(std::vector<json> original)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(original);
        auto _ret = this->_client->call("sim.copyTable", _args);
        return _ret[0].as<std::vector<json>>();
    }

    int sim::createCollection(int options)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(options);
        auto _ret = this->_client->call("sim.createCollection", _args);
        return _ret[0].as<int>();
    }

    int sim::createDummy(float size)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(size);
        auto _ret = this->_client->call("sim.createDummy", _args);
        return _ret[0].as<int>();
    }

    int sim::createForceSensor(int options, std::vector<int> intParams, std::vector<float> floatParams)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(options);
        _args.push_back(intParams);
        _args.push_back(floatParams);
        auto _ret = this->_client->call("sim.createForceSensor", _args);
        return _ret[0].as<int>();
    }

    int sim::createHeightfieldShape(int options, float shadingAngle, int xPointCount, int yPointCount, float xSize, std::vector<float> heights)
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
        return _ret[0].as<int>();
    }

    int sim::createJoint(int jointType, int jointMode, int options, std::optional<std::vector<float>> sizes)
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
        return _ret[0].as<int>();
    }

    int sim::createMeshShape(int options, float shadingAngle, std::vector<float> vertices, std::vector<int> indices)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(options);
        _args.push_back(shadingAngle);
        _args.push_back(vertices);
        _args.push_back(indices);
        auto _ret = this->_client->call("sim.createMeshShape", _args);
        return _ret[0].as<int>();
    }

    int sim::createOctree(float voxelSize, int options, float pointSize)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(voxelSize);
        _args.push_back(options);
        _args.push_back(pointSize);
        auto _ret = this->_client->call("sim.createOctree", _args);
        return _ret[0].as<int>();
    }

    int sim::createPath(std::vector<float> ctrlPts, std::optional<int> options, std::optional<int> subdiv, std::optional<float> smoothness, std::optional<int> orientationMode, std::optional<std::vector<float>> upVector)
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
        return _ret[0].as<int>();
    }

    int sim::createPointCloud(float maxVoxelSize, int maxPtCntPerVoxel, int options, float pointSize)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(maxVoxelSize);
        _args.push_back(maxPtCntPerVoxel);
        _args.push_back(options);
        _args.push_back(pointSize);
        auto _ret = this->_client->call("sim.createPointCloud", _args);
        return _ret[0].as<int>();
    }

    int sim::createProximitySensor(int sensorType, int subType, int options, std::vector<int> intParams, std::vector<float> floatParams)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(sensorType);
        _args.push_back(subType);
        _args.push_back(options);
        _args.push_back(intParams);
        _args.push_back(floatParams);
        auto _ret = this->_client->call("sim.createProximitySensor", _args);
        return _ret[0].as<int>();
    }

    int sim::createPureShape(int primitiveType, int options, std::vector<float> sizes, float mass, std::optional<std::vector<int>> precision)
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
        return _ret[0].as<int>();
    }

    std::tuple<int, int, std::vector<int>> sim::createTexture(std::string fileName, int options, std::optional<std::vector<float>> planeSizes, std::optional<std::vector<float>> scalingUV, std::optional<std::vector<float>> xy_g, std::optional<int> fixedResolution, std::optional<std::vector<int>> resolution)
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
        return std::make_tuple(_ret[0].as<int>(), _ret[1].as<int>(), _ret[2].as<std::vector<int>>());
    }

    int sim::createVisionSensor(int options, std::vector<int> intParams, std::vector<float> floatParams)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(options);
        _args.push_back(intParams);
        _args.push_back(floatParams);
        auto _ret = this->_client->call("sim.createVisionSensor", _args);
        return _ret[0].as<int>();
    }

    void sim::destroyCollection(int collectionHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(collectionHandle);
        auto _ret = this->_client->call("sim.destroyCollection", _args);
    }

    void sim::destroyGraphCurve(int graphHandle, int curveId)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(graphHandle);
        _args.push_back(curveId);
        auto _ret = this->_client->call("sim.destroyGraphCurve", _args);
    }

    int sim::duplicateGraphCurveToStatic(int graphHandle, int curveId, std::optional<std::string> curveName)
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
        return _ret[0].as<int>();
    }

    void sim::exportMesh(int fileformat, std::string pathAndFilename, int options, float scalingFactor, std::vector<float> vertices, std::vector<int> indices)
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

    int sim::floatingViewAdd(float posX, float posY, float sizeX, float sizeY, int options)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(posX);
        _args.push_back(posY);
        _args.push_back(sizeX);
        _args.push_back(sizeY);
        _args.push_back(options);
        auto _ret = this->_client->call("sim.floatingViewAdd", _args);
        return _ret[0].as<int>();
    }

    int sim::floatingViewRemove(int floatingViewHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(floatingViewHandle);
        auto _ret = this->_client->call("sim.floatingViewRemove", _args);
        return _ret[0].as<int>();
    }

    int sim::generateShapeFromPath(std::vector<float> path, std::vector<float> section, std::optional<int> options, std::optional<std::vector<float>> upVector)
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
        return _ret[0].as<int>();
    }

    int sim::generateTextShape(std::string txt, std::optional<std::vector<float>> color, std::optional<float> height, std::optional<bool> centered, std::optional<std::string> alphabetLocation)
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
        return _ret[0].as<int>();
    }

    std::tuple<std::vector<float>, std::vector<float>> sim::generateTimeOptimalTrajectory(std::vector<float> path, std::vector<float> pathLengths, std::vector<float> minMaxVel, std::vector<float> minMaxAccel, std::optional<int> trajPtSamples, std::optional<std::string> boundaryCondition, std::optional<float> timeout)
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
        return std::make_tuple(_ret[0].as<std::vector<float>>(), _ret[1].as<std::vector<float>>());
    }

    std::vector<float> sim::getAlternateConfigs(std::vector<int> jointHandles, std::vector<float> inputConfig, std::optional<int> tipHandle, std::optional<std::vector<float>> lowLimits, std::optional<std::vector<float>> ranges)
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
        return _ret[0].as<std::vector<float>>();
    }

    std::vector<std::string> sim::getApiFunc(int scriptHandleOrType, std::string apiWord)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(scriptHandleOrType);
        _args.push_back(apiWord);
        auto _ret = this->_client->call("sim.getApiFunc", _args);
        return _ret[0].as<std::vector<std::string>>();
    }

    std::string sim::getApiInfo(int scriptHandleOrType, std::string apiWord)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(scriptHandleOrType);
        _args.push_back(apiWord);
        auto _ret = this->_client->call("sim.getApiInfo", _args);
        return _ret[0].as<std::string>();
    }

    std::vector<float> sim::getArrayParam(int parameter)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(parameter);
        auto _ret = this->_client->call("sim.getArrayParam", _args);
        return _ret[0].as<std::vector<float>>();
    }

    bool sim::getBoolParam(int parameter)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(parameter);
        auto _ret = this->_client->call("sim.getBoolParam", _args);
        return _ret[0].as<bool>();
    }

    float sim::getClosestPosOnPath(std::vector<float> path, std::vector<float> pathLengths, std::vector<float> absPt)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(path);
        _args.push_back(pathLengths);
        _args.push_back(absPt);
        auto _ret = this->_client->call("sim.getClosestPosOnPath", _args);
        return _ret[0].as<float>();
    }

    std::vector<int> sim::getCollectionObjects(int collectionHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(collectionHandle);
        auto _ret = this->_client->call("sim.getCollectionObjects", _args);
        return _ret[0].as<std::vector<int>>();
    }

    float sim::getConfigDistance(std::vector<float> configA, std::vector<float> configB, std::optional<std::vector<float>> metric, std::optional<std::vector<int>> types)
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
        return _ret[0].as<float>();
    }

    std::tuple<std::vector<int>, std::vector<float>, std::vector<float>, std::vector<float>> sim::getContactInfo(int dynamicPass, int objectHandle, int index)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(dynamicPass);
        _args.push_back(objectHandle);
        _args.push_back(index);
        auto _ret = this->_client->call("sim.getContactInfo", _args);
        return std::make_tuple(_ret[0].as<std::vector<int>>(), _ret[1].as<std::vector<float>>(), _ret[2].as<std::vector<float>>(), _ret[3].as<std::vector<float>>());
    }

    std::tuple<std::vector<float>, std::vector<int>> sim::getDecimatedMesh(std::vector<float> verticesIn, std::vector<int> indicesIn, float decimationPercentage)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(verticesIn);
        _args.push_back(indicesIn);
        _args.push_back(decimationPercentage);
        auto _ret = this->_client->call("sim.getDecimatedMesh", _args);
        return std::make_tuple(_ret[0].as<std::vector<float>>(), _ret[1].as<std::vector<int>>());
    }

    float sim::getDoubleSignal(std::string signalName)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(signalName);
        auto _ret = this->_client->call("sim.getDoubleSignal", _args);
        return _ret[0].as<float>();
    }

    bool sim::getEngineBoolParam(int paramId, int objectHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(paramId);
        _args.push_back(objectHandle);
        auto _ret = this->_client->call("sim.getEngineBoolParam", _args);
        return _ret[0].as<bool>();
    }

    float sim::getEngineFloatParam(int paramId, int objectHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(paramId);
        _args.push_back(objectHandle);
        auto _ret = this->_client->call("sim.getEngineFloatParam", _args);
        return _ret[0].as<float>();
    }

    int sim::getEngineInt32Param(int paramId, int objectHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(paramId);
        _args.push_back(objectHandle);
        auto _ret = this->_client->call("sim.getEngineInt32Param", _args);
        return _ret[0].as<int>();
    }

    std::vector<float> sim::getEulerAnglesFromMatrix(std::vector<float> matrix)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(matrix);
        auto _ret = this->_client->call("sim.getEulerAnglesFromMatrix", _args);
        return _ret[0].as<std::vector<float>>();
    }

    int sim::getExplicitHandling(int objectHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(objectHandle);
        auto _ret = this->_client->call("sim.getExplicitHandling", _args);
        return _ret[0].as<int>();
    }

    std::string sim::getExtensionString(int objectHandle, int index, std::optional<std::string> key)
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

    float sim::getFloatParam(int parameter)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(parameter);
        auto _ret = this->_client->call("sim.getFloatParam", _args);
        return _ret[0].as<float>();
    }

    float sim::getFloatSignal(std::string signalName)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(signalName);
        auto _ret = this->_client->call("sim.getFloatSignal", _args);
        return _ret[0].as<float>();
    }

    std::vector<json> sim::getGenesisEvents()
    {
        bool _brk = false;
        json _args(json_array_arg);
        auto _ret = this->_client->call("sim.getGenesisEvents", _args);
        return _ret[0].as<std::vector<json>>();
    }

    std::tuple<std::string, int, std::vector<float>, std::vector<float>, std::vector<float>, std::vector<float>, int, int> sim::getGraphCurve(int graphHandle, int graphType, int curveIndex)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(graphHandle);
        _args.push_back(graphType);
        _args.push_back(curveIndex);
        auto _ret = this->_client->call("sim.getGraphCurve", _args);
        return std::make_tuple(_ret[0].as<std::string>(), _ret[1].as<int>(), _ret[2].as<std::vector<float>>(), _ret[3].as<std::vector<float>>(), _ret[4].as<std::vector<float>>(), _ret[5].as<std::vector<float>>(), _ret[6].as<int>(), _ret[7].as<int>());
    }

    std::tuple<int, std::vector<float>, std::vector<float>, int> sim::getGraphInfo(int graphHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(graphHandle);
        auto _ret = this->_client->call("sim.getGraphInfo", _args);
        return std::make_tuple(_ret[0].as<int>(), _ret[1].as<std::vector<float>>(), _ret[2].as<std::vector<float>>(), _ret[3].as<int>());
    }

    int sim::getInt32Param(int parameter)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(parameter);
        auto _ret = this->_client->call("sim.getInt32Param", _args);
        return _ret[0].as<int>();
    }

    int sim::getInt32Signal(std::string signalName)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(signalName);
        auto _ret = this->_client->call("sim.getInt32Signal", _args);
        return _ret[0].as<int>();
    }

    std::tuple<int, float, float> sim::getJointDependency(int jointHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(jointHandle);
        auto _ret = this->_client->call("sim.getJointDependency", _args);
        return std::make_tuple(_ret[0].as<int>(), _ret[1].as<float>(), _ret[2].as<float>());
    }

    float sim::getJointForce(int jointHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(jointHandle);
        auto _ret = this->_client->call("sim.getJointForce", _args);
        return _ret[0].as<float>();
    }

    std::tuple<bool, std::vector<float>> sim::getJointInterval(int objectHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(objectHandle);
        auto _ret = this->_client->call("sim.getJointInterval", _args);
        return std::make_tuple(_ret[0].as<bool>(), _ret[1].as<std::vector<float>>());
    }

    std::tuple<int, int> sim::getJointMode(int jointHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(jointHandle);
        auto _ret = this->_client->call("sim.getJointMode", _args);
        return std::make_tuple(_ret[0].as<int>(), _ret[1].as<int>());
    }

    float sim::getJointPosition(int objectHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(objectHandle);
        auto _ret = this->_client->call("sim.getJointPosition", _args);
        return _ret[0].as<float>();
    }

    float sim::getJointTargetForce(int jointHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(jointHandle);
        auto _ret = this->_client->call("sim.getJointTargetForce", _args);
        return _ret[0].as<float>();
    }

    std::tuple<int, float> sim::getJointTargetPosition(int objectHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(objectHandle);
        auto _ret = this->_client->call("sim.getJointTargetPosition", _args);
        return std::make_tuple(_ret[0].as<int>(), _ret[1].as<float>());
    }

    float sim::getJointTargetVelocity(int objectHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(objectHandle);
        auto _ret = this->_client->call("sim.getJointTargetVelocity", _args);
        return _ret[0].as<float>();
    }

    int sim::getJointType(int objectHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(objectHandle);
        auto _ret = this->_client->call("sim.getJointType", _args);
        return _ret[0].as<int>();
    }

    float sim::getJointVelocity(int jointHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(jointHandle);
        auto _ret = this->_client->call("sim.getJointVelocity", _args);
        return _ret[0].as<float>();
    }

    std::tuple<int, std::vector<float>, std::vector<float>, std::vector<float>> sim::getLightParameters(int lightHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(lightHandle);
        auto _ret = this->_client->call("sim.getLightParameters", _args);
        return std::make_tuple(_ret[0].as<int>(), _ret[1].as<std::vector<float>>(), _ret[2].as<std::vector<float>>(), _ret[3].as<std::vector<float>>());
    }

    int sim::getLinkDummy(int dummyHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(dummyHandle);
        auto _ret = this->_client->call("sim.getLinkDummy", _args);
        return _ret[0].as<int>();
    }

    std::vector<std::string> sim::getMatchingPersistentDataTags(std::string pattern)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(pattern);
        auto _ret = this->_client->call("sim.getMatchingPersistentDataTags", _args);
        return _ret[0].as<std::vector<std::string>>();
    }

    int sim::getModelProperty(int objectHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(objectHandle);
        auto _ret = this->_client->call("sim.getModelProperty", _args);
        return _ret[0].as<int>();
    }

    std::string sim::getModuleInfo(std::string moduleName, int infoType)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(moduleName);
        _args.push_back(infoType);
        auto _ret = this->_client->call("sim.getModuleInfo", _args);
        return _ret[0].as<std::string>();
    }

    std::tuple<std::string, int> sim::getModuleName(int index)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(index);
        auto _ret = this->_client->call("sim.getModuleName", _args);
        return std::make_tuple(_ret[0].as<std::string>(), _ret[1].as<int>());
    }

    std::vector<uint8_t> sim::getNamedStringParam(std::string paramName)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(paramName);
        auto _ret = this->_client->call("sim.getNamedStringParam", _args);
        return _ret[0].as<std::vector<uint8_t>>();
    }

    int sim::getNavigationMode()
    {
        bool _brk = false;
        json _args(json_array_arg);
        auto _ret = this->_client->call("sim.getNavigationMode", _args);
        return _ret[0].as<int>();
    }

    int sim::getObject(std::string path, std::optional<json> options)
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
        return _ret[0].as<int>();
    }

    std::string sim::getObjectAlias(int objectHandle, std::optional<int> options)
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

    int sim::getObjectChild(int objectHandle, int index)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(objectHandle);
        _args.push_back(index);
        auto _ret = this->_client->call("sim.getObjectChild", _args);
        return _ret[0].as<int>();
    }

    std::vector<float> sim::getObjectChildPose(int objectHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(objectHandle);
        auto _ret = this->_client->call("sim.getObjectChildPose", _args);
        return _ret[0].as<std::vector<float>>();
    }

    std::vector<float> sim::getObjectColor(int objectHandle, int index, int colorComponent)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(objectHandle);
        _args.push_back(index);
        _args.push_back(colorComponent);
        auto _ret = this->_client->call("sim.getObjectColor", _args);
        return _ret[0].as<std::vector<float>>();
    }

    float sim::getObjectFloatParam(int objectHandle, int parameterID)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(objectHandle);
        _args.push_back(parameterID);
        auto _ret = this->_client->call("sim.getObjectFloatParam", _args);
        return _ret[0].as<float>();
    }

    void sim::getObjectFromUid(int uid, std::optional<json> options)
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

    int sim::getObjectInt32Param(int objectHandle, int parameterID)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(objectHandle);
        _args.push_back(parameterID);
        auto _ret = this->_client->call("sim.getObjectInt32Param", _args);
        return _ret[0].as<int>();
    }

    std::vector<float> sim::getObjectMatrix(int objectHandle, int relativeToObjectHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(objectHandle);
        _args.push_back(relativeToObjectHandle);
        auto _ret = this->_client->call("sim.getObjectMatrix", _args);
        return _ret[0].as<std::vector<float>>();
    }

    std::vector<float> sim::getObjectOrientation(int objectHandle, int relativeToObjectHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(objectHandle);
        _args.push_back(relativeToObjectHandle);
        auto _ret = this->_client->call("sim.getObjectOrientation", _args);
        return _ret[0].as<std::vector<float>>();
    }

    int sim::getObjectParent(int objectHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(objectHandle);
        auto _ret = this->_client->call("sim.getObjectParent", _args);
        return _ret[0].as<int>();
    }

    std::vector<float> sim::getObjectPose(int objectHandle, int relativeToObjectHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(objectHandle);
        _args.push_back(relativeToObjectHandle);
        auto _ret = this->_client->call("sim.getObjectPose", _args);
        return _ret[0].as<std::vector<float>>();
    }

    std::vector<float> sim::getObjectPosition(int objectHandle, int relativeToObjectHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(objectHandle);
        _args.push_back(relativeToObjectHandle);
        auto _ret = this->_client->call("sim.getObjectPosition", _args);
        return _ret[0].as<std::vector<float>>();
    }

    int sim::getObjectProperty(int objectHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(objectHandle);
        auto _ret = this->_client->call("sim.getObjectProperty", _args);
        return _ret[0].as<int>();
    }

    std::vector<float> sim::getObjectQuaternion(int objectHandle, int relativeToObjectHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(objectHandle);
        _args.push_back(relativeToObjectHandle);
        auto _ret = this->_client->call("sim.getObjectQuaternion", _args);
        return _ret[0].as<std::vector<float>>();
    }

    std::vector<int> sim::getObjectSelection()
    {
        bool _brk = false;
        json _args(json_array_arg);
        auto _ret = this->_client->call("sim.getObjectSelection", _args);
        return _ret[0].as<std::vector<int>>();
    }

    float sim::getObjectSizeFactor(int ObjectHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(ObjectHandle);
        auto _ret = this->_client->call("sim.getObjectSizeFactor", _args);
        return _ret[0].as<float>();
    }

    int sim::getObjectSpecialProperty(int objectHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(objectHandle);
        auto _ret = this->_client->call("sim.getObjectSpecialProperty", _args);
        return _ret[0].as<int>();
    }

    std::vector<uint8_t> sim::getObjectStringParam(int objectHandle, int parameterID)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(objectHandle);
        _args.push_back(parameterID);
        auto _ret = this->_client->call("sim.getObjectStringParam", _args);
        return _ret[0].as<std::vector<uint8_t>>();
    }

    int sim::getObjectType(int objectHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(objectHandle);
        auto _ret = this->_client->call("sim.getObjectType", _args);
        return _ret[0].as<int>();
    }

    int sim::getObjectUid(int objectHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(objectHandle);
        auto _ret = this->_client->call("sim.getObjectUid", _args);
        return _ret[0].as<int>();
    }

    std::tuple<std::vector<float>, std::vector<float>> sim::getObjectVelocity(int objectHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(objectHandle);
        auto _ret = this->_client->call("sim.getObjectVelocity", _args);
        return std::make_tuple(_ret[0].as<std::vector<float>>(), _ret[1].as<std::vector<float>>());
    }

    int sim::getObjects(int index, int objectType)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(index);
        _args.push_back(objectType);
        auto _ret = this->_client->call("sim.getObjects", _args);
        return _ret[0].as<int>();
    }

    std::vector<int> sim::getObjectsInTree(int treeBaseHandle, std::optional<int> objectType, std::optional<int> options)
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
        return _ret[0].as<std::vector<int>>();
    }

    std::vector<float> sim::getOctreeVoxels(int octreeHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(octreeHandle);
        auto _ret = this->_client->call("sim.getOctreeVoxels", _args);
        return _ret[0].as<std::vector<float>>();
    }

    int sim::getPage()
    {
        bool _brk = false;
        json _args(json_array_arg);
        auto _ret = this->_client->call("sim.getPage", _args);
        return _ret[0].as<int>();
    }

    std::vector<float> sim::getPathInterpolatedConfig(std::vector<float> path, std::vector<float> pathLengths, float t, std::optional<json> method, std::optional<std::vector<int>> types)
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
        return _ret[0].as<std::vector<float>>();
    }

    std::tuple<std::vector<float>, float> sim::getPathLengths(std::vector<float> path, int dof, std::optional<std::string> distCallback)
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
        return std::make_tuple(_ret[0].as<std::vector<float>>(), _ret[1].as<float>());
    }

    std::vector<std::string> sim::getPersistentDataTags()
    {
        bool _brk = false;
        json _args(json_array_arg);
        auto _ret = this->_client->call("sim.getPersistentDataTags", _args);
        return _ret[0].as<std::vector<std::string>>();
    }

    std::tuple<float, int, int, float> sim::getPointCloudOptions(int pointCloudHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(pointCloudHandle);
        auto _ret = this->_client->call("sim.getPointCloudOptions", _args);
        return std::make_tuple(_ret[0].as<float>(), _ret[1].as<int>(), _ret[2].as<int>(), _ret[3].as<float>());
    }

    std::vector<float> sim::getPointCloudPoints(int pointCloudHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(pointCloudHandle);
        auto _ret = this->_client->call("sim.getPointCloudPoints", _args);
        return _ret[0].as<std::vector<float>>();
    }

    std::tuple<std::vector<float>, std::vector<int>> sim::getQHull(std::vector<float> verticesIn)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(verticesIn);
        auto _ret = this->_client->call("sim.getQHull", _args);
        return std::make_tuple(_ret[0].as<std::vector<float>>(), _ret[1].as<std::vector<int>>());
    }

    std::vector<float> sim::getQuaternionFromMatrix(std::vector<float> matrix)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(matrix);
        auto _ret = this->_client->call("sim.getQuaternionFromMatrix", _args);
        return _ret[0].as<std::vector<float>>();
    }

    float sim::getRandom(std::optional<int> seed)
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
        return _ret[0].as<float>();
    }

    int sim::getRealTimeSimulation()
    {
        bool _brk = false;
        json _args(json_array_arg);
        auto _ret = this->_client->call("sim.getRealTimeSimulation", _args);
        return _ret[0].as<int>();
    }

    std::vector<int> sim::getReferencedHandles(int objectHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(objectHandle);
        auto _ret = this->_client->call("sim.getReferencedHandles", _args);
        return _ret[0].as<std::vector<int>>();
    }

    std::tuple<std::vector<float>, float> sim::getRotationAxis(std::vector<float> matrixStart, std::vector<float> matrixGoal)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(matrixStart);
        _args.push_back(matrixGoal);
        auto _ret = this->_client->call("sim.getRotationAxis", _args);
        return std::make_tuple(_ret[0].as<std::vector<float>>(), _ret[1].as<float>());
    }

    std::tuple<std::vector<uint8_t>, std::vector<int>> sim::getScaledImage(std::vector<uint8_t> imageIn, std::vector<int> resolutionIn, std::vector<int> desiredResolutionOut, int options)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(bin(imageIn));
        _args.push_back(resolutionIn);
        _args.push_back(desiredResolutionOut);
        _args.push_back(options);
        auto _ret = this->_client->call("sim.getScaledImage", _args);
        return std::make_tuple(_ret[0].as<std::vector<uint8_t>>(), _ret[1].as<std::vector<int>>());
    }

    int sim::getScriptHandle(int scriptType, std::optional<std::string> scriptName)
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
        auto _ret = this->_client->call("sim.getScriptHandle", _args);
        return _ret[0].as<int>();
    }

    int sim::getScriptInt32Param(int scriptHandle, int parameterID)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(scriptHandle);
        _args.push_back(parameterID);
        auto _ret = this->_client->call("sim.getScriptInt32Param", _args);
        return _ret[0].as<int>();
    }

    std::vector<uint8_t> sim::getScriptStringParam(int scriptHandle, int parameterID)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(scriptHandle);
        _args.push_back(parameterID);
        auto _ret = this->_client->call("sim.getScriptStringParam", _args);
        return _ret[0].as<std::vector<uint8_t>>();
    }

    std::vector<float> sim::getShapeBB(int shapeHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(shapeHandle);
        auto _ret = this->_client->call("sim.getShapeBB", _args);
        return _ret[0].as<std::vector<float>>();
    }

    std::tuple<int, std::vector<float>> sim::getShapeColor(int shapeHandle, std::string colorName, int colorComponent)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(shapeHandle);
        _args.push_back(colorName);
        _args.push_back(colorComponent);
        auto _ret = this->_client->call("sim.getShapeColor", _args);
        return std::make_tuple(_ret[0].as<int>(), _ret[1].as<std::vector<float>>());
    }

    std::tuple<int, int, std::vector<float>> sim::getShapeGeomInfo(int shapeHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(shapeHandle);
        auto _ret = this->_client->call("sim.getShapeGeomInfo", _args);
        return std::make_tuple(_ret[0].as<int>(), _ret[1].as<int>(), _ret[2].as<std::vector<float>>());
    }

    std::tuple<std::vector<float>, std::vector<float>> sim::getShapeInertia(int shapeHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(shapeHandle);
        auto _ret = this->_client->call("sim.getShapeInertia", _args);
        return std::make_tuple(_ret[0].as<std::vector<float>>(), _ret[1].as<std::vector<float>>());
    }

    float sim::getShapeMass(int shapeHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(shapeHandle);
        auto _ret = this->_client->call("sim.getShapeMassAndInertia", _args);
        return _ret[0].as<float>();
    }

    std::tuple<std::vector<float>, std::vector<int>, std::vector<float>> sim::getShapeMesh(int shapeHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(shapeHandle);
        auto _ret = this->_client->call("sim.getShapeMesh", _args);
        return std::make_tuple(_ret[0].as<std::vector<float>>(), _ret[1].as<std::vector<int>>(), _ret[2].as<std::vector<float>>());
    }

    int sim::getShapeTextureId(int shapeHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(shapeHandle);
        auto _ret = this->_client->call("sim.getShapeTextureId", _args);
        return _ret[0].as<int>();
    }

    json sim::getShapeViz(int shapeHandle, int itemIndex)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(shapeHandle);
        _args.push_back(itemIndex);
        auto _ret = this->_client->call("sim.getShapeViz", _args);
        return _ret[0].as<json>();
    }

    std::string sim::getSignalName(int signalIndex, int signalType)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(signalIndex);
        _args.push_back(signalType);
        auto _ret = this->_client->call("sim.getSignalName", _args);
        return _ret[0].as<std::string>();
    }

    int sim::getSimulationState()
    {
        bool _brk = false;
        json _args(json_array_arg);
        auto _ret = this->_client->call("sim.getSimulationState", _args);
        return _ret[0].as<int>();
    }

    float sim::getSimulationTime()
    {
        bool _brk = false;
        json _args(json_array_arg);
        auto _ret = this->_client->call("sim.getSimulationTime", _args);
        return _ret[0].as<float>();
    }

    float sim::getSimulationTimeStep()
    {
        bool _brk = false;
        json _args(json_array_arg);
        auto _ret = this->_client->call("sim.getSimulationTimeStep", _args);
        return _ret[0].as<float>();
    }

    std::tuple<int, std::vector<int>, std::vector<int>> sim::getSimulatorMessage()
    {
        bool _brk = false;
        json _args(json_array_arg);
        auto _ret = this->_client->call("sim.getSimulatorMessage", _args);
        return std::make_tuple(_ret[0].as<int>(), _ret[1].as<std::vector<int>>(), _ret[2].as<std::vector<int>>());
    }

    std::string sim::getStackTraceback(std::optional<int> scriptHandle)
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

    std::string sim::getStringParam(int parameter)
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

    float sim::getSystemTime()
    {
        bool _brk = false;
        json _args(json_array_arg);
        auto _ret = this->_client->call("sim.getSystemTime", _args);
        return _ret[0].as<float>();
    }

    std::tuple<int, std::vector<int>> sim::getTextureId(std::string textureName)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(textureName);
        auto _ret = this->_client->call("sim.getTextureId", _args);
        return std::make_tuple(_ret[0].as<int>(), _ret[1].as<std::vector<int>>());
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

    int sim::getThreadId()
    {
        bool _brk = false;
        json _args(json_array_arg);
        auto _ret = this->_client->call("sim.getThreadId", _args);
        return _ret[0].as<int>();
    }

    bool sim::getThreadSwitchAllowed()
    {
        bool _brk = false;
        json _args(json_array_arg);
        auto _ret = this->_client->call("sim.getThreadSwitchAllowed", _args);
        return _ret[0].as<bool>();
    }

    int sim::getThreadSwitchTiming()
    {
        bool _brk = false;
        json _args(json_array_arg);
        auto _ret = this->_client->call("sim.getThreadSwitchTiming", _args);
        return _ret[0].as<int>();
    }

    std::vector<std::string> sim::getUserVariables()
    {
        bool _brk = false;
        json _args(json_array_arg);
        auto _ret = this->_client->call("sim.getUserVariables", _args);
        return _ret[0].as<std::vector<std::string>>();
    }

    std::tuple<std::vector<float>, std::vector<float>> sim::getVelocity(int shapeHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(shapeHandle);
        auto _ret = this->_client->call("sim.getVelocity", _args);
        return std::make_tuple(_ret[0].as<std::vector<float>>(), _ret[1].as<std::vector<float>>());
    }

    std::tuple<std::vector<uint8_t>, int, int> sim::getVisionSensorCharImage(int sensorHandle, std::optional<int> posX, std::optional<int> posY, std::optional<int> sizeX, std::optional<int> sizeY, std::optional<float> RgbaCutoff)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(sensorHandle);
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
        if(RgbaCutoff)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*RgbaCutoff);
        }
        else _brk = true;
        auto _ret = this->_client->call("sim.getVisionSensorCharImage", _args);
        return std::make_tuple(_ret[0].as<std::vector<uint8_t>>(), _ret[1].as<int>(), _ret[2].as<int>());
    }

    std::vector<float> sim::getVisionSensorDepthBuffer(int sensorHandle, std::optional<int> posX, std::optional<int> posY, std::optional<int> sizeX, std::optional<int> sizeY)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(sensorHandle);
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
        auto _ret = this->_client->call("sim.getVisionSensorDepthBuffer", _args);
        return _ret[0].as<std::vector<float>>();
    }

    std::vector<float> sim::getVisionSensorImage(int sensorHandle, std::optional<int> posX, std::optional<int> posY, std::optional<int> sizeX, std::optional<int> sizeY, std::optional<int> returnType)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(sensorHandle);
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
        if(returnType)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(*returnType);
        }
        else _brk = true;
        auto _ret = this->_client->call("sim.getVisionSensorImage", _args);
        return _ret[0].as<std::vector<float>>();
    }

    std::vector<int> sim::getVisionSensorResolution(int sensorHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(sensorHandle);
        auto _ret = this->_client->call("sim.getVisionSensorResolution", _args);
        return _ret[0].as<std::vector<int>>();
    }

    int sim::groupShapes(std::vector<int> shapeHandles, std::optional<bool> merge)
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
        return _ret[0].as<int>();
    }

    int sim::handleAddOnScripts(int callType)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(callType);
        auto _ret = this->_client->call("sim.handleAddOnScripts", _args);
        return _ret[0].as<int>();
    }

    int sim::handleChildScripts(int callType)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(callType);
        auto _ret = this->_client->call("sim.handleChildScripts", _args);
        return _ret[0].as<int>();
    }

    int sim::handleCustomizationScripts(int callType)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(callType);
        auto _ret = this->_client->call("sim.handleCustomizationScripts", _args);
        return _ret[0].as<int>();
    }

    int sim::handleDynamics(float deltaTime)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(deltaTime);
        auto _ret = this->_client->call("sim.handleDynamics", _args);
        return _ret[0].as<int>();
    }

    void sim::handleGraph(int objectHandle, float simulationTime)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(objectHandle);
        _args.push_back(simulationTime);
        auto _ret = this->_client->call("sim.handleGraph", _args);
    }

    std::tuple<int, float, std::vector<float>, int, std::vector<float>> sim::handleProximitySensor(int sensorHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(sensorHandle);
        auto _ret = this->_client->call("sim.handleProximitySensor", _args);
        return std::make_tuple(_ret[0].as<int>(), _ret[1].as<float>(), _ret[2].as<std::vector<float>>(), _ret[3].as<int>(), _ret[4].as<std::vector<float>>());
    }

    void sim::handleSandboxScript(int callType)
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

    std::tuple<std::vector<float>, std::vector<int>> sim::importMesh(int fileformat, std::string pathAndFilename, int options, float identicalVerticeTolerance, float scalingFactor)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(fileformat);
        _args.push_back(pathAndFilename);
        _args.push_back(options);
        _args.push_back(identicalVerticeTolerance);
        _args.push_back(scalingFactor);
        auto _ret = this->_client->call("sim.importMesh", _args);
        return std::make_tuple(_ret[0].as<std::vector<float>>(), _ret[1].as<std::vector<int>>());
    }

    int sim::importShape(int fileformat, std::string pathAndFilename, int options, float identicalVerticeTolerance, float scalingFactor)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(fileformat);
        _args.push_back(pathAndFilename);
        _args.push_back(options);
        _args.push_back(identicalVerticeTolerance);
        _args.push_back(scalingFactor);
        auto _ret = this->_client->call("sim.importShape", _args);
        return _ret[0].as<int>();
    }

    bool sim::initScript(int scriptHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(scriptHandle);
        auto _ret = this->_client->call("sim.initScript", _args);
        return _ret[0].as<bool>();
    }

    int sim::insertObjectIntoOctree(int octreeHandle, int objectHandle, int options, std::optional<std::vector<float>> color, std::optional<int> tag)
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
        return _ret[0].as<int>();
    }

    int sim::insertObjectIntoPointCloud(int pointCloudHandle, int objectHandle, int options, float gridSize, std::optional<std::vector<float>> color, std::optional<float> duplicateTolerance)
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
        return _ret[0].as<int>();
    }

    int sim::insertPointsIntoPointCloud(int pointCloudHandle, int options, std::vector<float> points, std::optional<std::vector<float>> color, std::optional<float> duplicateTolerance)
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
        return _ret[0].as<int>();
    }

    int sim::insertVoxelsIntoOctree(int octreeHandle, int options, std::vector<float> points, std::optional<std::vector<float>> color, std::optional<std::vector<int>> tag)
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
        return _ret[0].as<int>();
    }

    std::vector<float> sim::interpolateMatrices(std::vector<float> matrixIn1, std::vector<float> matrixIn2, float interpolFactor)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(matrixIn1);
        _args.push_back(matrixIn2);
        _args.push_back(interpolFactor);
        auto _ret = this->_client->call("sim.interpolateMatrices", _args);
        return _ret[0].as<std::vector<float>>();
    }

    int sim::intersectPointsWithPointCloud(int pointCloudHandle, int options, std::vector<float> points, float tolerance)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(pointCloudHandle);
        _args.push_back(options);
        _args.push_back(points);
        _args.push_back(tolerance);
        auto _ret = this->_client->call("sim.intersectPointsWithPointCloud", _args);
        return _ret[0].as<int>();
    }

    void sim::invertMatrix(std::vector<float> matrix)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(matrix);
        auto _ret = this->_client->call("sim.invertMatrix", _args);
    }

    int sim::isDeprecated(std::string funcOrConst)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(funcOrConst);
        auto _ret = this->_client->call("sim.isDeprecated", _args);
        return _ret[0].as<int>();
    }

    bool sim::isDynamicallyEnabled(int objectHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(objectHandle);
        auto _ret = this->_client->call("sim.isDynamicallyEnabled", _args);
        return _ret[0].as<bool>();
    }

    bool sim::isHandle(int objectHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(objectHandle);
        auto _ret = this->_client->call("sim.isHandle", _args);
        return _ret[0].as<bool>();
    }

    void sim::launchExecutable(std::string filename, std::optional<std::string> parameters, std::optional<int> showStatus)
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

    std::tuple<std::vector<uint8_t>, std::vector<int>> sim::loadImage(int options, std::string filename)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(options);
        _args.push_back(filename);
        auto _ret = this->_client->call("sim.loadImage", _args);
        return std::make_tuple(_ret[0].as<std::vector<uint8_t>>(), _ret[1].as<std::vector<int>>());
    }

    int sim::loadModel(std::string filename)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(filename);
        auto _ret = this->_client->call("sim.loadModel", _args);
        return _ret[0].as<int>();
    }

    int sim::loadModule(std::string filenameAndPath, std::string pluginName)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(filenameAndPath);
        _args.push_back(pluginName);
        auto _ret = this->_client->call("sim.loadModule", _args);
        return _ret[0].as<int>();
    }

    void sim::loadScene(std::string filename)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(filename);
        auto _ret = this->_client->call("sim.loadScene", _args);
    }

    int sim::moduleEntry(int handle, std::optional<std::string> label, std::optional<int> state)
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
        return _ret[0].as<int>();
    }

    std::tuple<std::vector<float>, std::vector<float>, std::vector<float>, float> sim::moveToConfig(int flags, std::vector<float> currentPos, std::vector<float> currentVel, std::vector<float> currentAccel, std::vector<float> maxVel, std::vector<float> maxAccel, std::vector<float> maxJerk, std::vector<float> targetPos, std::vector<float> targetVel, std::string callback, std::optional<json> auxData, std::optional<std::vector<bool>> cyclicJoints, std::optional<float> timeStep)
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
        return std::make_tuple(_ret[0].as<std::vector<float>>(), _ret[1].as<std::vector<float>>(), _ret[2].as<std::vector<float>>(), _ret[3].as<float>());
    }

    std::tuple<std::vector<float>, float> sim::moveToPose(int flags, std::vector<float> currentPose, std::vector<float> maxVel, std::vector<float> maxAccel, std::vector<float> maxJerk, std::vector<float> targetPose, std::string callback, std::optional<json> auxData, std::optional<std::vector<float>> metric, std::optional<float> timeStep)
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
        return std::make_tuple(_ret[0].as<std::vector<float>>(), _ret[1].as<float>());
    }

    std::vector<float> sim::multiplyMatrices(std::vector<float> matrixIn1, std::vector<float> matrixIn2)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(matrixIn1);
        _args.push_back(matrixIn2);
        auto _ret = this->_client->call("sim.multiplyMatrices", _args);
        return _ret[0].as<std::vector<float>>();
    }

    std::vector<float> sim::multiplyVector(std::vector<float> pose, std::vector<float> inVectors)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(pose);
        _args.push_back(inVectors);
        auto _ret = this->_client->call("sim.multiplyVector", _args);
        return _ret[0].as<std::vector<float>>();
    }

    std::vector<uint8_t> sim::packDoubleTable(std::vector<float> doubleNumbers, std::optional<int> startDoubleIndex, std::optional<int> doubleCount)
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

    std::vector<uint8_t> sim::packFloatTable(std::vector<float> floatNumbers, std::optional<int> startFloatIndex, std::optional<int> floatCount)
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

    std::vector<uint8_t> sim::packInt32Table(std::vector<int> int32Numbers, std::optional<int> startInt32Index, std::optional<int> int32Count)
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

    std::vector<uint8_t> sim::packTable(std::vector<json> aTable, std::optional<int> scheme)
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

    std::vector<uint8_t> sim::packUInt16Table(std::vector<int> uint16Numbers, std::optional<int> startUint16Index, std::optional<int> uint16Count)
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

    std::vector<uint8_t> sim::packUInt32Table(std::vector<int> uint32Numbers, std::optional<int> startUInt32Index, std::optional<int> uint32Count)
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

    std::vector<uint8_t> sim::packUInt8Table(std::vector<int> uint8Numbers, std::optional<int> startUint8Index, std::optional<int> uint8count)
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

    int sim::pauseSimulation()
    {
        bool _brk = false;
        json _args(json_array_arg);
        auto _ret = this->_client->call("sim.pauseSimulation", _args);
        return _ret[0].as<int>();
    }

    std::vector<uint8_t> sim::persistentDataRead(std::string dataTag)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(dataTag);
        auto _ret = this->_client->call("sim.persistentDataRead", _args);
        return _ret[0].as<std::vector<uint8_t>>();
    }

    void sim::persistentDataWrite(std::string dataTag, std::vector<uint8_t> dataValue, std::optional<int> options)
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

    void sim::pushUserEvent(std::string event, int handle, int uid, json eventData, std::optional<int> options)
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

    std::vector<uint8_t> sim::readCustomDataBlock(int objectHandle, std::string tagName)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(objectHandle);
        _args.push_back(tagName);
        auto _ret = this->_client->call("sim.readCustomDataBlock", _args);
        return _ret[0].as<std::vector<uint8_t>>();
    }

    std::vector<std::string> sim::readCustomDataBlockTags(int objectHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(objectHandle);
        auto _ret = this->_client->call("sim.readCustomDataBlockTags", _args);
        return _ret[0].as<std::vector<std::string>>();
    }

    json sim::readCustomTableData(int objectHandle, std::string tagName)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(objectHandle);
        _args.push_back(tagName);
        auto _ret = this->_client->call("sim.readCustomTableData", _args);
        return _ret[0].as<json>();
    }

    std::tuple<int, std::vector<float>, std::vector<float>> sim::readForceSensor(int objectHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(objectHandle);
        auto _ret = this->_client->call("sim.readForceSensor", _args);
        return std::make_tuple(_ret[0].as<int>(), _ret[1].as<std::vector<float>>(), _ret[2].as<std::vector<float>>());
    }

    std::tuple<int, float, std::vector<float>, int, std::vector<float>> sim::readProximitySensor(int sensorHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(sensorHandle);
        auto _ret = this->_client->call("sim.readProximitySensor", _args);
        return std::make_tuple(_ret[0].as<int>(), _ret[1].as<float>(), _ret[2].as<std::vector<float>>(), _ret[3].as<int>(), _ret[4].as<std::vector<float>>());
    }

    std::vector<uint8_t> sim::readTexture(int textureId, int options, std::optional<int> posX, std::optional<int> posY, std::optional<int> sizeX, std::optional<int> sizeY)
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

    std::tuple<int, std::vector<float>> sim::readVisionSensor(int sensorHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(sensorHandle);
        auto _ret = this->_client->call("sim.readVisionSensor", _args);
        return std::make_tuple(_ret[0].as<int>(), _ret[1].as<std::vector<float>>());
    }

    int sim::refreshDialogs(int refreshDegree)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(refreshDegree);
        auto _ret = this->_client->call("sim.refreshDialogs", _args);
        return _ret[0].as<int>();
    }

    int sim::registerScriptFuncHook(std::string funcToHook, std::string userFunc, bool execBefore)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(funcToHook);
        _args.push_back(userFunc);
        _args.push_back(execBefore);
        auto _ret = this->_client->call("sim.registerScriptFuncHook", _args);
        return _ret[0].as<int>();
    }

    int sim::registerScriptFunction(std::string funcNameAtPluginName, std::string callTips)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(funcNameAtPluginName);
        _args.push_back(callTips);
        auto _ret = this->_client->call("sim.registerScriptFunction", _args);
        return _ret[0].as<int>();
    }

    int sim::registerScriptVariable(std::string varNameAtPluginName)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(varNameAtPluginName);
        auto _ret = this->_client->call("sim.registerScriptVariable", _args);
        return _ret[0].as<int>();
    }

    void sim::removeDrawingObject(int drawingObjectHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(drawingObjectHandle);
        auto _ret = this->_client->call("sim.removeDrawingObject", _args);
    }

    int sim::removeModel(int objectHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(objectHandle);
        auto _ret = this->_client->call("sim.removeModel", _args);
        return _ret[0].as<int>();
    }

    void sim::removeObjects(std::vector<int> objectHandles)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(objectHandles);
        auto _ret = this->_client->call("sim.removeObjects", _args);
    }

    void sim::removeParticleObject(int particleObjectHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(particleObjectHandle);
        auto _ret = this->_client->call("sim.removeParticleObject", _args);
    }

    int sim::removePointsFromPointCloud(int pointCloudHandle, int options, std::vector<float> points, float tolerance)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(pointCloudHandle);
        _args.push_back(options);
        _args.push_back(points);
        _args.push_back(tolerance);
        auto _ret = this->_client->call("sim.removePointsFromPointCloud", _args);
        return _ret[0].as<int>();
    }

    void sim::removeScript(int scriptHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(scriptHandle);
        auto _ret = this->_client->call("sim.removeScript", _args);
    }

    int sim::removeVoxelsFromOctree(int octreeHandle, int options, std::vector<float> points)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(octreeHandle);
        _args.push_back(options);
        _args.push_back(points);
        auto _ret = this->_client->call("sim.removeVoxelsFromOctree", _args);
        return _ret[0].as<int>();
    }

    int sim::reorientShapeBoundingBox(int shapeHandle, int relativeToHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(shapeHandle);
        _args.push_back(relativeToHandle);
        auto _ret = this->_client->call("sim.reorientShapeBoundingBox", _args);
        return _ret[0].as<int>();
    }

    std::vector<float> sim::resamplePath(std::vector<float> path, std::vector<float> pathLengths, int finalConfigCnt, std::optional<json> method, std::optional<std::vector<int>> types)
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
        return _ret[0].as<std::vector<float>>();
    }

    void sim::resetDynamicObject(int objectHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(objectHandle);
        auto _ret = this->_client->call("sim.resetDynamicObject", _args);
    }

    void sim::resetGraph(int objectHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(objectHandle);
        auto _ret = this->_client->call("sim.resetGraph", _args);
    }

    void sim::resetProximitySensor(int objectHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(objectHandle);
        auto _ret = this->_client->call("sim.resetProximitySensor", _args);
    }

    void sim::resetVisionSensor(int sensorHandle)
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

    int sim::rmlPos(int dofs, float smallestTimeStep, int flags, std::vector<float> currentPosVelAccel, std::vector<float> maxVelAccelJerk, std::vector<int> selection, std::vector<float> targetPosVel)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(dofs);
        _args.push_back(smallestTimeStep);
        _args.push_back(flags);
        _args.push_back(currentPosVelAccel);
        _args.push_back(maxVelAccelJerk);
        _args.push_back(selection);
        _args.push_back(targetPosVel);
        auto _ret = this->_client->call("sim.rmlPos", _args);
        return _ret[0].as<int>();
    }

    void sim::rmlRemove(int handle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(handle);
        auto _ret = this->_client->call("sim.rmlRemove", _args);
    }

    std::tuple<int, std::vector<float>, float> sim::rmlStep(int handle, float timeStep)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(handle);
        _args.push_back(timeStep);
        auto _ret = this->_client->call("sim.rmlStep", _args);
        return std::make_tuple(_ret[0].as<int>(), _ret[1].as<std::vector<float>>(), _ret[2].as<float>());
    }

    int sim::rmlVel(int dofs, float smallestTimeStep, int flags, std::vector<float> currentPosVelAccel, std::vector<float> maxAccelJerk, std::vector<int> selection, std::vector<float> targetVel)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(dofs);
        _args.push_back(smallestTimeStep);
        _args.push_back(flags);
        _args.push_back(currentPosVelAccel);
        _args.push_back(maxAccelJerk);
        _args.push_back(selection);
        _args.push_back(targetVel);
        auto _ret = this->_client->call("sim.rmlVel", _args);
        return _ret[0].as<int>();
    }

    std::vector<float> sim::rotateAroundAxis(std::vector<float> matrixIn, std::vector<float> axis, std::vector<float> axisPos, float angle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(matrixIn);
        _args.push_back(axis);
        _args.push_back(axisPos);
        _args.push_back(angle);
        auto _ret = this->_client->call("sim.rotateAroundAxis", _args);
        return _ret[0].as<std::vector<float>>();
    }

    int sim::ruckigPos(int dofs, float smallestTimeStep, int flags, std::vector<float> currentPosVelAccel, std::vector<float> maxVelAccelJerk, std::vector<int> selection, std::vector<float> targetPosVel)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(dofs);
        _args.push_back(smallestTimeStep);
        _args.push_back(flags);
        _args.push_back(currentPosVelAccel);
        _args.push_back(maxVelAccelJerk);
        _args.push_back(selection);
        _args.push_back(targetPosVel);
        auto _ret = this->_client->call("sim.ruckigPos", _args);
        return _ret[0].as<int>();
    }

    void sim::ruckigRemove(int handle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(handle);
        auto _ret = this->_client->call("sim.ruckigRemove", _args);
    }

    std::tuple<int, std::vector<float>, float> sim::ruckigStep(int handle, float timeStep)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(handle);
        _args.push_back(timeStep);
        auto _ret = this->_client->call("sim.ruckigStep", _args);
        return std::make_tuple(_ret[0].as<int>(), _ret[1].as<std::vector<float>>(), _ret[2].as<float>());
    }

    int sim::ruckigVel(int dofs, float smallestTimeStep, int flags, std::vector<float> currentPosVelAccel, std::vector<float> maxAccelJerk, std::vector<int> selection, std::vector<float> targetVel)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(dofs);
        _args.push_back(smallestTimeStep);
        _args.push_back(flags);
        _args.push_back(currentPosVelAccel);
        _args.push_back(maxAccelJerk);
        _args.push_back(selection);
        _args.push_back(targetVel);
        auto _ret = this->_client->call("sim.ruckigVel", _args);
        return _ret[0].as<int>();
    }

    std::vector<uint8_t> sim::saveImage(std::vector<uint8_t> image, std::vector<int> resolution, int options, std::string filename, int quality)
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

    void sim::saveModel(int modelBaseHandle, std::string filename)
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

    void sim::scaleObject(int objectHandle, float xScale, float yScale, float zScale, std::optional<int> options)
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

    void sim::scaleObjects(std::vector<int> objectHandles, float scalingFactor, bool scalePositionsToo)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(objectHandles);
        _args.push_back(scalingFactor);
        _args.push_back(scalePositionsToo);
        auto _ret = this->_client->call("sim.scaleObjects", _args);
    }

    int sim::serialCheck(int portHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(portHandle);
        auto _ret = this->_client->call("sim.serialCheck", _args);
        return _ret[0].as<int>();
    }

    void sim::serialClose(int portHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(portHandle);
        auto _ret = this->_client->call("sim.serialClose", _args);
    }

    int sim::serialOpen(std::string portString, int baudrate)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(portString);
        _args.push_back(baudrate);
        auto _ret = this->_client->call("sim.serialOpen", _args);
        return _ret[0].as<int>();
    }

    std::vector<uint8_t> sim::serialRead(int portHandle, int dataLengthToRead, bool blockingOperation, std::optional<std::vector<uint8_t>> closingString, std::optional<float> timeout)
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

    int sim::serialSend(int portHandle, std::vector<uint8_t> data)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(portHandle);
        _args.push_back(bin(data));
        auto _ret = this->_client->call("sim.serialSend", _args);
        return _ret[0].as<int>();
    }

    void sim::setArrayParam(int parameter, std::vector<float> arrayOfValues)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(parameter);
        _args.push_back(arrayOfValues);
        auto _ret = this->_client->call("sim.setArrayParam", _args);
    }

    void sim::setBoolParam(int parameter, bool boolState)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(parameter);
        _args.push_back(boolState);
        auto _ret = this->_client->call("sim.setBoolParam", _args);
    }

    void sim::setDoubleSignal(std::string signalName, float signalValue)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(signalName);
        _args.push_back(signalValue);
        auto _ret = this->_client->call("sim.setDoubleSignal", _args);
    }

    void sim::setEngineBoolParam(int paramId, int objectHandle, bool boolParam)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(paramId);
        _args.push_back(objectHandle);
        _args.push_back(boolParam);
        auto _ret = this->_client->call("sim.setEngineBoolParam", _args);
    }

    void sim::setEngineFloatParam(int paramId, int objectHandle, float floatParam)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(paramId);
        _args.push_back(objectHandle);
        _args.push_back(floatParam);
        auto _ret = this->_client->call("sim.setEngineFloatParam", _args);
    }

    void sim::setEngineInt32Param(int paramId, int objectHandle, int int32Param)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(paramId);
        _args.push_back(objectHandle);
        _args.push_back(int32Param);
        auto _ret = this->_client->call("sim.setEngineInt32Param", _args);
    }

    void sim::setExplicitHandling(int objectHandle, int explicitHandlingFlags)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(objectHandle);
        _args.push_back(explicitHandlingFlags);
        auto _ret = this->_client->call("sim.setExplicitHandling", _args);
    }

    void sim::setFloatParam(int parameter, float floatState)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(parameter);
        _args.push_back(floatState);
        auto _ret = this->_client->call("sim.setFloatParam", _args);
    }

    void sim::setFloatSignal(std::string signalName, float signalValue)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(signalName);
        _args.push_back(signalValue);
        auto _ret = this->_client->call("sim.setFloatSignal", _args);
    }

    void sim::setGraphStreamTransformation(int graphHandle, int streamId, int trType, std::optional<float> mult, std::optional<float> off, std::optional<int> movAvgPeriod)
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

    void sim::setGraphStreamValue(int graphHandle, int streamId, float value)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(graphHandle);
        _args.push_back(streamId);
        _args.push_back(value);
        auto _ret = this->_client->call("sim.setGraphStreamValue", _args);
    }

    void sim::setInt32Param(int parameter, int intState)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(parameter);
        _args.push_back(intState);
        auto _ret = this->_client->call("sim.setInt32Param", _args);
    }

    void sim::setInt32Signal(std::string signalName, int signalValue)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(signalName);
        _args.push_back(signalValue);
        auto _ret = this->_client->call("sim.setInt32Signal", _args);
    }

    void sim::setJointDependency(int jointHandle, int masterJointHandle, float offset, float multCoeff)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(jointHandle);
        _args.push_back(masterJointHandle);
        _args.push_back(offset);
        _args.push_back(multCoeff);
        auto _ret = this->_client->call("sim.setJointDependency", _args);
    }

    void sim::setJointInterval(int objectHandle, bool cyclic, std::vector<float> interval)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(objectHandle);
        _args.push_back(cyclic);
        _args.push_back(interval);
        auto _ret = this->_client->call("sim.setJointInterval", _args);
    }

    void sim::setJointMode(int jointHandle, int jointMode, int options)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(jointHandle);
        _args.push_back(jointMode);
        _args.push_back(options);
        auto _ret = this->_client->call("sim.setJointMode", _args);
    }

    void sim::setJointPosition(int objectHandle, float position)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(objectHandle);
        _args.push_back(position);
        auto _ret = this->_client->call("sim.setJointPosition", _args);
    }

    void sim::setJointTargetForce(int objectHandle, float forceOrTorque, std::optional<bool> signedValue)
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

    void sim::setJointTargetPosition(int objectHandle, float targetPosition)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(objectHandle);
        _args.push_back(targetPosition);
        auto _ret = this->_client->call("sim.setJointTargetPosition", _args);
    }

    void sim::setJointTargetVelocity(int objectHandle, float targetVelocity)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(objectHandle);
        _args.push_back(targetVelocity);
        auto _ret = this->_client->call("sim.setJointTargetVelocity", _args);
    }

    void sim::setLightParameters(int lightHandle, int state, std::vector<float> reserved, std::vector<float> diffusePart, std::vector<float> specularPart)
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

    void sim::setLinkDummy(int dummyHandle, int linkDummyHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(dummyHandle);
        _args.push_back(linkDummyHandle);
        auto _ret = this->_client->call("sim.setLinkDummy", _args);
    }

    void sim::setModelProperty(int objectHandle, int property)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(objectHandle);
        _args.push_back(property);
        auto _ret = this->_client->call("sim.setModelProperty", _args);
    }

    void sim::setModuleInfo(std::string moduleName, int infoType, std::string info)
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

    void sim::setNavigationMode(int navigationMode)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(navigationMode);
        auto _ret = this->_client->call("sim.setNavigationMode", _args);
    }

    void sim::setObjectAlias(int objectHandle, std::string objectAlias)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(objectHandle);
        _args.push_back(objectAlias);
        auto _ret = this->_client->call("sim.setObjectAlias", _args);
    }

    void sim::setObjectChildPose(int objectHandle, std::vector<float> pose)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(objectHandle);
        _args.push_back(pose);
        auto _ret = this->_client->call("sim.setObjectChildPose", _args);
    }

    bool sim::setObjectColor(int objectHandle, int index, int colorComponent, std::vector<float> rgbData)
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

    void sim::setObjectFloatParam(int objectHandle, int parameterID, float parameter)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(objectHandle);
        _args.push_back(parameterID);
        _args.push_back(parameter);
        auto _ret = this->_client->call("sim.setObjectFloatParam", _args);
    }

    void sim::setObjectInt32Param(int objectHandle, int parameterID, int parameter)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(objectHandle);
        _args.push_back(parameterID);
        _args.push_back(parameter);
        auto _ret = this->_client->call("sim.setObjectInt32Param", _args);
    }

    void sim::setObjectMatrix(int objectHandle, int relativeToObjectHandle, std::vector<float> matrix)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(objectHandle);
        _args.push_back(relativeToObjectHandle);
        _args.push_back(matrix);
        auto _ret = this->_client->call("sim.setObjectMatrix", _args);
    }

    void sim::setObjectOrientation(int objectHandle, int relativeToObjectHandle, std::vector<float> eulerAngles)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(objectHandle);
        _args.push_back(relativeToObjectHandle);
        _args.push_back(eulerAngles);
        auto _ret = this->_client->call("sim.setObjectOrientation", _args);
    }

    void sim::setObjectParent(int objectHandle, int parentObjectHandle, bool keepInPlace)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(objectHandle);
        _args.push_back(parentObjectHandle);
        _args.push_back(keepInPlace);
        auto _ret = this->_client->call("sim.setObjectParent", _args);
    }

    void sim::setObjectPose(int objectHandle, int relativeToObjectHandle, std::vector<float> pose)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(objectHandle);
        _args.push_back(relativeToObjectHandle);
        _args.push_back(pose);
        auto _ret = this->_client->call("sim.setObjectPose", _args);
    }

    void sim::setObjectPosition(int objectHandle, int relativeToObjectHandle, std::vector<float> position)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(objectHandle);
        _args.push_back(relativeToObjectHandle);
        _args.push_back(position);
        auto _ret = this->_client->call("sim.setObjectPosition", _args);
    }

    void sim::setObjectProperty(int objectHandle, int property)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(objectHandle);
        _args.push_back(property);
        auto _ret = this->_client->call("sim.setObjectProperty", _args);
    }

    void sim::setObjectQuaternion(int objectHandle, int relativeToObjectHandle, std::vector<float> quaternion)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(objectHandle);
        _args.push_back(relativeToObjectHandle);
        _args.push_back(quaternion);
        auto _ret = this->_client->call("sim.setObjectQuaternion", _args);
    }

    void sim::setObjectSelection(std::vector<float> objectHandles)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(objectHandles);
        auto _ret = this->_client->call("sim.setObjectSelection", _args);
    }

    void sim::setObjectSpecialProperty(int objectHandle, int property)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(objectHandle);
        _args.push_back(property);
        auto _ret = this->_client->call("sim.setObjectSpecialProperty", _args);
    }

    void sim::setObjectStringParam(int objectHandle, int parameterID, std::vector<uint8_t> parameter)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(objectHandle);
        _args.push_back(parameterID);
        _args.push_back(bin(parameter));
        auto _ret = this->_client->call("sim.setObjectStringParam", _args);
    }

    void sim::setPage(int pageIndex)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(pageIndex);
        auto _ret = this->_client->call("sim.setPage", _args);
    }

    void sim::setPointCloudOptions(int pointCloudHandle, float maxVoxelSize, int maxPtCntPerVoxel, int options, float pointSize)
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

    void sim::setReferencedHandles(int objectHandle, std::vector<int> referencedHandles)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(objectHandle);
        _args.push_back(referencedHandles);
        auto _ret = this->_client->call("sim.setReferencedHandles", _args);
    }

    void sim::setScriptInt32Param(int scriptHandle, int parameterID, int parameter)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(scriptHandle);
        _args.push_back(parameterID);
        _args.push_back(parameter);
        auto _ret = this->_client->call("sim.setScriptInt32Param", _args);
    }

    void sim::setScriptStringParam(int scriptHandle, int parameterID, std::vector<uint8_t> parameter)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(scriptHandle);
        _args.push_back(parameterID);
        _args.push_back(bin(parameter));
        auto _ret = this->_client->call("sim.setScriptStringParam", _args);
    }

    void sim::setShapeBB(int shapeHandle, std::vector<float> size)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(shapeHandle);
        _args.push_back(size);
        auto _ret = this->_client->call("sim.setShapeBB", _args);
    }

    void sim::setShapeColor(int shapeHandle, std::string colorName, int colorComponent, std::vector<float> rgbData)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(shapeHandle);
        _args.push_back(colorName);
        _args.push_back(colorComponent);
        _args.push_back(rgbData);
        auto _ret = this->_client->call("sim.setShapeColor", _args);
    }

    void sim::setShapeInertia(int shapeHandle, std::vector<float> inertiaMatrix, std::vector<float> transformationMatrix)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(shapeHandle);
        _args.push_back(inertiaMatrix);
        _args.push_back(transformationMatrix);
        auto _ret = this->_client->call("sim.setShapeInertia", _args);
    }

    void sim::setShapeMass(int shapeHandle, float mass)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(shapeHandle);
        _args.push_back(mass);
        auto _ret = this->_client->call("sim.setShapeMass", _args);
    }

    void sim::setShapeMaterial(int shapeHandle, int materialIdOrShapeHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(shapeHandle);
        _args.push_back(materialIdOrShapeHandle);
        auto _ret = this->_client->call("sim.setShapeMaterial", _args);
    }

    void sim::setShapeTexture(int shapeHandle, int textureId, int mappingMode, int options, std::vector<float> uvScaling, std::optional<std::vector<float>> position, std::optional<std::vector<float>> orientation)
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

    void sim::setStringParam(int parameter, std::string stringState)
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

    int sim::setThreadAutomaticSwitch(bool automaticSwitch)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(automaticSwitch);
        auto _ret = this->_client->call("sim.setThreadAutomaticSwitch", _args);
        return _ret[0].as<int>();
    }

    int sim::setThreadSwitchAllowed(bool allowed)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(allowed);
        auto _ret = this->_client->call("sim.setThreadSwitchAllowed", _args);
        return _ret[0].as<int>();
    }

    void sim::setThreadSwitchTiming(int dtInMs)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(dtInMs);
        auto _ret = this->_client->call("sim.setThreadSwitchTiming", _args);
    }

    int sim::setVisionSensorCharImage(int sensorHandle, std::vector<uint8_t> imageBuffer)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(sensorHandle);
        _args.push_back(bin(imageBuffer));
        auto _ret = this->_client->call("sim.setVisionSensorCharImage", _args);
        return _ret[0].as<int>();
    }

    int sim::setVisionSensorImage(int sensorHandle, std::vector<uint8_t> image)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(sensorHandle);
        _args.push_back(bin(image));
        auto _ret = this->_client->call("sim.setVisionSensorImage", _args);
        return _ret[0].as<int>();
    }

    int sim::startSimulation()
    {
        bool _brk = false;
        json _args(json_array_arg);
        auto _ret = this->_client->call("sim.startSimulation", _args);
        return _ret[0].as<int>();
    }

    int sim::stopSimulation()
    {
        bool _brk = false;
        json _args(json_array_arg);
        auto _ret = this->_client->call("sim.stopSimulation", _args);
        return _ret[0].as<int>();
    }

    int sim::subtractObjectFromOctree(int octreeHandle, int objectHandle, int options)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(octreeHandle);
        _args.push_back(objectHandle);
        _args.push_back(options);
        auto _ret = this->_client->call("sim.subtractObjectFromOctree", _args);
        return _ret[0].as<int>();
    }

    int sim::subtractObjectFromPointCloud(int pointCloudHandle, int objectHandle, int options, float tolerance)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(pointCloudHandle);
        _args.push_back(objectHandle);
        _args.push_back(options);
        _args.push_back(tolerance);
        auto _ret = this->_client->call("sim.subtractObjectFromPointCloud", _args);
        return _ret[0].as<int>();
    }

    void sim::switchThread()
    {
        bool _brk = false;
        json _args(json_array_arg);
        auto _ret = this->_client->call("sim.switchThread", _args);
    }

    std::tuple<std::string, std::vector<int>, std::vector<int>> sim::textEditorClose(int handle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(handle);
        auto _ret = this->_client->call("sim.textEditorClose", _args);
        return std::make_tuple(_ret[0].as<std::string>(), _ret[1].as<std::vector<int>>(), _ret[2].as<std::vector<int>>());
    }

    std::tuple<std::string, std::vector<int>, std::vector<int>, bool> sim::textEditorGetInfo(int handle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(handle);
        auto _ret = this->_client->call("sim.textEditorGetInfo", _args);
        return std::make_tuple(_ret[0].as<std::string>(), _ret[1].as<std::vector<int>>(), _ret[2].as<std::vector<int>>(), _ret[3].as<bool>());
    }

    int sim::textEditorOpen(std::string initText, std::string properties)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(initText);
        _args.push_back(properties);
        auto _ret = this->_client->call("sim.textEditorOpen", _args);
        return _ret[0].as<int>();
    }

    void sim::textEditorShow(int handle, bool showState)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(handle);
        _args.push_back(showState);
        auto _ret = this->_client->call("sim.textEditorShow", _args);
    }

    std::vector<uint8_t> sim::transformBuffer(std::vector<uint8_t> inBuffer, int inFormat, float multiplier, float offset, int outFormat)
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

    void sim::transformImage(std::vector<uint8_t> image, std::vector<int> resolution, int options)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(bin(image));
        _args.push_back(resolution);
        _args.push_back(options);
        auto _ret = this->_client->call("sim.transformImage", _args);
    }

    std::vector<int> sim::ungroupShape(int shapeHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(shapeHandle);
        auto _ret = this->_client->call("sim.ungroupShape", _args);
        return _ret[0].as<std::vector<int>>();
    }

    int sim::unloadModule(int pluginHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(pluginHandle);
        auto _ret = this->_client->call("sim.unloadModule", _args);
        return _ret[0].as<int>();
    }

    std::vector<float> sim::unpackDoubleTable(std::vector<uint8_t> data, std::optional<int> startDoubleIndex, std::optional<int> doubleCount, std::optional<int> additionalByteOffset)
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
        return _ret[0].as<std::vector<float>>();
    }

    std::vector<float> sim::unpackFloatTable(std::vector<uint8_t> data, std::optional<int> startFloatIndex, std::optional<int> floatCount, std::optional<int> additionalByteOffset)
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
        return _ret[0].as<std::vector<float>>();
    }

    std::vector<int> sim::unpackInt32Table(std::vector<uint8_t> data, std::optional<int> startInt32Index, std::optional<int> int32Count, std::optional<int> additionalByteOffset)
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
        return _ret[0].as<std::vector<int>>();
    }

    json sim::unpackTable(std::vector<uint8_t> buffer)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(bin(buffer));
        auto _ret = this->_client->call("sim.unpackTable", _args);
        return _ret[0].as<json>();
    }

    std::vector<int> sim::unpackUInt16Table(std::vector<uint8_t> data, std::optional<int> startUint16Index, std::optional<int> uint16Count, std::optional<int> additionalByteOffset)
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
        return _ret[0].as<std::vector<int>>();
    }

    std::vector<int> sim::unpackUInt32Table(std::vector<uint8_t> data, std::optional<int> startUint32Index, std::optional<int> uint32Count, std::optional<int> additionalByteOffset)
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
        return _ret[0].as<std::vector<int>>();
    }

    std::vector<int> sim::unpackUInt8Table(std::vector<uint8_t> data, std::optional<int> startUint8Index, std::optional<int> uint8count)
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
        return _ret[0].as<std::vector<int>>();
    }

    float sim::wait(float dt, std::optional<bool> simulationTime)
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
        return _ret[0].as<float>();
    }

    json sim::waitForSignal(std::string sigName)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(sigName);
        auto _ret = this->_client->call("sim.waitForSignal", _args);
        return _ret[0].as<json>();
    }

    void sim::writeCustomDataBlock(int objectHandle, std::string tagName, std::vector<uint8_t> data)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(objectHandle);
        _args.push_back(tagName);
        _args.push_back(bin(data));
        auto _ret = this->_client->call("sim.writeCustomDataBlock", _args);
    }

    void sim::writeCustomTableData(int objectHandle, std::string tagName, std::vector<json> data)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(objectHandle);
        _args.push_back(tagName);
        _args.push_back(data);
        auto _ret = this->_client->call("sim.writeCustomTableData", _args);
    }

    void sim::writeTexture(int textureId, int options, std::vector<uint8_t> textureData, std::optional<int> posX, std::optional<int> posY, std::optional<int> sizeX, std::optional<int> sizeY, std::optional<float> interpol)
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

    std::tuple<float, float, float> sim::yawPitchRollToAlphaBetaGamma(float yawAngle, float pitchAngle, float rollAngle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(yawAngle);
        _args.push_back(pitchAngle);
        _args.push_back(rollAngle);
        auto _ret = this->_client->call("sim.yawPitchRollToAlphaBetaGamma", _args);
        return std::make_tuple(_ret[0].as<float>(), _ret[1].as<float>(), _ret[2].as<float>());
    }


    simIK::simIK(RemoteAPIClient *client)
        : _client(client)
    {
    }

    int simIK::addIkElement(int environmentHandle, int ikGroupHandle, int tipDummyHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(environmentHandle);
        _args.push_back(ikGroupHandle);
        _args.push_back(tipDummyHandle);
        auto _ret = this->_client->call("simIK.addIkElement", _args);
        return _ret[0].as<int>();
    }

    std::tuple<int, json> simIK::addIkElementFromScene(int environmentHandle, int ikGroup, int baseHandle, int tipHandle, int targetHandle, int constraints)
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
        return std::make_tuple(_ret[0].as<int>(), _ret[1].as<json>());
    }

    int simIK::applyIkEnvironmentToScene(int environmentHandle, int ikGroup, std::optional<bool> applyOnlyWhenSuccessful)
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
        return _ret[0].as<int>();
    }

    void simIK::applySceneToIkEnvironment(int environmentHandle, int ikGroup)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(environmentHandle);
        _args.push_back(ikGroup);
        auto _ret = this->_client->call("simIK.applySceneToIkEnvironment", _args);
    }

    bool simIK::computeJacobian(int environmentHandle, int ikGroupHandle, int options)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(environmentHandle);
        _args.push_back(ikGroupHandle);
        _args.push_back(options);
        auto _ret = this->_client->call("simIK.computeJacobian", _args);
        return _ret[0].as<bool>();
    }

    int simIK::createDummy(int environmentHandle, std::optional<std::string> dummyName)
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
        return _ret[0].as<int>();
    }

    int simIK::createEnvironment()
    {
        bool _brk = false;
        json _args(json_array_arg);
        auto _ret = this->_client->call("simIK.createEnvironment", _args);
        return _ret[0].as<int>();
    }

    int simIK::createIkGroup(int environmentHandle, std::optional<std::string> ikGroupName)
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
        return _ret[0].as<int>();
    }

    int simIK::createJoint(int environmentHandle, int jointType, std::optional<std::string> jointName)
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
        return _ret[0].as<int>();
    }

    bool simIK::doesIkGroupExist(int environmentHandle, std::string ikGroupName)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(environmentHandle);
        _args.push_back(ikGroupName);
        auto _ret = this->_client->call("simIK.doesIkGroupExist", _args);
        return _ret[0].as<bool>();
    }

    bool simIK::doesObjectExist(int environmentHandle, std::string objectName)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(environmentHandle);
        _args.push_back(objectName);
        auto _ret = this->_client->call("simIK.doesObjectExist", _args);
        return _ret[0].as<bool>();
    }

    int simIK::duplicateEnvironment(int environmentHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(environmentHandle);
        auto _ret = this->_client->call("simIK.duplicateEnvironment", _args);
        return _ret[0].as<int>();
    }

    void simIK::eraseEnvironment(int environmentHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(environmentHandle);
        auto _ret = this->_client->call("simIK.eraseEnvironment", _args);
    }

    void simIK::eraseObject(int environmentHandle, int objectHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(environmentHandle);
        _args.push_back(objectHandle);
        auto _ret = this->_client->call("simIK.eraseObject", _args);
    }

    std::vector<float> simIK::findConfig(int environmentHandle, int ikGroupHandle, std::vector<int> jointHandles, std::optional<float> thresholdDist, std::optional<float> maxTime, std::optional<std::vector<float>> metric, std::optional<std::string> validationCallback, std::optional<json> auxData)
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
        return _ret[0].as<std::vector<float>>();
    }

    std::vector<float> simIK::generatePath(int environmentHandle, int ikGroupHandle, std::vector<int> jointHandles, int tipHandle, int pathPointCount, std::optional<std::string> validationCallback, std::optional<json> auxData)
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
        return _ret[0].as<std::vector<float>>();
    }

    std::vector<float> simIK::getAlternateConfigs(int environmentHandle, std::vector<int> jointHandles, std::optional<std::vector<float>> lowLimits, std::optional<std::vector<float>> ranges)
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
        return _ret[0].as<std::vector<float>>();
    }

    std::tuple<int, int> simIK::getIkElementBase(int environmentHandle, int ikGroupHandle, int elementHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(environmentHandle);
        _args.push_back(ikGroupHandle);
        _args.push_back(elementHandle);
        auto _ret = this->_client->call("simIK.getIkElementBase", _args);
        return std::make_tuple(_ret[0].as<int>(), _ret[1].as<int>());
    }

    int simIK::getIkElementConstraints(int environmentHandle, int ikGroupHandle, int elementHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(environmentHandle);
        _args.push_back(ikGroupHandle);
        _args.push_back(elementHandle);
        auto _ret = this->_client->call("simIK.getIkElementConstraints", _args);
        return _ret[0].as<int>();
    }

    int simIK::getIkElementFlags(int environmentHandle, int ikGroupHandle, int elementHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(environmentHandle);
        _args.push_back(ikGroupHandle);
        _args.push_back(elementHandle);
        auto _ret = this->_client->call("simIK.getIkElementFlags", _args);
        return _ret[0].as<int>();
    }

    std::vector<float> simIK::getIkElementPrecision(int environmentHandle, int ikGroupHandle, int elementHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(environmentHandle);
        _args.push_back(ikGroupHandle);
        _args.push_back(elementHandle);
        auto _ret = this->_client->call("simIK.getIkElementPrecision", _args);
        return _ret[0].as<std::vector<float>>();
    }

    std::vector<float> simIK::getIkElementWeights(int environmentHandle, int ikGroupHandle, int elementHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(environmentHandle);
        _args.push_back(ikGroupHandle);
        _args.push_back(elementHandle);
        auto _ret = this->_client->call("simIK.getIkElementWeights", _args);
        return _ret[0].as<std::vector<float>>();
    }

    std::tuple<int, float, int> simIK::getIkGroupCalculation(int environmentHandle, int ikGroupHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(environmentHandle);
        _args.push_back(ikGroupHandle);
        auto _ret = this->_client->call("simIK.getIkGroupCalculation", _args);
        return std::make_tuple(_ret[0].as<int>(), _ret[1].as<float>(), _ret[2].as<int>());
    }

    int simIK::getIkGroupFlags(int environmentHandle, int ikGroupHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(environmentHandle);
        _args.push_back(ikGroupHandle);
        auto _ret = this->_client->call("simIK.getIkGroupFlags", _args);
        return _ret[0].as<int>();
    }

    int simIK::getIkGroupHandle(int environmentHandle, std::string ikGroupName)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(environmentHandle);
        _args.push_back(ikGroupName);
        auto _ret = this->_client->call("simIK.getIkGroupHandle", _args);
        return _ret[0].as<int>();
    }

    std::tuple<std::vector<float>, std::vector<int>> simIK::getJacobian(int environmentHandle, int ikGroupHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(environmentHandle);
        _args.push_back(ikGroupHandle);
        auto _ret = this->_client->call("simIK.getJacobian", _args);
        return std::make_tuple(_ret[0].as<std::vector<float>>(), _ret[1].as<std::vector<int>>());
    }

    std::tuple<int, float, float> simIK::getJointDependency(int environmentHandle, int jointHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(environmentHandle);
        _args.push_back(jointHandle);
        auto _ret = this->_client->call("simIK.getJointDependency", _args);
        return std::make_tuple(_ret[0].as<int>(), _ret[1].as<float>(), _ret[2].as<float>());
    }

    float simIK::getJointIkWeight(int environmentHandle, int jointHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(environmentHandle);
        _args.push_back(jointHandle);
        auto _ret = this->_client->call("simIK.getJointIkWeight", _args);
        return _ret[0].as<float>();
    }

    std::tuple<bool, std::vector<float>> simIK::getJointInterval(int environmentHandle, int jointHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(environmentHandle);
        _args.push_back(jointHandle);
        auto _ret = this->_client->call("simIK.getJointInterval", _args);
        return std::make_tuple(_ret[0].as<bool>(), _ret[1].as<std::vector<float>>());
    }

    std::vector<float> simIK::getJointMatrix(int environmentHandle, int jointHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(environmentHandle);
        _args.push_back(jointHandle);
        auto _ret = this->_client->call("simIK.getJointMatrix", _args);
        return _ret[0].as<std::vector<float>>();
    }

    float simIK::getJointMaxStepSize(int environmentHandle, int jointHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(environmentHandle);
        _args.push_back(jointHandle);
        auto _ret = this->_client->call("simIK.getJointMaxStepSize", _args);
        return _ret[0].as<float>();
    }

    int simIK::getJointMode(int environmentHandle, int jointHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(environmentHandle);
        _args.push_back(jointHandle);
        auto _ret = this->_client->call("simIK.getJointMode", _args);
        return _ret[0].as<int>();
    }

    float simIK::getJointPosition(int environmentHandle, int jointHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(environmentHandle);
        _args.push_back(jointHandle);
        auto _ret = this->_client->call("simIK.getJointPosition", _args);
        return _ret[0].as<float>();
    }

    float simIK::getJointScrewPitch(int environmentHandle, int jointHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(environmentHandle);
        _args.push_back(jointHandle);
        auto _ret = this->_client->call("simIK.getJointScrewPitch", _args);
        return _ret[0].as<float>();
    }

    std::tuple<std::vector<float>, std::vector<float>, std::vector<float>> simIK::getJointTransformation(int environmentHandle, int jointHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(environmentHandle);
        _args.push_back(jointHandle);
        auto _ret = this->_client->call("simIK.getJointTransformation", _args);
        return std::make_tuple(_ret[0].as<std::vector<float>>(), _ret[1].as<std::vector<float>>(), _ret[2].as<std::vector<float>>());
    }

    int simIK::getJointType(int environmentHandle, int jointHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(environmentHandle);
        _args.push_back(jointHandle);
        auto _ret = this->_client->call("simIK.getJointType", _args);
        return _ret[0].as<int>();
    }

    int simIK::getLinkedDummy(int environmentHandle, int dummyHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(environmentHandle);
        _args.push_back(dummyHandle);
        auto _ret = this->_client->call("simIK.getLinkedDummy", _args);
        return _ret[0].as<int>();
    }

    float simIK::getManipulability(int environmentHandle, int ikGroupHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(environmentHandle);
        _args.push_back(ikGroupHandle);
        auto _ret = this->_client->call("simIK.getManipulability", _args);
        return _ret[0].as<float>();
    }

    int simIK::getObjectHandle(int environmentHandle, std::string objectName)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(environmentHandle);
        _args.push_back(objectName);
        auto _ret = this->_client->call("simIK.getObjectHandle", _args);
        return _ret[0].as<int>();
    }

    std::vector<float> simIK::getObjectMatrix(int environmentHandle, int objectHandle, int relativeToObjectHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(environmentHandle);
        _args.push_back(objectHandle);
        _args.push_back(relativeToObjectHandle);
        auto _ret = this->_client->call("simIK.getObjectMatrix", _args);
        return _ret[0].as<std::vector<float>>();
    }

    int simIK::getObjectParent(int environmentHandle, int objectHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(environmentHandle);
        _args.push_back(objectHandle);
        auto _ret = this->_client->call("simIK.getObjectParent", _args);
        return _ret[0].as<int>();
    }

    std::vector<float> simIK::getObjectPose(int environmentHandle, int objectHandle, int relativeToObjectHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(environmentHandle);
        _args.push_back(objectHandle);
        _args.push_back(relativeToObjectHandle);
        auto _ret = this->_client->call("simIK.getObjectPose", _args);
        return _ret[0].as<std::vector<float>>();
    }

    std::tuple<std::vector<float>, std::vector<float>, std::vector<float>> simIK::getObjectTransformation(int environmentHandle, int objectHandle, int relativeToObjectHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(environmentHandle);
        _args.push_back(objectHandle);
        _args.push_back(relativeToObjectHandle);
        auto _ret = this->_client->call("simIK.getObjectTransformation", _args);
        return std::make_tuple(_ret[0].as<std::vector<float>>(), _ret[1].as<std::vector<float>>(), _ret[2].as<std::vector<float>>());
    }

    std::tuple<int, std::string, bool, int> simIK::getObjects(int environmentHandle, int index)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(environmentHandle);
        _args.push_back(index);
        auto _ret = this->_client->call("simIK.getObjects", _args);
        return std::make_tuple(_ret[0].as<int>(), _ret[1].as<std::string>(), _ret[2].as<bool>(), _ret[3].as<int>());
    }

    int simIK::handleIkGroup(int environmentHandle, std::optional<int> ikGroupHandle)
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
        return _ret[0].as<int>();
    }

    void simIK::load(int environmentHandle, std::string data)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(environmentHandle);
        _args.push_back(data);
        auto _ret = this->_client->call("simIK.load", _args);
    }

    std::string simIK::save(int environmentHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(environmentHandle);
        auto _ret = this->_client->call("simIK.save", _args);
        return _ret[0].as<std::string>();
    }

    void simIK::setIkElementBase(int environmentHandle, int ikGroupHandle, int elementHandle, int baseHandle, std::optional<int> constraintsBaseHandle)
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

    void simIK::setIkElementConstraints(int environmentHandle, int ikGroupHandle, int elementHandle, int constraints)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(environmentHandle);
        _args.push_back(ikGroupHandle);
        _args.push_back(elementHandle);
        _args.push_back(constraints);
        auto _ret = this->_client->call("simIK.setIkElementConstraints", _args);
    }

    void simIK::setIkElementFlags(int environmentHandle, int ikGroupHandle, int elementHandle, int flags)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(environmentHandle);
        _args.push_back(ikGroupHandle);
        _args.push_back(elementHandle);
        _args.push_back(flags);
        auto _ret = this->_client->call("simIK.setIkElementFlags", _args);
    }

    void simIK::setIkElementPrecision(int environmentHandle, int ikGroupHandle, int elementHandle, std::vector<float> precision)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(environmentHandle);
        _args.push_back(ikGroupHandle);
        _args.push_back(elementHandle);
        _args.push_back(precision);
        auto _ret = this->_client->call("simIK.setIkElementPrecision", _args);
    }

    void simIK::setIkElementWeights(int environmentHandle, int ikGroupHandle, int elementHandle, std::vector<float> weights)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(environmentHandle);
        _args.push_back(ikGroupHandle);
        _args.push_back(elementHandle);
        _args.push_back(weights);
        auto _ret = this->_client->call("simIK.setIkElementWeights", _args);
    }

    void simIK::setIkGroupCalculation(int environmentHandle, int ikGroupHandle, int method, float damping, int maxIterations)
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

    void simIK::setIkGroupFlags(int environmentHandle, int ikGroupHandle, int flags)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(environmentHandle);
        _args.push_back(ikGroupHandle);
        _args.push_back(flags);
        auto _ret = this->_client->call("simIK.setIkGroupFlags", _args);
    }

    void simIK::setJointDependency(int environmentHandle, int jointHandle, int depJointHandle, std::optional<float> offset, std::optional<float> mult)
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

    void simIK::setJointIkWeight(int environmentHandle, int jointHandle, float weight)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(environmentHandle);
        _args.push_back(jointHandle);
        _args.push_back(weight);
        auto _ret = this->_client->call("simIK.setJointIkWeight", _args);
    }

    void simIK::setJointInterval(int environmentHandle, int jointHandle, bool cyclic, std::optional<std::vector<float>> interval)
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

    void simIK::setJointMaxStepSize(int environmentHandle, int jointHandle, float stepSize)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(environmentHandle);
        _args.push_back(jointHandle);
        _args.push_back(stepSize);
        auto _ret = this->_client->call("simIK.setJointMaxStepSize", _args);
    }

    void simIK::setJointMode(int environmentHandle, int jointHandle, int jointMode)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(environmentHandle);
        _args.push_back(jointHandle);
        _args.push_back(jointMode);
        auto _ret = this->_client->call("simIK.setJointMode", _args);
    }

    void simIK::setJointPosition(int environmentHandle, int jointHandle, float position)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(environmentHandle);
        _args.push_back(jointHandle);
        _args.push_back(position);
        auto _ret = this->_client->call("simIK.setJointPosition", _args);
    }

    void simIK::setJointScrewPitch(int environmentHandle, int jointHandle, float pitch)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(environmentHandle);
        _args.push_back(jointHandle);
        _args.push_back(pitch);
        auto _ret = this->_client->call("simIK.setJointScrewPitch", _args);
    }

    void simIK::setLinkedDummy(int environmentHandle, int dummyHandle, int linkedDummyHandle)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(environmentHandle);
        _args.push_back(dummyHandle);
        _args.push_back(linkedDummyHandle);
        auto _ret = this->_client->call("simIK.setLinkedDummy", _args);
    }

    void simIK::setObjectMatrix(int environmentHandle, int objectHandle, int relativeToObjectHandle, std::vector<float> matrix)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(environmentHandle);
        _args.push_back(objectHandle);
        _args.push_back(relativeToObjectHandle);
        _args.push_back(matrix);
        auto _ret = this->_client->call("simIK.setObjectMatrix", _args);
    }

    void simIK::setObjectParent(int environmentHandle, int objectHandle, int parentObjectHandle, std::optional<bool> keepInPlace)
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

    void simIK::setObjectPose(int environmentHandle, int objectHandle, int relativeToObjectHandle, std::vector<float> pose)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(environmentHandle);
        _args.push_back(objectHandle);
        _args.push_back(relativeToObjectHandle);
        _args.push_back(pose);
        auto _ret = this->_client->call("simIK.setObjectPose", _args);
    }

    void simIK::setObjectTransformation(int environmentHandle, int objectHandle, int relativeToObjectHandle, std::vector<float> position, std::vector<float> eulerOrQuaternion)
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

    void simIK::setSphericalJointMatrix(int environmentHandle, int jointHandle, std::vector<float> matrix)
    {
        bool _brk = false;
        json _args(json_array_arg);
        _args.push_back(environmentHandle);
        _args.push_back(jointHandle);
        _args.push_back(matrix);
        auto _ret = this->_client->call("simIK.setSphericalJointMatrix", _args);
    }

    void simIK::setSphericalJointRotation(int environmentHandle, int jointHandle, std::vector<float> eulerOrQuaternion)
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

