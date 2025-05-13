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

std::vector<int64_t> sim::getObjectSelection()
{
    bool _brk = false;
    json _args(json_array_arg);
    auto _ret = this->_client->call("sim.getObjectSelection", _args);
    return _ret[0].as<std::vector<int64_t>>();
}

void sim::setObjectSelection(std::vector<double> objectHandles)
{
    bool _brk = false;
    json _args(json_array_arg);
    _args.push_back(objectHandles);
    auto _ret = this->_client->call("sim.setObjectSelection", _args);
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

std::vector<double> sim::getObjectQuaternion(int64_t objectHandle, int64_t relativeToObjectHandle)
{
    bool _brk = false;
    json _args(json_array_arg);
    _args.push_back(objectHandle);
    _args.push_back(relativeToObjectHandle);
    auto _ret = this->_client->call("sim.getObjectQuaternion", _args);
    return _ret[0].as<std::vector<double>>();
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

void sim::setObjectQuaternion(int64_t objectHandle, int64_t relativeToObjectHandle, std::vector<double> quaternion)
{
    bool _brk = false;
    json _args(json_array_arg);
    _args.push_back(objectHandle);
    _args.push_back(relativeToObjectHandle);
    _args.push_back(quaternion);
    auto _ret = this->_client->call("sim.setObjectQuaternion", _args);
}

std::vector<std::string> sim::getPersistentDataTags()
{
    bool _brk = false;
    json _args(json_array_arg);
    auto _ret = this->_client->call("sim.getPersistentDataTags", _args);
    return _ret[0].as<std::vector<std::string>>();
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

json sim::waitForSignal(std::string sigName)
{
    bool _brk = false;
    json _args(json_array_arg);
    _args.push_back(sigName);
    auto _ret = this->_client->call("sim.waitForSignal", _args);
    return _ret[0].as<json>();
}
