std::optional<std::vector<uint8_t>> sim::getStringSignal(std::string signalName)
{
    std::optional<std::vector<uint8_t>> retVal;
    auto r = this->_client->call("sim.getStringSignal", {signalName.c_str()});
    if (!r.empty())
    {
        if(r[0].is_string())
        {
            std::string r2=r[0].as<std::string>();
            std::vector<uint8_t> r3(r2.begin(),r2.end());
            retVal=std::make_optional<std::vector<uint8_t>>(r3);
        }
        else if(r[0].is_byte_string())
            retVal=std::make_optional<std::vector<uint8_t>>(r[0].as<std::vector<uint8_t>>());
    }
    return retVal;
}

std::optional<int64_t> sim::getInt32Signal(std::string signalName)
{
    std::optional<int64_t> retVal;
    auto r = this->_client->call("sim.getInt32Signal", {signalName.c_str()});
    if (!r.empty())
        retVal=std::make_optional<int64_t>(r[0].as<int64_t>());
    return retVal;
}

std::optional<double> sim::getFloatSignal(std::string signalName)
{
    std::optional<double> retVal;
    auto r = this->_client->call("sim.getInt32Signal", {signalName.c_str()});
    if (!r.empty())
        retVal=std::make_optional<double>(r[0].as<double>());
    return retVal;
}

json sim::callScriptFunction(std::string functionName, int64_t scriptHandle, json inArgs)
{
    bool _brk = false;
    json _args(json_array_arg);
    _args.push_back(functionName);
    _args.push_back(scriptHandle);
    if(!inArgs.is_array())
        throw std::runtime_error("inArgs must be an array");
    for(const auto& inArg : inArgs.array_range())
        _args.push_back(inArg);
    auto _ret = this->_client->call("sim.callScriptFunction", _args);
    return _ret;
}
