std::optional<std::vector<uint8_t>> getStringSignal(std::string signalName);
std::optional<int64_t> getInt32Signal(std::string signalName);
std::optional<double> getFloatSignal(std::string signalName);
json callScriptFunction(std::string functionName, int64_t scriptHandle, json inArgs = json::array());
