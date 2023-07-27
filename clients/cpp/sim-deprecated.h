        // DEPRECATED/BACKCOMPATIBILITY START
        double getJointMaxForce(int64_t jointHandle);
        void setJointMaxForce(int64_t objectHandle, double forceOrTorque);
        int64_t createPureShape(int64_t primitiveType, int64_t options, std::vector<double> sizes, double mass, std::optional<std::vector<int64_t>> precision = {});
        void removeObject(int64_t objectHandle);
        std::tuple<std::vector<uint8_t>, std::vector<int64_t>> getVisionSensorDepthBuffer(int64_t sensorHandle, std::optional<std::vector<int64_t>> pos = {}, std::optional<std::vector<int64_t>> size = {});
        std::tuple<std::vector<uint8_t>, std::vector<int64_t>> getVisionSensorCharImage(int64_t sensorHandle, std::optional<std::vector<int64_t>> pos = {}, std::optional<std::vector<int64_t>> size = {});
        void setVisionSensorCharImage(int64_t sensorHandle, std::vector<uint8_t> image);
        std::vector<int64_t> getObjectSelection();
        void setObjectSelection(std::vector<double> objectHandles);
        std::vector<double> getObjectPose(int64_t objectHandle, int64_t relativeToObjectHandle);
        std::vector<double> getObjectMatrix(int64_t objectHandle, int64_t relativeToObjectHandle);
        std::vector<double> getObjectPosition(int64_t objectHandle, int64_t relativeToObjectHandle);
        std::vector<double> getObjectQuaternion(int64_t objectHandle, int64_t relativeToObjectHandle);
        std::vector<double> getObjectOrientation(int64_t objectHandle, int64_t relativeToObjectHandle);
        void setObjectPose(int64_t objectHandle, int64_t relativeToObjectHandle, std::vector<double> pose);
        void setObjectMatrix(int64_t objectHandle, int64_t relativeToObjectHandle, std::vector<double> matrix);
        void setObjectPosition(int64_t objectHandle, int64_t relativeToObjectHandle, std::vector<double> position);
        void setObjectQuaternion(int64_t objectHandle, int64_t relativeToObjectHandle, std::vector<double> quaternion);
        void setObjectOrientation(int64_t objectHandle, int64_t relativeToObjectHandle, std::vector<double> eulerAngles);
        // DEPRECATED/BACKCOMPATIBILITY END
