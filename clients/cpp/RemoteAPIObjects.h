
class RemoteAPIClient;

namespace RemoteAPIObject
{
    class sim
    {
    protected:
        RemoteAPIClient *_client;
    public:
        sim(RemoteAPIClient *client);

        int addDrawingObject(int objectType, float size, float duplicateTolerance, int parentObjectHandle, int maxItemCount, std::optional<std::vector<float>> ambient_diffuse = {}, std::optional<std::vector<float>> reserved = {}, std::optional<std::vector<float>> specular = {}, std::optional<std::vector<float>> emission = {});
        int addDrawingObjectItem(int drawingObjectHandle, std::vector<float> itemData);
        void addForce(int shapeHandle, std::vector<float> position, std::vector<float> force);
        void addForceAndTorque(int shapeHandle, std::optional<std::vector<float>> force = {}, std::optional<std::vector<float>> torque = {});
        int addGraphCurve(int graphHandle, std::string curveName, int dim, std::vector<int> streamIds, std::vector<float> defaultValues, std::string unitStr, std::optional<int> options = {}, std::optional<std::vector<float>> color = {}, std::optional<int> curveWidth = {});
        int addGraphStream(int graphHandle, std::string streamName, std::string unit, std::optional<int> options = {}, std::optional<std::vector<float>> color = {}, std::optional<float> cyclicRange = {});
        void addItemToCollection(int collectionHandle, int what, int objectHandle, int options);
        void addLog(int verbosityLevel, std::string logMessage);
        int addParticleObject(int objectType, float size, float density, std::vector<float> params, float lifeTime, int maxItemCount, std::optional<std::vector<float>> ambient_diffuse = {}, std::optional<std::vector<float>> reserved = {}, std::optional<std::vector<float>> specular = {}, std::optional<std::vector<float>> emission = {});
        void addParticleObjectItem(int particleObjectHandle, std::vector<float> itemData);
        int addScript(int scriptType);
        int adjustView(int viewHandleOrIndex, int associatedViewableObjectHandle, int options, std::optional<std::string> viewLabel = {});
        std::tuple<float, float, float> alphaBetaGammaToYawPitchRoll(float alphaAngle, float betaAngle, float gammaAngle);
        int announceSceneContentChange();
        void associateScriptWithObject(int scriptHandle, int objectHandle);
        int auxiliaryConsoleClose(int consoleHandle);
        int auxiliaryConsoleOpen(std::string title, int maxLines, int mode, std::optional<std::vector<int>> position = {}, std::optional<std::vector<int>> size = {}, std::optional<std::vector<float>> textColor = {}, std::optional<std::vector<float>> backgroundColor = {});
        int auxiliaryConsolePrint(int consoleHandle, std::string text);
        int auxiliaryConsoleShow(int consoleHandle, bool showState);
        std::vector<float> buildIdentityMatrix();
        std::vector<float> buildMatrix(std::vector<float> position, std::vector<float> eulerAngles);
        std::vector<float> buildMatrixQ(std::vector<float> position, std::vector<float> quaternion);
        std::vector<float> buildPose(std::vector<float> position, std::vector<float> eulerAnglesOrAxis, std::optional<int> mode = {}, std::optional<std::vector<float>> axis2 = {});
        int cameraFitToView(int viewHandleOrIndex, std::optional<std::vector<int>> objectHandles = {}, std::optional<int> options = {}, std::optional<float> scaling = {});
        std::vector<json> changeEntityColor(int entityHandle, std::vector<float> newColor, std::optional<int> colorComponent = {});
        std::tuple<int, std::vector<int>> checkCollision(int entity1Handle, int entity2Handle);
        std::tuple<int, std::vector<float>> checkCollisionEx(int entity1Handle, int entity2Handle);
        std::tuple<int, std::vector<float>, std::vector<int>> checkDistance(int entity1Handle, int entity2Handle, std::optional<float> threshold = {});
        std::tuple<int, int, int, int> checkOctreePointOccupancy(int octreeHandle, int options, std::vector<float> points);
        std::tuple<int, float, std::vector<float>, int, std::vector<float>> checkProximitySensor(int sensorHandle, int entityHandle);
        std::tuple<int, float, std::vector<float>, int, std::vector<float>> checkProximitySensorEx(int sensorHandle, int entityHandle, int mode, float threshold, float maxAngle);
        std::tuple<int, float, std::vector<float>, std::vector<float>> checkProximitySensorEx2(int sensorHandle, std::vector<float> vertices, int itemType, int itemCount, int mode, float threshold, float maxAngle);
        std::vector<float> checkVisionSensorEx(int sensorHandle, int entityHandle, bool returnImage);
        void clearDoubleSignal(std::string signalName);
        void clearFloatSignal(std::string signalName);
        void clearInt32Signal(std::string signalName);
        void clearStringSignal(std::string signalName);
        int closeScene();
        std::vector<uint8_t> combineRgbImages(std::vector<uint8_t> img1, std::vector<int> img1Res, std::vector<uint8_t> img2, std::vector<int> img2Res, int operation);
        int computeMassAndInertia(int shapeHandle, float density);
        int convexDecompose(int shapeHandle, int options, std::vector<int> intParams, std::vector<float> floatParams);
        std::vector<int> copyPasteObjects(std::vector<int> objectHandles, int options);
        std::vector<json> copyTable(std::vector<json> original);
        int createCollection(int options);
        int createDummy(float size);
        int createForceSensor(int options, std::vector<int> intParams, std::vector<float> floatParams);
        int createHeightfieldShape(int options, float shadingAngle, int xPointCount, int yPointCount, float xSize, std::vector<float> heights);
        int createJoint(int jointType, int jointMode, int options, std::optional<std::vector<float>> sizes = {});
        int createMeshShape(int options, float shadingAngle, std::vector<float> vertices, std::vector<int> indices);
        int createOctree(float voxelSize, int options, float pointSize);
        int createPath(std::vector<float> ctrlPts, std::optional<int> options = {}, std::optional<int> subdiv = {}, std::optional<float> smoothness = {}, std::optional<int> orientationMode = {}, std::optional<std::vector<float>> upVector = {});
        int createPointCloud(float maxVoxelSize, int maxPtCntPerVoxel, int options, float pointSize);
        int createProximitySensor(int sensorType, int subType, int options, std::vector<int> intParams, std::vector<float> floatParams);
        int createPureShape(int primitiveType, int options, std::vector<float> sizes, float mass, std::optional<std::vector<int>> precision = {});
        std::tuple<int, int, std::vector<int>> createTexture(std::string fileName, int options, std::optional<std::vector<float>> planeSizes = {}, std::optional<std::vector<float>> scalingUV = {}, std::optional<std::vector<float>> xy_g = {}, std::optional<int> fixedResolution = {}, std::optional<std::vector<int>> resolution = {});
        int createVisionSensor(int options, std::vector<int> intParams, std::vector<float> floatParams);
        void destroyCollection(int collectionHandle);
        void destroyGraphCurve(int graphHandle, int curveId);
        int duplicateGraphCurveToStatic(int graphHandle, int curveId, std::optional<std::string> curveName = {});
        void exportMesh(int fileformat, std::string pathAndFilename, int options, float scalingFactor, std::vector<float> vertices, std::vector<int> indices);
        int floatingViewAdd(float posX, float posY, float sizeX, float sizeY, int options);
        int floatingViewRemove(int floatingViewHandle);
        int generateShapeFromPath(std::vector<float> path, std::vector<float> section, std::optional<int> options = {}, std::optional<std::vector<float>> upVector = {});
        int generateTextShape(std::string txt, std::optional<std::vector<float>> color = {}, std::optional<float> height = {}, std::optional<bool> centered = {}, std::optional<std::string> alphabetLocation = {});
        std::tuple<std::vector<float>, std::vector<float>> generateTimeOptimalTrajectory(std::vector<float> path, std::vector<float> pathLengths, std::vector<float> minMaxVel, std::vector<float> minMaxAccel, std::optional<int> trajPtSamples = {}, std::optional<std::string> boundaryCondition = {}, std::optional<float> timeout = {});
        std::vector<float> getAlternateConfigs(std::vector<int> jointHandles, std::vector<float> inputConfig, std::optional<int> tipHandle = {}, std::optional<std::vector<float>> lowLimits = {}, std::optional<std::vector<float>> ranges = {});
        std::vector<std::string> getApiFunc(int scriptHandleOrType, std::string apiWord);
        std::string getApiInfo(int scriptHandleOrType, std::string apiWord);
        std::vector<float> getArrayParam(int parameter);
        bool getBoolParam(int parameter);
        float getClosestPosOnPath(std::vector<float> path, std::vector<float> pathLengths, std::vector<float> absPt);
        std::vector<int> getCollectionObjects(int collectionHandle);
        float getConfigDistance(std::vector<float> configA, std::vector<float> configB, std::optional<std::vector<float>> metric = {}, std::optional<std::vector<int>> types = {});
        std::tuple<std::vector<int>, std::vector<float>, std::vector<float>, std::vector<float>> getContactInfo(int dynamicPass, int objectHandle, int index);
        std::tuple<std::vector<float>, std::vector<int>> getDecimatedMesh(std::vector<float> verticesIn, std::vector<int> indicesIn, float decimationPercentage);
        float getDoubleSignal(std::string signalName);
        bool getEngineBoolParam(int paramId, int objectHandle);
        float getEngineFloatParam(int paramId, int objectHandle);
        int getEngineInt32Param(int paramId, int objectHandle);
        std::vector<float> getEulerAnglesFromMatrix(std::vector<float> matrix);
        int getExplicitHandling(int objectHandle);
        std::string getExtensionString(int objectHandle, int index, std::optional<std::string> key = {});
        float getFloatParam(int parameter);
        float getFloatSignal(std::string signalName);
        std::vector<json> getGenesisEvents();
        std::tuple<std::string, int, std::vector<float>, std::vector<float>, std::vector<float>, std::vector<float>, int, int> getGraphCurve(int graphHandle, int graphType, int curveIndex);
        std::tuple<int, std::vector<float>, std::vector<float>, int> getGraphInfo(int graphHandle);
        int getInt32Param(int parameter);
        int getInt32Signal(std::string signalName);
        std::tuple<int, float, float> getJointDependency(int jointHandle);
        float getJointForce(int jointHandle);
        std::tuple<bool, std::vector<float>> getJointInterval(int objectHandle);
        std::tuple<int, int> getJointMode(int jointHandle);
        float getJointPosition(int objectHandle);
        float getJointTargetForce(int jointHandle);
        std::tuple<int, float> getJointTargetPosition(int objectHandle);
        float getJointTargetVelocity(int objectHandle);
        int getJointType(int objectHandle);
        float getJointVelocity(int jointHandle);
        std::tuple<int, std::vector<float>, std::vector<float>, std::vector<float>> getLightParameters(int lightHandle);
        int getLinkDummy(int dummyHandle);
        std::vector<std::string> getMatchingPersistentDataTags(std::string pattern);
        int getModelProperty(int objectHandle);
        std::string getModuleInfo(std::string moduleName, int infoType);
        std::tuple<std::string, int> getModuleName(int index);
        std::vector<uint8_t> getNamedStringParam(std::string paramName);
        int getNavigationMode();
        int getObject(std::string path, std::optional<json> options = {});
        std::string getObjectAlias(int objectHandle, std::optional<int> options = {});
        int getObjectChild(int objectHandle, int index);
        std::vector<float> getObjectChildPose(int objectHandle);
        std::vector<float> getObjectColor(int objectHandle, int index, int colorComponent);
        float getObjectFloatParam(int objectHandle, int parameterID);
        void getObjectFromUid(int uid, std::optional<json> options = {});
        int getObjectInt32Param(int objectHandle, int parameterID);
        std::vector<float> getObjectMatrix(int objectHandle, int relativeToObjectHandle);
        std::vector<float> getObjectOrientation(int objectHandle, int relativeToObjectHandle);
        int getObjectParent(int objectHandle);
        std::vector<float> getObjectPose(int objectHandle, int relativeToObjectHandle);
        std::vector<float> getObjectPosition(int objectHandle, int relativeToObjectHandle);
        int getObjectProperty(int objectHandle);
        std::vector<float> getObjectQuaternion(int objectHandle, int relativeToObjectHandle);
        std::vector<int> getObjectSelection();
        float getObjectSizeFactor(int ObjectHandle);
        int getObjectSpecialProperty(int objectHandle);
        std::vector<uint8_t> getObjectStringParam(int objectHandle, int parameterID);
        int getObjectType(int objectHandle);
        int getObjectUid(int objectHandle);
        std::tuple<std::vector<float>, std::vector<float>> getObjectVelocity(int objectHandle);
        int getObjects(int index, int objectType);
        std::vector<int> getObjectsInTree(int treeBaseHandle, std::optional<int> objectType = {}, std::optional<int> options = {});
        std::vector<float> getOctreeVoxels(int octreeHandle);
        int getPage();
        std::vector<float> getPathInterpolatedConfig(std::vector<float> path, std::vector<float> pathLengths, float t, std::optional<json> method = {}, std::optional<std::vector<int>> types = {});
        std::tuple<std::vector<float>, float> getPathLengths(std::vector<float> path, int dof, std::optional<std::string> distCallback = {});
        std::vector<std::string> getPersistentDataTags();
        std::tuple<float, int, int, float> getPointCloudOptions(int pointCloudHandle);
        std::vector<float> getPointCloudPoints(int pointCloudHandle);
        std::tuple<std::vector<float>, std::vector<int>> getQHull(std::vector<float> verticesIn);
        std::vector<float> getQuaternionFromMatrix(std::vector<float> matrix);
        float getRandom(std::optional<int> seed = {});
        int getRealTimeSimulation();
        std::vector<int> getReferencedHandles(int objectHandle);
        std::tuple<std::vector<float>, float> getRotationAxis(std::vector<float> matrixStart, std::vector<float> matrixGoal);
        std::tuple<std::vector<uint8_t>, std::vector<int>> getScaledImage(std::vector<uint8_t> imageIn, std::vector<int> resolutionIn, std::vector<int> desiredResolutionOut, int options);
        int getScriptHandle(int scriptType, std::optional<std::string> scriptName = {});
        int getScriptInt32Param(int scriptHandle, int parameterID);
        std::vector<uint8_t> getScriptStringParam(int scriptHandle, int parameterID);
        std::vector<float> getShapeBB(int shapeHandle);
        std::tuple<int, std::vector<float>> getShapeColor(int shapeHandle, std::string colorName, int colorComponent);
        std::tuple<int, int, std::vector<float>> getShapeGeomInfo(int shapeHandle);
        std::tuple<std::vector<float>, std::vector<float>> getShapeInertia(int shapeHandle);
        float getShapeMass(int shapeHandle);
        std::tuple<std::vector<float>, std::vector<int>, std::vector<float>> getShapeMesh(int shapeHandle);
        int getShapeTextureId(int shapeHandle);
        json getShapeViz(int shapeHandle, int itemIndex);
        std::string getSignalName(int signalIndex, int signalType);
        int getSimulationState();
        float getSimulationTime();
        float getSimulationTimeStep();
        std::tuple<int, std::vector<int>, std::vector<int>> getSimulatorMessage();
        std::string getStackTraceback(std::optional<int> scriptHandle = {});
        std::string getStringParam(int parameter);
        std::vector<uint8_t> getStringSignal(std::string signalName);
        float getSystemTime();
        std::tuple<int, std::vector<int>> getTextureId(std::string textureName);
        bool getThreadAutomaticSwitch();
        bool getThreadExistRequest();
        int getThreadId();
        bool getThreadSwitchAllowed();
        int getThreadSwitchTiming();
        std::vector<std::string> getUserVariables();
        std::tuple<std::vector<float>, std::vector<float>> getVelocity(int shapeHandle);
        std::tuple<std::vector<uint8_t>, int, int> getVisionSensorCharImage(int sensorHandle, std::optional<int> posX = {}, std::optional<int> posY = {}, std::optional<int> sizeX = {}, std::optional<int> sizeY = {}, std::optional<float> RgbaCutoff = {});
        std::vector<float> getVisionSensorDepthBuffer(int sensorHandle, std::optional<int> posX = {}, std::optional<int> posY = {}, std::optional<int> sizeX = {}, std::optional<int> sizeY = {});
        std::vector<float> getVisionSensorImage(int sensorHandle, std::optional<int> posX = {}, std::optional<int> posY = {}, std::optional<int> sizeX = {}, std::optional<int> sizeY = {}, std::optional<int> returnType = {});
        std::vector<int> getVisionSensorResolution(int sensorHandle);
        int groupShapes(std::vector<int> shapeHandles, std::optional<bool> merge = {});
        int handleAddOnScripts(int callType);
        int handleChildScripts(int callType);
        int handleCustomizationScripts(int callType);
        int handleDynamics(float deltaTime);
        void handleGraph(int objectHandle, float simulationTime);
        std::tuple<int, float, std::vector<float>, int, std::vector<float>> handleProximitySensor(int sensorHandle);
        void handleSandboxScript(int callType);
        void handleSensingStart();
        void handleSimulationStart();
        std::tuple<std::vector<float>, std::vector<int>> importMesh(int fileformat, std::string pathAndFilename, int options, float identicalVerticeTolerance, float scalingFactor);
        int importShape(int fileformat, std::string pathAndFilename, int options, float identicalVerticeTolerance, float scalingFactor);
        bool initScript(int scriptHandle);
        int insertObjectIntoOctree(int octreeHandle, int objectHandle, int options, std::optional<std::vector<float>> color = {}, std::optional<int> tag = {});
        int insertObjectIntoPointCloud(int pointCloudHandle, int objectHandle, int options, float gridSize, std::optional<std::vector<float>> color = {}, std::optional<float> duplicateTolerance = {});
        int insertPointsIntoPointCloud(int pointCloudHandle, int options, std::vector<float> points, std::optional<std::vector<float>> color = {}, std::optional<float> duplicateTolerance = {});
        int insertVoxelsIntoOctree(int octreeHandle, int options, std::vector<float> points, std::optional<std::vector<float>> color = {}, std::optional<std::vector<int>> tag = {});
        std::vector<float> interpolateMatrices(std::vector<float> matrixIn1, std::vector<float> matrixIn2, float interpolFactor);
        int intersectPointsWithPointCloud(int pointCloudHandle, int options, std::vector<float> points, float tolerance);
        void invertMatrix(std::vector<float> matrix);
        int isDeprecated(std::string funcOrConst);
        bool isDynamicallyEnabled(int objectHandle);
        bool isHandle(int objectHandle);
        void launchExecutable(std::string filename, std::optional<std::string> parameters = {}, std::optional<int> showStatus = {});
        std::tuple<std::vector<uint8_t>, std::vector<int>> loadImage(int options, std::string filename);
        int loadModel(std::string filename);
        int loadModule(std::string filenameAndPath, std::string pluginName);
        void loadScene(std::string filename);
        int moduleEntry(int handle, std::optional<std::string> label = {}, std::optional<int> state = {});
        std::tuple<std::vector<float>, std::vector<float>, std::vector<float>, float> moveToConfig(int flags, std::vector<float> currentPos, std::vector<float> currentVel, std::vector<float> currentAccel, std::vector<float> maxVel, std::vector<float> maxAccel, std::vector<float> maxJerk, std::vector<float> targetPos, std::vector<float> targetVel, std::string callback, std::optional<json> auxData = {}, std::optional<std::vector<bool>> cyclicJoints = {}, std::optional<float> timeStep = {});
        std::tuple<std::vector<float>, float> moveToPose(int flags, std::vector<float> currentPose, std::vector<float> maxVel, std::vector<float> maxAccel, std::vector<float> maxJerk, std::vector<float> targetPose, std::string callback, std::optional<json> auxData = {}, std::optional<std::vector<float>> metric = {}, std::optional<float> timeStep = {});
        std::vector<float> multiplyMatrices(std::vector<float> matrixIn1, std::vector<float> matrixIn2);
        std::vector<float> multiplyVector(std::vector<float> pose, std::vector<float> inVectors);
        std::vector<uint8_t> packDoubleTable(std::vector<float> doubleNumbers, std::optional<int> startDoubleIndex = {}, std::optional<int> doubleCount = {});
        std::vector<uint8_t> packFloatTable(std::vector<float> floatNumbers, std::optional<int> startFloatIndex = {}, std::optional<int> floatCount = {});
        std::vector<uint8_t> packInt32Table(std::vector<int> int32Numbers, std::optional<int> startInt32Index = {}, std::optional<int> int32Count = {});
        std::vector<uint8_t> packTable(std::vector<json> aTable, std::optional<int> scheme = {});
        std::vector<uint8_t> packUInt16Table(std::vector<int> uint16Numbers, std::optional<int> startUint16Index = {}, std::optional<int> uint16Count = {});
        std::vector<uint8_t> packUInt32Table(std::vector<int> uint32Numbers, std::optional<int> startUInt32Index = {}, std::optional<int> uint32Count = {});
        std::vector<uint8_t> packUInt8Table(std::vector<int> uint8Numbers, std::optional<int> startUint8Index = {}, std::optional<int> uint8count = {});
        int pauseSimulation();
        std::vector<uint8_t> persistentDataRead(std::string dataTag);
        void persistentDataWrite(std::string dataTag, std::vector<uint8_t> dataValue, std::optional<int> options = {});
        void pushUserEvent(std::string event, int handle, int uid, json eventData, std::optional<int> options = {});
        void quitSimulator();
        std::vector<uint8_t> readCustomDataBlock(int objectHandle, std::string tagName);
        std::vector<std::string> readCustomDataBlockTags(int objectHandle);
        json readCustomTableData(int objectHandle, std::string tagName);
        std::tuple<int, std::vector<float>, std::vector<float>> readForceSensor(int objectHandle);
        std::tuple<int, float, std::vector<float>, int, std::vector<float>> readProximitySensor(int sensorHandle);
        std::vector<uint8_t> readTexture(int textureId, int options, std::optional<int> posX = {}, std::optional<int> posY = {}, std::optional<int> sizeX = {}, std::optional<int> sizeY = {});
        std::tuple<int, std::vector<float>> readVisionSensor(int sensorHandle);
        int refreshDialogs(int refreshDegree);
        int registerScriptFuncHook(std::string funcToHook, std::string userFunc, bool execBefore);
        int registerScriptFunction(std::string funcNameAtPluginName, std::string callTips);
        int registerScriptVariable(std::string varNameAtPluginName);
        void removeDrawingObject(int drawingObjectHandle);
        int removeModel(int objectHandle);
        void removeObjects(std::vector<int> objectHandles);
        void removeParticleObject(int particleObjectHandle);
        int removePointsFromPointCloud(int pointCloudHandle, int options, std::vector<float> points, float tolerance);
        void removeScript(int scriptHandle);
        int removeVoxelsFromOctree(int octreeHandle, int options, std::vector<float> points);
        int reorientShapeBoundingBox(int shapeHandle, int relativeToHandle);
        std::vector<float> resamplePath(std::vector<float> path, std::vector<float> pathLengths, int finalConfigCnt, std::optional<json> method = {}, std::optional<std::vector<int>> types = {});
        void resetDynamicObject(int objectHandle);
        void resetGraph(int objectHandle);
        void resetProximitySensor(int objectHandle);
        void resetVisionSensor(int sensorHandle);
        void restoreEntityColor(std::vector<json> originalColorData);
        int rmlPos(int dofs, float smallestTimeStep, int flags, std::vector<float> currentPosVelAccel, std::vector<float> maxVelAccelJerk, std::vector<int> selection, std::vector<float> targetPosVel);
        void rmlRemove(int handle);
        std::tuple<int, std::vector<float>, float> rmlStep(int handle, float timeStep);
        int rmlVel(int dofs, float smallestTimeStep, int flags, std::vector<float> currentPosVelAccel, std::vector<float> maxAccelJerk, std::vector<int> selection, std::vector<float> targetVel);
        std::vector<float> rotateAroundAxis(std::vector<float> matrixIn, std::vector<float> axis, std::vector<float> axisPos, float angle);
        int ruckigPos(int dofs, float smallestTimeStep, int flags, std::vector<float> currentPosVelAccel, std::vector<float> maxVelAccelJerk, std::vector<int> selection, std::vector<float> targetPosVel);
        void ruckigRemove(int handle);
        std::tuple<int, std::vector<float>, float> ruckigStep(int handle, float timeStep);
        int ruckigVel(int dofs, float smallestTimeStep, int flags, std::vector<float> currentPosVelAccel, std::vector<float> maxAccelJerk, std::vector<int> selection, std::vector<float> targetVel);
        std::vector<uint8_t> saveImage(std::vector<uint8_t> image, std::vector<int> resolution, int options, std::string filename, int quality);
        void saveModel(int modelBaseHandle, std::string filename);
        void saveScene(std::string filename);
        void scaleObject(int objectHandle, float xScale, float yScale, float zScale, std::optional<int> options = {});
        void scaleObjects(std::vector<int> objectHandles, float scalingFactor, bool scalePositionsToo);
        int serialCheck(int portHandle);
        void serialClose(int portHandle);
        int serialOpen(std::string portString, int baudrate);
        std::vector<uint8_t> serialRead(int portHandle, int dataLengthToRead, bool blockingOperation, std::optional<std::vector<uint8_t>> closingString = {}, std::optional<float> timeout = {});
        int serialSend(int portHandle, std::vector<uint8_t> data);
        void setArrayParam(int parameter, std::vector<float> arrayOfValues);
        void setBoolParam(int parameter, bool boolState);
        void setDoubleSignal(std::string signalName, float signalValue);
        void setEngineBoolParam(int paramId, int objectHandle, bool boolParam);
        void setEngineFloatParam(int paramId, int objectHandle, float floatParam);
        void setEngineInt32Param(int paramId, int objectHandle, int int32Param);
        void setExplicitHandling(int objectHandle, int explicitHandlingFlags);
        void setFloatParam(int parameter, float floatState);
        void setFloatSignal(std::string signalName, float signalValue);
        void setGraphStreamTransformation(int graphHandle, int streamId, int trType, std::optional<float> mult = {}, std::optional<float> off = {}, std::optional<int> movAvgPeriod = {});
        void setGraphStreamValue(int graphHandle, int streamId, float value);
        void setInt32Param(int parameter, int intState);
        void setInt32Signal(std::string signalName, int signalValue);
        void setJointDependency(int jointHandle, int masterJointHandle, float offset, float multCoeff);
        void setJointInterval(int objectHandle, bool cyclic, std::vector<float> interval);
        void setJointMode(int jointHandle, int jointMode, int options);
        void setJointPosition(int objectHandle, float position);
        void setJointTargetForce(int objectHandle, float forceOrTorque, std::optional<bool> signedValue = {});
        void setJointTargetPosition(int objectHandle, float targetPosition);
        void setJointTargetVelocity(int objectHandle, float targetVelocity);
        void setLightParameters(int lightHandle, int state, std::vector<float> reserved, std::vector<float> diffusePart, std::vector<float> specularPart);
        void setLinkDummy(int dummyHandle, int linkDummyHandle);
        void setModelProperty(int objectHandle, int property);
        void setModuleInfo(std::string moduleName, int infoType, std::string info);
        void setNamedStringParam(std::string paramName, std::vector<uint8_t> stringParam);
        void setNavigationMode(int navigationMode);
        void setObjectAlias(int objectHandle, std::string objectAlias);
        void setObjectChildPose(int objectHandle, std::vector<float> pose);
        bool setObjectColor(int objectHandle, int index, int colorComponent, std::vector<float> rgbData);
        void setObjectFloatParam(int objectHandle, int parameterID, float parameter);
        void setObjectInt32Param(int objectHandle, int parameterID, int parameter);
        void setObjectMatrix(int objectHandle, int relativeToObjectHandle, std::vector<float> matrix);
        void setObjectOrientation(int objectHandle, int relativeToObjectHandle, std::vector<float> eulerAngles);
        void setObjectParent(int objectHandle, int parentObjectHandle, bool keepInPlace);
        void setObjectPose(int objectHandle, int relativeToObjectHandle, std::vector<float> pose);
        void setObjectPosition(int objectHandle, int relativeToObjectHandle, std::vector<float> position);
        void setObjectProperty(int objectHandle, int property);
        void setObjectQuaternion(int objectHandle, int relativeToObjectHandle, std::vector<float> quaternion);
        void setObjectSelection(std::vector<float> objectHandles);
        void setObjectSpecialProperty(int objectHandle, int property);
        void setObjectStringParam(int objectHandle, int parameterID, std::vector<uint8_t> parameter);
        void setPage(int pageIndex);
        void setPointCloudOptions(int pointCloudHandle, float maxVoxelSize, int maxPtCntPerVoxel, int options, float pointSize);
        void setReferencedHandles(int objectHandle, std::vector<int> referencedHandles);
        void setScriptInt32Param(int scriptHandle, int parameterID, int parameter);
        void setScriptStringParam(int scriptHandle, int parameterID, std::vector<uint8_t> parameter);
        void setShapeBB(int shapeHandle, std::vector<float> size);
        void setShapeColor(int shapeHandle, std::string colorName, int colorComponent, std::vector<float> rgbData);
        void setShapeInertia(int shapeHandle, std::vector<float> inertiaMatrix, std::vector<float> transformationMatrix);
        void setShapeMass(int shapeHandle, float mass);
        void setShapeMaterial(int shapeHandle, int materialIdOrShapeHandle);
        void setShapeTexture(int shapeHandle, int textureId, int mappingMode, int options, std::vector<float> uvScaling, std::optional<std::vector<float>> position = {}, std::optional<std::vector<float>> orientation = {});
        void setStringParam(int parameter, std::string stringState);
        void setStringSignal(std::string signalName, std::vector<uint8_t> signalValue);
        int setThreadAutomaticSwitch(bool automaticSwitch);
        int setThreadSwitchAllowed(bool allowed);
        void setThreadSwitchTiming(int dtInMs);
        int setVisionSensorCharImage(int sensorHandle, std::vector<uint8_t> imageBuffer);
        int setVisionSensorImage(int sensorHandle, std::vector<uint8_t> image);
        int startSimulation();
        int stopSimulation();
        int subtractObjectFromOctree(int octreeHandle, int objectHandle, int options);
        int subtractObjectFromPointCloud(int pointCloudHandle, int objectHandle, int options, float tolerance);
        void switchThread();
        std::tuple<std::string, std::vector<int>, std::vector<int>> textEditorClose(int handle);
        std::tuple<std::string, std::vector<int>, std::vector<int>, bool> textEditorGetInfo(int handle);
        int textEditorOpen(std::string initText, std::string properties);
        void textEditorShow(int handle, bool showState);
        std::vector<uint8_t> transformBuffer(std::vector<uint8_t> inBuffer, int inFormat, float multiplier, float offset, int outFormat);
        void transformImage(std::vector<uint8_t> image, std::vector<int> resolution, int options);
        std::vector<int> ungroupShape(int shapeHandle);
        int unloadModule(int pluginHandle);
        std::vector<float> unpackDoubleTable(std::vector<uint8_t> data, std::optional<int> startDoubleIndex = {}, std::optional<int> doubleCount = {}, std::optional<int> additionalByteOffset = {});
        std::vector<float> unpackFloatTable(std::vector<uint8_t> data, std::optional<int> startFloatIndex = {}, std::optional<int> floatCount = {}, std::optional<int> additionalByteOffset = {});
        std::vector<int> unpackInt32Table(std::vector<uint8_t> data, std::optional<int> startInt32Index = {}, std::optional<int> int32Count = {}, std::optional<int> additionalByteOffset = {});
        json unpackTable(std::vector<uint8_t> buffer);
        std::vector<int> unpackUInt16Table(std::vector<uint8_t> data, std::optional<int> startUint16Index = {}, std::optional<int> uint16Count = {}, std::optional<int> additionalByteOffset = {});
        std::vector<int> unpackUInt32Table(std::vector<uint8_t> data, std::optional<int> startUint32Index = {}, std::optional<int> uint32Count = {}, std::optional<int> additionalByteOffset = {});
        std::vector<int> unpackUInt8Table(std::vector<uint8_t> data, std::optional<int> startUint8Index = {}, std::optional<int> uint8count = {});
        float wait(float dt, std::optional<bool> simulationTime = {});
        json waitForSignal(std::string sigName);
        void writeCustomDataBlock(int objectHandle, std::string tagName, std::vector<uint8_t> data);
        void writeCustomTableData(int objectHandle, std::string tagName, std::vector<json> data);
        void writeTexture(int textureId, int options, std::vector<uint8_t> textureData, std::optional<int> posX = {}, std::optional<int> posY = {}, std::optional<int> sizeX = {}, std::optional<int> sizeY = {}, std::optional<float> interpol = {});
        std::tuple<float, float, float> yawPitchRollToAlphaBetaGamma(float yawAngle, float pitchAngle, float rollAngle);

#ifndef addonscriptcall_cleanup
        const int addonscriptcall_cleanup = 1;
#endif
#ifndef addonscriptcall_initialization
        const int addonscriptcall_initialization = 2;
#endif
#ifndef addonscriptcall_restarting
        const int addonscriptcall_restarting = 17;
#endif
#ifndef addonscriptcall_run
        const int addonscriptcall_run = 15;
#endif
#ifndef addonscriptcall_suspend
        const int addonscriptcall_suspend = 16;
#endif
#ifndef api_error_output
        const int api_error_output = 2;
#endif
#ifndef api_error_report
        const int api_error_report = 1;
#endif
#ifndef api_warning_output
        const int api_warning_output = 4;
#endif
#ifndef appobj_2delement_type
        const int appobj_2delement_type = 116;
#endif
#ifndef appobj_collection_type
        const int appobj_collection_type = 115;
#endif
#ifndef appobj_collision_type
        const int appobj_collision_type = 110;
#endif
#ifndef appobj_distance_type
        const int appobj_distance_type = 111;
#endif
#ifndef appobj_ik_type
        const int appobj_ik_type = 113;
#endif
#ifndef appobj_object_type
        const int appobj_object_type = 109;
#endif
#ifndef appobj_pathplanning_type
        const int appobj_pathplanning_type = 118;
#endif
#ifndef appobj_script_type
        const int appobj_script_type = 117;
#endif
#ifndef appobj_simulation_type
        const int appobj_simulation_type = 112;
#endif
#ifndef appobj_texture_type
        const int appobj_texture_type = 120;
#endif
#ifndef appobj_ui_type
        const int appobj_ui_type = 116;
#endif
#ifndef arrayparam_ambient_light
        const int arrayparam_ambient_light = 5;
#endif
#ifndef arrayparam_background_color1
        const int arrayparam_background_color1 = 3;
#endif
#ifndef arrayparam_background_color2
        const int arrayparam_background_color2 = 4;
#endif
#ifndef arrayparam_fog
        const int arrayparam_fog = 1;
#endif
#ifndef arrayparam_fog_color
        const int arrayparam_fog_color = 2;
#endif
#ifndef arrayparam_gravity
        const int arrayparam_gravity = 0;
#endif
#ifndef arrayparam_random_euler
        const int arrayparam_random_euler = 6;
#endif
#ifndef banner_backfaceculling
        const int banner_backfaceculling = 512;
#endif
#ifndef banner_bitmapfont
        const int banner_bitmapfont = 2048;
#endif
#ifndef banner_clickselectsparent
        const int banner_clickselectsparent = 32;
#endif
#ifndef banner_clicktriggersevent
        const int banner_clicktriggersevent = 64;
#endif
#ifndef banner_facingcamera
        const int banner_facingcamera = 128;
#endif
#ifndef banner_followparentvisibility
        const int banner_followparentvisibility = 16;
#endif
#ifndef banner_fullyfacingcamera
        const int banner_fullyfacingcamera = 256;
#endif
#ifndef banner_keepsamesize
        const int banner_keepsamesize = 1024;
#endif
#ifndef banner_left
        const int banner_left = 1;
#endif
#ifndef banner_nobackground
        const int banner_nobackground = 4;
#endif
#ifndef banner_overlay
        const int banner_overlay = 8;
#endif
#ifndef banner_right
        const int banner_right = 2;
#endif
#ifndef boolparam_aux_clip_planes_enabled
        const int boolparam_aux_clip_planes_enabled = 23;
#endif
#ifndef boolparam_browser_toolbarbutton_enabled
        const int boolparam_browser_toolbarbutton_enabled = 36;
#endif
#ifndef boolparam_browser_visible
        const int boolparam_browser_visible = 12;
#endif
#ifndef boolparam_calcmodules_toolbarbutton_enabled
        const int boolparam_calcmodules_toolbarbutton_enabled = 47;
#endif
#ifndef boolparam_collision_handling_enabled
        const int boolparam_collision_handling_enabled = 2;
#endif
#ifndef boolparam_console_visible
        const int boolparam_console_visible = 1;
#endif
#ifndef boolparam_display_enabled
        const int boolparam_display_enabled = 16;
#endif
#ifndef boolparam_distance_handling_enabled
        const int boolparam_distance_handling_enabled = 3;
#endif
#ifndef boolparam_dynamics_handling_enabled
        const int boolparam_dynamics_handling_enabled = 6;
#endif
#ifndef boolparam_exit_request
        const int boolparam_exit_request = 41;
#endif
#ifndef boolparam_fog_enabled
        const int boolparam_fog_enabled = 19;
#endif
#ifndef boolparam_force_calcstruct_all
        const int boolparam_force_calcstruct_all = 40;
#endif
#ifndef boolparam_force_calcstruct_all_visible
        const int boolparam_force_calcstruct_all_visible = 39;
#endif
#ifndef boolparam_force_show_wireless_emission
        const int boolparam_force_show_wireless_emission = 27;
#endif
#ifndef boolparam_force_show_wireless_reception
        const int boolparam_force_show_wireless_reception = 28;
#endif
#ifndef boolparam_full_model_copy_from_api
        const int boolparam_full_model_copy_from_api = 24;
#endif
#ifndef boolparam_fullscreen
        const int boolparam_fullscreen = 33;
#endif
#ifndef boolparam_gcs_handling_enabled
        const int boolparam_gcs_handling_enabled = 5;
#endif
#ifndef boolparam_headless
        const int boolparam_headless = 34;
#endif
#ifndef boolparam_hierarchy_toolbarbutton_enabled
        const int boolparam_hierarchy_toolbarbutton_enabled = 35;
#endif
#ifndef boolparam_hierarchy_visible
        const int boolparam_hierarchy_visible = 0;
#endif
#ifndef boolparam_ik_handling_enabled
        const int boolparam_ik_handling_enabled = 4;
#endif
#ifndef boolparam_infotext_visible
        const int boolparam_infotext_visible = 17;
#endif
#ifndef boolparam_mill_handling_enabled
        const int boolparam_mill_handling_enabled = 11;
#endif
#ifndef boolparam_mirrors_enabled
        const int boolparam_mirrors_enabled = 22;
#endif
#ifndef boolparam_objectrotate_toolbarbutton_enabled
        const int boolparam_objectrotate_toolbarbutton_enabled = 38;
#endif
#ifndef boolparam_objectshift_toolbarbutton_enabled
        const int boolparam_objectshift_toolbarbutton_enabled = 37;
#endif
#ifndef boolparam_objproperties_toolbarbutton_enabled
        const int boolparam_objproperties_toolbarbutton_enabled = 46;
#endif
#ifndef boolparam_pause_toolbarbutton_enabled
        const int boolparam_pause_toolbarbutton_enabled = 43;
#endif
#ifndef boolparam_play_toolbarbutton_enabled
        const int boolparam_play_toolbarbutton_enabled = 42;
#endif
#ifndef boolparam_proximity_sensor_handling_enabled
        const int boolparam_proximity_sensor_handling_enabled = 9;
#endif
#ifndef boolparam_realtime_simulation
        const int boolparam_realtime_simulation = 25;
#endif
#ifndef boolparam_rendering_sensor_handling_enabled
        const int boolparam_rendering_sensor_handling_enabled = 10;
#endif
#ifndef boolparam_rml2_available
        const int boolparam_rml2_available = 20;
#endif
#ifndef boolparam_rml4_available
        const int boolparam_rml4_available = 21;
#endif
#ifndef boolparam_rosinterface_donotrunmainscript
        const int boolparam_rosinterface_donotrunmainscript = 48;
#endif
#ifndef boolparam_scene_and_model_load_messages
        const int boolparam_scene_and_model_load_messages = 13;
#endif
#ifndef boolparam_scene_closing
        const int boolparam_scene_closing = 52;
#endif
#ifndef boolparam_shape_textures_are_visible
        const int boolparam_shape_textures_are_visible = 15;
#endif
#ifndef boolparam_show_w_emitters
        const int boolparam_show_w_emitters = 53;
#endif
#ifndef boolparam_show_w_receivers
        const int boolparam_show_w_receivers = 54;
#endif
#ifndef boolparam_statustext_open
        const int boolparam_statustext_open = 18;
#endif
#ifndef boolparam_stop_toolbarbutton_enabled
        const int boolparam_stop_toolbarbutton_enabled = 44;
#endif
#ifndef boolparam_use_glfinish_cmd
        const int boolparam_use_glfinish_cmd = 26;
#endif
#ifndef boolparam_video_recording_triggered
        const int boolparam_video_recording_triggered = 29;
#endif
#ifndef boolparam_vision_sensor_handling_enabled
        const int boolparam_vision_sensor_handling_enabled = 10;
#endif
#ifndef boolparam_waiting_for_trigger
        const int boolparam_waiting_for_trigger = 45;
#endif
#ifndef buffer_base64
        const int buffer_base64 = 12;
#endif
#ifndef buffer_clamp
        const int buffer_clamp = 256;
#endif
#ifndef buffer_double
        const int buffer_double = 7;
#endif
#ifndef buffer_float
        const int buffer_float = 6;
#endif
#ifndef buffer_int16
        const int buffer_int16 = 3;
#endif
#ifndef buffer_int32
        const int buffer_int32 = 5;
#endif
#ifndef buffer_int8
        const int buffer_int8 = 1;
#endif
#ifndef buffer_split
        const int buffer_split = 13;
#endif
#ifndef buffer_uint16
        const int buffer_uint16 = 2;
#endif
#ifndef buffer_uint32
        const int buffer_uint32 = 4;
#endif
#ifndef buffer_uint8
        const int buffer_uint8 = 0;
#endif
#ifndef buffer_uint8argb
        const int buffer_uint8argb = 11;
#endif
#ifndef buffer_uint8bgr
        const int buffer_uint8bgr = 9;
#endif
#ifndef buffer_uint8rgb
        const int buffer_uint8rgb = 8;
#endif
#ifndef buffer_uint8rgba
        const int buffer_uint8rgba = 10;
#endif
#ifndef bullet_body_angulardamping
        const int bullet_body_angulardamping = 6005;
#endif
#ifndef bullet_body_autoshrinkconvex
        const int bullet_body_autoshrinkconvex = 8004;
#endif
#ifndef bullet_body_bitcoded
        const int bullet_body_bitcoded = 7001;
#endif
#ifndef bullet_body_friction
        const int bullet_body_friction = 6003;
#endif
#ifndef bullet_body_lineardamping
        const int bullet_body_lineardamping = 6004;
#endif
#ifndef bullet_body_nondefaultcollisionmargingfactor
        const int bullet_body_nondefaultcollisionmargingfactor = 6006;
#endif
#ifndef bullet_body_nondefaultcollisionmargingfactorconvex
        const int bullet_body_nondefaultcollisionmargingfactorconvex = 6007;
#endif
#ifndef bullet_body_oldfriction
        const int bullet_body_oldfriction = 6002;
#endif
#ifndef bullet_body_restitution
        const int bullet_body_restitution = 6001;
#endif
#ifndef bullet_body_sticky
        const int bullet_body_sticky = 8001;
#endif
#ifndef bullet_body_usenondefaultcollisionmargin
        const int bullet_body_usenondefaultcollisionmargin = 8002;
#endif
#ifndef bullet_body_usenondefaultcollisionmarginconvex
        const int bullet_body_usenondefaultcollisionmarginconvex = 8003;
#endif
#ifndef bullet_constraintsolvertype_dantzig
        const int bullet_constraintsolvertype_dantzig = 2;
#endif
#ifndef bullet_constraintsolvertype_nncg
        const int bullet_constraintsolvertype_nncg = 1;
#endif
#ifndef bullet_constraintsolvertype_projectedgaussseidel
        const int bullet_constraintsolvertype_projectedgaussseidel = 3;
#endif
#ifndef bullet_constraintsolvertype_sequentialimpulse
        const int bullet_constraintsolvertype_sequentialimpulse = 0;
#endif
#ifndef bullet_global_bitcoded
        const int bullet_global_bitcoded = 1002;
#endif
#ifndef bullet_global_collisionmarginfactor
        const int bullet_global_collisionmarginfactor = 3;
#endif
#ifndef bullet_global_constraintsolvertype
        const int bullet_global_constraintsolvertype = 1003;
#endif
#ifndef bullet_global_constraintsolvingiterations
        const int bullet_global_constraintsolvingiterations = 1001;
#endif
#ifndef bullet_global_fullinternalscaling
        const int bullet_global_fullinternalscaling = 2001;
#endif
#ifndef bullet_global_internalscalingfactor
        const int bullet_global_internalscalingfactor = 2;
#endif
#ifndef bullet_global_stepsize
        const int bullet_global_stepsize = 1;
#endif
#ifndef bullet_joint_normalcfm
        const int bullet_joint_normalcfm = 3003;
#endif
#ifndef bullet_joint_stopcfm
        const int bullet_joint_stopcfm = 3002;
#endif
#ifndef bullet_joint_stoperp
        const int bullet_joint_stoperp = 3001;
#endif
#ifndef buttonproperty_borderless
        const int buttonproperty_borderless = 32;
#endif
#ifndef buttonproperty_button
        const int buttonproperty_button = 0;
#endif
#ifndef buttonproperty_closeaction
        const int buttonproperty_closeaction = 4096;
#endif
#ifndef buttonproperty_downupevent
        const int buttonproperty_downupevent = 16384;
#endif
#ifndef buttonproperty_editbox
        const int buttonproperty_editbox = 3;
#endif
#ifndef buttonproperty_enabled
        const int buttonproperty_enabled = 16;
#endif
#ifndef buttonproperty_horizontallycentered
        const int buttonproperty_horizontallycentered = 64;
#endif
#ifndef buttonproperty_ignoremouse
        const int buttonproperty_ignoremouse = 128;
#endif
#ifndef buttonproperty_isdown
        const int buttonproperty_isdown = 256;
#endif
#ifndef buttonproperty_label
        const int buttonproperty_label = 1;
#endif
#ifndef buttonproperty_nobackgroundcolor
        const int buttonproperty_nobackgroundcolor = 1024;
#endif
#ifndef buttonproperty_rollupaction
        const int buttonproperty_rollupaction = 2048;
#endif
#ifndef buttonproperty_slider
        const int buttonproperty_slider = 2;
#endif
#ifndef buttonproperty_staydown
        const int buttonproperty_staydown = 8;
#endif
#ifndef buttonproperty_transparent
        const int buttonproperty_transparent = 512;
#endif
#ifndef buttonproperty_verticallycentered
        const int buttonproperty_verticallycentered = 8192;
#endif
#ifndef callbackid_dynstep
        const int callbackid_dynstep = 2;
#endif
#ifndef callbackid_rossubscriber
        const int callbackid_rossubscriber = 1;
#endif
#ifndef callbackid_userdefined
        const int callbackid_userdefined = 1000;
#endif
#ifndef camerafloatparam_far_clipping
        const int camerafloatparam_far_clipping = 9009;
#endif
#ifndef camerafloatparam_near_clipping
        const int camerafloatparam_near_clipping = 9008;
#endif
#ifndef camerafloatparam_ortho_size
        const int camerafloatparam_ortho_size = 9002;
#endif
#ifndef camerafloatparam_perspective_angle
        const int camerafloatparam_perspective_angle = 9001;
#endif
#ifndef camerafloatparam_pov_aperture
        const int camerafloatparam_pov_aperture = 9006;
#endif
#ifndef camerafloatparam_pov_blur_distance
        const int camerafloatparam_pov_blur_distance = 9005;
#endif
#ifndef cameraintparam_disabled_light_components
        const int cameraintparam_disabled_light_components = 9000;
#endif
#ifndef cameraintparam_perspective_operation
        const int cameraintparam_perspective_operation = 9010;
#endif
#ifndef cameraintparam_pov_blur_samples
        const int cameraintparam_pov_blur_samples = 9007;
#endif
#ifndef cameraintparam_pov_focal_blur
        const int cameraintparam_pov_focal_blur = 9004;
#endif
#ifndef cameraintparam_remotecameramode
        const int cameraintparam_remotecameramode = 9012;
#endif
#ifndef cameraintparam_rendering_attributes
        const int cameraintparam_rendering_attributes = 9003;
#endif
#ifndef cameraintparam_trackedobject
        const int cameraintparam_trackedobject = 9011;
#endif
#ifndef childscriptattribute_automaticcascadingcalls
        const int childscriptattribute_automaticcascadingcalls = 3;
#endif
#ifndef childscriptattribute_enabled
        const int childscriptattribute_enabled = 4;
#endif
#ifndef childscriptcall_actuation
        const int childscriptcall_actuation = 6;
#endif
#ifndef childscriptcall_cleanup
        const int childscriptcall_cleanup = 1;
#endif
#ifndef childscriptcall_initialization
        const int childscriptcall_initialization = 2;
#endif
#ifndef childscriptcall_sensing
        const int childscriptcall_sensing = 7;
#endif
#ifndef colorcomponent_ambient
        const int colorcomponent_ambient = 0;
#endif
#ifndef colorcomponent_ambient_diffuse
        const int colorcomponent_ambient_diffuse = 0;
#endif
#ifndef colorcomponent_auxiliary
        const int colorcomponent_auxiliary = 5;
#endif
#ifndef colorcomponent_diffuse
        const int colorcomponent_diffuse = 1;
#endif
#ifndef colorcomponent_emission
        const int colorcomponent_emission = 3;
#endif
#ifndef colorcomponent_specular
        const int colorcomponent_specular = 2;
#endif
#ifndef colorcomponent_transparency
        const int colorcomponent_transparency = 4;
#endif
#ifndef customizationscriptattribute_activeduringsimulation
        const int customizationscriptattribute_activeduringsimulation = 0;
#endif
#ifndef customizationscriptattribute_cleanupbeforesave
        const int customizationscriptattribute_cleanupbeforesave = 5;
#endif
#ifndef customizationscriptcall_aftercopy
        const int customizationscriptcall_aftercopy = 14;
#endif
#ifndef customizationscriptcall_beforecopy
        const int customizationscriptcall_beforecopy = 13;
#endif
#ifndef customizationscriptcall_cleanup
        const int customizationscriptcall_cleanup = 1;
#endif
#ifndef customizationscriptcall_firstafterinstanceswitch
        const int customizationscriptcall_firstafterinstanceswitch = 12;
#endif
#ifndef customizationscriptcall_firstaftersimulation
        const int customizationscriptcall_firstaftersimulation = 5;
#endif
#ifndef customizationscriptcall_initialization
        const int customizationscriptcall_initialization = 2;
#endif
#ifndef customizationscriptcall_lastbeforeinstanceswitch
        const int customizationscriptcall_lastbeforeinstanceswitch = 11;
#endif
#ifndef customizationscriptcall_lastbeforesimulation
        const int customizationscriptcall_lastbeforesimulation = 4;
#endif
#ifndef customizationscriptcall_nonsimulation
        const int customizationscriptcall_nonsimulation = 3;
#endif
#ifndef customizationscriptcall_simulationactuation
        const int customizationscriptcall_simulationactuation = 6;
#endif
#ifndef customizationscriptcall_simulationpause
        const int customizationscriptcall_simulationpause = 8;
#endif
#ifndef customizationscriptcall_simulationpausefirst
        const int customizationscriptcall_simulationpausefirst = 9;
#endif
#ifndef customizationscriptcall_simulationpauselast
        const int customizationscriptcall_simulationpauselast = 10;
#endif
#ifndef customizationscriptcall_simulationsensing
        const int customizationscriptcall_simulationsensing = 7;
#endif
#ifndef displayattribute_colorcoded
        const int displayattribute_colorcoded = 8192;
#endif
#ifndef displayattribute_colorcodedpickpass
        const int displayattribute_colorcodedpickpass = 4096;
#endif
#ifndef displayattribute_colorcodedtriangles
        const int displayattribute_colorcodedtriangles = 134217728;
#endif
#ifndef displayattribute_depthpass
        const int displayattribute_depthpass = 2;
#endif
#ifndef displayattribute_dynamiccontentonly
        const int displayattribute_dynamiccontentonly = 131072;
#endif
#ifndef displayattribute_forbidedges
        const int displayattribute_forbidedges = 256;
#endif
#ifndef displayattribute_forbidwireframe
        const int displayattribute_forbidwireframe = 128;
#endif
#ifndef displayattribute_forcewireframe
        const int displayattribute_forcewireframe = 64;
#endif
#ifndef displayattribute_forvisionsensor
        const int displayattribute_forvisionsensor = 2048;
#endif
#ifndef displayattribute_ignorelayer
        const int displayattribute_ignorelayer = 1024;
#endif
#ifndef displayattribute_ignorerenderableflag
        const int displayattribute_ignorerenderableflag = 1048576;
#endif
#ifndef displayattribute_mainselection
        const int displayattribute_mainselection = 32;
#endif
#ifndef displayattribute_mirror
        const int displayattribute_mirror = 262144;
#endif
#ifndef displayattribute_nodrawingobjects
        const int displayattribute_nodrawingobjects = 33554432;
#endif
#ifndef displayattribute_noghosts
        const int displayattribute_noghosts = 8388608;
#endif
#ifndef displayattribute_noopenglcallbacks
        const int displayattribute_noopenglcallbacks = 2097152;
#endif
#ifndef displayattribute_noparticles
        const int displayattribute_noparticles = 67108864;
#endif
#ifndef displayattribute_nopointclouds
        const int displayattribute_nopointclouds = 16777216;
#endif
#ifndef displayattribute_originalcolors
        const int displayattribute_originalcolors = 512;
#endif
#ifndef displayattribute_pickpass
        const int displayattribute_pickpass = 4;
#endif
#ifndef displayattribute_renderpass
        const int displayattribute_renderpass = 1;
#endif
#ifndef displayattribute_selected
        const int displayattribute_selected = 8;
#endif
#ifndef displayattribute_thickEdges
        const int displayattribute_thickEdges = 65536;
#endif
#ifndef displayattribute_trianglewireframe
        const int displayattribute_trianglewireframe = 16384;
#endif
#ifndef displayattribute_useauxcomponent
        const int displayattribute_useauxcomponent = 524288;
#endif
#ifndef distcalcmethod_dac
        const int distcalcmethod_dac = 1;
#endif
#ifndef distcalcmethod_dac_if_nonzero
        const int distcalcmethod_dac_if_nonzero = 6;
#endif
#ifndef distcalcmethod_dl
        const int distcalcmethod_dl = 0;
#endif
#ifndef distcalcmethod_dl_and_dac
        const int distcalcmethod_dl_and_dac = 3;
#endif
#ifndef distcalcmethod_dl_if_nonzero
        const int distcalcmethod_dl_if_nonzero = 5;
#endif
#ifndef distcalcmethod_max_dl_dac
        const int distcalcmethod_max_dl_dac = 2;
#endif
#ifndef distcalcmethod_sqrt_dl2_and_dac2
        const int distcalcmethod_sqrt_dl2_and_dac2 = 4;
#endif
#ifndef dlgret_cancel
        const int dlgret_cancel = 2;
#endif
#ifndef dlgret_no
        const int dlgret_no = 4;
#endif
#ifndef dlgret_ok
        const int dlgret_ok = 1;
#endif
#ifndef dlgret_still_open
        const int dlgret_still_open = 0;
#endif
#ifndef dlgret_yes
        const int dlgret_yes = 3;
#endif
#ifndef dlgstyle_dont_center
        const int dlgstyle_dont_center = 32;
#endif
#ifndef dlgstyle_input
        const int dlgstyle_input = 1;
#endif
#ifndef dlgstyle_message
        const int dlgstyle_message = 0;
#endif
#ifndef dlgstyle_ok
        const int dlgstyle_ok = 2;
#endif
#ifndef dlgstyle_ok_cancel
        const int dlgstyle_ok_cancel = 3;
#endif
#ifndef dlgstyle_yes_no
        const int dlgstyle_yes_no = 4;
#endif
#ifndef drawing_12percenttransparency
        const int drawing_12percenttransparency = 32768;
#endif
#ifndef drawing_25percenttransparency
        const int drawing_25percenttransparency = 16384;
#endif
#ifndef drawing_50percenttransparency
        const int drawing_50percenttransparency = 8192;
#endif
#ifndef drawing_auxchannelcolor1
        const int drawing_auxchannelcolor1 = 2097152;
#endif
#ifndef drawing_auxchannelcolor2
        const int drawing_auxchannelcolor2 = 4194304;
#endif
#ifndef drawing_backfaceculling
        const int drawing_backfaceculling = 256;
#endif
#ifndef drawing_cubepoints
        const int drawing_cubepoints = 6;
#endif
#ifndef drawing_cubepts
        const int drawing_cubepts = 12;
#endif
#ifndef drawing_cyclic
        const int drawing_cyclic = 4096;
#endif
#ifndef drawing_discpoints
        const int drawing_discpoints = 5;
#endif
#ifndef drawing_discpts
        const int drawing_discpts = 11;
#endif
#ifndef drawing_emissioncolor
        const int drawing_emissioncolor = 65536;
#endif
#ifndef drawing_facingcamera
        const int drawing_facingcamera = 131072;
#endif
#ifndef drawing_followparentvisibility
        const int drawing_followparentvisibility = 2048;
#endif
#ifndef drawing_itemcolors
        const int drawing_itemcolors = 32;
#endif
#ifndef drawing_itemsizes
        const int drawing_itemsizes = 128;
#endif
#ifndef drawing_itemtransparency
        const int drawing_itemtransparency = 524288;
#endif
#ifndef drawing_lines
        const int drawing_lines = 1;
#endif
#ifndef drawing_linestrip
        const int drawing_linestrip = 8;
#endif
#ifndef drawing_overlay
        const int drawing_overlay = 262144;
#endif
#ifndef drawing_painttag
        const int drawing_painttag = 1024;
#endif
#ifndef drawing_persistent
        const int drawing_persistent = 1048576;
#endif
#ifndef drawing_points
        const int drawing_points = 0;
#endif
#ifndef drawing_quadpoints
        const int drawing_quadpoints = 4;
#endif
#ifndef drawing_quadpts
        const int drawing_quadpts = 10;
#endif
#ifndef drawing_spherepoints
        const int drawing_spherepoints = 7;
#endif
#ifndef drawing_spherepts
        const int drawing_spherepts = 7;
#endif
#ifndef drawing_trianglepoints
        const int drawing_trianglepoints = 3;
#endif
#ifndef drawing_trianglepts
        const int drawing_trianglepts = 9;
#endif
#ifndef drawing_triangles
        const int drawing_triangles = 2;
#endif
#ifndef drawing_vertexcolors
        const int drawing_vertexcolors = 64;
#endif
#ifndef drawing_wireframe
        const int drawing_wireframe = 512;
#endif
#ifndef dummy_linktype_dynamics_force_constraint
        const int dummy_linktype_dynamics_force_constraint = 1;
#endif
#ifndef dummy_linktype_dynamics_loop_closure
        const int dummy_linktype_dynamics_loop_closure = 0;
#endif
#ifndef dummy_linktype_gcs_loop_closure
        const int dummy_linktype_gcs_loop_closure = 2;
#endif
#ifndef dummy_linktype_gcs_target
        const int dummy_linktype_gcs_target = 4;
#endif
#ifndef dummy_linktype_gcs_tip
        const int dummy_linktype_gcs_tip = 3;
#endif
#ifndef dummy_linktype_ik_tip_target
        const int dummy_linktype_ik_tip_target = 5;
#endif
#ifndef dummyfloatparam_follow_path_offset
        const int dummyfloatparam_follow_path_offset = 10002;
#endif
#ifndef dummyfloatparam_size
        const int dummyfloatparam_size = 10003;
#endif
#ifndef dummyintparam_follow_path
        const int dummyintparam_follow_path = 10001;
#endif
#ifndef dummyintparam_link_type
        const int dummyintparam_link_type = 10000;
#endif
#ifndef dynmat_default
        const int dynmat_default = 2310013;
#endif
#ifndef dynmat_floor
        const int dynmat_floor = 2310021;
#endif
#ifndef dynmat_foot
        const int dynmat_foot = 2310018;
#endif
#ifndef dynmat_gripper
        const int dynmat_gripper = 2310020;
#endif
#ifndef dynmat_highfriction
        const int dynmat_highfriction = 2310014;
#endif
#ifndef dynmat_lowfriction
        const int dynmat_lowfriction = 2310015;
#endif
#ifndef dynmat_nofriction
        const int dynmat_nofriction = 2310016;
#endif
#ifndef dynmat_reststackgrasp
        const int dynmat_reststackgrasp = 2310017;
#endif
#ifndef dynmat_wheel
        const int dynmat_wheel = 2310019;
#endif
#ifndef filedlg_type_folder
        const int filedlg_type_folder = 3;
#endif
#ifndef filedlg_type_load
        const int filedlg_type_load = 0;
#endif
#ifndef filedlg_type_load_multiple
        const int filedlg_type_load_multiple = 2;
#endif
#ifndef filedlg_type_save
        const int filedlg_type_save = 1;
#endif
#ifndef filtercomponent_3x3filter
        const int filtercomponent_3x3filter = 18;
#endif
#ifndef filtercomponent_5x5filter
        const int filtercomponent_5x5filter = 19;
#endif
#ifndef filtercomponent_addbuffer1
        const int filtercomponent_addbuffer1 = 10;
#endif
#ifndef filtercomponent_addtobuffer1
        const int filtercomponent_addtobuffer1 = 30;
#endif
#ifndef filtercomponent_binary
        const int filtercomponent_binary = 28;
#endif
#ifndef filtercomponent_blobextraction
        const int filtercomponent_blobextraction = 34;
#endif
#ifndef filtercomponent_circularcut
        const int filtercomponent_circularcut = 23;
#endif
#ifndef filtercomponent_colorsegmentation
        const int filtercomponent_colorsegmentation = 33;
#endif
#ifndef filtercomponent_correlationwithbuffer1
        const int filtercomponent_correlationwithbuffer1 = 32;
#endif
#ifndef filtercomponent_customized
        const int filtercomponent_customized = 1000;
#endif
#ifndef filtercomponent_edge
        const int filtercomponent_edge = 21;
#endif
#ifndef filtercomponent_frombuffer1
        const int filtercomponent_frombuffer1 = 7;
#endif
#ifndef filtercomponent_frombuffer2
        const int filtercomponent_frombuffer2 = 8;
#endif
#ifndef filtercomponent_horizontalflip
        const int filtercomponent_horizontalflip = 13;
#endif
#ifndef filtercomponent_imagetocoord
        const int filtercomponent_imagetocoord = 35;
#endif
#ifndef filtercomponent_intensityscale
        const int filtercomponent_intensityscale = 25;
#endif
#ifndef filtercomponent_keeporremovecolors
        const int filtercomponent_keeporremovecolors = 26;
#endif
#ifndef filtercomponent_multiplywithbuffer1
        const int filtercomponent_multiplywithbuffer1 = 12;
#endif
#ifndef filtercomponent_normalize
        const int filtercomponent_normalize = 24;
#endif
#ifndef filtercomponent_originaldepth
        const int filtercomponent_originaldepth = 2;
#endif
#ifndef filtercomponent_originalimage
        const int filtercomponent_originalimage = 1;
#endif
#ifndef filtercomponent_pixelchange
        const int filtercomponent_pixelchange = 36;
#endif
#ifndef filtercomponent_rectangularcut
        const int filtercomponent_rectangularcut = 22;
#endif
#ifndef filtercomponent_resize
        const int filtercomponent_resize = 17;
#endif
#ifndef filtercomponent_rotate
        const int filtercomponent_rotate = 15;
#endif
#ifndef filtercomponent_scaleandoffsetcolors
        const int filtercomponent_scaleandoffsetcolors = 27;
#endif
#ifndef filtercomponent_sharpen
        const int filtercomponent_sharpen = 20;
#endif
#ifndef filtercomponent_shift
        const int filtercomponent_shift = 16;
#endif
#ifndef filtercomponent_subtractbuffer1
        const int filtercomponent_subtractbuffer1 = 11;
#endif
#ifndef filtercomponent_subtractfrombuffer1
        const int filtercomponent_subtractfrombuffer1 = 31;
#endif
#ifndef filtercomponent_swapbuffers
        const int filtercomponent_swapbuffers = 9;
#endif
#ifndef filtercomponent_swapwithbuffer1
        const int filtercomponent_swapwithbuffer1 = 29;
#endif
#ifndef filtercomponent_tobuffer1
        const int filtercomponent_tobuffer1 = 5;
#endif
#ifndef filtercomponent_tobuffer2
        const int filtercomponent_tobuffer2 = 6;
#endif
#ifndef filtercomponent_todepthoutput
        const int filtercomponent_todepthoutput = 38;
#endif
#ifndef filtercomponent_tooutput
        const int filtercomponent_tooutput = 4;
#endif
#ifndef filtercomponent_uniformimage
        const int filtercomponent_uniformimage = 3;
#endif
#ifndef filtercomponent_velodyne
        const int filtercomponent_velodyne = 37;
#endif
#ifndef filtercomponent_verticalflip
        const int filtercomponent_verticalflip = 14;
#endif
#ifndef floatparam_dynamic_step_size
        const int floatparam_dynamic_step_size = 3;
#endif
#ifndef floatparam_mouse_wheel_zoom_factor
        const int floatparam_mouse_wheel_zoom_factor = 4;
#endif
#ifndef floatparam_rand
        const int floatparam_rand = 0;
#endif
#ifndef floatparam_simulation_time_step
        const int floatparam_simulation_time_step = 1;
#endif
#ifndef floatparam_stereo_distance
        const int floatparam_stereo_distance = 2;
#endif
#ifndef forcefloatparam_error_a
        const int forcefloatparam_error_a = 5003;
#endif
#ifndef forcefloatparam_error_angle
        const int forcefloatparam_error_angle = 5007;
#endif
#ifndef forcefloatparam_error_b
        const int forcefloatparam_error_b = 5004;
#endif
#ifndef forcefloatparam_error_g
        const int forcefloatparam_error_g = 5005;
#endif
#ifndef forcefloatparam_error_pos
        const int forcefloatparam_error_pos = 5006;
#endif
#ifndef forcefloatparam_error_x
        const int forcefloatparam_error_x = 5000;
#endif
#ifndef forcefloatparam_error_y
        const int forcefloatparam_error_y = 5001;
#endif
#ifndef forcefloatparam_error_z
        const int forcefloatparam_error_z = 5002;
#endif
#ifndef graphintparam_needs_refresh
        const int graphintparam_needs_refresh = 10500;
#endif
#ifndef handle_all
        const int handle_all = -2;
#endif
#ifndef handle_all_except_explicit
        const int handle_all_except_explicit = -3;
#endif
#ifndef handle_all_except_self
        const int handle_all_except_self = -10;
#endif
#ifndef handle_app
        const int handle_app = -13;
#endif
#ifndef handle_chain
        const int handle_chain = -7;
#endif
#ifndef handle_default
        const int handle_default = -9;
#endif
#ifndef handle_inverse
        const int handle_inverse = -14;
#endif
#ifndef handle_main_script
        const int handle_main_script = -5;
#endif
#ifndef handle_parent
        const int handle_parent = -11;
#endif
#ifndef handle_scene
        const int handle_scene = -12;
#endif
#ifndef handle_self
        const int handle_self = -4;
#endif
#ifndef handle_single
        const int handle_single = -8;
#endif
#ifndef handle_tree
        const int handle_tree = -6;
#endif
#ifndef handle_world
        const int handle_world = -1;
#endif
#ifndef handleflag_abscoords
        const int handleflag_abscoords = 8388608;
#endif
#ifndef handleflag_altname
        const int handleflag_altname = 4194304;
#endif
#ifndef handleflag_assembly
        const int handleflag_assembly = 4194304;
#endif
#ifndef handleflag_axis
        const int handleflag_axis = 4194304;
#endif
#ifndef handleflag_camera
        const int handleflag_camera = 4194304;
#endif
#ifndef handleflag_codedstring
        const int handleflag_codedstring = 4194304;
#endif
#ifndef handleflag_depthbuffer
        const int handleflag_depthbuffer = 8388608;
#endif
#ifndef handleflag_depthbuffermeters
        const int handleflag_depthbuffermeters = 8388608;
#endif
#ifndef handleflag_extended
        const int handleflag_extended = 4194304;
#endif
#ifndef handleflag_greyscale
        const int handleflag_greyscale = 4194304;
#endif
#ifndef handleflag_keeporiginal
        const int handleflag_keeporiginal = 4194304;
#endif
#ifndef handleflag_model
        const int handleflag_model = 8388608;
#endif
#ifndef handleflag_rawvalue
        const int handleflag_rawvalue = 16777216;
#endif
#ifndef handleflag_reljointbaseframe
        const int handleflag_reljointbaseframe = 4194304;
#endif
#ifndef handleflag_resetforce
        const int handleflag_resetforce = 4194304;
#endif
#ifndef handleflag_resetforcetorque
        const int handleflag_resetforcetorque = 12582912;
#endif
#ifndef handleflag_resettorque
        const int handleflag_resettorque = 8388608;
#endif
#ifndef handleflag_setmultiple
        const int handleflag_setmultiple = 4194304;
#endif
#ifndef handleflag_silenterror
        const int handleflag_silenterror = 33554432;
#endif
#ifndef handleflag_togglevisibility
        const int handleflag_togglevisibility = 4194304;
#endif
#ifndef handleflag_wxyzquaternion
        const int handleflag_wxyzquaternion = 4194304;
#endif
#ifndef ik_alpha_beta_constraint
        const int ik_alpha_beta_constraint = 8;
#endif
#ifndef ik_damped_least_squares_method
        const int ik_damped_least_squares_method = 1;
#endif
#ifndef ik_gamma_constraint
        const int ik_gamma_constraint = 16;
#endif
#ifndef ik_jacobian_transpose_method
        const int ik_jacobian_transpose_method = 2;
#endif
#ifndef ik_pseudo_inverse_method
        const int ik_pseudo_inverse_method = 0;
#endif
#ifndef ik_undamped_pseudo_inverse_method
        const int ik_undamped_pseudo_inverse_method = 3;
#endif
#ifndef ik_x_constraint
        const int ik_x_constraint = 1;
#endif
#ifndef ik_y_constraint
        const int ik_y_constraint = 2;
#endif
#ifndef ik_z_constraint
        const int ik_z_constraint = 4;
#endif
#ifndef ikresult_fail
        const int ikresult_fail = 2;
#endif
#ifndef ikresult_not_performed
        const int ikresult_not_performed = 0;
#endif
#ifndef ikresult_success
        const int ikresult_success = 1;
#endif
#ifndef imgcomb_horizontal
        const int imgcomb_horizontal = 1;
#endif
#ifndef imgcomb_vertical
        const int imgcomb_vertical = 0;
#endif
#ifndef intparam_compilation_version
        const int intparam_compilation_version = 4;
#endif
#ifndef intparam_core_count
        const int intparam_core_count = 24;
#endif
#ifndef intparam_current_page
        const int intparam_current_page = 5;
#endif
#ifndef intparam_dlgverbosity
        const int intparam_dlgverbosity = 42;
#endif
#ifndef intparam_dynamic_engine
        const int intparam_dynamic_engine = 8;
#endif
#ifndef intparam_dynamic_iteration_count
        const int intparam_dynamic_iteration_count = 37;
#endif
#ifndef intparam_dynamic_step_divider
        const int intparam_dynamic_step_divider = 7;
#endif
#ifndef intparam_dynamic_warning_disabled_mask
        const int intparam_dynamic_warning_disabled_mask = 32;
#endif
#ifndef intparam_edit_mode_type
        const int intparam_edit_mode_type = 14;
#endif
#ifndef intparam_error_report_mode
        const int intparam_error_report_mode = 0;
#endif
#ifndef intparam_exitcode
        const int intparam_exitcode = 44;
#endif
#ifndef intparam_flymode_camera_handle
        const int intparam_flymode_camera_handle = 6;
#endif
#ifndef intparam_idle_fps
        const int intparam_idle_fps = 26;
#endif
#ifndef intparam_infotext_style
        const int intparam_infotext_style = 12;
#endif
#ifndef intparam_motionplanning_seed
        const int intparam_motionplanning_seed = 35;
#endif
#ifndef intparam_mouse_buttons
        const int intparam_mouse_buttons = 31;
#endif
#ifndef intparam_mouse_x
        const int intparam_mouse_x = 22;
#endif
#ifndef intparam_mouse_y
        const int intparam_mouse_y = 23;
#endif
#ifndef intparam_platform
        const int intparam_platform = 19;
#endif
#ifndef intparam_program_full_version
        const int intparam_program_full_version = 39;
#endif
#ifndef intparam_program_revision
        const int intparam_program_revision = 30;
#endif
#ifndef intparam_program_version
        const int intparam_program_version = 1;
#endif
#ifndef intparam_prox_sensor_select_down
        const int intparam_prox_sensor_select_down = 27;
#endif
#ifndef intparam_prox_sensor_select_up
        const int intparam_prox_sensor_select_up = 28;
#endif
#ifndef intparam_qt_version
        const int intparam_qt_version = 16;
#endif
#ifndef intparam_scene_index
        const int intparam_scene_index = 34;
#endif
#ifndef intparam_scene_unique_id
        const int intparam_scene_unique_id = 20;
#endif
#ifndef intparam_server_port_next
        const int intparam_server_port_next = 15;
#endif
#ifndef intparam_server_port_range
        const int intparam_server_port_range = 10;
#endif
#ifndef intparam_server_port_start
        const int intparam_server_port_start = 9;
#endif
#ifndef intparam_settings
        const int intparam_settings = 13;
#endif
#ifndef intparam_simulation_warning_disabled_mask
        const int intparam_simulation_warning_disabled_mask = 33;
#endif
#ifndef intparam_speedmodifier
        const int intparam_speedmodifier = 36;
#endif
#ifndef intparam_statusbarverbosity
        const int intparam_statusbarverbosity = 41;
#endif
#ifndef intparam_stop_request_counter
        const int intparam_stop_request_counter = 29;
#endif
#ifndef intparam_verbosity
        const int intparam_verbosity = 40;
#endif
#ifndef intparam_videoencoderindex
        const int intparam_videoencoderindex = 43;
#endif
#ifndef intparam_visible_layers
        const int intparam_visible_layers = 11;
#endif
#ifndef intparam_work_thread_calc_time_ms
        const int intparam_work_thread_calc_time_ms = 25;
#endif
#ifndef intparam_work_thread_count
        const int intparam_work_thread_count = 21;
#endif
#ifndef joint_prismatic_subtype
        const int joint_prismatic_subtype = 11;
#endif
#ifndef joint_revolute_subtype
        const int joint_revolute_subtype = 10;
#endif
#ifndef joint_spherical_subtype
        const int joint_spherical_subtype = 12;
#endif
#ifndef jointfloatparam_error_a
        const int jointfloatparam_error_a = 2025;
#endif
#ifndef jointfloatparam_error_angle
        const int jointfloatparam_error_angle = 2029;
#endif
#ifndef jointfloatparam_error_b
        const int jointfloatparam_error_b = 2026;
#endif
#ifndef jointfloatparam_error_g
        const int jointfloatparam_error_g = 2027;
#endif
#ifndef jointfloatparam_error_pos
        const int jointfloatparam_error_pos = 2028;
#endif
#ifndef jointfloatparam_error_x
        const int jointfloatparam_error_x = 2022;
#endif
#ifndef jointfloatparam_error_y
        const int jointfloatparam_error_y = 2023;
#endif
#ifndef jointfloatparam_error_z
        const int jointfloatparam_error_z = 2024;
#endif
#ifndef jointfloatparam_ik_weight
        const int jointfloatparam_ik_weight = 2021;
#endif
#ifndef jointfloatparam_intrinsic_qw
        const int jointfloatparam_intrinsic_qw = 2011;
#endif
#ifndef jointfloatparam_intrinsic_qx
        const int jointfloatparam_intrinsic_qx = 2008;
#endif
#ifndef jointfloatparam_intrinsic_qy
        const int jointfloatparam_intrinsic_qy = 2009;
#endif
#ifndef jointfloatparam_intrinsic_qz
        const int jointfloatparam_intrinsic_qz = 2010;
#endif
#ifndef jointfloatparam_intrinsic_x
        const int jointfloatparam_intrinsic_x = 2005;
#endif
#ifndef jointfloatparam_intrinsic_y
        const int jointfloatparam_intrinsic_y = 2006;
#endif
#ifndef jointfloatparam_intrinsic_z
        const int jointfloatparam_intrinsic_z = 2007;
#endif
#ifndef jointfloatparam_kc_c
        const int jointfloatparam_kc_c = 2019;
#endif
#ifndef jointfloatparam_kc_k
        const int jointfloatparam_kc_k = 2018;
#endif
#ifndef jointfloatparam_pid_d
        const int jointfloatparam_pid_d = 2004;
#endif
#ifndef jointfloatparam_pid_i
        const int jointfloatparam_pid_i = 2003;
#endif
#ifndef jointfloatparam_pid_p
        const int jointfloatparam_pid_p = 2002;
#endif
#ifndef jointfloatparam_screw_pitch
        const int jointfloatparam_screw_pitch = 2034;
#endif
#ifndef jointfloatparam_spherical_qw
        const int jointfloatparam_spherical_qw = 2016;
#endif
#ifndef jointfloatparam_spherical_qx
        const int jointfloatparam_spherical_qx = 2013;
#endif
#ifndef jointfloatparam_spherical_qy
        const int jointfloatparam_spherical_qy = 2014;
#endif
#ifndef jointfloatparam_spherical_qz
        const int jointfloatparam_spherical_qz = 2015;
#endif
#ifndef jointfloatparam_step_size
        const int jointfloatparam_step_size = 2035;
#endif
#ifndef jointfloatparam_upper_limit
        const int jointfloatparam_upper_limit = 2017;
#endif
#ifndef jointfloatparam_velocity
        const int jointfloatparam_velocity = 2012;
#endif
#ifndef jointfloatparam_vortex_dep_multiplication
        const int jointfloatparam_vortex_dep_multiplication = 2032;
#endif
#ifndef jointfloatparam_vortex_dep_offset
        const int jointfloatparam_vortex_dep_offset = 2033;
#endif
#ifndef jointintparam_ctrl_enabled
        const int jointintparam_ctrl_enabled = 2001;
#endif
#ifndef jointintparam_motor_enabled
        const int jointintparam_motor_enabled = 2000;
#endif
#ifndef jointintparam_velocity_lock
        const int jointintparam_velocity_lock = 2030;
#endif
#ifndef jointintparam_vortex_dep_handle
        const int jointintparam_vortex_dep_handle = 2031;
#endif
#ifndef jointmode_dependent
        const int jointmode_dependent = 4;
#endif
#ifndef jointmode_force
        const int jointmode_force = 5;
#endif
#ifndef jointmode_ik
        const int jointmode_ik = 2;
#endif
#ifndef jointmode_ikdependent
        const int jointmode_ikdependent = 3;
#endif
#ifndef jointmode_passive
        const int jointmode_passive = 0;
#endif
#ifndef light_directional_subtype
        const int light_directional_subtype = 3;
#endif
#ifndef light_omnidirectional_subtype
        const int light_omnidirectional_subtype = 1;
#endif
#ifndef light_spot_subtype
        const int light_spot_subtype = 2;
#endif
#ifndef lightfloatparam_const_attenuation
        const int lightfloatparam_const_attenuation = 8003;
#endif
#ifndef lightfloatparam_lin_attenuation
        const int lightfloatparam_lin_attenuation = 8004;
#endif
#ifndef lightfloatparam_quad_attenuation
        const int lightfloatparam_quad_attenuation = 8005;
#endif
#ifndef lightfloatparam_spot_cutoff
        const int lightfloatparam_spot_cutoff = 8002;
#endif
#ifndef lightfloatparam_spot_exponent
        const int lightfloatparam_spot_exponent = 8001;
#endif
#ifndef lightintparam_pov_casts_shadows
        const int lightintparam_pov_casts_shadows = 8000;
#endif
#ifndef mainscriptcall_cleanup
        const int mainscriptcall_cleanup = 1;
#endif
#ifndef mainscriptcall_initialization
        const int mainscriptcall_initialization = 2;
#endif
#ifndef mainscriptcall_regular
        const int mainscriptcall_regular = 6;
#endif
#ifndef message_bannerclicked
        const int message_bannerclicked = 7;
#endif
#ifndef message_keypress
        const int message_keypress = 6;
#endif
#ifndef message_model_loaded
        const int message_model_loaded = 4;
#endif
#ifndef message_object_selection_changed
        const int message_object_selection_changed = 2;
#endif
#ifndef message_pick_select_down
        const int message_pick_select_down = 11;
#endif
#ifndef message_prox_sensor_select_down
        const int message_prox_sensor_select_down = 9;
#endif
#ifndef message_prox_sensor_select_up
        const int message_prox_sensor_select_up = 10;
#endif
#ifndef message_scene_loaded
        const int message_scene_loaded = 8;
#endif
#ifndef message_ui_button_state_change
        const int message_ui_button_state_change = 0;
#endif
#ifndef mill_cone_subtype
        const int mill_cone_subtype = 43;
#endif
#ifndef mill_cylinder_subtype
        const int mill_cylinder_subtype = 41;
#endif
#ifndef mill_disc_subtype
        const int mill_disc_subtype = 42;
#endif
#ifndef mill_pyramid_subtype
        const int mill_pyramid_subtype = 40;
#endif
#ifndef millintparam_volume_type
        const int millintparam_volume_type = 11000;
#endif
#ifndef mirrorfloatparam_height
        const int mirrorfloatparam_height = 12001;
#endif
#ifndef mirrorfloatparam_reflectance
        const int mirrorfloatparam_reflectance = 12002;
#endif
#ifndef mirrorfloatparam_width
        const int mirrorfloatparam_width = 12000;
#endif
#ifndef mirrorintparam_enable
        const int mirrorintparam_enable = 12003;
#endif
#ifndef modelproperty_not_collidable
        const int modelproperty_not_collidable = 1;
#endif
#ifndef modelproperty_not_detectable
        const int modelproperty_not_detectable = 8;
#endif
#ifndef modelproperty_not_dynamic
        const int modelproperty_not_dynamic = 32;
#endif
#ifndef modelproperty_not_measurable
        const int modelproperty_not_measurable = 2;
#endif
#ifndef modelproperty_not_model
        const int modelproperty_not_model = 61440;
#endif
#ifndef modelproperty_not_renderable
        const int modelproperty_not_renderable = 4;
#endif
#ifndef modelproperty_not_reset
        const int modelproperty_not_reset = 128;
#endif
#ifndef modelproperty_not_respondable
        const int modelproperty_not_respondable = 64;
#endif
#ifndef modelproperty_not_showasinsidemodel
        const int modelproperty_not_showasinsidemodel = 1024;
#endif
#ifndef modelproperty_not_visible
        const int modelproperty_not_visible = 256;
#endif
#ifndef modelproperty_scripts_inactive
        const int modelproperty_scripts_inactive = 512;
#endif
#ifndef moduleinfo_builddatestr
        const int moduleinfo_builddatestr = 1;
#endif
#ifndef moduleinfo_extversionint
        const int moduleinfo_extversionint = 2;
#endif
#ifndef moduleinfo_extversionstr
        const int moduleinfo_extversionstr = 0;
#endif
#ifndef moduleinfo_statusbarverbosity
        const int moduleinfo_statusbarverbosity = 4;
#endif
#ifndef moduleinfo_verbosity
        const int moduleinfo_verbosity = 3;
#endif
#ifndef msgbox_buttons_ok
        const int msgbox_buttons_ok = 0;
#endif
#ifndef msgbox_buttons_okcancel
        const int msgbox_buttons_okcancel = 3;
#endif
#ifndef msgbox_buttons_yesno
        const int msgbox_buttons_yesno = 1;
#endif
#ifndef msgbox_buttons_yesnocancel
        const int msgbox_buttons_yesnocancel = 2;
#endif
#ifndef msgbox_return_cancel
        const int msgbox_return_cancel = 0;
#endif
#ifndef msgbox_return_error
        const int msgbox_return_error = 4;
#endif
#ifndef msgbox_return_no
        const int msgbox_return_no = 1;
#endif
#ifndef msgbox_return_ok
        const int msgbox_return_ok = 3;
#endif
#ifndef msgbox_return_yes
        const int msgbox_return_yes = 2;
#endif
#ifndef msgbox_type_critical
        const int msgbox_type_critical = 3;
#endif
#ifndef msgbox_type_info
        const int msgbox_type_info = 0;
#endif
#ifndef msgbox_type_question
        const int msgbox_type_question = 1;
#endif
#ifndef msgbox_type_warning
        const int msgbox_type_warning = 2;
#endif
#ifndef navigation_cameraangle
        const int navigation_cameraangle = 5;
#endif
#ifndef navigation_camerafly
        const int navigation_camerafly = 6;
#endif
#ifndef navigation_camerarotate
        const int navigation_camerarotate = 2;
#endif
#ifndef navigation_camerarotatemiddlebutton
        const int navigation_camerarotatemiddlebutton = 8192;
#endif
#ifndef navigation_camerarotaterightbutton
        const int navigation_camerarotaterightbutton = 8192;
#endif
#ifndef navigation_camerashift
        const int navigation_camerashift = 1;
#endif
#ifndef navigation_cameratilt
        const int navigation_cameratilt = 4;
#endif
#ifndef navigation_camerazoom
        const int navigation_camerazoom = 3;
#endif
#ifndef navigation_camerazoomwheel
        const int navigation_camerazoomwheel = 4096;
#endif
#ifndef navigation_clickselection
        const int navigation_clickselection = 512;
#endif
#ifndef navigation_createpathpoint
        const int navigation_createpathpoint = 256;
#endif
#ifndef navigation_ctrlselection
        const int navigation_ctrlselection = 1024;
#endif
#ifndef navigation_objectrotate
        const int navigation_objectrotate = 8;
#endif
#ifndef navigation_objectshift
        const int navigation_objectshift = 7;
#endif
#ifndef navigation_passive
        const int navigation_passive = 0;
#endif
#ifndef navigation_shiftselection
        const int navigation_shiftselection = 2048;
#endif
#ifndef newton_body_angulardrag
        const int newton_body_angulardrag = 33005;
#endif
#ifndef newton_body_bitcoded
        const int newton_body_bitcoded = 34001;
#endif
#ifndef newton_body_fastmoving
        const int newton_body_fastmoving = 35001;
#endif
#ifndef newton_body_kineticfriction
        const int newton_body_kineticfriction = 33002;
#endif
#ifndef newton_body_lineardrag
        const int newton_body_lineardrag = 33004;
#endif
#ifndef newton_body_restitution
        const int newton_body_restitution = 33003;
#endif
#ifndef newton_body_staticfriction
        const int newton_body_staticfriction = 33001;
#endif
#ifndef newton_global_bitcoded
        const int newton_global_bitcoded = 28002;
#endif
#ifndef newton_global_constraintsolvingiterations
        const int newton_global_constraintsolvingiterations = 28001;
#endif
#ifndef newton_global_contactmergetolerance
        const int newton_global_contactmergetolerance = 27002;
#endif
#ifndef newton_global_exactsolver
        const int newton_global_exactsolver = 29002;
#endif
#ifndef newton_global_highjointaccuracy
        const int newton_global_highjointaccuracy = 29003;
#endif
#ifndef newton_global_multithreading
        const int newton_global_multithreading = 29001;
#endif
#ifndef newton_global_stepsize
        const int newton_global_stepsize = 27001;
#endif
#ifndef newton_joint_dependencyfactor
        const int newton_joint_dependencyfactor = 30001;
#endif
#ifndef newton_joint_dependencyoffset
        const int newton_joint_dependencyoffset = 30002;
#endif
#ifndef newton_joint_dependentobjectid
        const int newton_joint_dependentobjectid = 31002;
#endif
#ifndef newton_joint_objectid
        const int newton_joint_objectid = 31001;
#endif
#ifndef object_camera_type
        const int object_camera_type = 3;
#endif
#ifndef object_dummy_type
        const int object_dummy_type = 4;
#endif
#ifndef object_forcesensor_type
        const int object_forcesensor_type = 12;
#endif
#ifndef object_graph_type
        const int object_graph_type = 2;
#endif
#ifndef object_joint_type
        const int object_joint_type = 1;
#endif
#ifndef object_light_type
        const int object_light_type = 13;
#endif
#ifndef object_mill_type
        const int object_mill_type = 11;
#endif
#ifndef object_mirror_type
        const int object_mirror_type = 14;
#endif
#ifndef object_no_subtype
        const int object_no_subtype = 200;
#endif
#ifndef object_octree_type
        const int object_octree_type = 15;
#endif
#ifndef object_path_type
        const int object_path_type = 8;
#endif
#ifndef object_pointcloud_type
        const int object_pointcloud_type = 16;
#endif
#ifndef object_proximitysensor_type
        const int object_proximitysensor_type = 5;
#endif
#ifndef object_renderingsensor_type
        const int object_renderingsensor_type = 9;
#endif
#ifndef object_shape_type
        const int object_shape_type = 0;
#endif
#ifndef object_visionsensor_type
        const int object_visionsensor_type = 9;
#endif
#ifndef objectproperty_cannotdelete
        const int objectproperty_cannotdelete = 8192;
#endif
#ifndef objectproperty_cannotdeleteduringsim
        const int objectproperty_cannotdeleteduringsim = 16384;
#endif
#ifndef objectproperty_canupdatedna
        const int objectproperty_canupdatedna = 1024;
#endif
#ifndef objectproperty_collapsed
        const int objectproperty_collapsed = 16;
#endif
#ifndef objectproperty_depthinvisible
        const int objectproperty_depthinvisible = 4096;
#endif
#ifndef objectproperty_dontshowasinsidemodel
        const int objectproperty_dontshowasinsidemodel = 256;
#endif
#ifndef objectproperty_hierarchyhiddenmodelchild
        const int objectproperty_hierarchyhiddenmodelchild = 32768;
#endif
#ifndef objectproperty_selectable
        const int objectproperty_selectable = 32;
#endif
#ifndef objectproperty_selectinvisible
        const int objectproperty_selectinvisible = 2048;
#endif
#ifndef objectproperty_selectmodelbaseinstead
        const int objectproperty_selectmodelbaseinstead = 128;
#endif
#ifndef objectspecialproperty_collidable
        const int objectspecialproperty_collidable = 1;
#endif
#ifndef objectspecialproperty_detectable
        const int objectspecialproperty_detectable = 496;
#endif
#ifndef objectspecialproperty_detectable_all
        const int objectspecialproperty_detectable_all = 496;
#endif
#ifndef objectspecialproperty_detectable_capacitive
        const int objectspecialproperty_detectable_capacitive = 256;
#endif
#ifndef objectspecialproperty_detectable_inductive
        const int objectspecialproperty_detectable_inductive = 128;
#endif
#ifndef objectspecialproperty_detectable_infrared
        const int objectspecialproperty_detectable_infrared = 32;
#endif
#ifndef objectspecialproperty_detectable_laser
        const int objectspecialproperty_detectable_laser = 64;
#endif
#ifndef objectspecialproperty_detectable_ultrasonic
        const int objectspecialproperty_detectable_ultrasonic = 16;
#endif
#ifndef objectspecialproperty_measurable
        const int objectspecialproperty_measurable = 2;
#endif
#ifndef objectspecialproperty_pathplanning_ignored
        const int objectspecialproperty_pathplanning_ignored = 2048;
#endif
#ifndef objectspecialproperty_renderable
        const int objectspecialproperty_renderable = 512;
#endif
#ifndef objfloatparam_abs_rot_velocity
        const int objfloatparam_abs_rot_velocity = 14;
#endif
#ifndef objfloatparam_abs_x_velocity
        const int objfloatparam_abs_x_velocity = 11;
#endif
#ifndef objfloatparam_abs_y_velocity
        const int objfloatparam_abs_y_velocity = 12;
#endif
#ifndef objfloatparam_abs_z_velocity
        const int objfloatparam_abs_z_velocity = 13;
#endif
#ifndef objfloatparam_modelbbox_max_x
        const int objfloatparam_modelbbox_max_x = 24;
#endif
#ifndef objfloatparam_modelbbox_max_y
        const int objfloatparam_modelbbox_max_y = 25;
#endif
#ifndef objfloatparam_modelbbox_max_z
        const int objfloatparam_modelbbox_max_z = 26;
#endif
#ifndef objfloatparam_modelbbox_min_x
        const int objfloatparam_modelbbox_min_x = 21;
#endif
#ifndef objfloatparam_modelbbox_min_y
        const int objfloatparam_modelbbox_min_y = 22;
#endif
#ifndef objfloatparam_modelbbox_min_z
        const int objfloatparam_modelbbox_min_z = 23;
#endif
#ifndef objfloatparam_objbbox_max_x
        const int objfloatparam_objbbox_max_x = 18;
#endif
#ifndef objfloatparam_objbbox_max_y
        const int objfloatparam_objbbox_max_y = 19;
#endif
#ifndef objfloatparam_objbbox_max_z
        const int objfloatparam_objbbox_max_z = 20;
#endif
#ifndef objfloatparam_objbbox_min_x
        const int objfloatparam_objbbox_min_x = 15;
#endif
#ifndef objfloatparam_objbbox_min_y
        const int objfloatparam_objbbox_min_y = 16;
#endif
#ifndef objfloatparam_objbbox_min_z
        const int objfloatparam_objbbox_min_z = 17;
#endif
#ifndef objfloatparam_size_factor
        const int objfloatparam_size_factor = 34;
#endif
#ifndef objfloatparam_transparency_offset
        const int objfloatparam_transparency_offset = 28;
#endif
#ifndef objintparam_child_role
        const int objintparam_child_role = 29;
#endif
#ifndef objintparam_collection_self_collision_indicator
        const int objintparam_collection_self_collision_indicator = 27;
#endif
#ifndef objintparam_illumination_handle
        const int objintparam_illumination_handle = 32;
#endif
#ifndef objintparam_manipulation_permissions
        const int objintparam_manipulation_permissions = 31;
#endif
#ifndef objintparam_parent_role
        const int objintparam_parent_role = 30;
#endif
#ifndef objintparam_unique_id
        const int objintparam_unique_id = 37;
#endif
#ifndef objintparam_visibility_layer
        const int objintparam_visibility_layer = 10;
#endif
#ifndef objintparam_visible
        const int objintparam_visible = 36;
#endif
#ifndef objstringparam_dna
        const int objstringparam_dna = 33;
#endif
#ifndef objstringparam_unique_id
        const int objstringparam_unique_id = 35;
#endif
#ifndef ode_body_angulardamping
        const int ode_body_angulardamping = 15005;
#endif
#ifndef ode_body_friction
        const int ode_body_friction = 15001;
#endif
#ifndef ode_body_lineardamping
        const int ode_body_lineardamping = 15004;
#endif
#ifndef ode_body_maxcontacts
        const int ode_body_maxcontacts = 16001;
#endif
#ifndef ode_body_softcfm
        const int ode_body_softcfm = 15003;
#endif
#ifndef ode_body_softerp
        const int ode_body_softerp = 15002;
#endif
#ifndef ode_global_bitcoded
        const int ode_global_bitcoded = 10002;
#endif
#ifndef ode_global_cfm
        const int ode_global_cfm = 9003;
#endif
#ifndef ode_global_constraintsolvingiterations
        const int ode_global_constraintsolvingiterations = 10001;
#endif
#ifndef ode_global_erp
        const int ode_global_erp = 9004;
#endif
#ifndef ode_global_fullinternalscaling
        const int ode_global_fullinternalscaling = 11001;
#endif
#ifndef ode_global_internalscalingfactor
        const int ode_global_internalscalingfactor = 9002;
#endif
#ifndef ode_global_quickstep
        const int ode_global_quickstep = 11002;
#endif
#ifndef ode_global_randomseed
        const int ode_global_randomseed = 10003;
#endif
#ifndef ode_global_stepsize
        const int ode_global_stepsize = 9001;
#endif
#ifndef ode_joint_bounce
        const int ode_joint_bounce = 12003;
#endif
#ifndef ode_joint_fudgefactor
        const int ode_joint_fudgefactor = 12004;
#endif
#ifndef ode_joint_normalcfm
        const int ode_joint_normalcfm = 12005;
#endif
#ifndef ode_joint_stopcfm
        const int ode_joint_stopcfm = 12002;
#endif
#ifndef ode_joint_stoperp
        const int ode_joint_stoperp = 12001;
#endif
#ifndef particle_cyclic
        const int particle_cyclic = 8192;
#endif
#ifndef particle_emissioncolor
        const int particle_emissioncolor = 16384;
#endif
#ifndef particle_ignoresgravity
        const int particle_ignoresgravity = 256;
#endif
#ifndef particle_invisible
        const int particle_invisible = 512;
#endif
#ifndef particle_itemcolors
        const int particle_itemcolors = 4096;
#endif
#ifndef particle_itemdensities
        const int particle_itemdensities = 2048;
#endif
#ifndef particle_itemsizes
        const int particle_itemsizes = 1024;
#endif
#ifndef particle_painttag
        const int particle_painttag = 65536;
#endif
#ifndef particle_particlerespondable
        const int particle_particlerespondable = 128;
#endif
#ifndef particle_points1
        const int particle_points1 = 0;
#endif
#ifndef particle_points2
        const int particle_points2 = 1;
#endif
#ifndef particle_points4
        const int particle_points4 = 2;
#endif
#ifndef particle_respondable1to4
        const int particle_respondable1to4 = 32;
#endif
#ifndef particle_respondable5to8
        const int particle_respondable5to8 = 64;
#endif
#ifndef particle_roughspheres
        const int particle_roughspheres = 3;
#endif
#ifndef particle_spheres
        const int particle_spheres = 4;
#endif
#ifndef particle_water
        const int particle_water = 32768;
#endif
#ifndef pathproperty_automatic_orientation
        const int pathproperty_automatic_orientation = 8;
#endif
#ifndef pathproperty_closed_path
        const int pathproperty_closed_path = 4;
#endif
#ifndef pathproperty_flat_path
        const int pathproperty_flat_path = 64;
#endif
#ifndef pathproperty_keep_x_up
        const int pathproperty_keep_x_up = 2048;
#endif
#ifndef pathproperty_show_line
        const int pathproperty_show_line = 1;
#endif
#ifndef pathproperty_show_orientation
        const int pathproperty_show_orientation = 2;
#endif
#ifndef pathproperty_show_position
        const int pathproperty_show_position = 128;
#endif
#ifndef physics_bullet
        const int physics_bullet = 0;
#endif
#ifndef physics_mujoco
        const int physics_mujoco = 4;
#endif
#ifndef physics_newton
        const int physics_newton = 3;
#endif
#ifndef physics_ode
        const int physics_ode = 1;
#endif
#ifndef physics_vortex
        const int physics_vortex = 2;
#endif
#ifndef proximitysensor_cone_subtype
        const int proximitysensor_cone_subtype = 33;
#endif
#ifndef proximitysensor_cylinder_subtype
        const int proximitysensor_cylinder_subtype = 31;
#endif
#ifndef proximitysensor_disc_subtype
        const int proximitysensor_disc_subtype = 32;
#endif
#ifndef proximitysensor_pyramid_subtype
        const int proximitysensor_pyramid_subtype = 30;
#endif
#ifndef proximitysensor_ray_subtype
        const int proximitysensor_ray_subtype = 34;
#endif
#ifndef proxintparam_entity_to_detect
        const int proxintparam_entity_to_detect = 4002;
#endif
#ifndef proxintparam_ray_invisibility
        const int proxintparam_ray_invisibility = 4000;
#endif
#ifndef proxintparam_volume_type
        const int proxintparam_volume_type = 4001;
#endif
#ifndef pure_primitive_cone
        const int pure_primitive_cone = 6;
#endif
#ifndef pure_primitive_cuboid
        const int pure_primitive_cuboid = 3;
#endif
#ifndef pure_primitive_cylinder
        const int pure_primitive_cylinder = 5;
#endif
#ifndef pure_primitive_disc
        const int pure_primitive_disc = 2;
#endif
#ifndef pure_primitive_heightfield
        const int pure_primitive_heightfield = 7;
#endif
#ifndef pure_primitive_none
        const int pure_primitive_none = 0;
#endif
#ifndef pure_primitive_plane
        const int pure_primitive_plane = 1;
#endif
#ifndef pure_primitive_spheroid
        const int pure_primitive_spheroid = 4;
#endif
#ifndef rml_disable_extremum_motion_states_calc
        const int rml_disable_extremum_motion_states_calc = 8;
#endif
#ifndef rml_keep_current_vel_if_fallback_strategy
        const int rml_keep_current_vel_if_fallback_strategy = 16;
#endif
#ifndef rml_keep_target_vel
        const int rml_keep_target_vel = 0;
#endif
#ifndef rml_no_sync
        const int rml_no_sync = 3;
#endif
#ifndef rml_only_phase_sync
        const int rml_only_phase_sync = 2;
#endif
#ifndef rml_only_time_sync
        const int rml_only_time_sync = 1;
#endif
#ifndef rml_phase_sync_if_possible
        const int rml_phase_sync_if_possible = 0;
#endif
#ifndef rml_recompute_trajectory
        const int rml_recompute_trajectory = 4;
#endif
#ifndef ruckig_minaccel
        const int ruckig_minaccel = 512;
#endif
#ifndef ruckig_minvel
        const int ruckig_minvel = 256;
#endif
#ifndef ruckig_nosync
        const int ruckig_nosync = 3;
#endif
#ifndef ruckig_phasesync
        const int ruckig_phasesync = 0;
#endif
#ifndef ruckig_timesync
        const int ruckig_timesync = 1;
#endif
#ifndef script_call_error
        const int script_call_error = 16;
#endif
#ifndef script_lua_error
        const int script_lua_error = 8;
#endif
#ifndef script_main_not_called
        const int script_main_not_called = 2;
#endif
#ifndef script_main_script_nonexistent
        const int script_main_script_nonexistent = 1;
#endif
#ifndef script_no_error
        const int script_no_error = 0;
#endif
#ifndef script_reentrance_error
        const int script_reentrance_error = 4;
#endif
#ifndef scriptattribute_debuglevel
        const int scriptattribute_debuglevel = 6;
#endif
#ifndef scriptattribute_enabled
        const int scriptattribute_enabled = 4;
#endif
#ifndef scriptattribute_executioncount
        const int scriptattribute_executioncount = 2;
#endif
#ifndef scriptattribute_executionorder
        const int scriptattribute_executionorder = 1;
#endif
#ifndef scriptattribute_scripthandle
        const int scriptattribute_scripthandle = 8;
#endif
#ifndef scriptattribute_scripttype
        const int scriptattribute_scripttype = 7;
#endif
#ifndef scriptdebug_allcalls
        const int scriptdebug_allcalls = 3;
#endif
#ifndef scriptdebug_callsandvars
        const int scriptdebug_callsandvars = 5;
#endif
#ifndef scriptdebug_none
        const int scriptdebug_none = 0;
#endif
#ifndef scriptdebug_syscalls
        const int scriptdebug_syscalls = 1;
#endif
#ifndef scriptdebug_vars
        const int scriptdebug_vars = 4;
#endif
#ifndef scriptdebug_vars_interval
        const int scriptdebug_vars_interval = 2;
#endif
#ifndef scriptexecorder_first
        const int scriptexecorder_first = 0;
#endif
#ifndef scriptexecorder_last
        const int scriptexecorder_last = 2;
#endif
#ifndef scriptexecorder_normal
        const int scriptexecorder_normal = 1;
#endif
#ifndef scriptintparam_enabled
        const int scriptintparam_enabled = 4;
#endif
#ifndef scriptintparam_execcount
        const int scriptintparam_execcount = 1;
#endif
#ifndef scriptintparam_execorder
        const int scriptintparam_execorder = 0;
#endif
#ifndef scriptintparam_handle
        const int scriptintparam_handle = 3;
#endif
#ifndef scriptintparam_objecthandle
        const int scriptintparam_objecthandle = 5;
#endif
#ifndef scriptintparam_type
        const int scriptintparam_type = 2;
#endif
#ifndef scriptstringparam_description
        const int scriptstringparam_description = 0;
#endif
#ifndef scriptstringparam_name
        const int scriptstringparam_name = 1;
#endif
#ifndef scriptstringparam_text
        const int scriptstringparam_text = 2;
#endif
#ifndef scriptthreadresume_actuation_first
        const int scriptthreadresume_actuation_first = 1;
#endif
#ifndef scriptthreadresume_actuation_last
        const int scriptthreadresume_actuation_last = 2;
#endif
#ifndef scriptthreadresume_allnotyetresumed
        const int scriptthreadresume_allnotyetresumed = -1;
#endif
#ifndef scriptthreadresume_custom
        const int scriptthreadresume_custom = 5;
#endif
#ifndef scriptthreadresume_default
        const int scriptthreadresume_default = 0;
#endif
#ifndef scriptthreadresume_sensing_first
        const int scriptthreadresume_sensing_first = 3;
#endif
#ifndef scriptthreadresume_sensing_last
        const int scriptthreadresume_sensing_last = 4;
#endif
#ifndef scripttype_addonfunction
        const int scripttype_addonfunction = 3;
#endif
#ifndef scripttype_addonscript
        const int scripttype_addonscript = 2;
#endif
#ifndef scripttype_childscript
        const int scripttype_childscript = 1;
#endif
#ifndef scripttype_customizationscript
        const int scripttype_customizationscript = 6;
#endif
#ifndef scripttype_mainscript
        const int scripttype_mainscript = 0;
#endif
#ifndef scripttype_sandboxscript
        const int scripttype_sandboxscript = 8;
#endif
#ifndef scripttype_threaded
        const int scripttype_threaded = 240;
#endif
#ifndef shape_multishape_subtype
        const int shape_multishape_subtype = 21;
#endif
#ifndef shape_simpleshape_subtype
        const int shape_simpleshape_subtype = 20;
#endif
#ifndef shapefloatparam_edge_angle
        const int shapefloatparam_edge_angle = 3026;
#endif
#ifndef shapefloatparam_init_ang_velocity_x
        const int shapefloatparam_init_ang_velocity_x = 3020;
#endif
#ifndef shapefloatparam_init_ang_velocity_y
        const int shapefloatparam_init_ang_velocity_y = 3021;
#endif
#ifndef shapefloatparam_init_ang_velocity_z
        const int shapefloatparam_init_ang_velocity_z = 3022;
#endif
#ifndef shapefloatparam_init_velocity_a
        const int shapefloatparam_init_velocity_a = 3020;
#endif
#ifndef shapefloatparam_init_velocity_b
        const int shapefloatparam_init_velocity_b = 3021;
#endif
#ifndef shapefloatparam_init_velocity_g
        const int shapefloatparam_init_velocity_g = 3022;
#endif
#ifndef shapefloatparam_init_velocity_x
        const int shapefloatparam_init_velocity_x = 3000;
#endif
#ifndef shapefloatparam_init_velocity_y
        const int shapefloatparam_init_velocity_y = 3001;
#endif
#ifndef shapefloatparam_init_velocity_z
        const int shapefloatparam_init_velocity_z = 3002;
#endif
#ifndef shapefloatparam_mass
        const int shapefloatparam_mass = 3005;
#endif
#ifndef shapefloatparam_shading_angle
        const int shapefloatparam_shading_angle = 3025;
#endif
#ifndef shapefloatparam_texture_a
        const int shapefloatparam_texture_a = 3009;
#endif
#ifndef shapefloatparam_texture_b
        const int shapefloatparam_texture_b = 3010;
#endif
#ifndef shapefloatparam_texture_g
        const int shapefloatparam_texture_g = 3011;
#endif
#ifndef shapefloatparam_texture_scaling_x
        const int shapefloatparam_texture_scaling_x = 3012;
#endif
#ifndef shapefloatparam_texture_scaling_y
        const int shapefloatparam_texture_scaling_y = 3013;
#endif
#ifndef shapefloatparam_texture_x
        const int shapefloatparam_texture_x = 3006;
#endif
#ifndef shapefloatparam_texture_y
        const int shapefloatparam_texture_y = 3007;
#endif
#ifndef shapefloatparam_texture_z
        const int shapefloatparam_texture_z = 3008;
#endif
#ifndef shapeintparam_component_cnt
        const int shapeintparam_component_cnt = 3028;
#endif
#ifndef shapeintparam_compound
        const int shapeintparam_compound = 3016;
#endif
#ifndef shapeintparam_convex
        const int shapeintparam_convex = 3017;
#endif
#ifndef shapeintparam_convex_check
        const int shapeintparam_convex_check = 3018;
#endif
#ifndef shapeintparam_culling
        const int shapeintparam_culling = 3014;
#endif
#ifndef shapeintparam_edge_borders_hidden
        const int shapeintparam_edge_borders_hidden = 3027;
#endif
#ifndef shapeintparam_edge_visibility
        const int shapeintparam_edge_visibility = 3024;
#endif
#ifndef shapeintparam_respondable
        const int shapeintparam_respondable = 3004;
#endif
#ifndef shapeintparam_respondable_mask
        const int shapeintparam_respondable_mask = 3019;
#endif
#ifndef shapeintparam_sleepmodestart
        const int shapeintparam_sleepmodestart = 3029;
#endif
#ifndef shapeintparam_static
        const int shapeintparam_static = 3003;
#endif
#ifndef shapeintparam_wireframe
        const int shapeintparam_wireframe = 3015;
#endif
#ifndef shapestringparam_color_name
        const int shapestringparam_color_name = 3023;
#endif
#ifndef simulation_advancing
        const int simulation_advancing = 16;
#endif
#ifndef simulation_advancing_abouttostop
        const int simulation_advancing_abouttostop = 21;
#endif
#ifndef simulation_advancing_firstafterpause
        const int simulation_advancing_firstafterpause = 20;
#endif
#ifndef simulation_advancing_firstafterstop
        const int simulation_advancing_firstafterstop = 16;
#endif
#ifndef simulation_advancing_lastbeforepause
        const int simulation_advancing_lastbeforepause = 19;
#endif
#ifndef simulation_advancing_lastbeforestop
        const int simulation_advancing_lastbeforestop = 22;
#endif
#ifndef simulation_advancing_running
        const int simulation_advancing_running = 17;
#endif
#ifndef simulation_paused
        const int simulation_paused = 8;
#endif
#ifndef simulation_stopped
        const int simulation_stopped = 0;
#endif
#ifndef stream_transf_cumulative
        const int stream_transf_cumulative = 3;
#endif
#ifndef stream_transf_derivative
        const int stream_transf_derivative = 1;
#endif
#ifndef stream_transf_integral
        const int stream_transf_integral = 2;
#endif
#ifndef stream_transf_raw
        const int stream_transf_raw = 0;
#endif
#ifndef stringparam_additionalpythonpath
        const int stringparam_additionalpythonpath = 135;
#endif
#ifndef stringparam_addonpath
        const int stringparam_addonpath = 131;
#endif
#ifndef stringparam_app_arg1
        const int stringparam_app_arg1 = 2;
#endif
#ifndef stringparam_app_arg2
        const int stringparam_app_arg2 = 3;
#endif
#ifndef stringparam_app_arg3
        const int stringparam_app_arg3 = 4;
#endif
#ifndef stringparam_app_arg4
        const int stringparam_app_arg4 = 5;
#endif
#ifndef stringparam_app_arg5
        const int stringparam_app_arg5 = 6;
#endif
#ifndef stringparam_app_arg6
        const int stringparam_app_arg6 = 7;
#endif
#ifndef stringparam_app_arg7
        const int stringparam_app_arg7 = 8;
#endif
#ifndef stringparam_app_arg8
        const int stringparam_app_arg8 = 9;
#endif
#ifndef stringparam_app_arg9
        const int stringparam_app_arg9 = 10;
#endif
#ifndef stringparam_application_path
        const int stringparam_application_path = 0;
#endif
#ifndef stringparam_datadir
        const int stringparam_datadir = 129;
#endif
#ifndef stringparam_defaultpython
        const int stringparam_defaultpython = 134;
#endif
#ifndef stringparam_dlgverbosity
        const int stringparam_dlgverbosity = 123;
#endif
#ifndef stringparam_importexportdir
        const int stringparam_importexportdir = 130;
#endif
#ifndef stringparam_logfilter
        const int stringparam_logfilter = 124;
#endif
#ifndef stringparam_luadir
        const int stringparam_luadir = 136;
#endif
#ifndef stringparam_machine_id
        const int stringparam_machine_id = 119;
#endif
#ifndef stringparam_machine_id_legacy
        const int stringparam_machine_id_legacy = 120;
#endif
#ifndef stringparam_modeldefaultdir
        const int stringparam_modeldefaultdir = 133;
#endif
#ifndef stringparam_pythondir
        const int stringparam_pythondir = 137;
#endif
#ifndef stringparam_remoteapi_temp_file_dir
        const int stringparam_remoteapi_temp_file_dir = 16;
#endif
#ifndef stringparam_scene_name
        const int stringparam_scene_name = 15;
#endif
#ifndef stringparam_scene_path
        const int stringparam_scene_path = 14;
#endif
#ifndef stringparam_scene_path_and_name
        const int stringparam_scene_path_and_name = 13;
#endif
#ifndef stringparam_scene_unique_id
        const int stringparam_scene_unique_id = 118;
#endif
#ifndef stringparam_scenedefaultdir
        const int stringparam_scenedefaultdir = 132;
#endif
#ifndef stringparam_statusbarverbosity
        const int stringparam_statusbarverbosity = 122;
#endif
#ifndef stringparam_tempdir
        const int stringparam_tempdir = 127;
#endif
#ifndef stringparam_tempscenedir
        const int stringparam_tempscenedir = 128;
#endif
#ifndef stringparam_uniqueid
        const int stringparam_uniqueid = 126;
#endif
#ifndef stringparam_verbosity
        const int stringparam_verbosity = 121;
#endif
#ifndef stringparam_video_filename
        const int stringparam_video_filename = 1;
#endif
#ifndef syscb_actuation
        const int syscb_actuation = 6;
#endif
#ifndef syscb_aftercopy
        const int syscb_aftercopy = 14;
#endif
#ifndef syscb_aftercreate
        const int syscb_aftercreate = 26;
#endif
#ifndef syscb_afterdelete
        const int syscb_afterdelete = 25;
#endif
#ifndef syscb_afterinstanceswitch
        const int syscb_afterinstanceswitch = 12;
#endif
#ifndef syscb_aftersimulation
        const int syscb_aftersimulation = 5;
#endif
#ifndef syscb_aos_resume
        const int syscb_aos_resume = 17;
#endif
#ifndef syscb_aos_run
        const int syscb_aos_run = 15;
#endif
#ifndef syscb_aos_suspend
        const int syscb_aos_suspend = 16;
#endif
#ifndef syscb_beforecopy
        const int syscb_beforecopy = 13;
#endif
#ifndef syscb_beforedelete
        const int syscb_beforedelete = 24;
#endif
#ifndef syscb_beforeinstanceswitch
        const int syscb_beforeinstanceswitch = 11;
#endif
#ifndef syscb_beforemainscript
        const int syscb_beforemainscript = 29;
#endif
#ifndef syscb_beforesimulation
        const int syscb_beforesimulation = 4;
#endif
#ifndef syscb_cleanup
        const int syscb_cleanup = 1;
#endif
#ifndef syscb_contactcallback
        const int syscb_contactcallback = 19;
#endif
#ifndef syscb_customcallback1
        const int syscb_customcallback1 = 20;
#endif
#ifndef syscb_customcallback2
        const int syscb_customcallback2 = 21;
#endif
#ifndef syscb_customcallback3
        const int syscb_customcallback3 = 22;
#endif
#ifndef syscb_customcallback4
        const int syscb_customcallback4 = 23;
#endif
#ifndef syscb_dyncallback
        const int syscb_dyncallback = 28;
#endif
#ifndef syscb_init
        const int syscb_init = 2;
#endif
#ifndef syscb_jointcallback
        const int syscb_jointcallback = 18;
#endif
#ifndef syscb_moduleentry
        const int syscb_moduleentry = 33;
#endif
#ifndef syscb_nonsimulation
        const int syscb_nonsimulation = 3;
#endif
#ifndef syscb_regular
        const int syscb_regular = 6;
#endif
#ifndef syscb_resume
        const int syscb_resume = 10;
#endif
#ifndef syscb_sensing
        const int syscb_sensing = 7;
#endif
#ifndef syscb_suspend
        const int syscb_suspend = 9;
#endif
#ifndef syscb_suspended
        const int syscb_suspended = 8;
#endif
#ifndef syscb_trigger
        const int syscb_trigger = 31;
#endif
#ifndef syscb_userconfig
        const int syscb_userconfig = 32;
#endif
#ifndef syscb_vision
        const int syscb_vision = 30;
#endif
#ifndef texturemap_cube
        const int texturemap_cube = 3;
#endif
#ifndef texturemap_cylinder
        const int texturemap_cylinder = 1;
#endif
#ifndef texturemap_plane
        const int texturemap_plane = 0;
#endif
#ifndef texturemap_sphere
        const int texturemap_sphere = 2;
#endif
#ifndef verbosity_debug
        const int verbosity_debug = 600;
#endif
#ifndef verbosity_default
        const int verbosity_default = 400;
#endif
#ifndef verbosity_errors
        const int verbosity_errors = 200;
#endif
#ifndef verbosity_infos
        const int verbosity_infos = 500;
#endif
#ifndef verbosity_loadinfos
        const int verbosity_loadinfos = 400;
#endif
#ifndef verbosity_msgs
        const int verbosity_msgs = 450;
#endif
#ifndef verbosity_none
        const int verbosity_none = 100;
#endif
#ifndef verbosity_questions
        const int verbosity_questions = 410;
#endif
#ifndef verbosity_scripterrors
        const int verbosity_scripterrors = 420;
#endif
#ifndef verbosity_scriptinfos
        const int verbosity_scriptinfos = 450;
#endif
#ifndef verbosity_scriptwarnings
        const int verbosity_scriptwarnings = 430;
#endif
#ifndef verbosity_trace
        const int verbosity_trace = 700;
#endif
#ifndef verbosity_traceall
        const int verbosity_traceall = 900;
#endif
#ifndef verbosity_tracelua
        const int verbosity_tracelua = 800;
#endif
#ifndef verbosity_undecorated
        const int verbosity_undecorated = 61440;
#endif
#ifndef verbosity_useglobal
        const int verbosity_useglobal = -1;
#endif
#ifndef verbosity_warnings
        const int verbosity_warnings = 300;
#endif
#ifndef visionfloatparam_far_clipping
        const int visionfloatparam_far_clipping = 1001;
#endif
#ifndef visionfloatparam_near_clipping
        const int visionfloatparam_near_clipping = 1000;
#endif
#ifndef visionfloatparam_ortho_size
        const int visionfloatparam_ortho_size = 1005;
#endif
#ifndef visionfloatparam_perspective_angle
        const int visionfloatparam_perspective_angle = 1004;
#endif
#ifndef visionfloatparam_pov_aperture
        const int visionfloatparam_pov_aperture = 1015;
#endif
#ifndef visionfloatparam_pov_blur_distance
        const int visionfloatparam_pov_blur_distance = 1014;
#endif
#ifndef visionintparam_disabled_light_components
        const int visionintparam_disabled_light_components = 1006;
#endif
#ifndef visionintparam_entity_to_render
        const int visionintparam_entity_to_render = 1008;
#endif
#ifndef visionintparam_perspective_operation
        const int visionintparam_perspective_operation = 1018;
#endif
#ifndef visionintparam_pov_blur_sampled
        const int visionintparam_pov_blur_sampled = 1016;
#endif
#ifndef visionintparam_pov_focal_blur
        const int visionintparam_pov_focal_blur = 1013;
#endif
#ifndef visionintparam_render_mode
        const int visionintparam_render_mode = 1017;
#endif
#ifndef visionintparam_rendering_attributes
        const int visionintparam_rendering_attributes = 1007;
#endif
#ifndef visionintparam_resolution_x
        const int visionintparam_resolution_x = 1002;
#endif
#ifndef visionintparam_resolution_y
        const int visionintparam_resolution_y = 1003;
#endif
#ifndef visionintparam_windowed_pos_x
        const int visionintparam_windowed_pos_x = 1011;
#endif
#ifndef visionintparam_windowed_pos_y
        const int visionintparam_windowed_pos_y = 1012;
#endif
#ifndef visionintparam_windowed_size_x
        const int visionintparam_windowed_size_x = 1009;
#endif
#ifndef visionintparam_windowed_size_y
        const int visionintparam_windowed_size_y = 1010;
#endif
#ifndef volume_cone
        const int volume_cone = 3;
#endif
#ifndef volume_cylinder
        const int volume_cylinder = 1;
#endif
#ifndef volume_disc
        const int volume_disc = 2;
#endif
#ifndef volume_pyramid
        const int volume_pyramid = 0;
#endif
#ifndef volume_randomizedray
        const int volume_randomizedray = 5;
#endif
#ifndef volume_ray
        const int volume_ray = 4;
#endif
#ifndef vortex_body_adhesiveforce
        const int vortex_body_adhesiveforce = 24015;
#endif
#ifndef vortex_body_angularvelocitydamping
        const int vortex_body_angularvelocitydamping = 24017;
#endif
#ifndef vortex_body_autoangulardamping
        const int vortex_body_autoangulardamping = 26009;
#endif
#ifndef vortex_body_autoangulardampingtensionratio
        const int vortex_body_autoangulardampingtensionratio = 24033;
#endif
#ifndef vortex_body_autosleepangularaccelthreshold
        const int vortex_body_autosleepangularaccelthreshold = 24031;
#endif
#ifndef vortex_body_autosleepangularspeedthreshold
        const int vortex_body_autosleepangularspeedthreshold = 24030;
#endif
#ifndef vortex_body_autosleeplinearaccelthreshold
        const int vortex_body_autosleeplinearaccelthreshold = 24029;
#endif
#ifndef vortex_body_autosleeplinearspeedthreshold
        const int vortex_body_autosleeplinearspeedthreshold = 24028;
#endif
#ifndef vortex_body_autosleepsteplivethreshold
        const int vortex_body_autosleepsteplivethreshold = 25007;
#endif
#ifndef vortex_body_autoslip
        const int vortex_body_autoslip = 26005;
#endif
#ifndef vortex_body_bitcoded
        const int vortex_body_bitcoded = 25006;
#endif
#ifndef vortex_body_compliance
        const int vortex_body_compliance = 24011;
#endif
#ifndef vortex_body_convexshapesasrandom
        const int vortex_body_convexshapesasrandom = 26002;
#endif
#ifndef vortex_body_damping
        const int vortex_body_damping = 24012;
#endif
#ifndef vortex_body_fastmoving
        const int vortex_body_fastmoving = 26004;
#endif
#ifndef vortex_body_linearvelocitydamping
        const int vortex_body_linearvelocitydamping = 24016;
#endif
#ifndef vortex_body_materialuniqueid
        const int vortex_body_materialuniqueid = 25008;
#endif
#ifndef vortex_body_normalangularaxisfriction
        const int vortex_body_normalangularaxisfriction = 24005;
#endif
#ifndef vortex_body_normalangularaxisslide
        const int vortex_body_normalangularaxisslide = 24022;
#endif
#ifndef vortex_body_normalangularaxisslip
        const int vortex_body_normalangularaxisslip = 24027;
#endif
#ifndef vortex_body_normalangularaxisstaticfrictionscale
        const int vortex_body_normalangularaxisstaticfrictionscale = 24010;
#endif
#ifndef vortex_body_normalmangulararaxisfrictionmodel
        const int vortex_body_normalmangulararaxisfrictionmodel = 25005;
#endif
#ifndef vortex_body_normangaxissameasprimangaxis
        const int vortex_body_normangaxissameasprimangaxis = 26008;
#endif
#ifndef vortex_body_primangulararaxisfrictionmodel
        const int vortex_body_primangulararaxisfrictionmodel = 25003;
#endif
#ifndef vortex_body_primangularaxisfriction
        const int vortex_body_primangularaxisfriction = 24003;
#endif
#ifndef vortex_body_primangularaxisslide
        const int vortex_body_primangularaxisslide = 24020;
#endif
#ifndef vortex_body_primangularaxisslip
        const int vortex_body_primangularaxisslip = 24025;
#endif
#ifndef vortex_body_primangularaxisstaticfrictionscale
        const int vortex_body_primangularaxisstaticfrictionscale = 24008;
#endif
#ifndef vortex_body_primaxisvectorx
        const int vortex_body_primaxisvectorx = 24034;
#endif
#ifndef vortex_body_primaxisvectory
        const int vortex_body_primaxisvectory = 24035;
#endif
#ifndef vortex_body_primaxisvectorz
        const int vortex_body_primaxisvectorz = 24036;
#endif
#ifndef vortex_body_primlinearaxisfriction
        const int vortex_body_primlinearaxisfriction = 24001;
#endif
#ifndef vortex_body_primlinearaxisfrictionmodel
        const int vortex_body_primlinearaxisfrictionmodel = 25001;
#endif
#ifndef vortex_body_primlinearaxisslide
        const int vortex_body_primlinearaxisslide = 24018;
#endif
#ifndef vortex_body_primlinearaxisslip
        const int vortex_body_primlinearaxisslip = 24023;
#endif
#ifndef vortex_body_primlinearaxisstaticfrictionscale
        const int vortex_body_primlinearaxisstaticfrictionscale = 24006;
#endif
#ifndef vortex_body_pureshapesasconvex
        const int vortex_body_pureshapesasconvex = 26001;
#endif
#ifndef vortex_body_randomshapesasterrain
        const int vortex_body_randomshapesasterrain = 26003;
#endif
#ifndef vortex_body_restitution
        const int vortex_body_restitution = 24013;
#endif
#ifndef vortex_body_restitutionthreshold
        const int vortex_body_restitutionthreshold = 24014;
#endif
#ifndef vortex_body_secangaxissameasprimangaxis
        const int vortex_body_secangaxissameasprimangaxis = 26007;
#endif
#ifndef vortex_body_secangularaxisfriction
        const int vortex_body_secangularaxisfriction = 24004;
#endif
#ifndef vortex_body_secangularaxisslide
        const int vortex_body_secangularaxisslide = 24021;
#endif
#ifndef vortex_body_secangularaxisslip
        const int vortex_body_secangularaxisslip = 24026;
#endif
#ifndef vortex_body_secangularaxisstaticfrictionscale
        const int vortex_body_secangularaxisstaticfrictionscale = 24009;
#endif
#ifndef vortex_body_seclinaxissameasprimlinaxis
        const int vortex_body_seclinaxissameasprimlinaxis = 26006;
#endif
#ifndef vortex_body_seclinearaxisfriction
        const int vortex_body_seclinearaxisfriction = 24002;
#endif
#ifndef vortex_body_seclinearaxisfrictionmodel
        const int vortex_body_seclinearaxisfrictionmodel = 25002;
#endif
#ifndef vortex_body_seclinearaxisslide
        const int vortex_body_seclinearaxisslide = 24019;
#endif
#ifndef vortex_body_seclinearaxisslip
        const int vortex_body_seclinearaxisslip = 24024;
#endif
#ifndef vortex_body_seclinearaxisstaticfrictionscale
        const int vortex_body_seclinearaxisstaticfrictionscale = 24007;
#endif
#ifndef vortex_body_secmangulararaxisfrictionmodel
        const int vortex_body_secmangulararaxisfrictionmodel = 25004;
#endif
#ifndef vortex_body_skinthickness
        const int vortex_body_skinthickness = 24032;
#endif
#ifndef vortex_bodyfrictionmodel_box
        const int vortex_bodyfrictionmodel_box = 0;
#endif
#ifndef vortex_bodyfrictionmodel_neutral
        const int vortex_bodyfrictionmodel_neutral = 5;
#endif
#ifndef vortex_bodyfrictionmodel_none
        const int vortex_bodyfrictionmodel_none = 6;
#endif
#ifndef vortex_bodyfrictionmodel_prophigh
        const int vortex_bodyfrictionmodel_prophigh = 3;
#endif
#ifndef vortex_bodyfrictionmodel_proplow
        const int vortex_bodyfrictionmodel_proplow = 2;
#endif
#ifndef vortex_bodyfrictionmodel_scaledbox
        const int vortex_bodyfrictionmodel_scaledbox = 1;
#endif
#ifndef vortex_bodyfrictionmodel_scaledboxfast
        const int vortex_bodyfrictionmodel_scaledboxfast = 4;
#endif
#ifndef vortex_global_autosleep
        const int vortex_global_autosleep = 20001;
#endif
#ifndef vortex_global_bitcoded
        const int vortex_global_bitcoded = 19001;
#endif
#ifndef vortex_global_constraintangularcompliance
        const int vortex_global_constraintangularcompliance = 18007;
#endif
#ifndef vortex_global_constraintangulardamping
        const int vortex_global_constraintangulardamping = 18008;
#endif
#ifndef vortex_global_constraintangularkineticloss
        const int vortex_global_constraintangularkineticloss = 18009;
#endif
#ifndef vortex_global_constraintlinearcompliance
        const int vortex_global_constraintlinearcompliance = 18004;
#endif
#ifndef vortex_global_constraintlineardamping
        const int vortex_global_constraintlineardamping = 18005;
#endif
#ifndef vortex_global_constraintlinearkineticloss
        const int vortex_global_constraintlinearkineticloss = 18006;
#endif
#ifndef vortex_global_contacttolerance
        const int vortex_global_contacttolerance = 18003;
#endif
#ifndef vortex_global_internalscalingfactor
        const int vortex_global_internalscalingfactor = 18002;
#endif
#ifndef vortex_global_multithreading
        const int vortex_global_multithreading = 20002;
#endif
#ifndef vortex_global_stepsize
        const int vortex_global_stepsize = 18001;
#endif
#ifndef vortex_joint_a0damping
        const int vortex_joint_a0damping = 21032;
#endif
#ifndef vortex_joint_a0frictioncoeff
        const int vortex_joint_a0frictioncoeff = 21033;
#endif
#ifndef vortex_joint_a0frictionloss
        const int vortex_joint_a0frictionloss = 21035;
#endif
#ifndef vortex_joint_a0frictionmaxforce
        const int vortex_joint_a0frictionmaxforce = 21034;
#endif
#ifndef vortex_joint_a0loss
        const int vortex_joint_a0loss = 21030;
#endif
#ifndef vortex_joint_a0stiffness
        const int vortex_joint_a0stiffness = 21031;
#endif
#ifndef vortex_joint_a1damping
        const int vortex_joint_a1damping = 21038;
#endif
#ifndef vortex_joint_a1frictioncoeff
        const int vortex_joint_a1frictioncoeff = 21039;
#endif
#ifndef vortex_joint_a1frictionloss
        const int vortex_joint_a1frictionloss = 21041;
#endif
#ifndef vortex_joint_a1frictionmaxforce
        const int vortex_joint_a1frictionmaxforce = 21040;
#endif
#ifndef vortex_joint_a1loss
        const int vortex_joint_a1loss = 21036;
#endif
#ifndef vortex_joint_a1stiffness
        const int vortex_joint_a1stiffness = 21037;
#endif
#ifndef vortex_joint_a2damping
        const int vortex_joint_a2damping = 21044;
#endif
#ifndef vortex_joint_a2frictioncoeff
        const int vortex_joint_a2frictioncoeff = 21045;
#endif
#ifndef vortex_joint_a2frictionloss
        const int vortex_joint_a2frictionloss = 21047;
#endif
#ifndef vortex_joint_a2frictionmaxforce
        const int vortex_joint_a2frictionmaxforce = 21046;
#endif
#ifndef vortex_joint_a2loss
        const int vortex_joint_a2loss = 21042;
#endif
#ifndef vortex_joint_a2stiffness
        const int vortex_joint_a2stiffness = 21043;
#endif
#ifndef vortex_joint_bitcoded
        const int vortex_joint_bitcoded = 22001;
#endif
#ifndef vortex_joint_dependencyfactor
        const int vortex_joint_dependencyfactor = 21048;
#endif
#ifndef vortex_joint_dependencyoffset
        const int vortex_joint_dependencyoffset = 21049;
#endif
#ifndef vortex_joint_dependentobjectid
        const int vortex_joint_dependentobjectid = 22006;
#endif
#ifndef vortex_joint_frictionenabledbc
        const int vortex_joint_frictionenabledbc = 22003;
#endif
#ifndef vortex_joint_frictionproportionalbc
        const int vortex_joint_frictionproportionalbc = 22004;
#endif
#ifndef vortex_joint_lowerlimitdamping
        const int vortex_joint_lowerlimitdamping = 21001;
#endif
#ifndef vortex_joint_lowerlimitmaxforce
        const int vortex_joint_lowerlimitmaxforce = 21007;
#endif
#ifndef vortex_joint_lowerlimitrestitution
        const int vortex_joint_lowerlimitrestitution = 21005;
#endif
#ifndef vortex_joint_lowerlimitstiffness
        const int vortex_joint_lowerlimitstiffness = 21003;
#endif
#ifndef vortex_joint_motorconstraintfrictioncoeff
        const int vortex_joint_motorconstraintfrictioncoeff = 21009;
#endif
#ifndef vortex_joint_motorconstraintfrictionloss
        const int vortex_joint_motorconstraintfrictionloss = 21011;
#endif
#ifndef vortex_joint_motorconstraintfrictionmaxforce
        const int vortex_joint_motorconstraintfrictionmaxforce = 21010;
#endif
#ifndef vortex_joint_motorfrictionenabled
        const int vortex_joint_motorfrictionenabled = 23001;
#endif
#ifndef vortex_joint_objectid
        const int vortex_joint_objectid = 22005;
#endif
#ifndef vortex_joint_p0damping
        const int vortex_joint_p0damping = 21014;
#endif
#ifndef vortex_joint_p0frictioncoeff
        const int vortex_joint_p0frictioncoeff = 21015;
#endif
#ifndef vortex_joint_p0frictionloss
        const int vortex_joint_p0frictionloss = 21017;
#endif
#ifndef vortex_joint_p0frictionmaxforce
        const int vortex_joint_p0frictionmaxforce = 21016;
#endif
#ifndef vortex_joint_p0loss
        const int vortex_joint_p0loss = 21012;
#endif
#ifndef vortex_joint_p0stiffness
        const int vortex_joint_p0stiffness = 21013;
#endif
#ifndef vortex_joint_p1damping
        const int vortex_joint_p1damping = 21020;
#endif
#ifndef vortex_joint_p1frictioncoeff
        const int vortex_joint_p1frictioncoeff = 21021;
#endif
#ifndef vortex_joint_p1frictionloss
        const int vortex_joint_p1frictionloss = 21023;
#endif
#ifndef vortex_joint_p1frictionmaxforce
        const int vortex_joint_p1frictionmaxforce = 21022;
#endif
#ifndef vortex_joint_p1loss
        const int vortex_joint_p1loss = 21018;
#endif
#ifndef vortex_joint_p1stiffness
        const int vortex_joint_p1stiffness = 21019;
#endif
#ifndef vortex_joint_p2damping
        const int vortex_joint_p2damping = 21026;
#endif
#ifndef vortex_joint_p2frictioncoeff
        const int vortex_joint_p2frictioncoeff = 21027;
#endif
#ifndef vortex_joint_p2frictionloss
        const int vortex_joint_p2frictionloss = 21029;
#endif
#ifndef vortex_joint_p2frictionmaxforce
        const int vortex_joint_p2frictionmaxforce = 21028;
#endif
#ifndef vortex_joint_p2loss
        const int vortex_joint_p2loss = 21024;
#endif
#ifndef vortex_joint_p2stiffness
        const int vortex_joint_p2stiffness = 21025;
#endif
#ifndef vortex_joint_proportionalmotorfriction
        const int vortex_joint_proportionalmotorfriction = 23002;
#endif
#ifndef vortex_joint_relaxationenabledbc
        const int vortex_joint_relaxationenabledbc = 22002;
#endif
#ifndef vortex_joint_upperlimitdamping
        const int vortex_joint_upperlimitdamping = 21002;
#endif
#ifndef vortex_joint_upperlimitmaxforce
        const int vortex_joint_upperlimitmaxforce = 21008;
#endif
#ifndef vortex_joint_upperlimitrestitution
        const int vortex_joint_upperlimitrestitution = 21006;
#endif
#ifndef vortex_joint_upperlimitstiffness
        const int vortex_joint_upperlimitstiffness = 21004;
#endif
    };

    class simIK
    {
    protected:
        RemoteAPIClient *_client;
    public:
        simIK(RemoteAPIClient *client);

        int addIkElement(int environmentHandle, int ikGroupHandle, int tipDummyHandle);
        std::tuple<int, json> addIkElementFromScene(int environmentHandle, int ikGroup, int baseHandle, int tipHandle, int targetHandle, int constraints);
        int applyIkEnvironmentToScene(int environmentHandle, int ikGroup, std::optional<bool> applyOnlyWhenSuccessful = {});
        void applySceneToIkEnvironment(int environmentHandle, int ikGroup);
        bool computeJacobian(int environmentHandle, int ikGroupHandle, int options);
        int createDummy(int environmentHandle, std::optional<std::string> dummyName = {});
        int createEnvironment();
        int createIkGroup(int environmentHandle, std::optional<std::string> ikGroupName = {});
        int createJoint(int environmentHandle, int jointType, std::optional<std::string> jointName = {});
        bool doesIkGroupExist(int environmentHandle, std::string ikGroupName);
        bool doesObjectExist(int environmentHandle, std::string objectName);
        int duplicateEnvironment(int environmentHandle);
        void eraseEnvironment(int environmentHandle);
        void eraseObject(int environmentHandle, int objectHandle);
        std::vector<float> findConfig(int environmentHandle, int ikGroupHandle, std::vector<int> jointHandles, std::optional<float> thresholdDist = {}, std::optional<float> maxTime = {}, std::optional<std::vector<float>> metric = {}, std::optional<std::string> validationCallback = {}, std::optional<json> auxData = {});
        std::vector<float> generatePath(int environmentHandle, int ikGroupHandle, std::vector<int> jointHandles, int tipHandle, int pathPointCount, std::optional<std::string> validationCallback = {}, std::optional<json> auxData = {});
        std::vector<float> getAlternateConfigs(int environmentHandle, std::vector<int> jointHandles, std::optional<std::vector<float>> lowLimits = {}, std::optional<std::vector<float>> ranges = {});
        std::tuple<int, int> getIkElementBase(int environmentHandle, int ikGroupHandle, int elementHandle);
        int getIkElementConstraints(int environmentHandle, int ikGroupHandle, int elementHandle);
        int getIkElementFlags(int environmentHandle, int ikGroupHandle, int elementHandle);
        std::vector<float> getIkElementPrecision(int environmentHandle, int ikGroupHandle, int elementHandle);
        std::vector<float> getIkElementWeights(int environmentHandle, int ikGroupHandle, int elementHandle);
        std::tuple<int, float, int> getIkGroupCalculation(int environmentHandle, int ikGroupHandle);
        int getIkGroupFlags(int environmentHandle, int ikGroupHandle);
        int getIkGroupHandle(int environmentHandle, std::string ikGroupName);
        std::tuple<std::vector<float>, std::vector<int>> getJacobian(int environmentHandle, int ikGroupHandle);
        std::tuple<int, float, float> getJointDependency(int environmentHandle, int jointHandle);
        float getJointIkWeight(int environmentHandle, int jointHandle);
        std::tuple<bool, std::vector<float>> getJointInterval(int environmentHandle, int jointHandle);
        std::vector<float> getJointMatrix(int environmentHandle, int jointHandle);
        float getJointMaxStepSize(int environmentHandle, int jointHandle);
        int getJointMode(int environmentHandle, int jointHandle);
        float getJointPosition(int environmentHandle, int jointHandle);
        float getJointScrewPitch(int environmentHandle, int jointHandle);
        std::tuple<std::vector<float>, std::vector<float>, std::vector<float>> getJointTransformation(int environmentHandle, int jointHandle);
        int getJointType(int environmentHandle, int jointHandle);
        int getLinkedDummy(int environmentHandle, int dummyHandle);
        float getManipulability(int environmentHandle, int ikGroupHandle);
        int getObjectHandle(int environmentHandle, std::string objectName);
        std::vector<float> getObjectMatrix(int environmentHandle, int objectHandle, int relativeToObjectHandle);
        int getObjectParent(int environmentHandle, int objectHandle);
        std::vector<float> getObjectPose(int environmentHandle, int objectHandle, int relativeToObjectHandle);
        std::tuple<std::vector<float>, std::vector<float>, std::vector<float>> getObjectTransformation(int environmentHandle, int objectHandle, int relativeToObjectHandle);
        std::tuple<int, std::string, bool, int> getObjects(int environmentHandle, int index);
        int handleIkGroup(int environmentHandle, std::optional<int> ikGroupHandle = {});
        void load(int environmentHandle, std::string data);
        std::string save(int environmentHandle);
        void setIkElementBase(int environmentHandle, int ikGroupHandle, int elementHandle, int baseHandle, std::optional<int> constraintsBaseHandle = {});
        void setIkElementConstraints(int environmentHandle, int ikGroupHandle, int elementHandle, int constraints);
        void setIkElementFlags(int environmentHandle, int ikGroupHandle, int elementHandle, int flags);
        void setIkElementPrecision(int environmentHandle, int ikGroupHandle, int elementHandle, std::vector<float> precision);
        void setIkElementWeights(int environmentHandle, int ikGroupHandle, int elementHandle, std::vector<float> weights);
        void setIkGroupCalculation(int environmentHandle, int ikGroupHandle, int method, float damping, int maxIterations);
        void setIkGroupFlags(int environmentHandle, int ikGroupHandle, int flags);
        void setJointDependency(int environmentHandle, int jointHandle, int depJointHandle, std::optional<float> offset = {}, std::optional<float> mult = {});
        void setJointIkWeight(int environmentHandle, int jointHandle, float weight);
        void setJointInterval(int environmentHandle, int jointHandle, bool cyclic, std::optional<std::vector<float>> interval = {});
        void setJointMaxStepSize(int environmentHandle, int jointHandle, float stepSize);
        void setJointMode(int environmentHandle, int jointHandle, int jointMode);
        void setJointPosition(int environmentHandle, int jointHandle, float position);
        void setJointScrewPitch(int environmentHandle, int jointHandle, float pitch);
        void setLinkedDummy(int environmentHandle, int dummyHandle, int linkedDummyHandle);
        void setObjectMatrix(int environmentHandle, int objectHandle, int relativeToObjectHandle, std::vector<float> matrix);
        void setObjectParent(int environmentHandle, int objectHandle, int parentObjectHandle, std::optional<bool> keepInPlace = {});
        void setObjectPose(int environmentHandle, int objectHandle, int relativeToObjectHandle, std::vector<float> pose);
        void setObjectTransformation(int environmentHandle, int objectHandle, int relativeToObjectHandle, std::vector<float> position, std::vector<float> eulerOrQuaternion);
        void setSphericalJointMatrix(int environmentHandle, int jointHandle, std::vector<float> matrix);
        void setSphericalJointRotation(int environmentHandle, int jointHandle, std::vector<float> eulerOrQuaternion);

#ifndef constraint_alpha_beta
        const int constraint_alpha_beta = 8;
#endif
#ifndef constraint_gamma
        const int constraint_gamma = 16;
#endif
#ifndef constraint_orientation
        const int constraint_orientation = 24;
#endif
#ifndef constraint_pose
        const int constraint_pose = 31;
#endif
#ifndef constraint_position
        const int constraint_position = 7;
#endif
#ifndef constraint_x
        const int constraint_x = 1;
#endif
#ifndef constraint_y
        const int constraint_y = 2;
#endif
#ifndef constraint_z
        const int constraint_z = 4;
#endif
#ifndef handle_all
        const int handle_all = -2;
#endif
#ifndef handle_parent
        const int handle_parent = -11;
#endif
#ifndef handleflag_tipdummy
        const int handleflag_tipdummy = 4194304;
#endif
#ifndef jointmode_dependent
        const int jointmode_dependent = 4;
#endif
#ifndef jointmode_force
        const int jointmode_force = 5;
#endif
#ifndef jointmode_ik
        const int jointmode_ik = 2;
#endif
#ifndef jointmode_passive
        const int jointmode_passive = 0;
#endif
#ifndef jointtype_prismatic
        const int jointtype_prismatic = 11;
#endif
#ifndef jointtype_revolute
        const int jointtype_revolute = 10;
#endif
#ifndef jointtype_spherical
        const int jointtype_spherical = 12;
#endif
#ifndef method_damped_least_squares
        const int method_damped_least_squares = 1;
#endif
#ifndef method_jacobian_transpose
        const int method_jacobian_transpose = 2;
#endif
#ifndef method_pseudo_inverse
        const int method_pseudo_inverse = 0;
#endif
#ifndef method_undamped_pseudo_inverse
        const int method_undamped_pseudo_inverse = 3;
#endif
#ifndef objecttype_dummy
        const int objecttype_dummy = 4;
#endif
#ifndef objecttype_joint
        const int objecttype_joint = 1;
#endif
#ifndef result_fail
        const int result_fail = 2;
#endif
#ifndef result_not_performed
        const int result_not_performed = 0;
#endif
#ifndef result_success
        const int result_success = 1;
#endif
    };

};

class RemoteAPIObjects
{
public:
    RemoteAPIObjects(RemoteAPIClient *client);
    RemoteAPIObject::sim sim();
    RemoteAPIObject::simIK simIK();
private:
    RemoteAPIClient *client;
};
