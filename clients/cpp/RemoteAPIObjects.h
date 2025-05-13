
class RemoteAPIClient;

namespace RemoteAPIObject
{
    class sim
    {
    protected:
        RemoteAPIClient *_client;
    public:
        sim(RemoteAPIClient *client);

#include "sim-deprecated.h"
#include "sim-special.h"

        json Object(int64_t handle);
        void acquireLock();
        int64_t addDrawingObject(int64_t objectType, double size, double duplicateTolerance, int64_t parentObjectHandle, int64_t maxItemCount, std::optional<std::vector<double>> color = {});
        int64_t addDrawingObjectItem(int64_t drawingObjectHandle, std::vector<double> itemData);
        void addForce(int64_t shapeHandle, std::vector<double> position, std::vector<double> force);
        void addForceAndTorque(int64_t shapeHandle, std::optional<std::vector<double>> force = {}, std::optional<std::vector<double>> torque = {});
        int64_t addGraphCurve(int64_t graphHandle, std::string curveName, int64_t dim, std::vector<int64_t> streamIds, std::vector<double> defaultValues, std::string unitStr, std::optional<int64_t> options = {}, std::optional<std::vector<double>> color = {}, std::optional<int64_t> curveWidth = {});
        int64_t addGraphStream(int64_t graphHandle, std::string streamName, std::string unit, std::optional<int64_t> options = {}, std::optional<std::vector<double>> color = {}, std::optional<double> cyclicRange = {});
        void addItemToCollection(int64_t collectionHandle, int64_t what, int64_t objectHandle, int64_t options);
        void addLog(int64_t verbosityLevel, std::string logMessage);
        int64_t addParticleObject(int64_t objectType, double size, double density, std::vector<double> params, double lifeTime, int64_t maxItemCount, std::optional<std::vector<double>> color = {});
        void addParticleObjectItem(int64_t objectHandle, std::vector<double> itemData);
        void addReferencedHandle(int64_t objectHandle, int64_t referencedHandle, std::optional<std::string> tag = {}, std::optional<json> opts = {});
        int64_t adjustView(int64_t viewHandleOrIndex, int64_t objectHandle, int64_t options, std::optional<std::string> viewLabel = {});
        int64_t alignShapeBB(int64_t shapeHandle, std::vector<double> pose);
        std::tuple<double, double, double> alphaBetaGammaToYawPitchRoll(double alphaAngle, double betaAngle, double gammaAngle);
        int64_t announceSceneContentChange();
        int64_t auxiliaryConsoleClose(int64_t consoleHandle);
        int64_t auxiliaryConsoleOpen(std::string title, int64_t maxLines, int64_t mode, std::optional<std::vector<int64_t>> position = {}, std::optional<std::vector<int64_t>> size = {}, std::optional<std::vector<double>> textColor = {}, std::optional<std::vector<double>> backgroundColor = {});
        int64_t auxiliaryConsolePrint(int64_t consoleHandle, std::string text);
        int64_t auxiliaryConsoleShow(int64_t consoleHandle, bool showState);
        void broadcastMsg(json message, std::optional<int64_t> options = {});
        std::vector<double> buildIdentityMatrix();
        std::vector<double> buildMatrix(std::vector<double> position, std::vector<double> eulerAngles);
        std::vector<double> buildPose(std::vector<double> position, std::vector<double> eulerAnglesOrAxis, std::optional<int64_t> mode = {}, std::optional<std::vector<double>> axis2 = {});
        int64_t cameraFitToView(int64_t viewHandleOrIndex, std::optional<std::vector<int64_t>> objectHandles = {}, std::optional<int64_t> options = {}, std::optional<double> scaling = {});
        bool cancelScheduledExecution(int64_t id);
        std::vector<json> changeEntityColor(int64_t entityHandle, std::vector<double> newColor, std::optional<int64_t> colorComponent = {});
        std::tuple<int64_t, std::vector<int64_t>> checkCollision(int64_t entity1Handle, int64_t entity2Handle);
        std::tuple<int64_t, std::vector<double>> checkCollisionEx(int64_t entity1Handle, int64_t entity2Handle);
        std::tuple<int64_t, std::vector<double>, std::vector<int64_t>> checkDistance(int64_t entity1Handle, int64_t entity2Handle, std::optional<double> threshold = {});
        std::tuple<int64_t, int64_t, int64_t, int64_t> checkOctreePointOccupancy(int64_t octreeHandle, int64_t options, std::vector<double> points);
        std::tuple<int64_t, double, std::vector<double>, int64_t, std::vector<double>> checkProximitySensor(int64_t sensorHandle, int64_t entityHandle);
        std::tuple<int64_t, double, std::vector<double>, int64_t, std::vector<double>> checkProximitySensorEx(int64_t sensorHandle, int64_t entityHandle, int64_t mode, double threshold, double maxAngle);
        std::tuple<int64_t, double, std::vector<double>, std::vector<double>> checkProximitySensorEx2(int64_t sensorHandle, std::vector<double> vertices, int64_t itemType, int64_t itemCount, int64_t mode, double threshold, double maxAngle);
        std::tuple<int64_t, std::vector<double>, std::vector<double>> checkVisionSensor(int64_t sensorHandle, int64_t entityHandle);
        std::vector<double> checkVisionSensorEx(int64_t sensorHandle, int64_t entityHandle, bool returnImage);
        void clearBufferSignal(std::string signalName);
        void clearFloatSignal(std::string signalName);
        void clearInt32Signal(std::string signalName);
        void clearStringSignal(std::string signalName);
        int64_t closeScene();
        std::vector<uint8_t> combineRgbImages(std::vector<uint8_t> img1, std::vector<int64_t> img1Res, std::vector<uint8_t> img2, std::vector<int64_t> img2Res, int64_t operation);
        int64_t computeMassAndInertia(int64_t shapeHandle, double density);
        json convertPropertyValue(json value, int64_t fromType, int64_t toType);
        std::vector<int64_t> copyPasteObjects(std::vector<int64_t> objectHandles, std::optional<int64_t> options = {});
        std::vector<json> copyTable(std::vector<json> original);
        int64_t createCollection(std::optional<int64_t> options = {});
        int64_t createDummy(double size);
        int64_t createForceSensor(int64_t options, std::vector<int64_t> intParams, std::vector<double> floatParams);
        int64_t createHeightfieldShape(int64_t options, double shadingAngle, int64_t xPointCount, int64_t yPointCount, double xSize, std::vector<double> heights);
        int64_t createJoint(int64_t jointType, int64_t jointMode, int64_t options, std::optional<std::vector<double>> sizes = {});
        int64_t createOctree(double voxelSize, int64_t options, double pointSize);
        int64_t createPath(std::vector<double> ctrlPts, std::optional<int64_t> options = {}, std::optional<int64_t> subdiv = {}, std::optional<double> smoothness = {}, std::optional<int64_t> orientationMode = {}, std::optional<std::vector<double>> upVector = {});
        int64_t createPointCloud(double maxVoxelSize, int64_t maxPtCntPerVoxel, int64_t options, double pointSize);
        int64_t createPrimitiveShape(int64_t primitiveType, std::vector<double> sizes, std::optional<int64_t> options = {});
        int64_t createProximitySensor(int64_t sensorType, int64_t subType, int64_t options, std::vector<int64_t> intParams, std::vector<double> floatParams);
        int64_t createScript(int64_t scriptType, std::string scriptString, std::optional<int64_t> options = {}, std::optional<std::string> lang = {});
        int64_t createShape(int64_t options, double shadingAngle, std::vector<double> vertices, std::vector<int64_t> indices, std::vector<double> normals, std::vector<double> textureCoordinates, std::vector<uint8_t> texture, std::vector<int64_t> textureResolution);
        std::tuple<int64_t, int64_t, std::vector<int64_t>> createTexture(std::string fileName, int64_t options, std::optional<std::vector<double>> planeSizes = {}, std::optional<std::vector<double>> scalingUV = {}, std::optional<std::vector<double>> xy_g = {}, std::optional<int64_t> fixedResolution = {}, std::optional<std::vector<int64_t>> resolution = {});
        int64_t createVisionSensor(int64_t options, std::vector<int64_t> intParams, std::vector<double> floatParams);
        void destroyCollection(int64_t collectionHandle);
        void destroyGraphCurve(int64_t graphHandle, int64_t curveId);
        int64_t duplicateGraphCurveToStatic(int64_t graphHandle, int64_t curveId, std::optional<std::string> curveName = {});
        std::tuple<bool, json> executeLuaCode(std::string theCode);
        std::tuple<int64_t, json> executeScriptString(std::string stringToExecute, int64_t scriptHandle);
        void exportMesh(int64_t fileformat, std::string pathAndFilename, int64_t options, double scalingFactor, std::vector<double> vertices, std::vector<int64_t> indices);
        void fastIdleLoop(bool enable);
        int64_t floatingViewAdd(double posX, double posY, double sizeX, double sizeY, int64_t options);
        int64_t floatingViewRemove(int64_t floatingViewHandle);
        int64_t generateShapeFromPath(std::vector<double> path, std::vector<double> section, std::optional<int64_t> options = {}, std::optional<std::vector<double>> upVector = {});
        int64_t generateTextShape(std::string txt, std::optional<std::vector<double>> color = {}, std::optional<double> height = {}, std::optional<bool> centered = {}, std::optional<std::string> alphabetLocation = {});
        std::tuple<std::vector<double>, std::vector<double>> generateTimeOptimalTrajectory(std::vector<double> path, std::vector<double> pathLengths, std::vector<double> minMaxVel, std::vector<double> minMaxAccel, std::optional<int64_t> trajPtSamples = {}, std::optional<std::string> boundaryCondition = {}, std::optional<double> timeout = {});
        std::vector<double> getAlternateConfigs(std::vector<int64_t> jointHandles, std::vector<double> inputConfig, std::optional<int64_t> tipHandle = {}, std::optional<std::vector<double>> lowLimits = {}, std::optional<std::vector<double>> ranges = {});
        std::vector<std::string> getApiFunc(int64_t scriptHandle, std::string apiWord);
        std::string getApiInfo(int64_t scriptHandle, std::string apiWord);
        std::vector<double> getArrayParam(int64_t parameter);
        double getAutoYieldDelay();
        bool getBoolParam(int64_t parameter);
        bool getBoolProperty(int64_t target, std::string pName, std::optional<json> options = {});
        std::vector<uint8_t> getBufferProperty(int64_t target, std::string pName, std::optional<json> options = {});
        std::vector<uint8_t> getBufferSignal(std::string signalName);
        double getClosestPosOnPath(std::vector<double> path, std::vector<double> pathLengths, std::vector<double> absPt);
        std::vector<int64_t> getCollectionObjects(int64_t collectionHandle);
        std::vector<double> getColorProperty(int64_t target, std::string pName, std::optional<json> options = {});
        double getConfigDistance(std::vector<double> configA, std::vector<double> configB, std::optional<std::vector<double>> metric = {}, std::optional<std::vector<int64_t>> types = {});
        std::tuple<std::vector<int64_t>, std::vector<double>, std::vector<double>, std::vector<double>> getContactInfo(int64_t dynamicPass, int64_t objectHandle, int64_t index);
        bool getEngineBoolParam(int64_t paramId, int64_t objectHandle);
        double getEngineFloatParam(int64_t paramId, int64_t objectHandle);
        int64_t getEngineInt32Param(int64_t paramId, int64_t objectHandle);
        std::vector<double> getEulerAnglesFromMatrix(std::vector<double> matrix);
        int64_t getExplicitHandling(int64_t objectHandle);
        std::string getExtensionString(int64_t objectHandle, int64_t index, std::optional<std::string> key = {});
        std::vector<double> getFloatArrayProperty(int64_t target, std::string pName, std::optional<json> options = {});
        double getFloatParam(int64_t parameter);
        double getFloatProperty(int64_t target, std::string pName, std::optional<json> options = {});
        std::vector<json> getGenesisEvents();
        std::tuple<std::string, int64_t, std::vector<double>, std::vector<double>, std::vector<double>, std::vector<double>, int64_t, int64_t> getGraphCurve(int64_t graphHandle, int64_t graphType, int64_t curveIndex);
        std::tuple<int64_t, std::vector<double>, std::vector<double>> getGraphInfo(int64_t graphHandle);
        int64_t getInt32Param(int64_t parameter);
        std::vector<int64_t> getIntArray2Property(int64_t target, std::string pName, std::optional<json> options = {});
        std::vector<int64_t> getIntArrayProperty(int64_t target, std::string pName, std::optional<json> options = {});
        int64_t getIntProperty(int64_t target, std::string pName, std::optional<json> options = {});
        int64_t getIsRealTimeSimulation();
        std::tuple<int64_t, double, double> getJointDependency(int64_t jointHandle);
        double getJointForce(int64_t jointHandle);
        std::tuple<bool, std::vector<double>> getJointInterval(int64_t objectHandle);
        std::tuple<int64_t, int64_t> getJointMode(int64_t jointHandle);
        double getJointPosition(int64_t objectHandle);
        double getJointTargetForce(int64_t jointHandle);
        double getJointTargetPosition(int64_t objectHandle);
        double getJointTargetVelocity(int64_t objectHandle);
        int64_t getJointType(int64_t objectHandle);
        double getJointVelocity(int64_t jointHandle);
        std::string getLastInfo();
        std::tuple<int64_t, std::vector<double>, std::vector<double>, std::vector<double>> getLightParameters(int64_t lightHandle);
        int64_t getLinkDummy(int64_t dummyHandle);
        std::vector<std::string> getLoadedPlugins();
        int64_t getLongProperty(int64_t target, std::string pName, std::optional<json> options = {});
        std::vector<std::string> getMatchingPersistentDataTags(std::string pattern);
        std::vector<double> getMatrixInverse(std::vector<double> matrix);
        std::vector<double> getModelBB(int64_t handle);
        int64_t getModelProperty(int64_t objectHandle);
        bool getNamedBoolParam(std::string name);
        double getNamedFloatParam(std::string name);
        int64_t getNamedInt32Param(std::string name);
        std::vector<uint8_t> getNamedStringParam(std::string paramName);
        int64_t getNavigationMode();
        int64_t getObject(std::string path, std::optional<json> options = {});
        std::string getObjectAlias(int64_t objectHandle, std::optional<int64_t> options = {});
        std::string getObjectAliasRelative(int64_t handle, int64_t baseHandle, std::optional<int64_t> options = {});
        int64_t getObjectChild(int64_t objectHandle, int64_t index);
        std::vector<double> getObjectChildPose(int64_t objectHandle);
        std::vector<double> getObjectColor(int64_t objectHandle, int64_t index, int64_t colorComponent);
        std::vector<double> getObjectFloatArrayParam(int64_t objectHandle, int64_t parameterID);
        double getObjectFloatParam(int64_t objectHandle, int64_t parameterID);
        void getObjectFromUid(int64_t uid, std::optional<json> options = {});
        int64_t getObjectHandle(std::string path, std::optional<json> options = {});
        std::tuple<int64_t, int64_t> getObjectHierarchyOrder(int64_t objectHandle);
        int64_t getObjectInt32Param(int64_t objectHandle, int64_t parameterID);
        std::vector<double> getObjectMatrix(int64_t objectHandle, std::optional<int64_t> relativeToObjectHandle = {});
        std::vector<double> getObjectOrientation(int64_t objectHandle, std::optional<int64_t> relativeToObjectHandle = {});
        int64_t getObjectParent(int64_t objectHandle);
        std::vector<double> getObjectPose(int64_t objectHandle, std::optional<int64_t> relativeToObjectHandle = {});
        std::vector<double> getObjectPosition(int64_t objectHandle, std::optional<int64_t> relativeToObjectHandle = {});
        int64_t getObjectProperty(int64_t objectHandle);
        std::vector<double> getObjectQuaternion(int64_t objectHandle, std::optional<int64_t> relativeToObjectHandle = {});
        std::vector<int64_t> getObjectSel();
        double getObjectSizeFactor(int64_t ObjectHandle);
        int64_t getObjectSpecialProperty(int64_t objectHandle);
        std::vector<uint8_t> getObjectStringParam(int64_t objectHandle, int64_t parameterID);
        int64_t getObjectType(int64_t objectHandle);
        int64_t getObjectUid(int64_t objectHandle);
        std::tuple<std::vector<double>, std::vector<double>> getObjectVelocity(int64_t objectHandle);
        int64_t getObjects(int64_t index, int64_t objectType);
        std::vector<int64_t> getObjectsInTree(int64_t treeBaseHandle, std::optional<int64_t> objectType = {}, std::optional<int64_t> options = {});
        std::vector<int64_t> getObjectsWithTag(std::string tagName, std::optional<bool> justModels = {});
        std::vector<double> getOctreeVoxels(int64_t octreeHandle);
        int64_t getPage();
        std::vector<double> getPathInterpolatedConfig(std::vector<double> path, std::vector<double> pathLengths, double t, std::optional<json> method = {}, std::optional<std::vector<int64_t>> types = {});
        std::tuple<std::vector<double>, double> getPathLengths(std::vector<double> path, int64_t dof, std::optional<std::string> distCallback = {});
        std::string getPluginInfo(std::string pluginName, int64_t infoType);
        std::string getPluginName(int64_t index);
        std::tuple<double, int64_t, int64_t, double> getPointCloudOptions(int64_t pointCloudHandle);
        std::vector<double> getPointCloudPoints(int64_t pointCloudHandle);
        std::vector<double> getPoseInverse(std::vector<double> pose);
        std::vector<double> getPoseProperty(int64_t target, std::string pName, std::optional<json> options = {});
        json getProperties(int64_t target, std::optional<json> opts = {});
        json getPropertiesInfos(int64_t target, std::optional<json> opts = {});
        json getProperty(int64_t target, std::string pName, std::optional<json> options = {});
        std::tuple<int64_t, int64_t, std::string> getPropertyInfo(int64_t target, std::string pName, std::optional<json> options = {});
        std::tuple<std::string, std::string> getPropertyName(int64_t target, int64_t index, std::optional<json> options = {});
        std::string getPropertyTypeString(int64_t pType);
        std::vector<double> getQuaternionInverse(std::vector<double> quat);
        std::vector<double> getQuaternionProperty(int64_t target, std::string pName, std::optional<json> options = {});
        double getRandom(std::optional<int64_t> seed = {});
        bool getRealTimeSimulation();
        int64_t getReferencedHandle(int64_t objectHandle, std::optional<std::string> tag = {});
        std::vector<int64_t> getReferencedHandles(int64_t objectHandle, std::optional<std::string> tag = {});
        std::vector<std::string> getReferencedHandlesTags(int64_t objectHandle);
        std::tuple<std::vector<double>, double> getRotationAxis(std::vector<double> matrixStart, std::vector<double> matrixGoal);
        std::tuple<std::vector<uint8_t>, std::vector<int64_t>> getScaledImage(std::vector<uint8_t> imageIn, std::vector<int64_t> resolutionIn, std::vector<int64_t> desiredResolutionOut, int64_t options);
        int64_t getScript(int64_t scriptType, std::optional<std::string> scriptName = {});
        json getScriptFunctions(int64_t scriptHandle);
        bool getSettingBool(std::string key);
        double getSettingFloat(std::string key);
        int64_t getSettingInt32(std::string key);
        std::string getSettingString(std::string key);
        json getShapeAppearance(int64_t handle, std::optional<json> opts = {});
        std::tuple<std::vector<double>, std::vector<double>> getShapeBB(int64_t shapeHandle);
        std::tuple<int64_t, std::vector<double>> getShapeColor(int64_t shapeHandle, std::string colorName, int64_t colorComponent);
        std::tuple<int64_t, int64_t, std::vector<double>> getShapeGeomInfo(int64_t shapeHandle);
        std::tuple<std::vector<double>, std::vector<double>> getShapeInertia(int64_t shapeHandle);
        double getShapeMass(int64_t shapeHandle);
        std::tuple<std::vector<double>, std::vector<int64_t>, std::vector<double>> getShapeMesh(int64_t shapeHandle);
        int64_t getShapeTextureId(int64_t shapeHandle);
        json getShapeViz(int64_t shapeHandle, int64_t itemIndex);
        std::string getSignalName(int64_t signalIndex, int64_t signalType);
        int64_t getSimulationState();
        bool getSimulationStopping();
        double getSimulationTime();
        double getSimulationTimeStep();
        std::tuple<int64_t, std::vector<int64_t>, std::vector<int64_t>> getSimulatorMessage();
        std::string getStackTraceback(std::optional<int64_t> scriptHandle = {});
        std::string getStringParam(int64_t parameter);
        std::string getStringProperty(int64_t target, std::string pName, std::optional<json> options = {});
        double getSystemTime();
        json getTableProperty(int64_t target, std::string pName, std::optional<json> options = {});
        std::tuple<int64_t, std::vector<int64_t>> getTextureId(std::string textureName);
        int64_t getThreadId();
        std::vector<std::string> getUserVariables();
        std::vector<double> getVector2Property(int64_t target, std::string pName, std::optional<json> options = {});
        std::vector<double> getVector3Property(int64_t target, std::string pName, std::optional<json> options = {});
        std::tuple<std::vector<double>, std::vector<double>> getVelocity(int64_t shapeHandle);
        std::tuple<std::vector<uint8_t>, std::vector<int64_t>> getVisionSensorDepth(int64_t sensorHandle, std::optional<int64_t> options = {}, std::optional<std::vector<int64_t>> pos = {}, std::optional<std::vector<int64_t>> size = {});
        std::tuple<std::vector<uint8_t>, std::vector<int64_t>> getVisionSensorImg(int64_t sensorHandle, std::optional<int64_t> options = {}, std::optional<double> rgbaCutOff = {}, std::optional<std::vector<int64_t>> pos = {}, std::optional<std::vector<int64_t>> size = {});
        void getVisionSensorRes(int64_t sensorHandle);
        int64_t groupShapes(std::vector<int64_t> shapeHandles, std::optional<bool> merge = {});
        int64_t handleAddOnScripts(int64_t callType);
        int64_t handleDynamics(double deltaTime);
        int64_t handleEmbeddedScripts(int64_t callType);
        void handleExtCalls();
        void handleGraph(int64_t objectHandle, double simulationTime);
        void handleJointMotion();
        std::tuple<int64_t, double, std::vector<double>, int64_t, std::vector<double>> handleProximitySensor(int64_t sensorHandle);
        void handleSandboxScript(int64_t callType);
        void handleSensingStart();
        int64_t handleSimulationScripts(int64_t callType);
        void handleSimulationStart();
        std::tuple<int64_t, std::vector<double>, std::vector<double>> handleVisionSensor(int64_t sensorHandle);
        std::tuple<std::vector<double>, std::vector<int64_t>> importMesh(int64_t fileformat, std::string pathAndFilename, int64_t options, double identicalVerticeTolerance, double scalingFactor);
        int64_t importShape(int64_t fileformat, std::string pathAndFilename, int64_t options, double identicalVerticeTolerance, double scalingFactor);
        void initScript(int64_t scriptHandle);
        int64_t insertObjectIntoOctree(int64_t octreeHandle, int64_t objectHandle, int64_t options, std::optional<std::vector<double>> color = {}, std::optional<int64_t> tag = {});
        int64_t insertObjectIntoPointCloud(int64_t pointCloudHandle, int64_t objectHandle, int64_t options, double gridSize, std::optional<std::vector<double>> color = {}, std::optional<double> duplicateTolerance = {});
        int64_t insertPointsIntoPointCloud(int64_t pointCloudHandle, int64_t options, std::vector<double> points, std::optional<std::vector<double>> color = {}, std::optional<double> duplicateTolerance = {});
        int64_t insertVoxelsIntoOctree(int64_t octreeHandle, int64_t options, std::vector<double> points, std::optional<std::vector<double>> color = {}, std::optional<std::vector<int64_t>> tag = {});
        std::vector<double> interpolateMatrices(std::vector<double> matrixIn1, std::vector<double> matrixIn2, double interpolFactor);
        std::vector<double> interpolatePoses(std::vector<double> poseIn1, std::vector<double> poseIn2, double interpolFactor);
        int64_t intersectPointsWithPointCloud(int64_t pointCloudHandle, int64_t options, std::vector<double> points, double tolerance);
        int64_t isDeprecated(std::string funcOrConst);
        bool isDynamicallyEnabled(int64_t objectHandle);
        bool isHandle(int64_t objectHandle);
        bool isPluginLoaded(std::string name);
        void launchExecutable(std::string filename, std::optional<std::string> parameters = {}, std::optional<int64_t> showStatus = {});
        std::tuple<std::vector<uint8_t>, std::vector<int64_t>> loadImage(int64_t options, std::string filename);
        int64_t loadModel(std::string filename);
        int64_t loadPlugin(std::string name);
        void loadScene(std::string filename);
        std::vector<double> matrixToPose(std::vector<double> matrix);
        int64_t moduleEntry(int64_t handle, std::optional<std::string> label = {}, std::optional<int64_t> state = {});
        json moveToConfig(json params);
        void moveToConfig_cleanup(json motionObject);
        json moveToConfig_init(json params);
        std::tuple<int64_t, json> moveToConfig_step(json motionObject);
        json moveToPose(json params);
        void moveToPose_cleanup(json motionObject);
        json moveToPose_init(json params);
        std::tuple<int64_t, json> moveToPose_step(json motionObject);
        std::vector<double> multiplyMatrices(std::vector<double> matrixIn1, std::vector<double> matrixIn2);
        std::vector<double> multiplyPoses(std::vector<double> poseIn1, std::vector<double> poseIn2);
        std::vector<double> multiplyVector(std::vector<double> matrix, std::vector<double> inVectors);
        std::vector<uint8_t> packDoubleTable(std::vector<double> doubleNumbers, std::optional<int64_t> startDoubleIndex = {}, std::optional<int64_t> doubleCount = {});
        std::vector<uint8_t> packFloatTable(std::vector<double> floatNumbers, std::optional<int64_t> startFloatIndex = {}, std::optional<int64_t> floatCount = {});
        std::vector<uint8_t> packInt32Table(std::vector<int64_t> int32Numbers, std::optional<int64_t> startInt32Index = {}, std::optional<int64_t> int32Count = {});
        std::vector<uint8_t> packTable(std::vector<json> aTable, std::optional<int64_t> scheme = {});
        std::vector<uint8_t> packUInt16Table(std::vector<int64_t> uint16Numbers, std::optional<int64_t> startUint16Index = {}, std::optional<int64_t> uint16Count = {});
        std::vector<uint8_t> packUInt32Table(std::vector<int64_t> uint32Numbers, std::optional<int64_t> startUInt32Index = {}, std::optional<int64_t> uint32Count = {});
        std::vector<uint8_t> packUInt8Table(std::vector<int64_t> uint8Numbers, std::optional<int64_t> startUint8Index = {}, std::optional<int64_t> uint8count = {});
        void pauseSimulation();
        std::vector<double> poseToMatrix(std::vector<double> pose);
        void pushUserEvent(std::string event, int64_t handle, int64_t uid, json eventData, std::optional<int64_t> options = {});
        void quitSimulator();
        std::vector<uint8_t> readCustomBufferData(int64_t objectHandle, std::string tagName);
        std::tuple<std::vector<uint8_t>, std::string> readCustomDataBlockEx(int64_t handle, std::string tag, std::optional<json> options = {});
        std::vector<std::string> readCustomDataTags(int64_t objectHandle);
        std::string readCustomStringData(int64_t objectHandle, std::string tagName);
        json readCustomTableData(int64_t handle, std::string tagName, std::optional<json> options = {});
        std::tuple<int64_t, std::vector<double>, std::vector<double>> readForceSensor(int64_t objectHandle);
        std::tuple<int64_t, double, std::vector<double>, int64_t, std::vector<double>> readProximitySensor(int64_t sensorHandle);
        std::vector<uint8_t> readTexture(int64_t textureId, int64_t options, std::optional<int64_t> posX = {}, std::optional<int64_t> posY = {}, std::optional<int64_t> sizeX = {}, std::optional<int64_t> sizeY = {});
        std::tuple<int64_t, std::vector<double>, std::vector<double>> readVisionSensor(int64_t sensorHandle);
        int64_t refreshDialogs(int64_t refreshDegree);
        int64_t registerScriptFuncHook(std::string funcToHook, std::string userFunc, bool execBefore);
        void releaseLock();
        int64_t relocateShapeFrame(int64_t shapeHandle, std::vector<double> pose);
        void removeDrawingObject(int64_t drawingObjectHandle);
        int64_t removeModel(int64_t objectHandle, std::optional<bool> delayedRemoval = {});
        void removeObjects(std::vector<int64_t> objectHandles, std::optional<bool> delayedRemoval = {});
        void removeParticleObject(int64_t particleObjectHandle);
        int64_t removePointsFromPointCloud(int64_t pointCloudHandle, int64_t options, std::vector<double> points, double tolerance);
        void removeProperty(int64_t target, std::string pName, std::optional<json> options = {});
        void removeReferencedObjects(int64_t objectHandle, std::optional<std::string> tag = {});
        int64_t removeVoxelsFromOctree(int64_t octreeHandle, int64_t options, std::vector<double> points);
        std::vector<double> resamplePath(std::vector<double> path, std::vector<double> pathLengths, int64_t finalConfigCnt, std::optional<json> method = {}, std::optional<std::vector<int64_t>> types = {});
        void resetDynamicObject(int64_t objectHandle);
        void resetGraph(int64_t objectHandle);
        void resetProximitySensor(int64_t objectHandle);
        void resetVisionSensor(int64_t sensorHandle);
        void restoreEntityColor(std::vector<json> originalColorData);
        std::vector<double> rotateAroundAxis(std::vector<double> matrixIn, std::vector<double> axis, std::vector<double> axisPos, double angle);
        int64_t ruckigPos(int64_t dofs, double baseCycleTime, int64_t flags, std::vector<double> currentPosVelAccel, std::vector<double> maxVelAccelJerk, std::vector<int64_t> selection, std::vector<double> targetPosVel);
        void ruckigRemove(int64_t handle);
        std::tuple<int64_t, std::vector<double>, double> ruckigStep(int64_t handle, double cycleTime);
        int64_t ruckigVel(int64_t dofs, double baseCycleTime, int64_t flags, std::vector<double> currentPosVelAccel, std::vector<double> maxAccelJerk, std::vector<int64_t> selection, std::vector<double> targetVel);
        std::vector<uint8_t> saveImage(std::vector<uint8_t> image, std::vector<int64_t> resolution, int64_t options, std::string filename, int64_t quality);
        void saveModel(int64_t modelBaseHandle, std::string filename);
        void saveScene(std::string filename);
        void scaleObject(int64_t objectHandle, double xScale, double yScale, double zScale, std::optional<int64_t> options = {});
        void scaleObjects(std::vector<int64_t> objectHandles, double scalingFactor, bool scalePositionsToo);
        int64_t scheduleExecution(std::string f, std::vector<json> args, double timePoint, std::optional<bool> simTime = {});
        int64_t serialCheck(int64_t portHandle);
        void serialClose(int64_t portHandle);
        int64_t serialOpen(std::string portString, int64_t baudrate);
        std::vector<uint8_t> serialRead(int64_t portHandle, int64_t dataLengthToRead, bool blockingOperation, std::optional<std::vector<uint8_t>> closingString = {}, std::optional<double> timeout = {});
        int64_t serialSend(int64_t portHandle, std::vector<uint8_t> data);
        void setArrayParam(int64_t parameter, std::vector<double> arrayOfValues);
        void setAutoYieldDelay(double dt);
        void setBoolParam(int64_t parameter, bool boolState);
        void setBoolProperty(int64_t target, std::string pName, bool pValue, std::optional<json> options = {});
        void setBufferProperty(int64_t target, std::string pName, std::vector<uint8_t> pValue, std::optional<json> options = {});
        void setBufferSignal(std::string signalName, std::vector<uint8_t> signalValue);
        void setColorProperty(int64_t target, std::string pName, std::vector<double> pValue, std::optional<json> options = {});
        void setEngineBoolParam(int64_t paramId, int64_t objectHandle, bool boolParam);
        void setEngineFloatParam(int64_t paramId, int64_t objectHandle, double floatParam);
        void setEngineInt32Param(int64_t paramId, int64_t objectHandle, int64_t int32Param);
        void setEventFilters(std::optional<json> filters = {});
        void setExplicitHandling(int64_t objectHandle, int64_t explicitHandlingFlags);
        void setFloatArrayProperty(int64_t target, std::string pName, std::vector<double> pValue, std::optional<json> options = {});
        void setFloatParam(int64_t parameter, double floatState);
        void setFloatProperty(int64_t target, std::string pName, double pValue, std::optional<json> options = {});
        void setFloatSignal(std::string signalName, double signalValue);
        void setGraphStreamTransformation(int64_t graphHandle, int64_t streamId, int64_t trType, std::optional<double> mult = {}, std::optional<double> off = {}, std::optional<int64_t> movAvgPeriod = {});
        void setGraphStreamValue(int64_t graphHandle, int64_t streamId, double value);
        void setInt32Param(int64_t parameter, int64_t intState);
        void setInt32Signal(std::string signalName, int64_t signalValue);
        void setIntArray2Property(int64_t target, std::string pName, std::vector<int64_t> pValue, std::optional<json> options = {});
        void setIntArrayProperty(int64_t target, std::string pName, std::vector<int64_t> pValue, std::optional<json> options = {});
        void setIntProperty(int64_t target, std::string pName, int64_t pValue, std::optional<json> options = {});
        void setJointDependency(int64_t jointHandle, int64_t masterJointHandle, double offset, double multCoeff);
        void setJointInterval(int64_t objectHandle, bool cyclic, std::vector<double> interval);
        void setJointMode(int64_t jointHandle, int64_t jointMode);
        void setJointPosition(int64_t objectHandle, double position);
        void setJointTargetForce(int64_t objectHandle, double forceOrTorque, std::optional<bool> signedValue = {});
        void setJointTargetPosition(int64_t objectHandle, double targetPosition, std::optional<std::vector<double>> motionParams = {});
        void setJointTargetVelocity(int64_t objectHandle, double targetVelocity, std::optional<std::vector<double>> motionParams = {});
        void setLightParameters(int64_t lightHandle, int64_t state, std::vector<double> reserved, std::vector<double> diffusePart, std::vector<double> specularPart);
        void setLinkDummy(int64_t dummyHandle, int64_t linkDummyHandle);
        void setLongProperty(int64_t target, std::string pName, int64_t pValue, std::optional<json> options = {});
        void setModelProperty(int64_t objectHandle, int64_t property);
        void setNamedBoolParam(std::string name, bool value);
        void setNamedFloatParam(std::string name, double value);
        void setNamedInt32Param(std::string name, int64_t value);
        void setNamedStringParam(std::string paramName, std::vector<uint8_t> stringParam);
        void setNavigationMode(int64_t navigationMode);
        void setObjectAlias(int64_t objectHandle, std::string objectAlias);
        void setObjectChildPose(int64_t objectHandle, std::vector<double> pose);
        bool setObjectColor(int64_t objectHandle, int64_t index, int64_t colorComponent, std::vector<double> rgbData);
        void setObjectFloatArrayParam(int64_t objectHandle, int64_t parameterID, std::vector<double> params);
        void setObjectFloatParam(int64_t objectHandle, int64_t parameterID, double parameter);
        void setObjectHierarchyOrder(int64_t objectHandle, int64_t order);
        void setObjectInt32Param(int64_t objectHandle, int64_t parameterID, int64_t parameter);
        void setObjectMatrix(int64_t objectHandle, std::vector<double> matrix, std::optional<int64_t> relativeToObjectHandle = {});
        void setObjectOrientation(int64_t objectHandle, std::vector<double> eulerAngles, std::optional<int64_t> relativeToObjectHandle = {});
        void setObjectParent(int64_t objectHandle, int64_t parentObjectHandle, std::optional<bool> keepInPlace = {});
        void setObjectPose(int64_t objectHandle, std::vector<double> pose, std::optional<int64_t> relativeToObjectHandle = {});
        void setObjectPosition(int64_t objectHandle, std::vector<double> position, std::optional<int64_t> relativeToObjectHandle = {});
        void setObjectProperty(int64_t objectHandle, int64_t property);
        void setObjectQuaternion(int64_t objectHandle, std::vector<double> quaternion, std::optional<int64_t> relativeToObjectHandle = {});
        void setObjectSel(std::vector<int64_t> objectHandles);
        void setObjectSpecialProperty(int64_t objectHandle, int64_t property);
        void setObjectStringParam(int64_t objectHandle, int64_t parameterID, std::vector<uint8_t> parameter);
        void setPage(int64_t pageIndex);
        void setPluginInfo(std::string pluginName, int64_t infoType, std::string info);
        void setPointCloudOptions(int64_t pointCloudHandle, double maxVoxelSize, int64_t maxPtCntPerVoxel, int64_t options, double pointSize);
        void setPoseProperty(int64_t target, std::string pName, std::vector<double> pValue, std::optional<json> options = {});
        void setProperties(int64_t target, json props);
        void setProperty(int64_t target, std::string pName, json pValue, std::optional<int64_t> pType = {});
        void setQuaternionProperty(int64_t target, std::string pName, std::vector<double> pValue, std::optional<json> options = {});
        void setReferencedHandles(int64_t objectHandle, std::vector<int64_t> referencedHandles, std::optional<std::string> tag = {});
        int64_t setShapeAppearance(int64_t handle, json savedData, std::optional<json> opts = {});
        void setShapeBB(int64_t shapeHandle, std::vector<double> size);
        void setShapeColor(int64_t shapeHandle, std::string colorName, int64_t colorComponent, std::vector<double> rgbData);
        void setShapeInertia(int64_t shapeHandle, std::vector<double> inertiaMatrix, std::vector<double> comMatrix);
        void setShapeMass(int64_t shapeHandle, double mass);
        void setShapeMaterial(int64_t shapeHandle, int64_t materialIdOrShapeHandle);
        void setShapeTexture(int64_t shapeHandle, int64_t textureId, int64_t mappingMode, int64_t options, std::vector<double> uvScaling, std::optional<std::vector<double>> position = {}, std::optional<std::vector<double>> orientation = {});
        int64_t setStepping(bool enabled);
        void setStringParam(int64_t parameter, std::string stringState);
        void setStringProperty(int64_t target, std::string pName, std::string pValue, std::optional<json> options = {});
        void setStringSignal(std::string signalName, std::string signalValue);
        void setTableProperty(int64_t target, std::string pName, json pValue, std::optional<json> options = {});
        void setVector2Property(int64_t target, std::string pName, std::vector<double> pValue, std::optional<json> options = {});
        void setVector3Property(int64_t target, std::string pName, std::vector<double> pValue, std::optional<json> options = {});
        void setVisionSensorImg(int64_t sensorHandle, std::vector<uint8_t> image, std::optional<int64_t> options = {}, std::optional<std::vector<int64_t>> pos = {}, std::optional<std::vector<int64_t>> size = {});
        void startSimulation();
        void step();
        void stopSimulation(std::optional<bool> wait = {});
        int64_t subtractObjectFromOctree(int64_t octreeHandle, int64_t objectHandle, int64_t options);
        int64_t subtractObjectFromPointCloud(int64_t pointCloudHandle, int64_t objectHandle, int64_t options, double tolerance);
        void systemSemaphore(std::string key, bool acquire);
        int64_t testCB(int64_t a, std::string cb, int64_t b);
        std::tuple<std::string, std::vector<int64_t>, std::vector<int64_t>> textEditorClose(int64_t handle);
        std::tuple<std::string, std::vector<int64_t>, std::vector<int64_t>, bool> textEditorGetInfo(int64_t handle);
        int64_t textEditorOpen(std::string initText, std::string properties);
        void textEditorShow(int64_t handle, bool showState);
        std::vector<uint8_t> transformBuffer(std::vector<uint8_t> inBuffer, int64_t inFormat, double multiplier, double offset, int64_t outFormat);
        std::vector<uint8_t> transformImage(std::vector<uint8_t> image, std::vector<int64_t> resolution, int64_t options);
        std::vector<int64_t> ungroupShape(int64_t shapeHandle);
        std::vector<double> unpackDoubleTable(std::vector<uint8_t> data, std::optional<int64_t> startDoubleIndex = {}, std::optional<int64_t> doubleCount = {}, std::optional<int64_t> additionalByteOffset = {});
        std::vector<double> unpackFloatTable(std::vector<uint8_t> data, std::optional<int64_t> startFloatIndex = {}, std::optional<int64_t> floatCount = {}, std::optional<int64_t> additionalByteOffset = {});
        std::vector<int64_t> unpackInt32Table(std::vector<uint8_t> data, std::optional<int64_t> startInt32Index = {}, std::optional<int64_t> int32Count = {}, std::optional<int64_t> additionalByteOffset = {});
        json unpackTable(std::vector<uint8_t> buffer);
        std::vector<int64_t> unpackUInt16Table(std::vector<uint8_t> data, std::optional<int64_t> startUint16Index = {}, std::optional<int64_t> uint16Count = {}, std::optional<int64_t> additionalByteOffset = {});
        std::vector<int64_t> unpackUInt32Table(std::vector<uint8_t> data, std::optional<int64_t> startUint32Index = {}, std::optional<int64_t> uint32Count = {}, std::optional<int64_t> additionalByteOffset = {});
        std::vector<int64_t> unpackUInt8Table(std::vector<uint8_t> data, std::optional<int64_t> startUint8Index = {}, std::optional<int64_t> uint8count = {});
        void visitTree(int64_t rootHandle, std::string visitorFunc, std::optional<json> options = {});
        double wait(double dt, std::optional<bool> simulationTime = {});
        json waitForSignal(int64_t target, std::string sigName);
        void writeCustomBufferData(int64_t objectHandle, std::string tagName, std::vector<uint8_t> data);
        void writeCustomDataBlockEx(int64_t handle, std::string tag, std::vector<uint8_t> data, std::optional<json> options = {});
        void writeCustomStringData(int64_t objectHandle, std::string tagName, std::string data);
        void writeCustomTableData(int64_t handle, std::string tagName, json theTable, std::optional<json> options = {});
        void writeTexture(int64_t textureId, int64_t options, std::vector<uint8_t> textureData, std::optional<int64_t> posX = {}, std::optional<int64_t> posY = {}, std::optional<int64_t> sizeX = {}, std::optional<int64_t> sizeY = {}, std::optional<double> interpol = {});
        std::tuple<double, double, double> yawPitchRollToAlphaBetaGamma(double yawAngle, double pitchAngle, double rollAngle);
        void yield();

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
#ifndef arrayparam_raydirection
        const int arrayparam_raydirection = 8;
#endif
#ifndef arrayparam_rayorigin
        const int arrayparam_rayorigin = 7;
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
#ifndef boolparam_cansave
        const int boolparam_cansave = 60;
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
#ifndef boolparam_execunsafe
        const int boolparam_execunsafe = 58;
#endif
#ifndef boolparam_execunsafeext
        const int boolparam_execunsafeext = 59;
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
#ifndef boolparam_rayvalid
        const int boolparam_rayvalid = 56;
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
#ifndef boolparam_usingscriptobjects
        const int boolparam_usingscriptobjects = 61;
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
#ifndef bullet_global_computeinertias
        const int bullet_global_computeinertias = 2002;
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
#ifndef bullet_joint_pospid1
        const int bullet_joint_pospid1 = 3006;
#endif
#ifndef bullet_joint_pospid2
        const int bullet_joint_pospid2 = 3007;
#endif
#ifndef bullet_joint_pospid3
        const int bullet_joint_pospid3 = 3008;
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
#ifndef camerafarrayparam_viewfrustum
        const int camerafarrayparam_viewfrustum = 9013;
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
#ifndef drawing_local
        const int drawing_local = 8388608;
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
#ifndef dummyintparam_dummytype
        const int dummyintparam_dummytype = 10000;
#endif
#ifndef dummyintparam_follow_path
        const int dummyintparam_follow_path = 10001;
#endif
#ifndef dummyintparam_link_type
        const int dummyintparam_link_type = 10000;
#endif
#ifndef dummylink_dynloopclosure
        const int dummylink_dynloopclosure = 0;
#endif
#ifndef dummylink_dyntendon
        const int dummylink_dyntendon = 7;
#endif
#ifndef dummystringparam_assemblytag
        const int dummystringparam_assemblytag = 10004;
#endif
#ifndef dummytype_assembly
        const int dummytype_assembly = 9;
#endif
#ifndef dummytype_default
        const int dummytype_default = 8;
#endif
#ifndef dummytype_dynloopclosure
        const int dummytype_dynloopclosure = 0;
#endif
#ifndef dummytype_dyntendon
        const int dummytype_dyntendon = 7;
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
#ifndef floatparam_maxtrisizeabs
        const int floatparam_maxtrisizeabs = 6;
#endif
#ifndef floatparam_mintrisizerel
        const int floatparam_mintrisizerel = 7;
#endif
#ifndef floatparam_mouse_wheel_zoom_factor
        const int floatparam_mouse_wheel_zoom_factor = 4;
#endif
#ifndef floatparam_physicstimestep
        const int floatparam_physicstimestep = 5;
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
#ifndef handle_appstorage
        const int handle_appstorage = -15;
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
#ifndef handle_mainscript
        const int handle_mainscript = -5;
#endif
#ifndef handle_mesh
        const int handle_mesh = -18;
#endif
#ifndef handle_parent
        const int handle_parent = -11;
#endif
#ifndef handle_sandbox
        const int handle_sandbox = -17;
#endif
#ifndef handle_scene
        const int handle_scene = -12;
#endif
#ifndef handle_sceneobject
        const int handle_sceneobject = -16;
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
#ifndef handleflag_addmultiple
        const int handleflag_addmultiple = 16777216;
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
#ifndef handleflag_wxyzquat
        const int handleflag_wxyzquat = 16777216;
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
#ifndef intparam_hierarchychangecounter
        const int intparam_hierarchychangecounter = 50;
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
#ifndef intparam_mouseclickcounterdown
        const int intparam_mouseclickcounterdown = 46;
#endif
#ifndef intparam_mouseclickcounterup
        const int intparam_mouseclickcounterup = 47;
#endif
#ifndef intparam_notifydeprecated
        const int intparam_notifydeprecated = 51;
#endif
#ifndef intparam_objectcreationcounter
        const int intparam_objectcreationcounter = 48;
#endif
#ifndef intparam_objectdestructioncounter
        const int intparam_objectdestructioncounter = 49;
#endif
#ifndef intparam_platform
        const int intparam_platform = 19;
#endif
#ifndef intparam_processcnt
        const int intparam_processcnt = 53;
#endif
#ifndef intparam_processid
        const int intparam_processid = 52;
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
#ifndef joint_prismatic
        const int joint_prismatic = 11;
#endif
#ifndef joint_prismatic_subtype
        const int joint_prismatic_subtype = 11;
#endif
#ifndef joint_revolute
        const int joint_revolute = 10;
#endif
#ifndef joint_revolute_subtype
        const int joint_revolute_subtype = 10;
#endif
#ifndef joint_spherical
        const int joint_spherical = 12;
#endif
#ifndef joint_spherical_subtype
        const int joint_spherical_subtype = 12;
#endif
#ifndef jointdynctrl_callback
        const int jointdynctrl_callback = 16;
#endif
#ifndef jointdynctrl_force
        const int jointdynctrl_force = 1;
#endif
#ifndef jointdynctrl_free
        const int jointdynctrl_free = 0;
#endif
#ifndef jointdynctrl_position
        const int jointdynctrl_position = 8;
#endif
#ifndef jointdynctrl_spring
        const int jointdynctrl_spring = 12;
#endif
#ifndef jointdynctrl_velocity
        const int jointdynctrl_velocity = 4;
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
#ifndef jointfloatparam_maxaccel
        const int jointfloatparam_maxaccel = 2037;
#endif
#ifndef jointfloatparam_maxjerk
        const int jointfloatparam_maxjerk = 2038;
#endif
#ifndef jointfloatparam_maxvel
        const int jointfloatparam_maxvel = 2036;
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
#ifndef jointfloatparam_screwlead
        const int jointfloatparam_screwlead = 2042;
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
#ifndef jointintparam_dynctrlmode
        const int jointintparam_dynctrlmode = 2039;
#endif
#ifndef jointintparam_dynposctrltype
        const int jointintparam_dynposctrltype = 2041;
#endif
#ifndef jointintparam_dynvelctrltype
        const int jointintparam_dynvelctrltype = 2040;
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
#ifndef jointmode_dynamic
        const int jointmode_dynamic = 5;
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
#ifndef jointmode_kinematic
        const int jointmode_kinematic = 0;
#endif
#ifndef jointmode_passive
        const int jointmode_passive = 0;
#endif
#ifndef light_directional
        const int light_directional = 3;
#endif
#ifndef light_directional_subtype
        const int light_directional_subtype = 3;
#endif
#ifndef light_omnidirectional
        const int light_omnidirectional = 1;
#endif
#ifndef light_omnidirectional_subtype
        const int light_omnidirectional_subtype = 1;
#endif
#ifndef light_spot
        const int light_spot = 2;
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
#ifndef mujoco_body_condim
        const int mujoco_body_condim = 44001;
#endif
#ifndef mujoco_body_friction1
        const int mujoco_body_friction1 = 43001;
#endif
#ifndef mujoco_body_friction2
        const int mujoco_body_friction2 = 43002;
#endif
#ifndef mujoco_body_friction3
        const int mujoco_body_friction3 = 43003;
#endif
#ifndef mujoco_body_margin
        const int mujoco_body_margin = 43012;
#endif
#ifndef mujoco_body_priority
        const int mujoco_body_priority = 44002;
#endif
#ifndef mujoco_body_solimp1
        const int mujoco_body_solimp1 = 43006;
#endif
#ifndef mujoco_body_solimp2
        const int mujoco_body_solimp2 = 43007;
#endif
#ifndef mujoco_body_solimp3
        const int mujoco_body_solimp3 = 43008;
#endif
#ifndef mujoco_body_solimp4
        const int mujoco_body_solimp4 = 43009;
#endif
#ifndef mujoco_body_solimp5
        const int mujoco_body_solimp5 = 43010;
#endif
#ifndef mujoco_body_solmix
        const int mujoco_body_solmix = 43011;
#endif
#ifndef mujoco_body_solref1
        const int mujoco_body_solref1 = 43004;
#endif
#ifndef mujoco_body_solref2
        const int mujoco_body_solref2 = 43005;
#endif
#ifndef mujoco_dummy_bitcoded
        const int mujoco_dummy_bitcoded = 47001;
#endif
#ifndef mujoco_dummy_damping
        const int mujoco_dummy_damping = 46013;
#endif
#ifndef mujoco_dummy_limited
        const int mujoco_dummy_limited = 48001;
#endif
#ifndef mujoco_dummy_margin
        const int mujoco_dummy_margin = 46010;
#endif
#ifndef mujoco_dummy_proxyjointid
        const int mujoco_dummy_proxyjointid = 47002;
#endif
#ifndef mujoco_dummy_range1
        const int mujoco_dummy_range1 = 46001;
#endif
#ifndef mujoco_dummy_range2
        const int mujoco_dummy_range2 = 46002;
#endif
#ifndef mujoco_dummy_solimplimit1
        const int mujoco_dummy_solimplimit1 = 46005;
#endif
#ifndef mujoco_dummy_solimplimit2
        const int mujoco_dummy_solimplimit2 = 46006;
#endif
#ifndef mujoco_dummy_solimplimit3
        const int mujoco_dummy_solimplimit3 = 46007;
#endif
#ifndef mujoco_dummy_solimplimit4
        const int mujoco_dummy_solimplimit4 = 46008;
#endif
#ifndef mujoco_dummy_solimplimit5
        const int mujoco_dummy_solimplimit5 = 46009;
#endif
#ifndef mujoco_dummy_solreflimit1
        const int mujoco_dummy_solreflimit1 = 46003;
#endif
#ifndef mujoco_dummy_solreflimit2
        const int mujoco_dummy_solreflimit2 = 46004;
#endif
#ifndef mujoco_dummy_springlength
        const int mujoco_dummy_springlength = 46011;
#endif
#ifndef mujoco_dummy_stiffness
        const int mujoco_dummy_stiffness = 46012;
#endif
#ifndef mujoco_global_balanceinertias
        const int mujoco_global_balanceinertias = 39004;
#endif
#ifndef mujoco_global_bitcoded
        const int mujoco_global_bitcoded = 38001;
#endif
#ifndef mujoco_global_boundinertia
        const int mujoco_global_boundinertia = 37009;
#endif
#ifndef mujoco_global_boundmass
        const int mujoco_global_boundmass = 37008;
#endif
#ifndef mujoco_global_computeinertias
        const int mujoco_global_computeinertias = 39001;
#endif
#ifndef mujoco_global_cone
        const int mujoco_global_cone = 38007;
#endif
#ifndef mujoco_global_density
        const int mujoco_global_density = 37006;
#endif
#ifndef mujoco_global_impratio
        const int mujoco_global_impratio = 37002;
#endif
#ifndef mujoco_global_integrator
        const int mujoco_global_integrator = 38003;
#endif
#ifndef mujoco_global_iterations
        const int mujoco_global_iterations = 38002;
#endif
#ifndef mujoco_global_kininertia
        const int mujoco_global_kininertia = 37019;
#endif
#ifndef mujoco_global_kinmass
        const int mujoco_global_kinmass = 37018;
#endif
#ifndef mujoco_global_multiccd
        const int mujoco_global_multiccd = 39003;
#endif
#ifndef mujoco_global_multithreaded
        const int mujoco_global_multithreaded = 39002;
#endif
#ifndef mujoco_global_nconmax
        const int mujoco_global_nconmax = 38006;
#endif
#ifndef mujoco_global_njmax
        const int mujoco_global_njmax = 38005;
#endif
#ifndef mujoco_global_nstack
        const int mujoco_global_nstack = 38009;
#endif
#ifndef mujoco_global_overridecontacts
        const int mujoco_global_overridecontacts = 39005;
#endif
#ifndef mujoco_global_overridekin
        const int mujoco_global_overridekin = 38008;
#endif
#ifndef mujoco_global_overridemargin
        const int mujoco_global_overridemargin = 37010;
#endif
#ifndef mujoco_global_overridesolimp1
        const int mujoco_global_overridesolimp1 = 37013;
#endif
#ifndef mujoco_global_overridesolimp2
        const int mujoco_global_overridesolimp2 = 37014;
#endif
#ifndef mujoco_global_overridesolimp3
        const int mujoco_global_overridesolimp3 = 37015;
#endif
#ifndef mujoco_global_overridesolimp4
        const int mujoco_global_overridesolimp4 = 37016;
#endif
#ifndef mujoco_global_overridesolimp5
        const int mujoco_global_overridesolimp5 = 37017;
#endif
#ifndef mujoco_global_overridesolref1
        const int mujoco_global_overridesolref1 = 37011;
#endif
#ifndef mujoco_global_overridesolref2
        const int mujoco_global_overridesolref2 = 37012;
#endif
#ifndef mujoco_global_rebuildtrigger
        const int mujoco_global_rebuildtrigger = 38010;
#endif
#ifndef mujoco_global_solver
        const int mujoco_global_solver = 38004;
#endif
#ifndef mujoco_global_viscosity
        const int mujoco_global_viscosity = 37007;
#endif
#ifndef mujoco_global_wind1
        const int mujoco_global_wind1 = 37003;
#endif
#ifndef mujoco_global_wind2
        const int mujoco_global_wind2 = 37004;
#endif
#ifndef mujoco_global_wind3
        const int mujoco_global_wind3 = 37005;
#endif
#ifndef mujoco_joint_armature
        const int mujoco_joint_armature = 40021;
#endif
#ifndef mujoco_joint_damping
        const int mujoco_joint_damping = 40017;
#endif
#ifndef mujoco_joint_dependentobjectid
        const int mujoco_joint_dependentobjectid = 41002;
#endif
#ifndef mujoco_joint_frictionloss
        const int mujoco_joint_frictionloss = 40008;
#endif
#ifndef mujoco_joint_margin
        const int mujoco_joint_margin = 40022;
#endif
#ifndef mujoco_joint_polycoef1
        const int mujoco_joint_polycoef1 = 40023;
#endif
#ifndef mujoco_joint_polycoef2
        const int mujoco_joint_polycoef2 = 40024;
#endif
#ifndef mujoco_joint_polycoef3
        const int mujoco_joint_polycoef3 = 40025;
#endif
#ifndef mujoco_joint_polycoef4
        const int mujoco_joint_polycoef4 = 40026;
#endif
#ifndef mujoco_joint_polycoef5
        const int mujoco_joint_polycoef5 = 40027;
#endif
#ifndef mujoco_joint_pospid1
        const int mujoco_joint_pospid1 = 40028;
#endif
#ifndef mujoco_joint_pospid2
        const int mujoco_joint_pospid2 = 40029;
#endif
#ifndef mujoco_joint_pospid3
        const int mujoco_joint_pospid3 = 40030;
#endif
#ifndef mujoco_joint_solimpfriction1
        const int mujoco_joint_solimpfriction1 = 40011;
#endif
#ifndef mujoco_joint_solimpfriction2
        const int mujoco_joint_solimpfriction2 = 40012;
#endif
#ifndef mujoco_joint_solimpfriction3
        const int mujoco_joint_solimpfriction3 = 40013;
#endif
#ifndef mujoco_joint_solimpfriction4
        const int mujoco_joint_solimpfriction4 = 40014;
#endif
#ifndef mujoco_joint_solimpfriction5
        const int mujoco_joint_solimpfriction5 = 40015;
#endif
#ifndef mujoco_joint_solimplimit1
        const int mujoco_joint_solimplimit1 = 40003;
#endif
#ifndef mujoco_joint_solimplimit2
        const int mujoco_joint_solimplimit2 = 40004;
#endif
#ifndef mujoco_joint_solimplimit3
        const int mujoco_joint_solimplimit3 = 40005;
#endif
#ifndef mujoco_joint_solimplimit4
        const int mujoco_joint_solimplimit4 = 40006;
#endif
#ifndef mujoco_joint_solimplimit5
        const int mujoco_joint_solimplimit5 = 40007;
#endif
#ifndef mujoco_joint_solreffriction1
        const int mujoco_joint_solreffriction1 = 40009;
#endif
#ifndef mujoco_joint_solreffriction2
        const int mujoco_joint_solreffriction2 = 40010;
#endif
#ifndef mujoco_joint_solreflimit1
        const int mujoco_joint_solreflimit1 = 40001;
#endif
#ifndef mujoco_joint_solreflimit2
        const int mujoco_joint_solreflimit2 = 40002;
#endif
#ifndef mujoco_joint_springdamper1
        const int mujoco_joint_springdamper1 = 40019;
#endif
#ifndef mujoco_joint_springdamper2
        const int mujoco_joint_springdamper2 = 40020;
#endif
#ifndef mujoco_joint_springref
        const int mujoco_joint_springref = 40018;
#endif
#ifndef mujoco_joint_stiffness
        const int mujoco_joint_stiffness = 40016;
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
#ifndef newton_global_computeinertias
        const int newton_global_computeinertias = 29004;
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
#ifndef newton_joint_pospid1
        const int newton_joint_pospid1 = 30003;
#endif
#ifndef newton_joint_pospid2
        const int newton_joint_pospid2 = 30004;
#endif
#ifndef newton_joint_pospid3
        const int newton_joint_pospid3 = 30005;
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
#ifndef object_script_type
        const int object_script_type = 17;
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
#ifndef objectproperty_hiddenforsimulation
        const int objectproperty_hiddenforsimulation = 65536;
#endif
#ifndef objectproperty_hierarchyhiddenmodelchild
        const int objectproperty_hierarchyhiddenmodelchild = 32768;
#endif
#ifndef objectproperty_ignoreviewfitting
        const int objectproperty_ignoreviewfitting = 1;
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
#ifndef objecttype_collection
        const int objecttype_collection = 115;
#endif
#ifndef objecttype_interfacestack
        const int objecttype_interfacestack = 123;
#endif
#ifndef objecttype_mesh
        const int objecttype_mesh = 122;
#endif
#ifndef objecttype_sceneobject
        const int objecttype_sceneobject = 109;
#endif
#ifndef objecttype_script
        const int objecttype_script = 117;
#endif
#ifndef objecttype_texture
        const int objecttype_texture = 120;
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
#ifndef objintparam_hierarchycolor
        const int objintparam_hierarchycolor = 38;
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
#ifndef octreefloatparam_voxelsize
        const int octreefloatparam_voxelsize = 13000;
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
#ifndef ode_global_computeinertias
        const int ode_global_computeinertias = 11003;
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
#ifndef ode_joint_pospid1
        const int ode_joint_pospid1 = 12006;
#endif
#ifndef ode_joint_pospid2
        const int ode_joint_pospid2 = 12007;
#endif
#ifndef ode_joint_pospid3
        const int ode_joint_pospid3 = 12008;
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
#ifndef physics_drake
        const int physics_drake = 5;
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
#ifndef plugininfo_builddatestr
        const int plugininfo_builddatestr = 1;
#endif
#ifndef plugininfo_extversionint
        const int plugininfo_extversionint = 2;
#endif
#ifndef plugininfo_extversionstr
        const int plugininfo_extversionstr = 0;
#endif
#ifndef plugininfo_statusbarverbosity
        const int plugininfo_statusbarverbosity = 4;
#endif
#ifndef plugininfo_verbosity
        const int plugininfo_verbosity = 3;
#endif
#ifndef primitiveshape_capsule
        const int primitiveshape_capsule = 8;
#endif
#ifndef primitiveshape_cone
        const int primitiveshape_cone = 6;
#endif
#ifndef primitiveshape_cuboid
        const int primitiveshape_cuboid = 3;
#endif
#ifndef primitiveshape_cylinder
        const int primitiveshape_cylinder = 5;
#endif
#ifndef primitiveshape_disc
        const int primitiveshape_disc = 2;
#endif
#ifndef primitiveshape_heightfield
        const int primitiveshape_heightfield = 7;
#endif
#ifndef primitiveshape_none
        const int primitiveshape_none = 0;
#endif
#ifndef primitiveshape_plane
        const int primitiveshape_plane = 1;
#endif
#ifndef primitiveshape_spheroid
        const int primitiveshape_spheroid = 4;
#endif
#ifndef propertyinfo_deprecated
        const int propertyinfo_deprecated = 16;
#endif
#ifndef propertyinfo_largedata
        const int propertyinfo_largedata = 256;
#endif
#ifndef propertyinfo_modelhashexclude
        const int propertyinfo_modelhashexclude = 8;
#endif
#ifndef propertyinfo_notreadable
        const int propertyinfo_notreadable = 2;
#endif
#ifndef propertyinfo_notwritable
        const int propertyinfo_notwritable = 1;
#endif
#ifndef propertyinfo_removable
        const int propertyinfo_removable = 4;
#endif
#ifndef propertytype_bool
        const int propertytype_bool = 0;
#endif
#ifndef propertytype_buffer
        const int propertytype_buffer = 4;
#endif
#ifndef propertytype_color
        const int propertytype_color = 10;
#endif
#ifndef propertytype_float
        const int propertytype_float = 2;
#endif
#ifndef propertytype_floatarray
        const int propertytype_floatarray = 11;
#endif
#ifndef propertytype_int
        const int propertytype_int = 1;
#endif
#ifndef propertytype_intarray
        const int propertytype_intarray = 13;
#endif
#ifndef propertytype_intarray2
        const int propertytype_intarray2 = 15;
#endif
#ifndef propertytype_long
        const int propertytype_long = 16;
#endif
#ifndef propertytype_matrix3x3
        const int propertytype_matrix3x3 = 8;
#endif
#ifndef propertytype_matrix4x4
        const int propertytype_matrix4x4 = 9;
#endif
#ifndef propertytype_pose
        const int propertytype_pose = 7;
#endif
#ifndef propertytype_quaternion
        const int propertytype_quaternion = 6;
#endif
#ifndef propertytype_string
        const int propertytype_string = 3;
#endif
#ifndef propertytype_table
        const int propertytype_table = 12;
#endif
#ifndef propertytype_vector2
        const int propertytype_vector2 = 14;
#endif
#ifndef propertytype_vector3
        const int propertytype_vector3 = 5;
#endif
#ifndef proximitysensor_cone
        const int proximitysensor_cone = 33;
#endif
#ifndef proximitysensor_cone_subtype
        const int proximitysensor_cone_subtype = 33;
#endif
#ifndef proximitysensor_cylinder
        const int proximitysensor_cylinder = 31;
#endif
#ifndef proximitysensor_cylinder_subtype
        const int proximitysensor_cylinder_subtype = 31;
#endif
#ifndef proximitysensor_disc
        const int proximitysensor_disc = 32;
#endif
#ifndef proximitysensor_disc_subtype
        const int proximitysensor_disc_subtype = 32;
#endif
#ifndef proximitysensor_pyramid
        const int proximitysensor_pyramid = 30;
#endif
#ifndef proximitysensor_pyramid_subtype
        const int proximitysensor_pyramid_subtype = 30;
#endif
#ifndef proximitysensor_ray
        const int proximitysensor_ray = 34;
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
        const int rml_disable_extremum_motion_states_calc = 0;
#endif
#ifndef rml_keep_current_vel_if_fallback_strategy
        const int rml_keep_current_vel_if_fallback_strategy = 0;
#endif
#ifndef rml_keep_target_vel
        const int rml_keep_target_vel = 0;
#endif
#ifndef rml_no_sync
        const int rml_no_sync = 3;
#endif
#ifndef rml_only_phase_sync
        const int rml_only_phase_sync = 1;
#endif
#ifndef rml_only_time_sync
        const int rml_only_time_sync = 1;
#endif
#ifndef rml_phase_sync_if_possible
        const int rml_phase_sync_if_possible = 0;
#endif
#ifndef rml_recompute_trajectory
        const int rml_recompute_trajectory = 0;
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
#ifndef sceneobject_camera
        const int sceneobject_camera = 3;
#endif
#ifndef sceneobject_dummy
        const int sceneobject_dummy = 4;
#endif
#ifndef sceneobject_forcesensor
        const int sceneobject_forcesensor = 12;
#endif
#ifndef sceneobject_graph
        const int sceneobject_graph = 2;
#endif
#ifndef sceneobject_joint
        const int sceneobject_joint = 1;
#endif
#ifndef sceneobject_light
        const int sceneobject_light = 13;
#endif
#ifndef sceneobject_mill
        const int sceneobject_mill = 11;
#endif
#ifndef sceneobject_mirror
        const int sceneobject_mirror = 14;
#endif
#ifndef sceneobject_octree
        const int sceneobject_octree = 15;
#endif
#ifndef sceneobject_path
        const int sceneobject_path = 8;
#endif
#ifndef sceneobject_pointcloud
        const int sceneobject_pointcloud = 16;
#endif
#ifndef sceneobject_proximitysensor
        const int sceneobject_proximitysensor = 5;
#endif
#ifndef sceneobject_renderingsensor
        const int sceneobject_renderingsensor = 9;
#endif
#ifndef sceneobject_script
        const int sceneobject_script = 17;
#endif
#ifndef sceneobject_shape
        const int sceneobject_shape = 0;
#endif
#ifndef sceneobject_visionsensor
        const int sceneobject_visionsensor = 9;
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
#ifndef scriptintparam_autorestartonerror
        const int scriptintparam_autorestartonerror = 10107;
#endif
#ifndef scriptintparam_enabled
        const int scriptintparam_enabled = 10104;
#endif
#ifndef scriptintparam_execcount
        const int scriptintparam_execcount = 10101;
#endif
#ifndef scriptintparam_execorder
        const int scriptintparam_execorder = 10100;
#endif
#ifndef scriptintparam_handle
        const int scriptintparam_handle = 10103;
#endif
#ifndef scriptintparam_lang
        const int scriptintparam_lang = 10106;
#endif
#ifndef scriptintparam_objecthandle
        const int scriptintparam_objecthandle = 10105;
#endif
#ifndef scriptintparam_type
        const int scriptintparam_type = 10102;
#endif
#ifndef scriptstringparam_description
        const int scriptstringparam_description = 10108;
#endif
#ifndef scriptstringparam_lang
        const int scriptstringparam_lang = 10112;
#endif
#ifndef scriptstringparam_name
        const int scriptstringparam_name = 10109;
#endif
#ifndef scriptstringparam_nameext
        const int scriptstringparam_nameext = 10111;
#endif
#ifndef scriptstringparam_text
        const int scriptstringparam_text = 10110;
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
#ifndef scripttype_addon
        const int scripttype_addon = 2;
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
#ifndef scripttype_customization
        const int scripttype_customization = 6;
#endif
#ifndef scripttype_customizationscript
        const int scripttype_customizationscript = 6;
#endif
#ifndef scripttype_main
        const int scripttype_main = 0;
#endif
#ifndef scripttype_mainscript
        const int scripttype_mainscript = 0;
#endif
#ifndef scripttype_passive
        const int scripttype_passive = 9;
#endif
#ifndef scripttype_sandbox
        const int scripttype_sandbox = 8;
#endif
#ifndef scripttype_sandboxscript
        const int scripttype_sandboxscript = 8;
#endif
#ifndef scripttype_simulation
        const int scripttype_simulation = 1;
#endif
#ifndef scripttype_threaded
        const int scripttype_threaded = 240;
#endif
#ifndef shape_compound
        const int shape_compound = 21;
#endif
#ifndef shape_multishape_subtype
        const int shape_multishape_subtype = 21;
#endif
#ifndef shape_simple
        const int shape_simple = 20;
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
#ifndef shapeintparam_kinematic
        const int shapeintparam_kinematic = 3030;
#endif
#ifndef shapeintparam_respondable
        const int shapeintparam_respondable = 3004;
#endif
#ifndef shapeintparam_respondable_mask
        const int shapeintparam_respondable_mask = 3019;
#endif
#ifndef shapeintparam_respondablesuspendcnt
        const int shapeintparam_respondablesuspendcnt = 3031;
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
#ifndef shapestringparam_colorname
        const int shapestringparam_colorname = 3032;
#endif
#ifndef sim_lang_lua
        const int sim_lang_lua = 0;
#endif
#ifndef sim_lang_python
        const int sim_lang_python = 1;
#endif
#ifndef sim_lang_undefined
        const int sim_lang_undefined = -1;
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
#ifndef stringparam_addondir
        const int stringparam_addondir = 143;
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
#ifndef stringparam_legacymachinetag
        const int stringparam_legacymachinetag = 142;
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
#ifndef stringparam_mujocodir
        const int stringparam_mujocodir = 138;
#endif
#ifndef stringparam_pythondir
        const int stringparam_pythondir = 137;
#endif
#ifndef stringparam_remoteapi_temp_file_dir
        const int stringparam_remoteapi_temp_file_dir = 16;
#endif
#ifndef stringparam_resourcesdir
        const int stringparam_resourcesdir = 141;
#endif
#ifndef stringparam_sandboxlang
        const int stringparam_sandboxlang = 144;
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
#ifndef stringparam_systemdir
        const int stringparam_systemdir = 140;
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
#ifndef stringparam_usersettingsdir
        const int stringparam_usersettingsdir = 139;
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
#ifndef syscb_contact
        const int syscb_contact = 41;
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
#ifndef syscb_data
        const int syscb_data = 45;
#endif
#ifndef syscb_dyn
        const int syscb_dyn = 40;
#endif
#ifndef syscb_dyncallback
        const int syscb_dyncallback = 28;
#endif
#ifndef syscb_init
        const int syscb_init = 2;
#endif
#ifndef syscb_joint
        const int syscb_joint = 42;
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
#ifndef syscb_selchange
        const int syscb_selchange = 44;
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
#ifndef syscb_thread
        const int syscb_thread = 43;
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
#ifndef verbosity_once
        const int verbosity_once = 131072;
#endif
#ifndef verbosity_onlyterminal
        const int verbosity_onlyterminal = 65536;
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
#ifndef visionfarrayparam_viewfrustum
        const int visionfarrayparam_viewfrustum = 1019;
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
#ifndef visionintparam_depthignored
        const int visionintparam_depthignored = 1021;
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
#ifndef visionintparam_rgbignored
        const int visionintparam_rgbignored = 1020;
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
#ifndef vortex_body_normalangularaxisfrictionmodel
        const int vortex_body_normalangularaxisfrictionmodel = 25005;
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
#ifndef vortex_body_secangularaxisfrictionmodel
        const int vortex_body_secangularaxisfrictionmodel = 25004;
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
#ifndef vortex_global_computeinertias
        const int vortex_global_computeinertias = 20005;
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
#ifndef vortex_joint_pospid1
        const int vortex_joint_pospid1 = 21052;
#endif
#ifndef vortex_joint_pospid2
        const int vortex_joint_pospid2 = 21053;
#endif
#ifndef vortex_joint_pospid3
        const int vortex_joint_pospid3 = 21054;
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

        int64_t addElement(int64_t environmentHandle, int64_t ikGroupHandle, int64_t tipDummyHandle);
        std::tuple<int64_t, json, json> addElementFromScene(int64_t environmentHandle, int64_t ikGroup, int64_t baseHandle, int64_t tipHandle, int64_t targetHandle, int64_t constraints);
        std::tuple<std::vector<double>, std::vector<double>> computeGroupJacobian(int64_t environmentHandle, int64_t ikGroupHandle);
        std::tuple<std::vector<double>, std::vector<double>> computeJacobian(int64_t environmentHandle, int64_t baseObject, int64_t lastJoint, int64_t constraints, std::vector<double> tipMatrix, std::optional<std::vector<double>> targetMatrix = {}, std::optional<std::vector<double>> constrBaseMatrix = {});
        int64_t createDebugOverlay(int64_t environmentHandle, int64_t tipHandle, std::optional<int64_t> baseHandle = {});
        int64_t createDummy(int64_t environmentHandle, std::optional<std::string> dummyName = {});
        int64_t createEnvironment(std::optional<int64_t> flags = {});
        int64_t createGroup(int64_t environmentHandle, std::optional<std::string> ikGroupName = {});
        int64_t createJoint(int64_t environmentHandle, int64_t jointType, std::optional<std::string> jointName = {});
        bool doesGroupExist(int64_t environmentHandle, std::string ikGroupName);
        bool doesObjectExist(int64_t environmentHandle, std::string objectName);
        int64_t duplicateEnvironment(int64_t environmentHandle);
        void eraseDebugOverlay(int64_t debugObject);
        void eraseEnvironment(int64_t environmentHandle);
        void eraseObject(int64_t environmentHandle, int64_t objectHandle);
        std::vector<json> findConfigs(int64_t envHandle, int64_t ikGroupHandle, std::vector<int64_t> jointHandles, std::optional<json> params = {}, std::optional<std::vector<json>> configs = {});
        std::vector<double> generatePath(int64_t environmentHandle, int64_t ikGroupHandle, std::vector<int64_t> jointHandles, int64_t tipHandle, int64_t pathPointCount, std::optional<std::string> validationCallback = {}, std::optional<json> auxData = {});
        std::vector<double> getAlternateConfigs(int64_t environmentHandle, std::vector<int64_t> jointHandles, std::optional<std::vector<double>> lowLimits = {}, std::optional<std::vector<double>> ranges = {});
        std::tuple<int64_t, int64_t> getElementBase(int64_t environmentHandle, int64_t ikGroupHandle, int64_t elementHandle);
        int64_t getElementConstraints(int64_t environmentHandle, int64_t ikGroupHandle, int64_t elementHandle);
        int64_t getElementFlags(int64_t environmentHandle, int64_t ikGroupHandle, int64_t elementHandle);
        std::vector<double> getElementPrecision(int64_t environmentHandle, int64_t ikGroupHandle, int64_t elementHandle);
        std::vector<double> getElementWeights(int64_t environmentHandle, int64_t ikGroupHandle, int64_t elementHandle);
        std::string getFailureDescription(int64_t reason);
        std::tuple<int64_t, double, int64_t> getGroupCalculation(int64_t environmentHandle, int64_t ikGroupHandle);
        int64_t getGroupFlags(int64_t environmentHandle, int64_t ikGroupHandle);
        int64_t getGroupHandle(int64_t environmentHandle, std::string ikGroupName);
        std::tuple<std::vector<int64_t>, std::vector<double>> getGroupJointLimitHits(int64_t environmentHandle, int64_t ikGroupHandle);
        std::vector<int64_t> getGroupJoints(int64_t environmentHandle, int64_t ikGroupHandle);
        std::tuple<int64_t, double, double> getJointDependency(int64_t environmentHandle, int64_t jointHandle);
        std::tuple<bool, std::vector<double>> getJointInterval(int64_t environmentHandle, int64_t jointHandle);
        double getJointLimitMargin(int64_t environmentHandle, int64_t jointHandle);
        std::vector<double> getJointMatrix(int64_t environmentHandle, int64_t jointHandle);
        double getJointMaxStepSize(int64_t environmentHandle, int64_t jointHandle);
        int64_t getJointMode(int64_t environmentHandle, int64_t jointHandle);
        double getJointPosition(int64_t environmentHandle, int64_t jointHandle);
        double getJointScrewLead(int64_t environmentHandle, int64_t jointHandle);
        std::tuple<std::vector<double>, std::vector<double>, std::vector<double>> getJointTransformation(int64_t environmentHandle, int64_t jointHandle);
        int64_t getJointType(int64_t environmentHandle, int64_t jointHandle);
        double getJointWeight(int64_t environmentHandle, int64_t jointHandle);
        int64_t getObjectHandle(int64_t environmentHandle, std::string objectName);
        std::vector<double> getObjectMatrix(int64_t environmentHandle, int64_t objectHandle, std::optional<int64_t> relativeToObjectHandle = {});
        int64_t getObjectParent(int64_t environmentHandle, int64_t objectHandle);
        std::vector<double> getObjectPose(int64_t environmentHandle, int64_t objectHandle, std::optional<int64_t> relativeToObjectHandle = {});
        std::tuple<std::vector<double>, std::vector<double>, std::vector<double>> getObjectTransformation(int64_t environmentHandle, int64_t objectHandle, std::optional<int64_t> relativeToObjectHandle = {});
        int64_t getObjectType(int64_t environmentHandle, int64_t objectHandle);
        std::tuple<int64_t, std::string, bool, int64_t> getObjects(int64_t environmentHandle, int64_t index);
        int64_t getTargetDummy(int64_t environmentHandle, int64_t dummyHandle);
        std::tuple<int64_t, int64_t, std::vector<double>> handleGroup(int64_t environmentHandle, int64_t ikGroup, std::optional<json> options = {});
        std::tuple<int64_t, int64_t, std::vector<double>> handleGroups(int64_t environmentHandle, std::vector<int64_t> ikGroups, std::optional<json> options = {});
        void load(int64_t environmentHandle, std::string data);
        std::string save(int64_t environmentHandle);
        void setElementBase(int64_t environmentHandle, int64_t ikGroupHandle, int64_t elementHandle, int64_t baseHandle, std::optional<int64_t> constraintsBaseHandle = {});
        void setElementConstraints(int64_t environmentHandle, int64_t ikGroupHandle, int64_t elementHandle, int64_t constraints);
        void setElementFlags(int64_t environmentHandle, int64_t ikGroupHandle, int64_t elementHandle, int64_t flags);
        void setElementPrecision(int64_t environmentHandle, int64_t ikGroupHandle, int64_t elementHandle, std::vector<double> precision);
        void setElementWeights(int64_t environmentHandle, int64_t ikGroupHandle, int64_t elementHandle, std::vector<double> weights);
        void setGroupCalculation(int64_t environmentHandle, int64_t ikGroupHandle, int64_t method, double damping, int64_t maxIterations);
        void setGroupFlags(int64_t environmentHandle, int64_t ikGroupHandle, int64_t flags);
        void setJointDependency(int64_t environmentHandle, int64_t jointHandle, int64_t masterJointHandle, std::optional<double> offset = {}, std::optional<double> mult = {}, std::optional<std::string> callback = {});
        void setJointInterval(int64_t environmentHandle, int64_t jointHandle, bool cyclic, std::optional<std::vector<double>> interval = {});
        void setJointLimitMargin(int64_t environmentHandle, int64_t jointHandle, double margin);
        void setJointMaxStepSize(int64_t environmentHandle, int64_t jointHandle, double stepSize);
        void setJointMode(int64_t environmentHandle, int64_t jointHandle, int64_t jointMode);
        void setJointPosition(int64_t environmentHandle, int64_t jointHandle, double position);
        void setJointScrewLead(int64_t environmentHandle, int64_t jointHandle, double lead);
        void setJointWeight(int64_t environmentHandle, int64_t jointHandle, double weight);
        void setObjectMatrix(int64_t environmentHandle, int64_t objectHandle, std::vector<double> matrix, std::optional<int64_t> relativeToObjectHandle = {});
        void setObjectParent(int64_t environmentHandle, int64_t objectHandle, int64_t parentObjectHandle, std::optional<bool> keepInPlace = {});
        void setObjectPose(int64_t environmentHandle, int64_t objectHandle, std::vector<double> pose, std::optional<int64_t> relativeToObjectHandle = {});
        void setObjectTransformation(int64_t environmentHandle, int64_t objectHandle, std::vector<double> position, std::vector<double> eulerOrQuaternion, std::optional<int64_t> relativeToObjectHandle = {});
        void setSphericalJointMatrix(int64_t environmentHandle, int64_t jointHandle, std::vector<double> matrix);
        void setSphericalJointRotation(int64_t environmentHandle, int64_t jointHandle, std::vector<double> eulerOrQuaternion);
        void setTargetDummy(int64_t environmentHandle, int64_t dummyHandle, int64_t targetDummyHandle);
        void syncFromSim(int64_t environmentHandle, std::vector<int64_t> ikGroups);
        void syncToSim(int64_t environmentHandle, std::vector<int64_t> ikGroups);

#ifndef calc_cannotinvert
        const int calc_cannotinvert = 2;
#endif
#ifndef calc_invalidcallbackdata
        const int calc_invalidcallbackdata = 128;
#endif
#ifndef calc_limithit
        const int calc_limithit = 64;
#endif
#ifndef calc_notperformed
        const int calc_notperformed = 1;
#endif
#ifndef calc_notwithintolerance
        const int calc_notwithintolerance = 16;
#endif
#ifndef calc_stepstoobig
        const int calc_stepstoobig = 32;
#endif
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
#ifndef group_avoidlimits
        const int group_avoidlimits = 64;
#endif
#ifndef group_enabled
        const int group_enabled = 1;
#endif
#ifndef group_ignoremaxsteps
        const int group_ignoremaxsteps = 2;
#endif
#ifndef group_restoreonbadangtol
        const int group_restoreonbadangtol = 8;
#endif
#ifndef group_restoreonbadlintol
        const int group_restoreonbadlintol = 4;
#endif
#ifndef group_stoponlimithit
        const int group_stoponlimithit = 16;
#endif
#ifndef handle_all
        const int handle_all = -2;
#endif
#ifndef handle_parent
        const int handle_parent = -11;
#endif
#ifndef handle_world
        const int handle_world = -1;
#endif
#ifndef handleflag_tipdummy
        const int handleflag_tipdummy = 4194304;
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
#ifndef pluginHandle
        const int pluginHandle = 13;
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
