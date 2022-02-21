client = RemoteAPIClient();
sim = client.getObject('sim');

% get some object handles:
visionSensorHandle = sim.getObject('/VisionSensor');
passiveVisionSensorHandle = sim.getObject('/PassiveVisionSensor');

% read vision sensor image:
[img, resX, resY] = sim.getVisionSensorCharImage(visionSensorHandle);

% display image in MATLAB:
imshow(flip(permute(reshape(img, 3, resY, resX), [3 2 1]), 1));

% write image to another sensor:
sim.setVisionSensorCharImage(passiveVisionSensorHandle, img);