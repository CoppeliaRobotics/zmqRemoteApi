client = RemoteAPIClient();
sim = client.getObject('sim');
[handle] = sim.getObject('/Floor');
fprintf('Handle of /Floor is %d\n', handle);
