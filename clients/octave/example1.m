client = RemoteAPIClient();
h = client.call("sim.getObject", {"/Floor"});
printf("sim.getObject('/Floor') -> %d", h);
