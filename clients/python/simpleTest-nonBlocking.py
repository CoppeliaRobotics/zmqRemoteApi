# Make sure to have the add-on "ZMQ remote API"
# running in CoppeliaSim
#
# Some CoppeliaSim commands will run in non-blocking mode
# For a simpler, blocking mode only example, see simpleTest.py

import asyncio
import sys

from zmqRemoteApi.asyncio import RemoteAPIClient


async def mainFunc():
    print('Program started')

    async with RemoteAPIClient() as client:
        sim = await client.getObject('sim')

        # When simulation is not running, ZMQ message handling could be a bit
        # slow, since the idle loop runs at 8 Hz by default. So let's make
        # sure that the idle loop runs at full speed for this program:
        defaultIdlsFps = await sim.getInt32Param(sim.intparam_idle_fps)
        await sim.setInt32Param(sim.intparam_idle_fps, 0)

        # Create a few dummies (below executes "concurrently", i.e. without
        # waiting for the result of one call, before sending the request for
        # the next call; this will significantly improve speed when order of
        # calls is not important; see also the doc of asyncio.gather):
        handles = await asyncio.gather(*[
            sim.createDummy(0.01, 12 * [0]) for _ in range(50)
        ])
        # Set positions of the dummies (executes "concurrently", see above):
        await asyncio.gather(*[
            sim.setObjectPosition(h, -1, [0.01 * i, 0.01 * i, 0.01 * i])
            for i, h in enumerate(handles)
        ])

        # Run a simulation in asynchronous mode:
        await sim.startSimulation()
        while (t := await sim.getSimulationTime()) < 3:
            s = f'Simulation time: {t:.2f} [s] (simulation running '\
                'asynchronously to client, i.e. non-stepped)'
            print(s)
            await sim.addLog(sim.verbosity_scriptinfos, s)
        await sim.stopSimulation()
        # If you need to make sure we really stopped:
        while sim.simulation_stopped != await sim.getSimulationState():
            await asyncio.sleep(0.1)

        # Run a simulation in stepping mode:
        await client.setStepping(True)
        await sim.startSimulation()
        while (t := await sim.getSimulationTime()) < 3:
            s = f'Simulation time: {t:.2f} [s] (simulation running '\
                'synchronously to client, i.e. stepped)'
            print(s)
            await sim.addLog(sim.verbosity_scriptinfos, s)
            await client.step()  # triggers next simulation step
        await sim.stopSimulation()

        # Remove the dummies created earlier
        # (executes "concurrently", see above):
        await asyncio.gather(*[sim.removeObject(h) for h in handles])

        # Restore the original idle loop frequency:
        await sim.setInt32Param(sim.intparam_idle_fps, defaultIdlsFps)

    print('Program ended')

if sys.platform == 'win32' and sys.version_info >= (3, 8, 0):
    asyncio.set_event_loop_policy(asyncio.WindowsSelectorEventLoopPolicy())
asyncio.run(mainFunc())
