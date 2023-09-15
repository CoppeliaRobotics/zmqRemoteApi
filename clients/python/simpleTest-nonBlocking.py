# Make sure to have the add-on "ZMQ remote API"
# running in CoppeliaSim
#
# Some CoppeliaSim commands will run in non-blocking mode
# For a simpler, blocking mode only example, see simpleTest.py

import asyncio
import sys

from coppeliasim_zmqremoteapi_client.asyncio import RemoteAPIClient


async def mainFunc():
    print('Program started')

    async with RemoteAPIClient() as client:
        sim = await client.require('sim')

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
        await sim.setStepping(True)
        await sim.startSimulation()
        while (t := await sim.getSimulationTime()) < 3:
            s = f'Simulation time: {t:.2f} [s] (simulation running '\
                'synchronously to client, i.e. stepped)'
            print(s)
            await sim.addLog(sim.verbosity_scriptinfos, s)
            await sim.step()  # triggers next simulation step
        await sim.stopSimulation()

        # Remove the dummies created earlier
        # (executes "concurrently", see above):
        await asyncio.gather(*[sim.removeObject(h) for h in handles])

    print('Program ended')

if sys.platform == 'win32' and sys.version_info >= (3, 8, 0):
    asyncio.set_event_loop_policy(asyncio.WindowsSelectorEventLoopPolicy())
asyncio.run(mainFunc())
