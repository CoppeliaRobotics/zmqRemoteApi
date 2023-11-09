sim = require 'sim'
removeLazyLoaders()

zmqRemoteApi = {}

function sim.setThreadAutomaticSwitch()
    -- Shadow the original function
end

sim.stopSimulation = wrap(sim.stopSimulation, function(origFunc)
    return function()
        for k, v in pairs(allClients) do
            v.steppingLevel = 0
            v.holdCalls = 0
            v.desiredStep = currentStep + 10
        end
        origFunc()
    end
end)

sim.acquireLock = wrap(sim.acquireLock, function(origFunc)
    return function()
        if currentFunction == 'sim.acquireLock' then
            currentClientInfo.holdCalls = currentClientInfo.holdCalls + 1
        else
            origFunc()
        end
    end
end)

sim.releaseLock = wrap(sim.releaseLock, function(origFunc)
    return function()
        if currentFunction == 'sim.releaseLock' then
            if currentClientInfo.holdCalls > 0 then
                currentClientInfo.holdCalls = currentClientInfo.holdCalls - 1
            end
        else
            origFunc()
        end
    end
end)

function sim.restartServer()
    leaveRequest = true
end

function sim.setThreadSwitchTiming(switchTiming)
    -- Shadow the original func
    -- 0=disabled, otherwise switchTiming
    if switchTiming == 0 then switchTiming = 999999999 end
    if sim.getSimulationState() == sim.simulation_stopped then
        msgQueueTimeout_idle = switchTiming
    else
        msgQueueTimeout_running = switchTiming
    end
end

sim.wait = wrap(sim.wait, function(origFunc)
    return function(dt, simTime)
        if not simTime then
            local st = sim.getSystemTime()
            while sim.getSystemTime() - st < dt do _yield() end
        else
            origFunc(dt, true)
        end
    end
end)

sim.setStepping = wrap(sim.setStepping, function(origFunc)
    -- Shadow original function:
    -- When stepping is true, CoppeliaSim ALWAYS blocks while Python runs some code
    -- When stepping is false, CoppeliaSim run concurently to Python, i.e. Python is "free" (until a request from Python comes)
    return function(enabled)
        local retVal = 0
        if currentFunction == 'sim.setStepping' then
            retVal = currentClientInfo.steppingLevel
            if enabled then
                if currentClientInfo.steppingLevel == 0 then
                    currentClientInfo.desiredStep = currentStep
                end
                currentClientInfo.steppingLevel = currentClientInfo.steppingLevel + 1
            else
                if currentClientInfo.steppingLevel > 0 then
                    currentClientInfo.steppingLevel = currentClientInfo.steppingLevel - 1
                end
            end
        else
            retVal = origFunc(enabled)
        end
        return retVal
    end
end)

sim.step = wrap(sim.step, function(origFunc)
    return function()
        if currentFunction == 'sim.step' then -- when called from external client
            if sim.getSimulationState() ~= sim.simulation_stopped then
                if currentClientInfo.steppingLevel > 0 then
                    currentClientInfo.desiredStep = currentClientInfo.desiredStep + 1
                    while currentClientInfo.desiredStep > currentStep do
                        _yield() -- by default we wait for the simulation time to increase before returning
                    end
                end
            end
        else
            origFunc()
        end
    end
end)

sim.readCustomDataBlock = wrap(sim.readCustomDataBlock, function(origFunc)
    -- via the remote API, we should always return a string:
    return function(obj, tag)
        local retVal = origFunc(obj, tag)
        if retVal == nil then retVal = '' end
        return retVal
    end
end)

-- Special handling of sim.yield:
originalYield = sim.yield
function sim.yield()
    currentClientInfo.ignoreCallDepth = true
    if sim.getSimulationState() ~= sim.simulation_stopped and sim.getSimulationState() ~=
        sim.simulation_paused then
        if currentClientInfo.steppingLevel > 0 then
            currentClientInfo.desiredStep = currentStep + 1
        end
        local cs = currentStep
        while cs == currentStep do
            -- stays inside here until we are ready with next simulation step
            _yield()
        end
    else
        _yield()
    end
    currentClientInfo.ignoreCallDepth = false
end

function _yield()
    -- Reentrant. Stays in here until the client returned '_*executed*_'
    if not currentClientInfo.replySent then
        -- We are switching before we sent a reply. This is a blocking command and
        -- we need the Python client to wait:
        zmqRemoteApi.send({func = '_*wait*_', args = {}}) -- Tell the client to wait and send '_*executed*_' back
    end
    currentClientInfo.replySent = false
    originalYield()
    -- The coroutine is resuming from here exactly. A new command must have arrived
    while currentClientInfo.lastReq.func ~= '_*executed*_' do
        local req = currentClientInfo.lastReq
        currentClientInfo.lastReq = nil
        local reply = zmqRemoteApi.handleRequest(req)
        zmqRemoteApi.send(reply)
        currentClientInfo.replySent = true
        _yield()
    end
end

-- Special handling of sim.switchThread:
function sim.switchThread()
    sim.yield() -- using the modified version above
end

function tobin(data)
    local d = {data = data}
    setmetatable(d, {
        __tocbor = function(self)
            return cbor.TYPE.BIN(self.data)
        end,
    })
    return d
end

function totxt(data)
    local d = {data = data}
    setmetatable(d, {
        __tocbor = function(self)
            return cbor.TYPE.TEXT(self.data)
        end,
    })
    return d
end

function toarray(data)
    local d = {data = data}
    setmetatable(d, {
        __tocbor = function(self)
            return cbor.TYPE.ARRAY(self.data)
        end,
    })
    return d
end

function tomap(data)
    local d = {data = data}
    setmetatable(d, {
        __tocbor = function(self)
            return cbor.TYPE.MAP(self.data)
        end,
    })
    return d
end

function zmqRemoteApi.verbose()
    return sim.getNamedInt32Param('zmqRemoteApi.verbose') or 0
end

function zmqRemoteApi.require(name)
    _G[name] = require(name)
    zmqRemoteApi.parseFuncsReturnTypes(name)
    if not sim.getBoolParam(sim.boolparam_execunsafeext) then
        sim.executeScriptString = nil
        sim.launchExecutable = nil
        if simSubprocess then
            simSubprocess.exec = nil
            simSubprocess.execAsync = nil
        end
        -- more in sysCall_init and zmqRemoteApi.handleRequest
    end
end

function zmqRemoteApi.parseFuncsReturnTypes(nameSpace)
    local funcs = sim.getApiFunc(-1, '+' .. nameSpace .. '.')
    for i = 1, #funcs, 1 do
        local func = funcs[i]
        local inf = sim.getApiInfo(-1, func)
        local p = string.find(inf, '(', 1, true)
        if p then
            inf = string.sub(inf, 1, p - 1)
            p = string.find(inf, '=')
            if p then
                inf = string.sub(inf, 1, p - 1)
                local t = {}
                local i = 1
                for token in (inf .. ","):gmatch("([^,]*),") do
                    p = string.find(token, ' ')
                    if p then
                        token = string.sub(token, 1, p - 1)
                        if token == 'string' then
                            t[i] = 1
                        elseif token == 'buffer' then
                            t[i] = 2
                        elseif token == 'map' then
                            t[i] = 3
                        else
                            t[i] = 0
                        end
                    else
                        t[i] = 0
                    end
                    i = i + 1
                end
                returnTypes[func] = t
            else
                returnTypes[func] = {}
            end
        end
    end
end

function zmqRemoteApi.info(obj)
    if type(obj) == 'string' then obj = zmqRemoteApi.getField(obj) end
    if type(obj) ~= 'table' then return obj end
    local ret = {}
    for k, v in pairs(obj) do
        if type(v) == 'table' then
            ret[k] = zmqRemoteApi.info(v)
        elseif type(v) == 'function' then
            ret[k] = {func = {}}
        elseif type(v) ~= 'function' then
            ret[k] = {const = v}
        end
    end
    return ret
end

function zmqRemoteApi.getField(f)
    local v = _G
    for w in string.gmatch(f, '[%w_]+') do
        v = v[w]
        if not v then return nil end
    end
    return v
end

function zmqRemoteApi.handleRequest(req)
    currentClientInfo.callDepth = currentClientInfo.callDepth + 1
    -- Handle buffered async calls first:
    local tmp = asyncFuncCalls
    asyncFuncCalls = {}
    for i = 1, #tmp, 1 do zmqRemoteApi.callRemoteFunction(tmp[i].func, tmp[i].args) end

    if zmqRemoteApi.verbose() > 1 then print('Received request:', req) end
    local resp = {}
    if req['func'] ~= nil and req['func'] ~= '' then
        local func = zmqRemoteApi.getField(req['func'])
        local args = req['args'] or {}
        if not func then
            resp['err'] = 'No such function: ' .. req['func']
        else
            if func == sim.switchThread or func == sim.yield then func = sim.step end

            currentFunction = req['func']

            if func == sim.callScriptFunction then
                if #args > 0 and (args[1] == '_evalExec' or args[1] == '_evalExecRet') and
                    not sim.getBoolParam(sim.boolparam_execunsafeext) then
                    args[1] = "FORBIDDEN"
                end
                -- more in sysCall_init and zmqRemoteApi.require
            end

            -- Handle function arguments and possible nil values:
            for i = 1, #args, 1 do
                if type(args[i]) == 'string' then
                    if args[i]:sub(-5) == "@func" then
                        local nm = args[i]:sub(1, -6)
                        args[i] = function(...) return zmqRemoteApi.callRemoteFunction(nm, {...}, true) end
                    elseif args[i] == '_*NIL*_' then
                        args[i] = nil
                    end
                end
            end

            local function errHandler(err)
                local trace = debug.traceback(err)
                local p = string.find(trace, "\nstack traceback:")
                if p then
                    trace = trace:sub(1, p - 1) -- strip traceback from xpcall
                end
                -- Make sure the string survives the passage to Python unmodified:
                trace = string.gsub(trace, "\n", "_=NL=_")
                trace = string.gsub(trace, "\t", "_=TB=_")
                return trace
            end
            local status, retvals = xpcall(function()
                local ret = {func(unpack(args, 1, req.argsL))}
                -- Try to assign correct types to text and buffers:
                local args = returnTypes[req['func']]
                if args then
                    local cnt = math.min(#ret, #args)
                    for i = 1, cnt, 1 do
                        if args[i] == 1 then
                            ret[i] = totxt(ret[i])
                        elseif args[i] == 2 then
                            ret[i] = tobin(ret[i])
                        elseif type(ret[i]) == 'table' then
                            if table.isarray(ret[i]) then
                                ret[i] = toarray(ret[i])
                            else
                                ret[i] = tomap(ret[i])
                            end
                        end
                    end
                end
                return ret
            end, errHandler)

            resp[status and 'ret' or 'err'] = retvals
        end
        currentFunction = nil
    elseif req['eval'] ~= nil and req['eval'] ~= '' then
        local status, retvals = pcall(
                                    function()
                -- cannot prefix 'return ' here, otherwise non-trivial code breaks
                local ret = {loadstring(req['eval'])()}
                return ret
            end
                                )
        resp[status and 'ret' or 'err'] = retvals
    end
    currentClientInfo.callDepth = currentClientInfo.callDepth - 1
    return resp
end

function zmqRemoteApi.poll()
    local retVal = false
    if receiveIsNext then
        local rc, revents = simZMQ.poll({rpcSocket}, {simZMQ.POLLIN}, 0)
        retVal = (rc > 0)
    else
        error('Trying to receive data from Python where a send is expected')
    end
    return retVal
end

function zmqRemoteApi.receive()
    local retVal = nil
    if receiveIsNext then
        local rc, dat = simZMQ.recv(rpcSocket, 0)
        receiveIsNext = false
        rc, retVal = pcall(cbor.decode, dat)
        if not rc then
            error(
                retVal .. "\n" ..
                    sim.transformBuffer(dat, sim.buffer_uint8, 1, 0, sim.buffer_base64)
            )
        end
    else
        error('Trying to receive data from Python where a send is expected')
    end
    return retVal
end

function zmqRemoteApi.send(reply)
    if not receiveIsNext then
        local dat = reply
        status, reply = pcall(cbor.encode, reply)
        if not status then error(reply .. "\n" .. getAsString(dat)) end
        currentClientInfo.idleSince = sim.getSystemTime()
        simZMQ.send(rpcSocket, reply, 0)
        receiveIsNext = true
    else
        error('Trying to send data to Python where a receive is expected')
    end
end

function zmqRemoteApi.handleQueue()
    local startTime = sim.getSystemTime()

    -- First remove old clients:
    for k, v in pairs(allClients) do
        if startTime - v.idleSince > 10 * 60 then
            allClients[k] = nil
            break
        end
    end

    local clients = {}
    local clientCnt = 0
    local msgCnt = 0
    while not leaveRequest do
        local dataPresent = zmqRemoteApi.poll()
        if dataPresent then
            msgCnt = msgCnt + 1
            local req = zmqRemoteApi.receive()
            if req.uuid then
                -- req.ver currently 2. Is sent only for the first contact
                if req.lang then
                    auxFunc('stts', 'zmqRemoteApiConnection-' .. req.lang)
                end
                if req.func == '_*end*_' then
                    zmqRemoteApi.send({})
                    allClients[req.uuid] = nil -- the client left
                else
                    zmqRemoteApi.setClientInfoFromUUID(req.uuid)
                    currentClientInfo.lastReq = req
                    zmqRemoteApi.resumeCoroutine() -- simZMQ.send in there
                    if clients[req.uuid] == nil then
                        clientCnt = clientCnt + 1
                        clients[req.uuid] = true
                    end
                end
            else
                auxFunc('stts', 'zmqRemoteApiConnection-???')
                -- Previous version of ZMQ remote API
                currentClientInfo = {} -- to avoid error in next:
                zmqRemoteApi.send({
                    success = false,
                    error = "The client ZeroMQ remote API version does not match CoppeliaSim's version",
                })
            end
        end

        local maxWaitTime = -1
        if msgCnt > 0 then
            if sim.getSimulationState() & sim.simulation_advancing == 0 then
                maxWaitTime = msgQueueTimeout_idle
            else
                maxWaitTime = msgQueueTimeout_running * clientCnt
            end
        end

        if sim.getSystemTime() - startTime > maxWaitTime then break end
    end
end

function zmqRemoteApi.setClientInfoFromUUID(uuid)
    currentClientInfo = allClients[uuid]
    if currentClientInfo == nil then
        local cor = coroutine.create(coroutineMain)
        currentClientInfo = {
            corout = cor,
            idleSince = systTime,
            lastReq = nil,
            steppingLevel = 0,
            desiredStep = currentStep,
            callDepth = 0,
            holdCalls = 0,
        }

        allClients[uuid] = currentClientInfo
    end
end

function zmqRemoteApi.resumeCoroutine()
    if coroutine.status(currentClientInfo.corout) ~= 'dead' then
        local ok, errorMsg = coroutine.resume(currentClientInfo.corout)
        currentClientInfo.idleSince = sim.getSystemTime()
        if errorMsg then error(debug.traceback(currentClientInfo.corout, errorMsg), 2) end
    end
end

function sim.testCB(a, cb, b)
    for i = 1, 99, 1 do cb(a, b) end
    return cb(a, b)
end

function coroutineMain()
    while true do
        -- We resume the coroutine here, if no blocking function is underway
        local req = currentClientInfo.lastReq
        currentClientInfo.lastReq = nil
        local reply = zmqRemoteApi.handleRequest(req) -- We might switchThread in there, with blocking functions or callback functions
        zmqRemoteApi.send(reply)
        currentClientInfo.replySent = true
        _yield()
    end
end

function sysCall_info()
    return {
        autoStart = sim.getNamedBoolParam('zmqRemoteApi.autoStart') ~= false,
        menu = 'Connectivity\nZMQ remote API server',
    }
end

function sysCall_init()
    if not sim.getBoolParam(sim.boolparam_execunsafeext) then
        load = nil
        loadfile = nil
        dofile = nil
        io.open = nil
        io.popen = nil
        os.execute = nil
        os.remove = nil
        sim.executeScriptString = nil
        sim.launchExecutable = nil
        -- more in zmqRemoteApi.require and zmqRemoteApi.handleRequest
    end

    returnTypes = {}
    simZMQ = require 'simZMQ'
    simZMQ.__raiseErrors(true) -- so we don't need to check retval with every call
    zmqRemoteApi.parseFuncsReturnTypes('sim')
    zmqRemoteApi.parseFuncsReturnTypes('simZMQ')
    rpcPort = sim.getNamedInt32Param('zmqRemoteApi.rpcPort') or 23000
    msgQueueTimeout_idle = 0.05
    msgQueueTimeout_running = 0.002

    if zmqRemoteApi.verbose() > 0 then
        sim.addLog(
            sim.verbosity_scriptinfos,
            string.format('ZeroMQ Remote API server starting (rpcPort=%d)...', rpcPort)
        )
    end
    cbor = require 'org.conman.cbor'
    context = simZMQ.ctx_new()
    rpcSocket = simZMQ.socket(context, simZMQ.REP)
    simZMQ.bind(rpcSocket, string.format('tcp://*:%d', rpcPort))
    if zmqRemoteApi.verbose() > 0 then
        sim.addLog(sim.verbosity_scriptinfos, 'ZeroMQ Remote API server started')
    end

    setAutoYield(false)
    currentStep = 0
    insideExtCall = 0
    receiveIsNext = true
    asyncFuncCalls = {}
    currentClientInfo = nil
    allClients = {} -- uuid is the key, e.g.:
    -- allClients.uuidXXX.corout
    -- allClients.uuidXXX.idleSince
    -- allClients.uuidXXX.lastReq
    -- allClients.uuidXXX.steppingLevel
    -- allClients.uuidXXX.desiredStep
    -- allClients.uuidXXX.holdCalls
    -- allClients.uuidXXX.callDepth
    -- allClients.uuidXXX.ignoreCallDepth
    initSuccessful = true
end

function zmqRemoteApi.callRemoteFunction(functionName, _args, cb)
    -- This is called when a CoppeliaSim function (e.g. sim.moveToConfig) calls a callback
    zmqRemoteApi.send({func = functionName, args = _args})
    if insideExtCall > 0 and cb then
        -- Yielding has no effect, since we might be in a callback from a c routine (after a lua - c-boundary crossing, yieling doesn't work)
        while true do
            if zmqRemoteApi.poll() then
                local req = zmqRemoteApi.receive()
                if currentClientInfo == allClients[req.uuid] then
                    if req.func == '_*executed*_' then
                        currentClientInfo.lastReq = {args = req.args}
                        break
                    end
                    local reply = zmqRemoteApi.handleRequest(req)
                    zmqRemoteApi.send(reply)
                else
                    error('Multiple clients not allowed when lock acquired and in a callback')
                end
            end
        end
    else
        currentClientInfo.replySent = true
        _yield() -- Stays in here until '_*executed*_' received
    end
    return unpack(currentClientInfo.lastReq.args)
end

function sysCall_cleanup()
    if initSuccessful then
        sim.setScriptInt32Param(sim.handle_self, sim.scriptintparam_autorestartonerror, 1)
    end
    if simZMQ then
        simZMQ.close(rpcSocket)
        simZMQ.ctx_term(context)
        if zmqRemoteApi.verbose() > 0 then
            sim.addLog(sim.verbosity_scriptinfos, 'ZeroMQ Remote API server stopped')
        end
    end
end

function sysCall_addOnScriptSuspend()
    return {cmd = 'cleanup'}
end

function sysCall_addOnScriptSuspended()
    return {cmd = 'cleanup'}
end

function sysCall_nonSimulation()
    local retVal
    local holdCalls = true
    while holdCalls do
        zmqRemoteApi.handleQueue()
        holdCalls = false
        if not leaveRequest then
            for k, v in pairs(allClients) do
                if v.holdCalls > 0 then holdCalls = true end
                if v.callDepth > 0 and not v.ignoreCallDepth then waitForStep = true end
            end
        end
    end
    if leaveRequest then retVal = {cmd = 'restart'} end
    return retVal
end

function sysCall_actuation()
    local retVal
    local waitForStep = true
    while waitForStep do
        zmqRemoteApi.handleQueue()
        waitForStep = false
        if not leaveRequest then
            for k, v in pairs(allClients) do
                if v.steppingLevel > 0 then
                    if v.desiredStep <= currentStep then waitForStep = true end
                else
                    if v.callDepth > 0 and not v.ignoreCallDepth then
                        waitForStep = true
                    end
                end
                if v.holdCalls > 0 then waitForStep = true end
            end
        end
    end
    currentStep = currentStep + 1
    if leaveRequest then retVal = {cmd = 'restart'} end
    return retVal
end

function sysCall_ext(funcName, ...)
    local retVal
    insideExtCall = insideExtCall + 1
    
    local fun = _G
    if string.find(funcName, "%.") then
        for w in funcName:gmatch("[^%.]+") do -- handle cases like sim.func or similar too
            if fun[w] then fun = fun[w] end
        end
    else
        fun = fun[funcName]
    end
    if type(fun) == 'function' then
        retVal = fun(...)
    else
        asyncFuncCalls[#asyncFuncCalls + 1] = {func = funcName, args = {...}}
    end
    insideExtCall = insideExtCall - 1
    return retVal
end

function sysCall_afterSimulation()
    currentStep = 0
    --[[ disable following to keep a similar behaviour as Lua scripts, that keep their stepping behaviour past a simulation run
    for k,v in pairs(allClients) do
        v.steppingLevel = 0
    end
    --]]
end
