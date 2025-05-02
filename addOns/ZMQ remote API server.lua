sim = require 'sim'
_removeLazyLoaders()

zmqRemoteApi = {}

function sim.setThreadAutomaticSwitch()
    -- Shadow the original function
end

sim.stopSimulation = wrap(sim.stopSimulation, function(origFunc)
    return function(wait)
        for k, v in pairs(allClients) do
            v.steppingLevel = 0
            v.holdCalls = 0
            v.desiredStep = currentStep + 10
        end
        origFunc(wait)
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

--[[
sim.wait = wrap(sim.wait, function(origFunc)
    return function(dt, simTime)
        if not simTime then
            local st = sim.getSystemTime()
            while sim.getSystemTime() - st < dt do sim.step() end
        else
            origFunc(dt, true)
        end
    end
end)
--]]

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

nakedYield = sim.yield
function sim.step()
    currentClientInfo.ignoreCallDepth = true
    local running = ( sim.getSimulationState() ~= sim.simulation_stopped and sim.getSimulationState() ~= sim.simulation_paused )
    if running then
        if currentClientInfo.steppingLevel > 0 then
            currentClientInfo.desiredStep = currentStep + 1
        end
    end
    local cs = currentStep
    while cs == currentStep do
        -- stays inside here until we are ready with next simulation step (if running)

        -- Handle buffered async calls first:
        insideExtCall = insideExtCall + 1
        local tmp = currentClientInfo.asyncFuncCalls
        currentClientInfo.asyncFuncCalls = {}
        for i = 1, #tmp, 1 do
            zmqRemoteApi.callRemoteFunction(tmp[i].func, tmp[i].args, true)
        end
        insideExtCall = insideExtCall - 1

        zmqRemoteApi.send({func = '_*wait*_', args = {}}) -- Tell the client to wait and send '_*executed*_' back
        nakedYield()
        -- if we arrived here, we have received the '_*executed*_' reply from the same client
        if not running then
            break
        end
    end
    currentClientInfo.ignoreCallDepth = false
end
sim.switchThread = sim.step
sim.yield = sim.step

function sim.readCustomDataBlock(obj, tag)
    -- via the remote API, we should always return a string, for backw. comp.
    local retVal = sim.readCustomStringData(obj, tag)
    if retVal == nil then
        retVal = ''
    end
    return retVal
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

cbornil = {
    __tocbor = function(self)
        return cbor.SIMPLE.null()
    end,
}

function tonil()
    local d = {}
    setmetatable(d, cbornil)
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

    insideExtCall = insideExtCall + 1
    local tmp = currentClientInfo.asyncFuncCalls
    currentClientInfo.asyncFuncCalls = {}
    for i = 1, #tmp, 1 do
        zmqRemoteApi.callRemoteFunction(tmp[i].func, tmp[i].args, true)
    end
    insideExtCall = insideExtCall - 1

    if zmqRemoteApi.verbose() > 1 then print('Received request:', req) end
    local resp = {}
    if req['func'] ~= nil and req['func'] ~= '' then
        local reqFunc = req['func']
        local func = zmqRemoteApi.getField(reqFunc)
        local args = req['args'] or {}
        if not func then
            resp['err'] = 'No such function: ' .. reqFunc
        else
            currentFunction = reqFunc

            if func == sim.callScriptFunction then
                if #args > 0 and (args[1] == '_evalExec' or args[1] == '_evalExecRet') and
                    not sim.getBoolParam(sim.boolparam_execunsafeext) then
                    args[1] = "FORBIDDEN"
                end
                -- more in sysCall_init and zmqRemoteApi.require
            end

            -- Handle function arguments (up to a depth of 2), and possible nil values:
            for i = 1, #args, 1 do
                if type(args[i]) == 'string' then
                    -- depth 1
                    if args[i]:sub(-5) == "@func" then
                        local nm = args[i]:sub(1, -6)
                        local fff = function(...) return zmqRemoteApi.callRemoteFunction(nm, {...}, true) end
                        args[i] = fff
                        if not _S.pythonCallbacks then
                            _S.pythonCallbacks = {}
                        end
                        _S.pythonCallbacks[fff] = true -- so that we can identify a Python callback
                    elseif args[i] == '_*NIL*_' then
                        args[i] = nil
                    end
                else
                    if type(args[i]) == 'table' then
                        -- depth 2
                        local cnt = 0
                        for k, v in pairs(args[i]) do
                            if type(v) == 'string' and v:sub(-5) == "@func" then
                                local nm = v:sub(1, -6)
                                v = function(...) return zmqRemoteApi.callRemoteFunction(nm, {...}, true) end
                                if not _S.pythonCallbacks then
                                    _S.pythonCallbacks = {}
                                end
                                _S.pythonCallbacks[v] = true -- so that we can identify a Python callback
                                args[i][k] = v
                            end
                            cnt = cnt + 1
                            if cnt >= 16 then
                                break -- parse no more than 16 items
                            end
                        end
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
                local pret = table.pack(func(unpack(args, 1, req.argsL)))
                local ret = {}
                for i = 1, pret.n do
                    if pret[i] ~= nil then
                        ret[i] = pret[i]
                    else
                        ret[i] = tonil()
                    end
                end
                --local ret = {func(unpack(args, 1, req.argsL))}

                -- Try to assign correct types to text and buffers:
                local args = returnTypes[reqFunc]
                if args then
                    local cnt = math.min(#ret, #args)
                    for i = 1, cnt, 1 do
                        if args[i] == 1 and type(ret[i]) == 'string' then
                            ret[i] = totxt(ret[i])
                        elseif args[i] == 2 and type(ret[i]) == 'string' then
                            ret[i] = tobin(ret[i])
                        elseif type(ret[i]) == 'table' then
                            if (not isbuffer(ret[i])) and (getmetatable(ret[i]) ~= cbornil) then 
                                if table.isarray(ret[i]) then
                                    ret[i] = toarray(ret[i])
                                else
                                    ret[i] = tomap(ret[i])
                                end
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
        rc, retVal = pcall(cbor.decode, tostring(dat))
        if not rc then
            if #dat < 2000 then
                error(retVal .. "\n" .. sim.transformBuffer(dat, sim.buffer_uint8, 1, 0, sim.buffer_base64))
            else
                error('Error trying to decode received data:\n' .. retVal)
            end
        end
    else
        error('Trying to receive data from Python where a send is expected')
    end
    return retVal
end

function zmqRemoteApi.send(reply)
    if not receiveIsNext then
        local dat = reply
        local status, reply = pcall(cbor.encode, reply)
        if not status then
            local s2, rep2 = pcall(getAsString, dat)
            if s2 then
                error(reply .. "\n" .. rep2)
            else
                error('Error while trying to encode data to send:\n' .. reply)
            end
        end
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
        local to = 10 * 60
        if v.timeout then
            to =v.timeout
        end
        if startTime - v.idleSince > to then
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
                    zmqRemoteApi.setClientInfoFromUUID(req.uuid, req.timeout)
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

function zmqRemoteApi.setClientInfoFromUUID(uuid, optionalTimeoutInSecs)
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
            timeout = optionalTimeoutInSecs,
            asyncFuncCalls = {}
        }

        allClients[uuid] = currentClientInfo
    end
end

function zmqRemoteApi.resumeCoroutine()
    if coroutine.status(currentClientInfo.corout) ~= 'dead' then
        local ok, errorMsg = coroutine.resume(currentClientInfo.corout)
        currentClientInfo.idleSince = sim.getSystemTime()
        if errorMsg then
            error(debug.traceback(currentClientInfo.corout, errorMsg), 2)
        end
    end
end

function sim.testCB(a, cb, b, iterations)
    iterations = iterations or 99
    for i = 1, iterations, 1 do cb(a, b) end
    return cb(a, b)
end

function coroutineMain()
    -- each client stays in here until timeout. Socket reads always happend in the 'handleQueue' (with an exception in callRemoteFunction)
    while true do
        -- Here we always have a request pending that we need to process and answer
        local req = currentClientInfo.lastReq
        local reply = zmqRemoteApi.handleRequest(req) -- We might yield in there, with blocking functions or callback functions
        zmqRemoteApi.send(reply)
        nakedYield() -- we only resume once a new request has arrived from the same client
    end
end

function sysCall_info()
    return {
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
    local defaultRpcPort = 23000 + sim.getInt32Param(sim.intparam_processid)
    local rpcPort = sim.getNamedInt32Param('zmqRemoteApi.rpcPort') or defaultRpcPort
    sim.setNamedInt32Param('zmqRemoteApi.rpcPort', rpcPort)
    msgQueueTimeout_idle = 0.05
    msgQueueTimeout_running = 0.002

--    if zmqRemoteApi.verbose() > 0 then
        sim.addLog(
            sim.verbosity_scriptinfos,
            string.format('ZeroMQ Remote API server starting (rpcPort=%d)...', rpcPort)
        )
--    end
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
    -- allClients.uuidXXX.timeout
    -- allClients.uuidXXX.asyncFuncCalls
    initSuccessful = true
end

function zmqRemoteApi.callRemoteFunction(functionName, _args, cb)
    -- This is called when a CoppeliaSim function (e.g. sim.moveToConfig) calls a callback
    zmqRemoteApi.send({func = functionName, args = _args})
    local retVal
    if insideExtCall > 0 and cb then
        -- Yielding has no effect, since we might be in a callback from a c routine (after a lua - c-boundary crossing, yieling doesn't work)
        while true do
            if zmqRemoteApi.poll() then
                local req = zmqRemoteApi.receive()
                if currentClientInfo == allClients[req.uuid] then
                    if req.func ~= '_*executed*_' then
                        local reply = zmqRemoteApi.handleRequest(req)
                        zmqRemoteApi.send(reply)
                    else
                        retVal = req.args
                        break
                    end
                else
                    zmqRemoteApi.send({func = '_*repeat*_', args = {}}) -- Tell the client to repeat the request, since we are busy now
                end
            end
        end
    else
        while true do
            nakedYield()
            -- if we arrived here, we have received a reply from the client, either the '_*executed*_' reply or a request to call a function
            local req = currentClientInfo.lastReq
            if req.func ~= '_*executed*_' then
                local reply = zmqRemoteApi.handleRequest(req)
                zmqRemoteApi.send(reply)
            else
                retVal = req.args
                break
            end
        end
    end
    return unpack(retVal)
end

function sysCall_cleanup()
    if initSuccessful then
        sim.setObjectInt32Param(sim.getScript(sim.handle_self), sim.scriptintparam_autorestartonerror, 1)
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

function sysCall_suspended()
    return sysCall_nonSimulation()
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
    local retVal = table.pack(nil)
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
        retVal = table.pack(fun(...))
    else
        for k,v in pairs(allClients) do
            v.asyncFuncCalls[#v.asyncFuncCalls + 1] = {func = funcName, args = {...}}
        end
    end
    insideExtCall = insideExtCall - 1
    return table.unpack(retVal)
end

function sysCall_afterSimulation()
    currentStep = 0
    --[[ disable following to keep a similar behaviour as Lua scripts, that keep their stepping behaviour past a simulation run
    for k,v in pairs(allClients) do
        v.steppingLevel = 0
    end
    --]]
end

require('addOns.autoStart').setup{ns = 'zmqRemoteApi', default = true}
