sim=require'sim'
removeLazyLoaders()

zmqRemoteApi={}

function zmqRemoteApi.verbose()
    return sim.getNamedInt32Param('zmqRemoteApi.verbose') or 0
end

function zmqRemoteApi.require(name)
    _G[name]=require(name)
    zmqRemoteApi.parseFuncsReturnTypes(name)
end

function zmqRemoteApi.parseFuncsReturnTypes(nameSpace)
    local funcs=sim.getApiFunc(-1,'+'..nameSpace..'.')
    for i=1,#funcs,1 do
        local func=funcs[i]
        local inf=sim.getApiInfo(-1,func)
        local p=string.find(inf,'(',1,true)
        if p then
            inf=string.sub(inf,1,p-1)
            p=string.find(inf,'=')
            if p then
                inf=string.sub(inf,1,p-1)
                local t={}
                local i=1
                for token in (inf..","):gmatch("([^,]*),") do
                    p=string.find(token,' ')
                    if p then
                        token=string.sub(token,1,p-1)
                        if token=='string' then
                            t[i]=1
                        elseif token=='buffer' then
                            t[i]=2
                        else
                            t[i]=0
                        end
                    else
                        t[i]=0
                    end
                    i=i+1
                end
                returnTypes[func]=t
            else
                returnTypes[func]={}
            end
        end
    end
end

function zmqRemoteApi.info(obj)
    if type(obj)=='string' then obj=zmqRemoteApi.getField(obj) end
    if type(obj)~='table' then return obj end
    local ret={}
    for k,v in pairs(obj) do
        if type(v)=='table' then
            ret[k]=zmqRemoteApi.info(v)
        elseif type(v)=='function' then
            ret[k]={func={}}
        elseif type(v)~='function' then
            ret[k]={const=v}
        end
    end
    return ret
end

function zmqRemoteApi.getField(f)
    local v=_G
    for w in string.gmatch(f,'[%w_]+') do
        v=v[w]
        if not v then return nil end
    end
    return v
end

function zmqRemoteApi.handleRequest(req)
    -- Handle buffered async calls first:

    local tmp=asyncFuncCalls
    asyncFuncCalls={}
    for i=1,#tmp,1 do
        zmqRemoteApi.callRemoteFunction(tmp[i].func,tmp[i].args)
    end

    if zmqRemoteApi.verbose()>1 then
        print('Received request:',req)
    end
    local resp={}
    if req['func']~=nil and req['func']~='' then
        local func=zmqRemoteApi.getField(req['func'])
        local args=req['args'] or {}
        if not func then
            resp['err']='No such function: '..req['func']
        else
            -- Handle function arguments:
            local prefix="<function "
            local cbi=1
            for i=1,#args,1 do
                if type(args[i])=='string' then
                    local p=string.find(args[i],prefix)
                    if p==1 then
                        local nm=string.sub(args[i],#prefix+1)
                        p=string.find(nm," ")
                        if p then
                            nm=string.sub(nm,1,p-1)
                            args[i]=pythonCallbacks[cbi]
                            currentClientInfo.pythonCallbackStrs[cbi]=nm
                            cbi=cbi+1
                        end
                    end
                end
            end

            local function errHandler(err)
                local trace = debug.traceback(err)
                local p=string.find(trace,"\nstack traceback:")
                if p then
                    trace=trace:sub(1,p-1) -- strip traceback from xpcall
                end
                -- Make sure the string survives the passage to Python unmodified:
                trace=string.gsub(trace,"\n","_=NL=_")
                trace=string.gsub(trace,"\t","_=TB=_")
                return trace
            end


            local status,retvals=xpcall(function()
                local ret={func(unpack(args))}
                -- Try to assign correct types to text and buffers:
                local args=returnTypes[req['func']]
                if args then
                    local cnt=math.min(#ret,#args)
                    for i=1,cnt,1 do
                        if args[i]==1 then
                            ret[i]=ret[i]..'@:txt:'
                        elseif args[i]==2 then
                            ret[i]=ret[i]..'@:dat:'
                        end
                    end
                end
                return ret
            end,errHandler)

            resp[status and 'ret' or 'err']=retvals
        end
    elseif req['eval']~=nil and req['eval']~='' then
        local status,retvals=pcall(function()
            -- cannot prefix 'return ' here, otherwise non-trivial code breaks
            local ret={loadstring(req['eval'])()}
            return ret
        end)
        resp[status and 'ret' or 'err']=retvals
    end

    return resp
end

function zmqRemoteApi.poll()
    local retVal=false
    if receiveIsNext then
        local rc,revents=simZMQ.poll({rpcSocket},{simZMQ.POLLIN},0)
        retVal=(rc>0)
    else
        error('Trying to receive data from Python where a send is expected')
    end
    return retVal
end

function zmqRemoteApi.receive()
    local retVal=nil
    if receiveIsNext then
        local rc,dat=simZMQ.recv(rpcSocket,0)
        receiveIsNext=false
        rc,retVal=pcall(cborDecode.decode,dat)
        if not rc then
            error('CBOR decode error: '..sim.transformBuffer(dat,sim.buffer_uint8,1,0,sim.buffer_base64))
        end
    else
        error('Trying to receive data from Python where a send is expected')
    end
    return retVal
end

function zmqRemoteApi.send(reply)
    if not receiveIsNext then
        local dat=reply
        status,reply=pcall(sim.packCbor,reply)
        if not status then
            error('CBOR encode error: '..getAsString(dat))
        end
        currentClientInfo.idleSince=sim.getSystemTime()
        simZMQ.send(rpcSocket,reply,0)
        receiveIsNext=true
    else
        error('Trying to send data to Python where a receive is expected')
    end
end

function zmqRemoteApi.handleQueue()
    local startTime=sim.getSystemTime()
    local clients={}
    local clientCnt=0
    local msgCnt=0
    while true do
        local dataPresent=zmqRemoteApi.poll()
        if dataPresent then
            msgCnt=msgCnt+1
            local req=zmqRemoteApi.receive()
            zmqRemoteApi.setClientInfoFromUUID(req.uuid)
            currentClientInfo.lastReq=req
            zmqRemoteApi.resumeCoroutine() -- simZMQ.send in there
            if clients[req.uuid]==nil then
                clientCnt=clientCnt+1
                clients[req.uuid]=true
            end
        end

        local maxWaitTime=-1
        if msgCnt>0 then
            if sim.getSimulationState()&sim.simulation_advancing==0 then
                maxWaitTime=msgQueueTimeout_idle
            else
                maxWaitTime=msgQueueTimeout_running*clientCnt
            end
        end

        if sim.getSystemTime()-startTime>maxWaitTime then
            break
        end
    end
end

function zmqRemoteApi.setClientInfoFromUUID(uuid)
    -- First remove old clients that are not used anymore, not sure it makes a big difference
    local systTime=sim.getSystemTime()
    for k,v in pairs(allClients) do
        if systTime-v.idleSince>10*60 then
            allClients[k]=nil
            break
        end
    end

    currentClientInfo=allClients[uuid]
    if currentClientInfo==nil then
        local cor=coroutine.create(coroutineMain)
        currentClientInfo={corout=cor,pythonCallbackStrs={'','',''},idleSince=systTime,lastReq=nil,stepping=false,desiredStep=currentStep}

        allClients[uuid]=currentClientInfo
    end
end

function zmqRemoteApi.resumeCoroutine()
    if coroutine.status(currentClientInfo.corout)~='dead' then
        local ok,errorMsg=coroutine.resume(currentClientInfo.corout)
        currentClientInfo.idleSince=sim.getSystemTime()
        if errorMsg then
            error(debug.traceback(currentClientInfo.corout,errorMsg),2)
        end
    end
end

function sim.handleExtCalls()
    -- can be called by Python in order to trigger possible callbacks
end

function sim.setThreadAutomaticSwitch()
    -- Shadow the original function
end

originalWait=sim.wait
function sim.wait(dt,simTime)
    if not simTime then
        local st=sim.getSystemTime()
        while sim.getSystemTime()-st<dt do
            _simSwitchThread()
        end
    else
        originalWait(dt,true)
    end
end

-- Special handling of sim.switchThread:
originalSwitchThread=sim.switchThread
function sim.switchThread()
    if sim.getSimulationState()~=sim.simulation_stopped and sim.getSimulationState()~=sim.simulation_paused then
        if currentClientInfo.stepping then
            currentClientInfo.desiredStep=currentStep+1
        end
        local cs=currentStep
        while cs==currentStep do
            -- stays inside here until we are ready with next simulation step
            _simSwitchThread()
        end
    else
        _simSwitchThread()
    end
end

function _simSwitchThread()
    -- Reentrant. Stays in here until the client returned '_*executed*_'
    if not currentClientInfo.replySent then
        -- We are switching before we sent a reply. This is a blocking command and
        -- we need the Python client to wait:
        zmqRemoteApi.send({func='_*wait*_',args={}}) -- Tell the client to wait and send '_*executed*_' back
    end
    currentClientInfo.replySent=false
    originalSwitchThread()
    -- The coroutine is resuming from here exactly. A new command must have arrived
    while currentClientInfo.lastReq.func~='_*executed*_' do
        local req=currentClientInfo.lastReq
        currentClientInfo.lastReq=nil
        local reply=zmqRemoteApi.handleRequest(req)
        zmqRemoteApi.send(reply)
        currentClientInfo.replySent=true
        _simSwitchThread()
    end
end

function sim.testCB(a,cb,b)
    return cb(a,b)
end

function coroutineMain()
    while true do
        -- We resume the coroutine here, if no blocking function is underway
        local req=currentClientInfo.lastReq
        currentClientInfo.lastReq=nil
        local reply=zmqRemoteApi.handleRequest(req) -- We might switchThread in there, with blocking functions or callback functions
        zmqRemoteApi.send(reply)
        currentClientInfo.replySent=true
        _simSwitchThread()
    end
end

function sysCall_info()
    return {autoStart=sim.getNamedBoolParam('zmqRemoteApi.autoStart')~=false,menu='Connectivity\nZMQ remote API server'}
end

function sysCall_init()
    returnTypes={}
    simZMQ=require'simZMQ'
    simZMQ.__raiseErrors(true) -- so we don't need to check retval with every call
    zmqRemoteApi.parseFuncsReturnTypes('sim')
    zmqRemoteApi.parseFuncsReturnTypes('simZMQ')
    rpcPort=sim.getNamedInt32Param('zmqRemoteApi.rpcPort') or 23000
    msgQueueTimeout_stepped=sim.getNamedFloatParam('zmqRemoteApi.msgQueueTimeout_stepped') or 0.05
    msgQueueTimeout_idle=sim.getNamedFloatParam('zmqRemoteApi.msgQueueTimeout_idle') or 0.05
    msgQueueTimeout_running=sim.getNamedFloatParam('zmqRemoteApi.msgQueueTimeout_running') or 0.002

    if zmqRemoteApi.verbose()>0 then
        sim.addLog(sim.verbosity_scriptinfos,string.format('ZeroMQ Remote API server starting (rpcPort=%d)...',rpcPort))
    end
    -- cbor=require 'cbor' -- encodes strings as buffers, always. DO NOT USE!!
    cborDecode=require'org.conman.cbor' -- use only for decoding. For encoding use sim.packCbor
    context=simZMQ.ctx_new()
    rpcSocket=simZMQ.socket(context,simZMQ.REP)
    simZMQ.bind(rpcSocket,string.format('tcp://*:%d',rpcPort))
    if zmqRemoteApi.verbose()>0 then
        sim.addLog(sim.verbosity_scriptinfos,'ZeroMQ Remote API server started')
    end

    setThreadAutomaticSwitch(false)
    currentStep=0
    receiveIsNext=true
    pythonCallbacks={pythonCallback1,pythonCallback2,pythonCallback3}
    asyncFuncCalls={}
    currentClientInfo=nil
    allClients={} -- uuid is the key, e.g.:
                  -- allClients.uuidXXX.corout
                  -- allClients.uuidXXX.pythonCallbackStrs[3]
                  -- allClients.uuidXXX.idleSince
                  -- allClients.uuidXXX.lastReq
                  -- allClients.uuidXXX.stepping
                  -- allClients.uuidXXX.desiredStep
end

function pythonCallback1(...)
    return zmqRemoteApi.callRemoteFunction(currentClientInfo.pythonCallbackStrs[1],{...})
end

function pythonCallback2(...)
    return zmqRemoteApi.callRemoteFunction(currentClientInfo.pythonCallbackStrs[2],{...})
end

function pythonCallback3(...)
    return zmqRemoteApi.callRemoteFunction(currentClientInfo.pythonCallbackStrs[3],{...})
end

function zmqRemoteApi.callRemoteFunction(functionName,_args)
    -- This is called when a CoppeliaSim function (e.g. sim.moveToConfig) calls a callback
    zmqRemoteApi.send({func=functionName,args=_args})
    currentClientInfo.replySent=true
    _simSwitchThread() -- Stays in here until '_*executed*_' received
    return unpack(currentClientInfo.lastReq.args)
end

function sysCall_cleanup()
    if simZMQ then
        simZMQ.close(rpcSocket)
        simZMQ.ctx_term(context)
        if zmqRemoteApi.verbose()>0 then
            sim.addLog(sim.verbosity_scriptinfos,'ZeroMQ Remote API server stopped')
        end
    end
end

function sysCall_addOnScriptSuspend()
    return {cmd='cleanup'}
end

function sysCall_addOnScriptSuspended()
    return {cmd='cleanup'}
end

function getMsgQueueTimeouts()
    return {idle=msgQueueTimeout_idle,running=msgQueueTimeout_running,stepped=msgQueueTimeout_stepped}
end

function setMsgQueueTimeouts(timeout)
    msgQueueTimeout_idle=timeout.idle or msgQueueTimeout_idle
    msgQueueTimeout_running=timeout.running or msgQueueTimeout_running
    msgQueueTimeout_stepped=timeout.stepped or msgQueueTimeout_stepped
end

function sysCall_nonSimulation()
    zmqRemoteApi.handleQueue()
end

function sysCall_beforeMainScript()
    zmqRemoteApi.handleQueue()
    for k,v in pairs(allClients) do
        if v.stepping and v.desiredStep<=currentStep then
            return {doNotRunMainScript=true}
        end
    end
end

function sysCall_ext(funcName,...)
    local _args={...}
    if _G[funcName] then -- for now ignore functions in tables
        return _G[funcName](_args)
    else
        asyncFuncCalls[#asyncFuncCalls+1]={func=funcName,args=_args}
    end
end

function sysCall_actuation()
    currentStep=currentStep+1
end

function sysCall_afterSimulation()
    currentStep=0
    for k,v in pairs(allClients) do
        v.stepping=false -- important, since we can't really notice when a client leaves
    end
end

function zmqRemoteApi.setStepping(enable)
    currentClientInfo.stepping=enable
    currentClientInfo.desiredStep=currentStep
end

function zmqRemoteApi.step()
    currentClientInfo.desiredStep=currentClientInfo.desiredStep+1
end

function zmqRemoteApi.waitForStep()
    while currentClientInfo.desiredStep>currentStep do
        _simSwitchThread()
    end
end

-- via the remote API, we should always return a string:
_S.readCustomDataBlock=sim.readCustomDataBlock
function sim.readCustomDataBlock(obj,tag)
    local retVal=_S.readCustomDataBlock(obj,tag)
    if retVal==nil then
        retVal=''
    end
    return retVal
end
