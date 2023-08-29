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
    if zmqRemoteApi.verbose()>1 then
        print('Received request:',req)
    end
    local resp={}
    if req['func']~=nil and req['func']~='' then
        local func=zmqRemoteApi.getField(req['func'])
        local args=req['args'] or {}
        if not func then
            resp['error']='No such function: '..req['func']
        else
            local status,retvals=pcall(function()
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
            end)
            resp[status and 'ret' or 'error']=retvals
        end
    elseif req['eval']~=nil and req['eval']~='' then
        local status,retvals=pcall(function()
            -- cannot prefix 'return ' here, otherwise non-trivial code breaks
            local ret={loadstring(req['eval'])()}
            return ret
        end)
        resp[status and 'ret' or 'error']=retvals
    end
    resp['success']=resp['error']==nil
    if zmqRemoteApi.verbose()>1 then
        print('Sending response:',resp)
    end
    return resp
end

function zmqRemoteApi.handleRawMessage(rawReq)
    local status,req=pcall(cborDecode.decode,rawReq)
    if status then
        local resp=zmqRemoteApi.handleRequest(req)
        local status,resp=pcall(sim.packCbor,resp)
        if status then return resp end
        return sim.packCbor({success=false,error=resp})
    else
        sim.addLog(sim.verbosity_errors,'Decode error: '..req)
        return ''
    end
end

function zmqRemoteApi.handleQueue()
    function dumpBytes(x)
        if sim.getNamedStringParam('zmqRemoteApi.debugBinaryFormat')=='base64' then
            return 'base64='..sim.transformBuffer(req,sim.buffer_uint8,0,0,sim.buffer_base64)
        else
            return table.join(map(partial(string.format,'%02X'),string.bytes(x)),' ')
        end
    end

    local startTime=sim.getSystemTime()
    local msgCnt=0
    while true do
        local rc,revents=simZMQ.poll({rpcSocket},{simZMQ.POLLIN},0)
        if rc>0 then
            msgCnt=msgCnt+1
            local rc,req=simZMQ.recv(rpcSocket,0)

            if zmqRemoteApi.verbose()>2 then
                print('Received raw request: (len='..#req..') '..dumpBytes(req))
            end

            local resp=zmqRemoteApi.handleRawMessage(req)

            if zmqRemoteApi.verbose()>2 then
                print('Sending raw response: (len='..#resp..') '..dumpBytes(resp))
            end

            simZMQ.send(rpcSocket,resp,0)
        end
        
        local waitingForStep=false
        if sim.getSimulationState()~=sim.simulation_stopped then
            if next(steppingClients)~=nil and sim.getSimulationState()~=sim.simulation_stopped then
                for uuid,v in pairs(steppingClients) do
                    if steppedClients[uuid]==nil then
                        waitingForStep=true
                        break
                    end
                end
            end
        end
        
        local maxWaitTime=-1
        if waitingForStep then
            maxWaitTime=msgQueueTimeout_stepped
        else
            if msgCnt>0 then
                if sim.getSimulationState()&sim.simulation_advancing==0 then
                    maxWaitTime=msgQueueTimeout_idle
                else
                    maxWaitTime=msgQueueTimeout_running
                end
            end
        end
        
        if sim.getSystemTime()-startTime>maxWaitTime then
            break
        end
    end
end

function zmqRemoteApi.publishStepCount()
    if zmqRemoteApi.verbose()>3 then
        print('Publishing simulationTimeStepCount='..simulationTimeStepCount)
    end
    simZMQ.send(cntSocket,sim.packUInt32Table{simulationTimeStepCount},0)
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
    cntPort=sim.getNamedInt32Param('zmqRemoteApi.cntPort') or (rpcPort+1)
    maxTimeSlot=sim.getNamedFloatParam('zmqRemoteApi.maxTimeSlot') or 0.005
    msgQueueTimeout_stepped=sim.getNamedFloatParam('zmqRemoteApi.msgQueueTimeout_stepped') or 1.0
    msgQueueTimeout_idle=sim.getNamedFloatParam('zmqRemoteApi.msgQueueTimeout_idle') or 0.2
    msgQueueTimeout_running=sim.getNamedFloatParam('zmqRemoteApi.msgQueueTimeout_running') or 0.005
    
    if zmqRemoteApi.verbose()>0 then
        sim.addLog(sim.verbosity_scriptinfos,string.format('ZeroMQ Remote API server starting (rpcPort=%d, cntPort=%d)...',rpcPort,cntPort))
    end
    -- cbor=require 'cbor' -- encodes strings as buffers, always. DO NOT USE!!
    cborDecode=require'org.conman.cbor' -- use only for decoding. For encoding use sim.packCbor
    context=simZMQ.ctx_new()
    rpcSocket=simZMQ.socket(context,simZMQ.REP)
    simZMQ.bind(rpcSocket,string.format('tcp://*:%d',rpcPort))
    cntSocket=simZMQ.socket(context,simZMQ.PUB)
    simZMQ.setsockopt(cntSocket,simZMQ.CONFLATE,sim.packUInt32Table{1})
    simZMQ.bind(cntSocket,string.format('tcp://*:%d',cntPort))
    if zmqRemoteApi.verbose()>0 then
        sim.addLog(sim.verbosity_scriptinfos,'ZeroMQ Remote API server started')
    end
    simulationTimeStepCount=0
    steppingClients={}
    steppedClients={}
end

function sysCall_cleanup()
    if not simZMQ then return end
    simZMQ.close(cntSocket)
    simZMQ.close(rpcSocket)
    simZMQ.ctx_term(context)
    if zmqRemoteApi.verbose()>0 then
        sim.addLog(sim.verbosity_scriptinfos,'ZeroMQ Remote API server stopped')
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

function sysCall_suspended()
    zmqRemoteApi.handleQueue()
end

function sysCall_realTimeIdle()
    zmqRemoteApi.handleQueue()
end

function sysCall_beforeMainScript()
    zmqRemoteApi.handleQueue()
    local outData
    if next(steppingClients)~=nil then
        local canStep=true
        for uuid,v in pairs(steppingClients) do
            if steppedClients[uuid]==nil then
                canStep=false
                break
            end
        end
        outData={doNotRunMainScript=(not canStep)}
    end
    return outData
end

function sysCall_beforeSimulation()
    simulationTimeStepCount=0
    zmqRemoteApi.publishStepCount()
end

function sysCall_actuation()
    steppedClients={}
    simulationTimeStepCount=simulationTimeStepCount+1
    zmqRemoteApi.publishStepCount()
end

function sysCall_afterSimulation()
    zmqRemoteApi.publishStepCount() -- so that the last client.step(True) doesn't block
    steppingClients={}
    steppedClients={}
end

function setStepping(enable,uuid)
    if uuid==nil then
        uuid='ANY' -- to support older clients
    end
    if enable then
        steppingClients[uuid]=true
    else
        steppingClients[uuid]=nil
    end
    steppedClients[uuid]=nil
end

function step(uuid)
    if uuid==nil then
        uuid='ANY' -- to support older clients
    end
    steppedClients[uuid]=true
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
