zmqRemoteApi={}

function zmqRemoteApi.verbose()
    return tonumber(sim.getStringNamedParam('zmqRemoteApi.verbose') or '0')
end

function zmqRemoteApi.info(obj)
    if type(obj)=='string' then obj=zmqRemoteApi.getField(obj) end
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
        print('request received:',req)
    end
    local resp={}
    local func,args=zmqRemoteApi.getField(req['func']),req['args']
    if not func then
        resp['error']='No such function: '..req['func']
    else
        local status,retvals=pcall(function()
            local ret={func(unpack(args))}
            return ret
        end)
        resp[status and 'ret' or 'error']=retvals
    end
    resp['success']=resp['error']==nil
    if zmqRemoteApi.verbose()>1 then
        print('returning response:',resp)
    end
    return resp
end

function zmqRemoteApi.handleRawMessage(rawReq)
    -- if first byte is '{', it *might* be a JSON payload
    if rawReq:byte(1)==123 then
        local status,req=pcall(json.decode,rawReq)
        if status then
            local resp=zmqRemoteApi.handleRequest(req)
            return json.encode(resp)
        end
    end

    -- if we are here, it should be a CBOR payload
    local status,req=pcall(cbor.decode,rawReq)
    if status then
        local resp=zmqRemoteApi.handleRequest(req)
        return cbor.encode(resp)
    end

    sim.addLog(sim.verbosity_errors,'cannot decode message: no suitable decoder')
    return ''
end

function zmqRemoteApi.handleQueue()
    while true do
        local rc,revents=simZMQ.poll({socket},{simZMQ.POLLIN},0)
        if rc<=0 then break end
        local rc,req=simZMQ.recv(socket,0)
        local resp=zmqRemoteApi.handleRawMessage(req)
        simZMQ.send(socket,resp,0)
    end
end

function sysCall_info()
    return {autoStart=true}
end

function sysCall_init()
    if not simZMQ then
        sim.addLog(sim.verbosity_errors,'zmqRemoteApi: the ZMQ plugin is not available')
        return {cmd='cleanup'}
    end
    json=require 'dkjson'
    cbor=require 'cbor'
    context=simZMQ.ctx_new()
    socket=simZMQ.socket(context,simZMQ.REP)
    local rc=simZMQ.bind(socket,'tcp://*:23000')
    if rc~=0 then
        error('bind() failed')
    end
    if zmqRemoteApi.verbose()>0 then
        sim.addLog(sim.verbosity_scriptinfos,'ZeroMQ Remote API started')
    end
end

function sysCall_cleanup()
    if not simZMQ then return end
    simZMQ.close(socket)
    simZMQ.ctx_term(context)
    if zmqRemoteApi.verbose()>0 then
        sim.addLog(sim.verbosity_scriptinfos,'ZeroMQ Remote API stopped')
    end
end

function sysCall_addOnScriptSuspend()
    return {cmd='cleanup'}
end

function sysCall_addOnScriptSuspended()
    return {cmd='cleanup'}
end

function sysCall_nonSimulation()
    zmqRemoteApi.handleQueue()
end

function sysCall_actuation()
    zmqRemoteApi.handleQueue()
end

function sysCall_sensing()
end
