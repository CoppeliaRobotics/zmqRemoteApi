function flatten(key, map)
    if key ~= '' then
        key = key .. '_'
    end
    local retMap = {}
    for k, v in pairs(map) do
        if type(v) == 'table' then
            local m = flatten(key .. k, v)
            for k2, v2 in pairs(m) do
                retMap[k2] = v2
            end
        elseif type(v) == 'number' then
            retMap[key .. k] = v
        end
    end
    return retMap
end

function loadAuxModule(nm)
    return require(nm)
end

r={}
local allConst = sim.getApiFunc(-1, '-')
local loadedModules = {}
for i = 1, #allConst do
    local v = allConst[i]
    local dotPos = v:find('%.')
    if v:sub(1, 3) == 'sim' then 
        v = v:sub(1, dotPos - 1)
        if loadedModules[v] == nil then
            loadedModules[v] = true
            local res, dat = pcall(loadAuxModule, v)
            if not res then
                print(dat)
            else
                r[v] = flatten('', dat)
            end
        end
    end
end

return r
