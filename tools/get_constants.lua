function stripFunctions(t)
    if type(t)=='table' then
        local r={}
        for k,v in pairs(t) do
            if type(v)~='function' then
                r[k]=stripFunctions(v)
            end
        end
        if next(r) then return r end
    else
        return t
    end
end
r={}
for k,v in pairs(_G) do
    if type(v)=='table' and k:sub(1,3)=='sim' then
        r[k]=stripFunctions(v)
    end
end
return r
