if __name__ == '__main__':
    import sys
    from calltip import FuncDef
    from pprint import PrettyPrinter
    pp = PrettyPrinter(indent=4)
    c = 'int objectHandle=sim.getObject(string path,map options={})'
    func_defs = FuncDef.from_calltip(c)
    pp.pprint(func_defs)
