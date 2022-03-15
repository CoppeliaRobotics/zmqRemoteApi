if __name__ == '__main__':
    import sys
    from calltip import FuncDef
    from pprint import PrettyPrinter
    pp = PrettyPrinter(indent=4)
    try:
        if len(sys.argv) > 1:
            func_defs = FuncDef.get(sys.argv[1:])
        else:
            func_defs = FuncDef.get_all()
        pp.pprint(func_defs)
    except Exception as e:
        print(f'error: {e}')
