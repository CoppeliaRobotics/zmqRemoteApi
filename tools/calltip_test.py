if __name__ == '__main__':
    from calltip import FuncDef
    from pprint import PrettyPrinter
    pp = PrettyPrinter(indent=4)
    try:
        func_defs = FuncDef.get_all()
        pp.pprint(func_defs)
    except Exception as e:
        print(f'error: {e}')
