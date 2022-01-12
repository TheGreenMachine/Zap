
class HWCheck:
    def __init__(self):
        pass
    def CheckZed(self):
        try:
            import pyzed.sl as sl
            return True
        except:
            print('ZED is not installed')
            return False