import math
import smach

class Pattern2D:
    def __init__(self, **kwargs):
        print("pattern constructor")
        self.id = kwargs["id"]
        self.height = kwargs["height"]
        self.width = kwargs["width"]
        self.center = kwargs["center"]
        self.vortexs = self.generate_votexs()

    def generate_votexs(self):
        print ("default generator")
        return iter([([0,0,0], [0,0,0])])

    def __call__(self):
        pass

class Rectangle2D(Pattern2D, smach.State):
    def __init__(self, **kwargs):
        Pattern2D.__init__(self,**kwargs)
        smach.State.__init__(self, outcomes=kwargs["outcomes"], output_keys = kwargs["output_keys"])
        print ("vertex", self.vortexs)

    def __call__(self):
        try:
            s,e = next(self.vortexs)
        except:
            s,e = None, None
        return s,e

    def execute(self,userdata):
        s,e = self.__call__()

        if s is None:
            self.vortexs = self.generate_votexs()
            print ("restart")
            s,e = self.__call__()
        userdata.start = s
        userdata.end = e
        return 'succeeded'



    def generate_votexs(self):
        print ("new generator")
        s = list()
        e = list()
        #FIRST COORDINATES
        s.append(self.center)
        e.append([self.center[0]+self.height, self.center[1], self.center[2]])

        s.append([self.center[0]+self.height, self.center[1], self.center[2]+math.pi/2])

        e.append([self.center[0]+self.height, self.center[1]+ self.width, self.center[2]+math.pi/2])

        s.append([self.center[0]+self.height, self.center[1]+ self.width, self.center[2]+math.pi])
        e.append([self.center[0], self.center[1]+self.width, self.center[2]+math.pi + math.pi])

        s.append([self.center[0], self.center[1]+self.width, self.center[2]+math.pi + math.pi/2])
        e.append([self.center[0], self.center[1], self.center[2]+math.pi + math.pi/2])

        return iter(zip(s,e))
