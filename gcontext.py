from direct.showbase.ShowBase import ShowBase
from direct.task import Task
from direct.interval.IntervalGlobal import Sequence
from panda3d.core import Point3
from math import pi, sin, cos

class Context(ShowBase):
    def __init__(self):
        ShowBase.__init__(self, updatePModel)

        self.scene = self.loader.loadModel('environment')
        self.scene.reparentTo(self.render)
        self.scene.setScale(0.25, 0.25, 0.25)
        self.scene.setPos(-8, 42, 0)

        self.panda = self.loader.loadModel('panda-model')
        self.panda.setScale(0.005, 0.005, 0.005)
        self.panda.reparentTo(self.render)

        self.taskMgr.add(self._updatePModelTask, 'UpdatePhysicalModel')
        self._lastPModelUpdate = 0
        self._updatePModel = updatePModel


    def _updatePModelTask(self, task):
        elapsedTime = task.time - self._lastPModelUpdate
        self._updatePModel(elapsedTime)
        self._lastPModelUpdate = task.time
        return Task.cont