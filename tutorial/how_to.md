# RoboComp components for V-REP simulator



The objective of this guide is to explain how to work with V-REP and RoboComp, creating RoboComp components that interact with V-REP simulator.



## Requirements

- Having RoboComp installed on your computer, if you don't know how to do it you can find the tutorial [here](https://github.com/robocomp/robocomp)

- Having V-REP PRO (or PRO EDU version) in your computer. You can find it [here](http://www.coppeliarobotics.com/downloads.html)



## Creating the component

_To explain how to create and use the component, the tutorial will use an already made example to illustrate all the steps_



The RoboComp components can be built using RoboCompDSL tool. This tool generates a CDSL file template that we will modify with our specifications and properties.

In a terminal, we move to the components' folder and we create a folder that will contain the component.

```sh
cd ~/robocomp/components/
mkdir newComponent
cd newComponent
```

After that, we use the RoboCompDSL tool to create the cdsl file.

```sh
robocompdsl newComponent.cdsl
```

Next, we have a CDSL file created inside the component folder named _newComponent.cdsl_. The CDSL files and an introduction to components are described [in this link](https://github.com/robocomp/robocomp/blob/stable/doc/components.md). The generated file will have this code inside

```c
import "import1.idsl";
import "import2.idsl";

Component newComponent
{
        Communications
        {
                implements interfaceName;
                requires otherName;
                subscribesTo topicToSubscribeTo;
                publishes topicToPublish;
        };
        language Cpp//Cpp11//python;
        gui Qt(QWidget//QDialog//QMainWindow);
        //options agmagent;
        //options InnerModelViewer;
};
```



We will design a component that controls an e-Puck robot model, already saved on V-REP models. This robot has differential robot characteristics, so we will use a differential robot interface to communicate with it if it was necessary. This means that we will need the _differentialRobot_ interface implemented in our component. So we will alter the CDSL file.

```c
import "DifferentialRobot.idsl";

Component newComponent
{
        Communications
        {
                implements DifferentialRobot;
        };
        language python;
};

```

As we can see in the code, we import _DifferentialRobot.idsl_ to use in the communications (_implements DifferentialRobot_) and we specified the language, in this case, python.

After that, we will generate the RoboComp component using RoboCompDSL.

```bash
robocompdsl newComponent.cdsl .
```



When we did it, the folder will have these files:


```bash
.
├── DoxyFile
├── etc
│   └── config
├── newComponent.cdsl
├── README.md
└── src
    ├── differentialrobotI.py
    ├── newComponent.py
    ├── genericworker.py
    └── specificworker.py
```

As we can see, we have in `etc/` the _config_ file and in `src/` some _.py_ files. The specific worker will be something like the following code 

```python
import sys, os, traceback, time

from PySide import QtGui, QtCore
from genericworker import *

# If RoboComp was compiled with Python bindings you can use InnerModel in Python
# sys.path.append('/opt/robocomp/lib')
# import librobocomp_qmat
# import librobocomp_osgviewer
# import librobocomp_innermodel

class SpecificWorker(GenericWorker):
        def __init__(self, proxy_map):
                super(SpecificWorker, self).__init__(proxy_map)
                self.timer.timeout.connect(self.compute)
                self.Period = 2000
                self.timer.start(self.Period)

        def setParams(self, params):
                #try:
                #       self.innermodel = InnerModel(params["InnerModelPath"])
                #except:
                #       traceback.print_exc()
                #       print "Error reading config params"
                return True

        @QtCore.Slot()
        def compute(self):
                print 'SpecificWorker.compute...'
                #computeCODE
                #try:
                #       self.differentialrobot_proxy.setSpeedBase(100, 0)
                #except Ice.Exception, e:
                #       traceback.print_exc()
                #       print e

                # The API of python-innermodel is not exactly the same as the C++ version
                # self.innermodel.updateTransformValues("head_rot_tilt_pose", 0, 0, 0, 1.3, 0, 0)
                # z = librobocomp_qmat.QVec(3,0)
                # r = self.innermodel.transform("rgbd", z, "laser")
                # r.printvector("d")
                # print r[0], r[1], r[2]

                return True


        #
        # correctOdometer
        #
        def correctOdometer(self, x, z, alpha):
                #
                #implementCODE
                #
                pass


        #
        # getBasePose
        #
        def getBasePose(self):
                #
                #implementCODE
                #
                x = int()
                z = int()
                alpha = float()
                return [x, z, alpha]

        #
        # resetOdometer
        #
        def resetOdometer(self):
                #
                #implementCODE
                #
                pass


        #
        # setOdometer
        #
        def setOdometer(self, state):
                #
                #implementCODE
                #
                pass


        #
        # getBaseState
        #
        def getBaseState(self):
                #
                #implementCODE
                #
                state = RoboCompGenericBase.TBaseState()
                return state


        #
        # setOdometerPose
        #
        def setOdometerPose(self, x, z, alpha):
                #
                #implementCODE
                #
                pass


        #
        # stopBase
        #
        def stopBase(self):
                #
                #implementCODE
                #
                pass


        #
        # setSpeedBase
        #
        def setSpeedBase(self, adv, rot):
                #
                #implementCODE
                #
                pass
```



All of these empty methods needs to be implemented because belongs to the _DifferentialRobot_ interface, we just need to complete the code to have a fully functional component.



## Implementing the VREP remote API

The VREP remote API can be implemented just copying the files in the `src/` folder. The folder will look like the next scheme.

```bash
src
├── differentialrobotI.py
├── newComponent.py
├── extApi.o
├── extApiPlatform.o
├── genericworker.py
├── remoteApi.dll
├── remoteApi.so
├── specificworker.py
├── vrep_client_controller.py
├── vrepConst.py
├── vrep.py
```



## Using V-REP remote API in the component

Now, we just need to import to the `.py` code where we will use it



### Example: How EV3_LEGO component implements the V-REP library

In this example, the component uses a class that works as a controller of V-REP named  [`EV3_LEGO_controller.py`](https://github.com/JMAgundezG/V-REP/blob/master/components/EV3_VREP/src/EV3_LEGO_controller.py) which inherits from another class that works as a client of V-REP named [`vrep_client_controller.py`](https://github.com/JMAgundezG/V-REP/blob/master/components/EV3_VREP/src/vrep_client_controller.py) 



#### How the component uses this controller

There are some solutions to that, the simplest one is just importing `EV3_LEGO_controller.py` class in `specificWorker.py` and using it as a handler.

```python
class SpecificWorker(GenericWorker):
        def __init__(self, proxy_map):
                super(SpecificWorker, self).__init__(proxy_map)
                self.handler = EV3Controller("158.49.227.95", 19998, debug=True)
                self.timer.timeout.connect(self.compute)
                self.Period = 2000
                self.timer.start(self.Period)

```

As we can see in the code, we create an _EV3Controller_ object using the V-REP ip host and the V-REP opened port as parameters.



### Running the component



After implementing the code and having in the  `compute()` method the instructions of the component, we just need to run our component. We need to have V-REP already running on any computer. and execute this command on our component folder

```bash
python src/EV3_VREP.py etc/config 
```



After that, our RoboComp component will work using the V-REP simulator.

