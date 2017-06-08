#!/usr/bin/env python
import time, threading, os
import Tkinter as tk

import clipsFunctions
from clipsFunctions import clips, _clipsLock

import pyRobotics.BB as BB
from pyRobotics.Messages import Command, Response

from knowledge_msgs.msg import *
from knowledge_msgs.srv import *
from interprete import intSpeech
from std_msgs.msg import Bool, String

import BBFunctions

import rospy

defaultTimeout = 2000
defaultAttempts = 1

def setLogLevelTest():
        _clipsLock.acquire()
        clips.SendCommand('(bind ?*logLevel* ' + 'getloglevel' + ')')
        clipsFunctions.PrintOutput()
        _clipsLock.release()

def callbackCommandResponse(data):
    print "callbackCommandResponse name command:" + data.name
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.name)
    clipsFunctions.Assert('(BB_received "{0}" {1} {2} "{3}")'.format(data.name, data.id, data.successful, data.params))
    clipsFunctions.PrintOutput()
    clipsFunctions.Run('') # aqui se manda el numero de pasos que ejecutara CLIPS con [''] se ejecutan todos los pasos sin detenerse
    clipsFunctions.PrintOutput()

def callbackCommandRunCLIPS(data):
    print "callbackCommandRUNCLIPS "
    clipsFunctions.Run('') # aqui se manda el numero de pasos que ejecutara CLIPS con [''] se ejecutan todos los pasos sin detenerse
    clipsFunctions.PrintOutput()

def callbackCommandResetCLIPS(data):
    clipsFunctions.Reset()
    print 'Facts were reset!'
    setLogLevelTest()

def callbackCommandFactCLIPS(data):
    print 'LIST OF FACTS'
    clipsFunctions.PrintFacts()

def callbackCommandRuleCLIPS(data):
    print 'LIST OF RULES'
    clipsFunctions.PrintRules()

def callbackCommandAgendaCLIPS(data):
    print 'AGENDA'
    clipsFunctions.PrintAgenda()

def callbackCommandSendCLIPS(data):
    print 'SEND COMMAND'
    _clipsLock.acquire()
    clips.SendCommand(data.data, True)
    clipsFunctions.PrintOutput()
    _clipsLock.release()

def callbackCommandSendAndRunClips(data):
    print 'SEND AND RUN COMMAND'
    _clipsLock.acquire()
    clips.SendCommand(data.data, True)
    clipsFunctions.PrintOutput()
    _clipsLock.release()
    clipsFunctions.Run('')
    clipsFunctions.PrintOutput()

def callbackCommandLoadCLIPS(data):
    print 'LOAD FILE'
    filePath = data.data
    if not filePath:
        print 'OPEN FILE, Click on the botton and select a file to be loaded.'
        return

    if filePath[-3:] == 'clp':
        _clipsLock.acquire()
        clips.BatchStar(filePath)
        clipsFunctions.PrintOutput()
        _clipsLock.release()
        print 'File Loaded!'
        return

    path = os.path.dirname(os.path.abspath(filePath))
    f = open(filePath, 'r')
    line = f.readline()
    _clipsLock.acquire()
    while line:
        clips.BatchStar((path + os.sep + line).strip())
        line = f.readline()
    f.close()
    clipsFunctions.PrintOutput()
    _clipsLock.release()

    print 'Files Loaded!'

    clipsFunctions.Reset()
    print 'Facts were reset!'
    setLogLevelTest()

def setCmdTimer(t, cmd, cmdId):
    t = threading.Thread(target=cmdTimerThread, args = (t, cmd, cmdId))
    t.daemon = True
    t.start()
    return True

def cmdTimerThread(t, cmd, cmdId):
    print 'cmdTimerThread function'
    time.sleep(t/1000)
    clipsFunctions.Assert('(BB_timer "{0}" {1})'.format(cmd, cmdId))
    clipsFunctions.PrintOutput()
    #clipsFunctions.Run(gui.getRunTimes())
    #clipsFunctions.PrintOutput()

def setTimer(t, sym):
    print 'setTime function'
    t = threading.Thread(target=timerThread, args = (t, sym))
    t.daemon = True
    t.start()
    return True

def timerThread(t, sym):
    print 'timerThread function'
    time.sleep(t/1000)
    clipsFunctions.Assert('(BB_timer {0})'.format(sym))
    clipsFunctions.PrintOutput()
    #clipsFunctions.Run(gui.getRunTimes())
    #clipsFunctions.PrintOutput()

def SendCommand(cmdName, params, timeout = defaultTimeout, attempts = defaultAttempts):
    global pubUnknown
    print 'Function name ' + cmdName
    cmd = Command(cmdName, params)
    func = fmap.get(cmdName)
    if func != None:
        func(cmd)
    else:
        request = PlanningCmdClips(cmd.name, cmd.params, cmd._id, False)
        pubUnknown.publish(request)
    return cmd._id

def cmd_query_kdb(req):
    print "Receive: [%s]"%(req.query)
    _clipsLock.acquire()
    clips.SendCommand(req.query, True)
    clipsFunctions.PrintOutput()
    _clipsLock.release()
    clipsFunctions.Run('')
    clipsFunctions.PrintOutput()


#def SendResponse(cmdName, cmd_id, result, response):
    #result = str(result).lower() not in ['false', '0']
    #r = Response(cmdName, result, response)
    #r._id = cmd_id
    #BB.Send(r)

def Initialize():
    clips.Memory.Conserve = True
    clips.Memory.EnvironmentErrorsEnabled = True
    
    clips.RegisterPythonFunction(SendCommand)
    #clips.RegisterPythonFunction(SendResponse)
    clips.RegisterPythonFunction(setCmdTimer)
    clips.RegisterPythonFunction(setTimer)
    #clips.RegisterPythonFunction(CreateSharedVar)
    #clips.RegisterPythonFunction(WriteSharedVar)
    #clips.RegisterPythonFunction(SubscribeToSharedVar)
    
    clips.BuildGlobal('defaultTimeout', defaultTimeout)
    clips.BuildGlobal('defaultAttempts', defaultAttempts)
    
    filePath = os.path.dirname(os.path.abspath(__file__))
    clips.BatchStar(filePath + os.sep + 'CLIPS' + os.sep + 'BB_interface.clp')
    
    file_gpsr = filePath + '/virbot_gpsr/speechTest.dat'
    print file_gpsr
    #BBFunctions.gui.putFileName(file_gpsr)
    #BBFunctions.gui.loadFile()
    #BBFunctions.gui.reset()

#Funcions to fmap, this functions are publish to topics to do the tasks
def cmd_speech(cmd):
    global pubCmdSpeech
    print "Executing function:" + cmd.name;
    request = PlanningCmdClips(cmd.name, cmd.params, cmd._id, False)
    pubCmdSpeech.publish(request)
    return cmd._id

def cmd_int(cmd):
    global pubCmdInt
    print "Executing function:" + cmd.name;
    request = PlanningCmdClips(cmd.name, cmd.params, cmd._id, False)
    pubCmdInt.publish(request)
    return cmd._id

def cmd_conf(cmd):
    global pubCmdInt
    print "Executing function:" + cmd.name;
    request = PlanningCmdClips(cmd.name, cmd.params, cmd._id, False)
    pubCmdConf.publish(request)
    return cmd._id

def cmd_task(cmd):
    global pubCmdInt
    print "Executing function:" + cmd.name;
    request = PlanningCmdClips(cmd.name, cmd.params, cmd._id, False)
    pubCmdGetTask.publish(request)
    return cmd._id

def goto(cmd):
    global pubCmdGoto
    print "Executing function:" + cmd.name;
    request = PlanningCmdClips(cmd.name, cmd.params, cmd._id, False)
    pubCmdGoto.publish(request)
    print "send pub"
    return cmd._id

def answer(cmd):
    global pubCmdAnswer
    print "Executing function:" + cmd.name;
    request = PlanningCmdClips(cmd.name, cmd.params, cmd._id, False)
    pubCmdAnswer.publish(request)
    return cmd._id

def find_object(cmd):
    global pubCmdFindObject
    print "Executing function:" + cmd.name;
    request = PlanningCmdClips(cmd.name, cmd.params, cmd._id, False)
    pubCmdFindObject.publish(request)
    return cmd._id

def ask_for(cmd):
    global pubCmdAskFor
    print "Executing function:" + cmd.name;
    request = PlanningCmdClips(cmd.name, cmd.params, cmd._id, False)
    pubCmdAskFor.publish(request)
    return cmd._id

def status_object(cmd):
    global pubCmdStatusObject
    print "Executing function:" + cmd.name;
    request = PlanningCmdClips(cmd.name, cmd.params, cmd._id, False)
    pubCmdStatusObject.publish(request)
    return cmd._id

def move_actuator(cmd):
    global pubCmdMoveActuator
    print "Executing function:" + cmd.name;
    request = PlanningCmdClips(cmd.name, cmd.params, cmd._id, False)
    pubCmdMoveActuator.publish(request)
    return cmd._id

def drop(cmd):
    global pubDrop
    print "Executing function:" + cmd.name;
    request = PlanningCmdClips(cmd.name, cmd.params, cmd._id, False)
    pubDrop.publish(request)
    return cmd._id

def ask_person(cmd):
    global pubCmdAskPerson
    print "Executing function:" + cmd.name;
    request = PlanningCmdClips(cmd.name, cmd.params, cmd._id, False)
    pubCmdAskPerson.publish(request)
    return cmd._id

def find_category(cmd):
    global pubCmdFindCategory
    print "Executing function:" + cmd.name;
    request = PlanningCmdClips(cmd.name, cmd.params, cmd._id, False)
    pubCmdFindCategory.publish(request)
    return cmd._id

def many_obj(cmd):
    global pubCmdManyObjects
    print "Executing function:" + cmd.name
    request = PlanningCmdClips(cmd.name, cmd.params, cmd._id, False)
    pubCmdManyObjects.publish(request)
    return cmd._id

def query_result(cmd):
    global pubCmdAskQues
    print "Executing function:" + cmd.name
    request = PlanningCmdClips(cmd.name, cmd.params, cmd._id, False)
    pubCmdQueryResult.publish(request)
    return cmd._id

#Define the function map, this function are the functions that represent of task in the clips rules.
fmap = {
    'cmd_speech': cmd_speech,
    'cmd_int': cmd_int,
    'cmd_conf': cmd_conf,
    'cmd_task': cmd_task,
    'find_object': find_object,
    'move_actuator': move_actuator,
    #'grab': grab,
    'drop': drop,
    'status_object': status_object,
    'goto': goto,
    #'speak': speak,
    'ask_for': ask_for,
    'answer' : answer,
    'ask_person':ask_person,
    'find_category': find_category,
    'many_obj': many_obj,
    'query_result': query_result
}

def quit():
    global tk
    tk.quit()

def main():

    global pubCmdSpeech, pubCmdInt, pubCmdConf, pubCmdGetTask, pubUnknown
    global pubCmdGoto, pubCmdAnswer, pubCmdFindObject, pubCmdAskFor, pubCmdStatusObject, pubCmdMoveActuator, pubDrop, pubCmdAskPerson
    global pubCmdFindCategory, pubCmdManyObjects, pubCmdQueryResult

    rospy.init_node('knowledge_representation')
    rospy.Subscriber("/planning_clips/command_response", PlanningCmdClips, callbackCommandResponse)
    rospy.Subscriber("/planning_clips/command_runCLIPS",Bool, callbackCommandRunCLIPS)
    rospy.Subscriber("/planning_clips/command_resetCLIPS",Bool, callbackCommandResetCLIPS)
    rospy.Subscriber("/planning_clips/command_factCLIPS",Bool, callbackCommandFactCLIPS)
    rospy.Subscriber("/planning_clips/command_ruleCLIPS",Bool, callbackCommandRuleCLIPS)
    rospy.Subscriber("/planning_clips/command_agendaCLIPS",Bool, callbackCommandAgendaCLIPS)

    rospy.Subscriber("/planning_clips/command_sendCLIPS",String, callbackCommandSendCLIPS)
    rospy.Subscriber("/planning_clips/command_sendAndRunCLIPS", String, callbackCommandSendAndRunClips)
    rospy.Subscriber("/planning_clips/command_loadCLIPS",String, callbackCommandLoadCLIPS)

    pubCmdSpeech = rospy.Publisher('/planning_clips/cmd_speech', PlanningCmdClips, queue_size=1)
    pubCmdInt = rospy.Publisher('/planning_clips/cmd_int', PlanningCmdClips, queue_size=1)
    pubCmdConf = rospy.Publisher('/planning_clips/cmd_conf', PlanningCmdClips, queue_size=1)
    pubCmdGetTask = rospy.Publisher('/planning_clips/cmd_task', PlanningCmdClips, queue_size=1)
    pubCmdGoto = rospy.Publisher('/planning_clips/cmd_goto', PlanningCmdClips, queue_size=1)
    pubCmdAnswer = rospy.Publisher('/planning_clips/cmd_answer', PlanningCmdClips, queue_size=1)
    pubCmdFindObject = rospy.Publisher('/planning_clips/cmd_find_object', PlanningCmdClips, queue_size=1)
    pubCmdAskFor = rospy.Publisher('/planning_clips/cmd_ask_for', PlanningCmdClips, queue_size=1)
    pubCmdStatusObject = rospy.Publisher('/planning_clips/cmd_status_object', PlanningCmdClips, queue_size=1)
    pubCmdMoveActuator = rospy.Publisher('/planning_clips/cmd_move_actuator', PlanningCmdClips, queue_size=1)
    pubDrop = rospy.Publisher('/planning_clips/cmd_drop', PlanningCmdClips, queue_size=1)
    pubUnknown = rospy.Publisher('/planning_clips/cmd_unknown', PlanningCmdClips, queue_size=1)
    pubCmdAskPerson = rospy.Publisher('/planning_clips/cmd_ask_person', PlanningCmdClips, queue_size=1)
    pubCmdFindCategory = rospy.Publisher('/planning_clips/cmd_find_category', PlanningCmdClips, queue_size=1)
    pubCmdManyObjects = rospy.Publisher('/planning_clips/cmd_many_obj', PlanningCmdClips, queue_size=1)
    pubCmdQueryResult = rospy.Publisher('/planning_clips/cmd_query_result', PlanningCmdClips, queue_size=1)

    Initialize()
    
    rospy.spin()
    #tk.mainloop()

if __name__ == "__main__":
    main()
