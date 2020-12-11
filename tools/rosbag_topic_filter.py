#!/usr/bin/env python
import os, sys
import argparse

import rosbag
from tf.msg import tfMessage

def plist(head,values,dplace=2,sep=', '):
    if values is None: return
    try:
        if(isinstance(values[0],int)): print(head + sep.join(str(val) for val in values))
        elif(isinstance(values[0],float)): print(head + sep.join(str(round(val,dplace)) for val in values))
        else: print(head + sep.join(str(val) for val in values))
    except:
        print(head + sep.join(str(val) for val in values))
        pass

def check_topic_for_keeping(pString, keep_patterns=[], remove_patterns=[],  verbose=False):
    if pString == '': return False
    # if(verbose):
    #     print("[INFO] check_topic_for_keeping() --- Checking whether to keep given topic = \'%s\'..." % pString)
    #     print("[INFO] check_topic_for_keeping() --- Keep Patterns Given = %s" % str(keep_patterns))
    #     print("[INFO] check_topic_for_keeping() --- Remove Patterns Given = %s" % str(remove_patterns))

    foundBadApple = False
    for p in remove_patterns:
        if(p in pString):
            if(verbose): print("[INFO] check_topic_for_keeping() --- Found a match to the remove_pattern = \'%s\' with the given topic = \'%s\'. This topic should NOT be kept." % (p, pString))
            foundBadApple = True

    foundAKeeper = False
    for p in keep_patterns:
        if(p in pString):
            if(verbose): print("[INFO] check_topic_for_keeping() --- Found a match to the keep_pattern = \'%s\' with the given topic = \'%s\'. This topic should be kept." % (p, pString))
            foundAKeeper = True
    # Prevent falsely identifying topic as a bad apple if no keep patterns are given
    if(len(keep_patterns) <= 0): foundAKeeper = True
    isAKeeper = (foundAKeeper and not foundBadApple)

    if(verbose): print("[INFO] check_topic_for_keeping() --- Input topic \'%s\' -- is a keeper = %s. (isBadApple = %s, isKeeper = %s)" % (pString, str(isAKeeper), str(foundBadApple), str(foundAKeeper)) )
    return isAKeeper

def yes_or_no(question):
    while "the answer is invalid":
        reply = str(raw_input(question+' (y/n): ')).lower().strip()
        if reply[0] == 'y': return True
        if reply[0] == 'n': return False


if __name__ == "__main__" :

    topicKeepList = []
    keepPatterns = []
    removePatterns = []
    inputBagPath = "input.bag"
    newBagPath = "output.bag"

    # Setup commandline argument(s) structures
    ap = argparse.ArgumentParser(description='Extract certain rostopics in a recorded rosbag file into a new rosbag file.')
    ap.add_argument("--input", "-i", type=str, metavar='PATH', help="File path to input rosbag file")
    ap.add_argument("--output", "-o", type=str, metavar='PATH', default='rosbag_subsets.bag', help="File path of the new rosbag file to generate")
    ap.add_argument("--keep-patterns", "-k", nargs='+', type=str, metavar='LIST', help="List of string patterns that should be found in the name of rostopics you want to keep in the new rosbag")
    ap.add_argument("--remove-patterns", "-r", nargs='+', type=str, metavar='LIST', help="List of string patterns that should be found in the name of rostopics you DONT want to keep in the new rosbag")
    # Store parsed arguments into array of variables
    args = vars(ap.parse_args())

    # Extract stored arguments array into individual variables for later usage in script
    inputFpath = args["input"]
    outputFpath = args["output"]
    keepPats = args["keep_patterns"]
    removePats = args["remove_patterns"]
    print("[INFO] rosbag_topic_filter.py --- Parsed Input arguments:")
    print("[INFO] rosbag_topic_filter.py --- --- Input rosbag path = %s" % (inputFpath))
    print("[INFO] rosbag_topic_filter.py --- --- Output rosbag path = %s" % (outputFpath))
    plist("[INFO] rosbag_topic_filter.py --- --- Keep Patterns = ", keepPats)
    plist("[INFO] rosbag_topic_filter.py --- --- Remove Patterns = ", removePats)
    print("[INFO] rosbag_topic_filter.py --- ------------------- ")
    if inputFpath is None:
        print("[INFO] rosbag_topic_filter.py --- No path was given to an input rosbag! Exitting...")
        sys.exit(-1)

    if keepPats is not None: [keepPatterns.append(r'{}'.format(pat)) for pat in keepPats]
    if removePats is not None: [removePatterns.append(r'{}'.format(pat)) for pat in removePats]

    inputBagPath = inputFpath
    inputBagDir = os.path.dirname(inputFpath)
    newBagPath = inputBagDir + "/" + outputFpath + ".bag"
    print(inputBagDir)
    print(newBagPath)

    bagIn = rosbag.Bag(inputBagPath)
    bagData = bagIn.read_messages()
    nData = int(bagIn.get_message_count())
    print("[INFO] rosbag_topic_filter.py --- Input rosbag file \'%s\' contains %d msgs." % (inputBagPath, nData))

    newCnt = 0
    inputCnt = 0
    newBagData = []
    keptTopicHistory = []
    for topic, msg, stamp in bagData:
        inputCnt += 1
        isAKeeper = check_topic_for_keeping(topic, keepPatterns, removePatterns)
        if isAKeeper:
            if(topic not in keptTopicHistory):
                keptTopicHistory.append(topic)
                print("[INFO] rosbag_topic_filter.py ---- New rosbag will have topic --- \'%s\'" % (topic) )
            newBagData.append([topic, msg, stamp])
            newCnt += 1

    do_save_to_file = yes_or_no("Do you want to save these topics to the new rosbag at \'%s\'?" % newBagPath)

    if(do_save_to_file):
        print("[INFO] rosbag_topic_filter.py --- Writing %d / %d ROS msgs into new rosbag file..." % (newCnt, nData))
        with rosbag.Bag(newBagPath, mode='w') as outbag:
            [outbag.write(tp, m, stmp) for tp, m, stmp in newBagData]
        print("rosbag filtering finished.")
    else: print("[INFO] rosbag_topic_filter.py --- Try running this script again w/ different keep/remove patterns.")
