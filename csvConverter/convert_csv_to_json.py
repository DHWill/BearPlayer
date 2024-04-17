import csv
import json

def calcPosition(_parentState):
    positions = {"Left":0, "Front":1, "Right":2, "Idle":1, "PB":1}
    isEarlyExit = False
    if(len(_parentState.split('_')) > 1):
        currentPosition = _parentState.split('_')[0]
        targetPosition = _parentState.split('_')[1]
    else:
        currentPosition = _parentState
        targetPosition = _parentState
    position = positions.get(currentPosition)
    targetPosition = positions.get(targetPosition)

    return position, targetPosition

def getDataFromClipName(_clipName, dictList):
    for reg in dictList:
        if(reg['Clip Name'] == _clipName):
            return reg

def csv_to_json(sm_file, videoSM_file, json_file):
    sm_data = []
    videoSM_data = []
    output_data = []
    
    # Read the CSV file
    with open(sm_file, 'r') as file1:
        sm_reader = csv.DictReader(file1)
        for row in sm_reader:
            sm_data.append(row)
    
    with open(videoSM_file, 'r') as file2:
        videoSM_reader = csv.DictReader(file2)
        for row in videoSM_reader:
            videoSM_data.append(row)
    
    for sm_anim in sm_data:
        for vid_anim in videoSM_data:
            if(vid_anim['Clip Name'] == sm_anim['Clip Name']):
                clipName = vid_anim['Clip Name']
                startTime = int(vid_anim['From'])
                endTime = int(vid_anim['To'])
                parentState = sm_anim['Parents']
                transitionTime = 0
            
                isEarlyExit = False
                if(len(parentState.split('/')) > 1):
                    parentAnimation = parentState.split('/')[1]
                    parentState = parentState.split('/')[0]
                    isEarlyExit = True

                    parentStateSM = getDataFromClipName(parentAnimation,sm_data)
                    parentStateVid = getDataFromClipName(parentAnimation,videoSM_data)
                    stateSM = getDataFromClipName(clipName,sm_data)
                    
                    transitionOffset = int(stateSM['Transition Frame']) - int(parentStateSM['From'])
                    transitionOffset *= 2
                    transitionTime = int(parentStateVid['From']) + transitionOffset

                else:
                    parentAnimation = "None"

                
                if(sm_anim['IsActive'] == "TRUE"):
                    isActive = True
                else: isActive = False

                position, targetPosition = calcPosition(parentState)
                
                output_data.append({'name':clipName, 
                                    'startTime':startTime, 
                                    'endTime':endTime,
                                    'transitionFromParent': transitionTime,
                                    'parentAnimation' : parentAnimation,
                                    'parentState':parentState,
                                    'position':position,
                                    'targetPosition':targetPosition,
                                    'isEarlyExit':isEarlyExit,
                                    'isActive':isActive})
    # Write the data to a JSON file
    with open(json_file, 'w') as file:
        json.dump(output_data, file, indent=4)
        
        




    
        
    
# Replace 'inputcsv' and 'output.json' with the actual file names


csv_to_json('SM.csv', 'Video_SM.csv', 'video_states.json')
