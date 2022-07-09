import numpy as np

def getAVID(ID, AV_number, platoon_bool) : 
    # Define the spatial formation of the AVs
    
    if AV_number == 4:
        if platoon_bool:
            ID[8] = 1
            ID[9] = 1
            ID[10] = 1
            ID[11] = 1
        else:
            ID[2] = 1
            ID[7] = 1
            ID[12] = 1
            ID[17] = 1
            
    if AV_number == 2:
        if platoon_bool:
            ID[9] = 1
            ID[10] = 1
        else:
            ID[4] = 1
            ID[14] = 1
            
    if AV_number == 1:
        ID[19] = 1

    return ID