import numpy as np

def getAVIDPercentile(N, platoon_bool, ID, AV_number) : 
    # Define the spatial formation of the AVs
    
    if N == 70 :
        if platoon_bool:
            if AV_number == 70:
                ID = np.ones([N])

            if AV_number == 53:
                ID[5] = 1
                ID[6] = 1
                ID[7] = 1
                ID[8] = 1
                ID[9] = 1
                ID[10] = 1
                ID[11] = 1
                ID[12] = 1
                ID[13] = 1
                ID[14] = 1
                ID[15] = 1
                ID[16] = 1
                ID[17] = 1
                ID[18] = 1
                ID[19] = 1
                ID[20] = 1
                ID[21] = 1
                ID[22] = 1
                ID[23] = 1
                ID[24] = 1
                ID[25] = 1
                ID[26] = 1
                ID[27] = 1
                ID[28] = 1
                ID[29] = 1
                ID[30] = 1
                ID[31] = 1
                ID[32] = 1
                ID[33] = 1
                ID[34] = 1
                ID[35] = 1
                ID[36] = 1
                ID[37] = 1
                ID[38] = 1
                ID[39] = 1
                ID[40] = 1
                ID[41] = 1
                ID[42] = 1
                ID[43] = 1
                ID[44] = 1
                ID[45] = 1
                ID[46] = 1
                ID[47] = 1
                ID[48] = 1
                ID[49] = 1
                ID[50] = 1
                ID[51] = 1
                ID[52] = 1
                ID[53] = 1
                ID[54] = 1
                ID[55] = 1
                ID[56] = 1
                ID[57] = 1
            
            if AV_number == 35:
                ID[5] = 1
                ID[6] = 1
                ID[7] = 1
                ID[8] = 1
                ID[9] = 1
                ID[10] = 1
                ID[11] = 1
                ID[12] = 1
                ID[13] = 1
                ID[14] = 1
                ID[15] = 1
                ID[16] = 1
                ID[17] = 1
                ID[18] = 1
                ID[19] = 1
                ID[20] = 1
                ID[21] = 1
                ID[22] = 1
                ID[23] = 1
                ID[24] = 1
                ID[25] = 1
                ID[26] = 1
                ID[27] = 1
                ID[28] = 1
                ID[29] = 1
                ID[30] = 1
                ID[31] = 1
                ID[32] = 1
                ID[33] = 1
                ID[34] = 1
                ID[35] = 1
                ID[36] = 1
                ID[37] = 1
                ID[38] = 1
                ID[39] = 1

            if AV_number == 32:
                ID[5] = 1
                ID[6] = 1
                ID[7] = 1
                ID[8] = 1
                ID[9] = 1
                ID[10] = 1
                ID[11] = 1
                ID[12] = 1
                ID[13] = 1
                ID[14] = 1
                ID[15] = 1
                ID[16] = 1
                ID[17] = 1
                ID[18] = 1
                ID[19] = 1
                ID[20] = 1
                ID[21] = 1
                ID[22] = 1
                ID[23] = 1
                ID[24] = 1
                ID[25] = 1
                ID[26] = 1
                ID[27] = 1
                ID[28] = 1
                ID[29] = 1
                ID[30] = 1
                ID[31] = 1
                ID[32] = 1
                ID[33] = 1
                ID[34] = 1
                ID[35] = 1
                ID[36] = 1

            if AV_number == 28:
                ID[5] = 1
                ID[6] = 1
                ID[7] = 1
                ID[8] = 1
                ID[9] = 1
                ID[10] = 1
                ID[11] = 1
                ID[12] = 1
                ID[13] = 1
                ID[14] = 1
                ID[15] = 1
                ID[16] = 1
                ID[17] = 1
                ID[18] = 1
                ID[19] = 1
                ID[20] = 1
                ID[21] = 1
                ID[22] = 1
                ID[23] = 1
                ID[24] = 1
                ID[25] = 1
                ID[26] = 1
                ID[27] = 1
                ID[28] = 1
                ID[29] = 1
                ID[30] = 1
                ID[31] = 1
                ID[32] = 1

            if AV_number == 25:
                ID[5] = 1
                ID[6] = 1
                ID[7] = 1
                ID[8] = 1
                ID[9] = 1
                ID[10] = 1
                ID[11] = 1
                ID[12] = 1
                ID[13] = 1
                ID[14] = 1
                ID[15] = 1
                ID[16] = 1
                ID[17] = 1
                ID[18] = 1
                ID[19] = 1
                ID[20] = 1
                ID[21] = 1
                ID[22] = 1
                ID[23] = 1
                ID[24] = 1
                ID[25] = 1
                ID[26] = 1
                ID[27] = 1
                ID[28] = 1
                ID[29] = 1

            if AV_number == 21:
                ID[5] = 1
                ID[6] = 1
                ID[7] = 1
                ID[8] = 1
                ID[9] = 1
                ID[10] = 1
                ID[11] = 1
                ID[12] = 1
                ID[13] = 1
                ID[14] = 1
                ID[15] = 1
                ID[16] = 1
                ID[17] = 1
                ID[18] = 1
                ID[19] = 1
                ID[20] = 1
                ID[21] = 1
                ID[22] = 1
                ID[23] = 1
                ID[24] = 1
                ID[25] = 1

            if AV_number == 18:
                ID[5] = 1
                ID[6] = 1
                ID[7] = 1
                ID[8] = 1
                ID[9] = 1
                ID[10] = 1
                ID[11] = 1
                ID[12] = 1
                ID[13] = 1
                ID[14] = 1
                ID[15] = 1
                ID[16] = 1
                ID[17] = 1
                ID[18] = 1
                ID[19] = 1
                ID[20] = 1
                ID[21] = 1
                ID[22] = 1

            if AV_number == 14:
                ID[5] = 1
                ID[6] = 1
                ID[7] = 1
                ID[8] = 1
                ID[9] = 1
                ID[10] = 1
                ID[11] = 1
                ID[12] = 1
                ID[13] = 1
                ID[14] = 1
                ID[15] = 1
                ID[16] = 1
                ID[17] = 1
                ID[18] = 1

            if AV_number == 11:
                ID[5] = 1
                ID[6] = 1
                ID[7] = 1
                ID[8] = 1
                ID[9] = 1
                ID[10] = 1
                ID[11] = 1
                ID[12] = 1
                ID[13] = 1
                ID[14] = 1
                ID[15] = 1
                    
            if AV_number == 7:
                ID[5] = 1
                ID[6] = 1
                ID[7] = 1
                ID[8] = 1
                ID[9] = 1
                ID[10] = 1
                ID[11] = 1

            if AV_number == 4:
                ID[5] = 1
                ID[6] = 1
                ID[7] = 1
                ID[8] = 1          
        else:
            if AV_number == 70:
                ID = np.ones([N])

            if AV_number == 53:
                ID[5] = 1
                ID[6] = 1
                ID[8] = 1
                ID[9] = 1
                ID[11] = 1
                ID[12] = 1
                ID[13] = 1
                ID[14] = 1
                ID[15] = 1
                ID[17] = 1
                ID[19] = 1
                ID[18] = 1
                ID[20] = 1
                ID[21] = 1
                ID[22] = 1
                ID[23] = 1
                ID[24] = 1
                ID[26] = 1
                ID[27] = 1
                ID[29] = 1
                ID[30] = 1
                ID[31] = 1
                ID[32] = 1
                ID[33] = 1
                ID[35] = 1
                ID[36] = 1
                ID[38] = 1
                ID[39] = 1
                ID[41] = 1
                ID[42] = 1
                ID[44] = 1
                ID[45] = 1
                ID[47] = 1
                ID[48] = 1
                ID[50] = 1
                ID[51] = 1
                ID[52] = 1
                ID[53] = 1
                ID[54] = 1
                ID[56] = 1
                ID[57] = 1
                ID[59] = 1
                ID[60] = 1
                ID[62] = 1
                ID[63] = 1
                ID[64] = 1
                ID[65] = 1
                ID[66] = 1
                ID[68] = 1
                ID[69] = 1
                ID[1] = 1
                ID[2] = 1
                ID[3] = 1
            
            if AV_number == 35:
                ID[5] = 1
                ID[7] = 1
                ID[9] = 1
                ID[11] = 1
                ID[13] = 1
                ID[15] = 1
                ID[17] = 1
                ID[19] = 1
                ID[21] = 1
                ID[23] = 1
                ID[25] = 1
                ID[27] = 1
                ID[29] = 1
                ID[31] = 1
                ID[33] = 1
                ID[35] = 1
                ID[37] = 1
                ID[39] = 1
                ID[41] = 1
                ID[43] = 1
                ID[45] = 1
                ID[47] = 1
                ID[49] = 1
                ID[51] = 1
                ID[53] = 1
                ID[55] = 1
                ID[57] = 1
                ID[59] = 1
                ID[61] = 1
                ID[63] = 1
                ID[65] = 1
                ID[67] = 1
                ID[69] = 1
                ID[1] = 1
                ID[3] = 1

            if AV_number == 32:
                ID[5] = 1
                ID[7] = 1
                ID[9] = 1
                ID[11] = 1
                ID[13] = 1
                ID[15] = 1
                ID[17] = 1
                ID[19] = 1
                ID[21] = 1
                ID[23] = 1
                ID[25] = 1
                ID[27] = 1
                ID[29] = 1
                ID[31] = 1
                ID[33] = 1
                ID[35] = 1
                ID[37] = 1
                ID[39] = 1
                ID[41] = 1
                ID[43] = 1
                ID[45] = 1
                ID[47] = 1
                ID[49] = 1
                ID[51] = 1
                ID[53] = 1
                ID[55] = 1
                ID[57] = 1
                ID[59] = 1
                ID[61] = 1
                ID[63] = 1
                ID[65] = 1
                ID[67] = 1

            if AV_number == 28:
                ID[5] = 1
                ID[7] = 1
                ID[10] = 1
                ID[12] = 1
                ID[15] = 1
                ID[17] = 1
                ID[20] = 1
                ID[22] = 1
                ID[25] = 1
                ID[27] = 1
                ID[30] = 1
                ID[32] = 1
                ID[35] = 1
                ID[37] = 1
                ID[40] = 1
                ID[42] = 1
                ID[45] = 1
                ID[47] = 1
                ID[50] = 1
                ID[52] = 1
                ID[55] = 1
                ID[57] = 1
                ID[60] = 1
                ID[62] = 1
                ID[65] = 1
                ID[67] = 1
                ID[0] = 1
                ID[2] = 1

            if AV_number == 25:
                ID[5] = 1
                ID[7] = 1
                ID[10] = 1
                ID[12] = 1
                ID[15] = 1
                ID[17] = 1
                ID[20] = 1
                ID[22] = 1
                ID[25] = 1
                ID[27] = 1
                ID[30] = 1
                ID[32] = 1
                ID[35] = 1
                ID[37] = 1
                ID[40] = 1
                ID[42] = 1
                ID[45] = 1
                ID[47] = 1
                ID[50] = 1
                ID[52] = 1
                ID[55] = 1
                ID[57] = 1
                ID[60] = 1
                ID[62] = 1
                ID[65] = 1

            if AV_number == 21:
                ID[5] = 1
                ID[8] = 1
                ID[11] = 1
                ID[14] = 1
                ID[17] = 1
                ID[20] = 1
                ID[23] = 1
                ID[26] = 1
                ID[29] = 1
                ID[32] = 1
                ID[35] = 1
                ID[38] = 1
                ID[41] = 1
                ID[44] = 1
                ID[47] = 1
                ID[50] = 1
                ID[53] = 1
                ID[56] = 1
                ID[59] = 1
                ID[62] = 1
                ID[65] = 1

            if AV_number == 18:
                ID[5] = 1
                ID[8] = 1
                ID[12] = 1
                ID[15] = 1
                ID[19] = 1
                ID[22] = 1
                ID[26] = 1
                ID[30] = 1
                ID[34] = 1
                ID[37] = 1
                ID[41] = 1
                ID[44] = 1
                ID[48] = 1
                ID[52] = 1
                ID[55] = 1
                ID[59] = 1
                ID[62] = 1
                ID[66] = 1

            if AV_number == 14:
                ID[5] = 1
                ID[10] = 1
                ID[15] = 1
                ID[20] = 1
                ID[25] = 1
                ID[30] = 1
                ID[35] = 1
                ID[40] = 1
                ID[45] = 1
                ID[50] = 1
                ID[55] = 1
                ID[60] = 1
                ID[65] = 1
                ID[0] = 1

            if AV_number == 11:
                ID[5] = 1
                ID[11] = 1
                ID[17] = 1
                ID[25] = 1
                ID[31] = 1
                ID[37] = 1
                ID[43] = 1
                ID[51] = 1
                ID[57] = 1
                ID[63] = 1
                ID[69] = 1
                    
            if AV_number == 7:
                ID[5] = 1
                ID[15] = 1
                ID[25] = 1
                ID[35] = 1
                ID[45] = 1
                ID[55] = 1
                ID[65] = 1

            if AV_number == 4:
                ID[5] = 1
                ID[22] = 1
                ID[38] = 1
                ID[55] = 1

    if N == 45 :
        if platoon_bool:
            if AV_number == 45:
                ID = np.ones([N])

            if AV_number == 32:
                ID[5] = 1
                ID[6] = 1
                ID[7] = 1
                ID[8] = 1
                ID[9] = 1
                ID[10] = 1
                ID[11] = 1
                ID[12] = 1
                ID[13] = 1
                ID[14] = 1
                ID[15] = 1
                ID[16] = 1
                ID[17] = 1
                ID[18] = 1
                ID[19] = 1
                ID[20] = 1
                ID[21] = 1
                ID[22] = 1
                ID[23] = 1
                ID[24] = 1
                ID[25] = 1
                ID[26] = 1
                ID[27] = 1
                ID[28] = 1
                ID[29] = 1
                ID[30] = 1
                ID[31] = 1
                ID[32] = 1
                ID[33] = 1
                ID[34] = 1
                ID[35] = 1
                ID[36] = 1

            if AV_number == 23:
                ID[5] = 1
                ID[6] = 1
                ID[7] = 1
                ID[8] = 1
                ID[9] = 1
                ID[10] = 1
                ID[11] = 1
                ID[12] = 1
                ID[13] = 1
                ID[14] = 1
                ID[15] = 1
                ID[16] = 1
                ID[17] = 1
                ID[18] = 1
                ID[19] = 1
                ID[20] = 1
                ID[21] = 1
                ID[22] = 1
                ID[23] = 1
                ID[24] = 1
                ID[25] = 1
                ID[26] = 1
                ID[27] = 1

            if AV_number == 21:
                ID[5] = 1
                ID[6] = 1
                ID[7] = 1
                ID[8] = 1
                ID[9] = 1
                ID[10] = 1
                ID[11] = 1
                ID[12] = 1
                ID[13] = 1
                ID[14] = 1
                ID[15] = 1
                ID[16] = 1
                ID[17] = 1
                ID[18] = 1
                ID[19] = 1
                ID[20] = 1
                ID[21] = 1
                ID[22] = 1
                ID[23] = 1
                ID[24] = 1
                ID[25] = 1

            if AV_number == 18:
                ID[5] = 1
                ID[6] = 1
                ID[7] = 1
                ID[8] = 1
                ID[9] = 1
                ID[10] = 1
                ID[11] = 1
                ID[12] = 1
                ID[13] = 1
                ID[14] = 1
                ID[15] = 1
                ID[16] = 1
                ID[17] = 1
                ID[18] = 1
                ID[19] = 1
                ID[20] = 1
                ID[21] = 1
                ID[22] = 1

            if AV_number == 16:
                ID[5] = 1
                ID[6] = 1
                ID[7] = 1
                ID[8] = 1
                ID[9] = 1
                ID[10] = 1
                ID[11] = 1
                ID[12] = 1
                ID[13] = 1
                ID[14] = 1
                ID[15] = 1
                ID[16] = 1
                ID[17] = 1
                ID[18] = 1
                ID[19] = 1
                ID[20] = 1

            if AV_number == 14:
                ID[5] = 1
                ID[6] = 1
                ID[7] = 1
                ID[8] = 1
                ID[9] = 1
                ID[10] = 1
                ID[11] = 1
                ID[12] = 1
                ID[13] = 1
                ID[14] = 1
                ID[15] = 1
                ID[16] = 1
                ID[17] = 1
                ID[18] = 1

            if AV_number == 12:
                ID[5] = 1
                ID[6] = 1
                ID[7] = 1
                ID[8] = 1
                ID[9] = 1
                ID[10] = 1
                ID[11] = 1
                ID[12] = 1
                ID[13] = 1
                ID[14] = 1
                ID[15] = 1
                ID[16] = 1

            if AV_number == 9:
                ID[5] = 1
                ID[6] = 1
                ID[7] = 1
                ID[8] = 1
                ID[9] = 1
                ID[10] = 1
                ID[11] = 1
                ID[12] = 1
                ID[13] = 1

            if AV_number == 7:
                ID[5] = 1
                ID[6] = 1
                ID[7] = 1
                ID[8] = 1
                ID[9] = 1
                ID[10] = 1
                ID[11] = 1
                    
            if AV_number == 5:
                ID[5] = 1
                ID[6] = 1
                ID[7] = 1
                ID[8] = 1
                ID[9] = 1

            if AV_number == 3:
                ID[5] = 1
                ID[6] = 1
                ID[7] = 1
        else :
            if AV_number == 45:
                ID = np.ones([N])

            if AV_number == 32:
                ID[5] = 1
                ID[6] = 1
                ID[8] = 1
                ID[9] = 1
                ID[11] = 1
                ID[12] = 1
                ID[14] = 1
                ID[15] = 1
                ID[16] = 1
                ID[17] = 1
                ID[18] = 1
                ID[20] = 1
                ID[21] = 1
                ID[23] = 1
                ID[24] = 1
                ID[26] = 1
                ID[27] = 1
                ID[28] = 1
                ID[29] = 1
                ID[30] = 1
                ID[32] = 1
                ID[33] = 1
                ID[35] = 1
                ID[36] = 1
                ID[38] = 1
                ID[39] = 1
                ID[41] = 1
                ID[42] = 1
                ID[44] = 1
                ID[0] = 1
                ID[2] = 1
                ID[3] = 1

            if AV_number == 23:
                ID[5] = 1
                ID[7] = 1
                ID[9] = 1
                ID[11] = 1
                ID[13] = 1
                ID[15] = 1
                ID[17] = 1
                ID[19] = 1
                ID[21] = 1
                ID[23] = 1
                ID[25] = 1
                ID[27] = 1
                ID[29] = 1
                ID[31] = 1
                ID[33] = 1
                ID[35] = 1
                ID[37] = 1
                ID[39] = 1
                ID[41] = 1
                ID[43] = 1
                ID[0] = 1
                ID[2] = 1
                ID[3] = 1

            if AV_number == 21:
                ID[5] = 1
                ID[7] = 1
                ID[9] = 1
                ID[11] = 1
                ID[13] = 1
                ID[15] = 1
                ID[17] = 1
                ID[20] = 1
                ID[22] = 1
                ID[24] = 1
                ID[26] = 1
                ID[28] = 1
                ID[30] = 1
                ID[33] = 1
                ID[35] = 1
                ID[37] = 1
                ID[39] = 1
                ID[41] = 1
                ID[43] = 1
                ID[0] = 1
                ID[2] = 1

            if AV_number == 18:
                ID[5] = 1
                ID[7] = 1
                ID[10] = 1
                ID[12] = 1
                ID[15] = 1
                ID[17] = 1
                ID[20] = 1
                ID[22] = 1
                ID[25] = 1
                ID[27] = 1
                ID[30] = 1
                ID[32] = 1
                ID[35] = 1
                ID[37] = 1
                ID[40] = 1
                ID[42] = 1
                ID[0] = 1
                ID[2] = 1

            if AV_number == 16:
                ID[5] = 1
                ID[7] = 1
                ID[10] = 1
                ID[12] = 1
                ID[15] = 1
                ID[17] = 1
                ID[20] = 1
                ID[22] = 1
                ID[25] = 1
                ID[27] = 1
                ID[30] = 1
                ID[32] = 1
                ID[35] = 1
                ID[37] = 1
                ID[40] = 1
                ID[42] = 1

            if AV_number == 14:
                ID[5] = 1
                ID[8] = 1
                ID[11] = 1
                ID[14] = 1
                ID[17] = 1
                ID[20] = 1
                ID[23] = 1
                ID[26] = 1
                ID[29] = 1
                ID[32] = 1
                ID[35] = 1
                ID[38] = 1
                ID[41] = 1
                ID[44] = 1

            if AV_number == 12:
                ID[5] = 1
                ID[8] = 1
                ID[12] = 1
                ID[15] = 1
                ID[19] = 1
                ID[22] = 1
                ID[26] = 1
                ID[29] = 1
                ID[33] = 1
                ID[36] = 1
                ID[40] = 1
                ID[43] = 1

            if AV_number == 9:
                ID[5] = 1
                ID[10] = 1
                ID[15] = 1
                ID[20] = 1
                ID[25] = 1
                ID[30] = 1
                ID[35] = 1
                ID[40] = 1
                ID[0] = 1

            if AV_number == 7:
                ID[5] = 1
                ID[11] = 1
                ID[17] = 1
                ID[25] = 1
                ID[31] = 1
                ID[39] = 1
                ID[0] = 1
                    
            if AV_number == 5:
                ID[5] = 1
                ID[14] = 1
                ID[23] = 1
                ID[32] = 1
                ID[41] = 1

            if AV_number == 3:
                ID[5] = 1
                ID[20] = 1
                ID[35] = 1

    if N == 20 :
        if platoon_bool:
            if AV_number == 20:
                ID = np.ones([20])
            
            if AV_number == 15:
                ID[5] = 1
                ID[6] = 1
                ID[7] = 1
                ID[8] = 1
                ID[9] = 1
                ID[10] = 1
                ID[11] = 1
                ID[12] = 1
                ID[13] = 1
                ID[14] = 1
                ID[15] = 1
                ID[16] = 1
                ID[17] = 1
                ID[18] = 1
                ID[19] = 1
            
            if AV_number == 10:
                ID[5] = 1
                ID[6] = 1
                ID[7] = 1
                ID[8] = 1
                ID[9] = 1
                ID[10] = 1
                ID[11] = 1
                ID[12] = 1
                ID[13] = 1
                ID[14] = 1

            if AV_number == 9:
                ID[5] = 1
                ID[6] = 1
                ID[7] = 1
                ID[8] = 1
                ID[9] = 1
                ID[10] = 1
                ID[11] = 1
                ID[12] = 1
                ID[13] = 1
            
            if AV_number == 8:
                ID[5] = 1
                ID[6] = 1
                ID[7] = 1
                ID[8] = 1
                ID[9] = 1
                ID[10] = 1
                ID[11] = 1
                ID[12] = 1
            
            if AV_number == 7:
                ID[5] = 1
                ID[6] = 1
                ID[7] = 1
                ID[8] = 1
                ID[9] = 1
                ID[10] = 1
                ID[11] = 1
                
            if AV_number == 6:
                ID[5] = 1
                ID[6] = 1
                ID[7] = 1
                ID[8] = 1
                ID[9] = 1
                ID[10] = 1

            if AV_number == 5:
                ID[5] = 1
                ID[6] = 1
                ID[7] = 1
                ID[8] = 1
                ID[9] = 1

            if AV_number == 4:
                ID[5] = 1
                ID[6] = 1
                ID[7] = 1
                ID[8] = 1

            if AV_number == 3:
                ID[5] = 1
                ID[6] = 1
                ID[7] = 1
                    
            if AV_number == 2:
                ID[5] = 1
                ID[6] = 1

            if AV_number == 1:
                ID[5] = 1
        
        else :
            if AV_number == 20:
                ID = np.ones([20])
            
            if AV_number == 15:
                ID[5] = 1
                ID[7] = 1
                ID[8] = 1
                ID[10] = 1
                ID[11] = 1
                ID[12] = 1
                ID[13] = 1
                ID[14] = 1
                ID[16] = 1
                ID[17] = 1
                ID[18] = 1
                ID[19] = 1
                ID[0] = 1
                ID[2] = 1
                ID[3] = 1
            
            if AV_number == 10:
                ID[5] = 1
                ID[7] = 1
                ID[9] = 1
                ID[11] = 1
                ID[13] = 1
                ID[15] = 1
                ID[17] = 1
                ID[19] = 1
                ID[1] = 1
                ID[3] = 1

            if AV_number == 9:
                ID[5] = 1
                ID[7] = 1
                ID[10] = 1
                ID[12] = 1
                ID[15] = 1
                ID[17] = 1
                ID[19] = 1
                ID[1] = 1
                ID[3] = 1
            
            if AV_number == 8:
                ID[5] = 1
                ID[7] = 1
                ID[9] = 1
                ID[11] = 1
                ID[13] = 1
                ID[15] = 1
                ID[17] = 1
                ID[19] = 1
            
            if AV_number == 7:
                ID[5] = 1
                ID[7] = 1
                ID[10] = 1
                ID[12] = 1
                ID[15] = 1
                ID[17] = 1
                ID[0] = 1
                
            if AV_number == 6:
                ID[5] = 1
                ID[8] = 1
                ID[11] = 1
                ID[14] = 1
                ID[17] = 1
                ID[0] = 1

            if AV_number == 5:
                ID[5] = 1
                ID[9] = 1
                ID[13] = 1
                ID[17] = 1
                ID[1] = 1

            if AV_number == 4:
                ID[5] = 1
                ID[10] = 1
                ID[15] = 1
                ID[0] = 1

            if AV_number == 3:
                ID[5] = 1
                ID[12] = 1
                ID[18] = 1
                    
            if AV_number == 2:
                ID[5] = 1
                ID[15] = 1

            if AV_number == 1:
                ID[5] = 1
            
    return ID
