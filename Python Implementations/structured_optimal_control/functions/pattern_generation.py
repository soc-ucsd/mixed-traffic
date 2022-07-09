import numpy as np

def pattern_generation(N, AV_number, CR):

    if AV_number == 1:
        K_Pattern = np.zeros((1, 2 * N))

        for i in range(1, CR + 1):
            K_Pattern[0, 2 * i - 2: 2 * i] = [1, 1]

        for i in range(N-CR, N):
            K_Pattern[0, 2 * i - 2: 2 * i] = [1, 1]
            K_Pattern[0, 2 * N - 2: 2 * N] = [1, 1]

    if AV_number == 2:
        if CR >= N - np.floor(N / 2):
            K_Pattern = np.ones((2, 2 * N))
        else:
            K_Pattern = np.zeros((2, 2 * N))
            # row 1
            for i in range(np.floor(N / 2) - CR, np.floor(N / 2) + CR + 1):
                K_Pattern[0, 2 * i - 2 : 2 * i] = [1, 1]

            # row 2
            for i in range(1, CR + 1):
                K_Pattern[1, 2 * i - 2 : 2 * i] = [1, 1]

            for i in range(N - CR, N):
                K_Pattern[1, 2 * i - 2 : 2 * i] = [1, 1]

            K_Pattern[1, 2 * N - 2 : 2 * N] = [1, 1]
            
    return K_Pattern