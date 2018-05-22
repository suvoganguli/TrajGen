def problemMaxIterData(N, ns, no, V0, sf_T):

    # Units
    mph2fps = 4.4 / 3

    if abs(V0 - 5*mph2fps) <= 10**(-3):
        if no == 0:
            if N == 4:
                mpciterations = 35/sf_T # 35
            elif N == 6:
                mpciterations = 33/sf_T # 33
            elif N == 8:
                mpciterations = 31/sf_T # 31
            elif N == 10:
                mpciterations = 29/sf_T  # 29


        elif no == 1:
            if N == 4:
                if ns == 4:
                    mpciterations = 36/sf_T  # 36
                elif ns == 6:
                    mpciterations = 18/sf_T # 18
            elif N == 6:
                if ns == 4:
                    mpciterations = 34/sf_T # 34
                elif ns == 6:
                    mpciterations = 34/sf_T  # 38 - check mpciterations
            elif N == 8:
                if ns == 4:
                    mpciterations = 32/sf_T  # 32
                elif ns == 6:
                    mpciterations = 36/sf_T  #24 - check mpciterations
            elif N == 10:
                if ns == 4:
                    mpciterations = 30/sf_T  # ?
                elif ns == 6:
                    mpciterations = 30/sf_T  # ?

        elif no == 2:
            if N == 4:
                if ns == 4:
                    mpciterations = 41/sf_T  # 41
                elif ns == 6:
                    mpciterations = 14/sf_T  # 14 = wider dy with V terminal constraint, unstable
            elif N == 6:
                if ns == 4:
                    mpciterations = 38/sf_T  # 38
                elif ns == 6:
                    mpciterations = 36/sf_T  # 20 = wider dy with V terminal constraint, stops
            elif N == 8:
                if ns == 4:
                    mpciterations = 36/sf_T  # 36
                elif ns == 6:
                    mpciterations = 34/sf_T  # 24 = wider dy with V terminal constraint,
                                        # unstable at 2nd turn "No solution found in runningCons". Why?
            elif N == 10:
                if ns == 4:
                    mpciterations = 14/sf_T  # 30 (for total run)
                elif ns == 6:
                    mpciterations = 12/sf_T  # 30 (for total run)

            elif N == 9:
                if ns == 4:
                    mpciterations = 28/sf_T  # 30 (for total run)
                elif ns == 6:
                    mpciterations = 10/sf_T  # 30 (for total run)

    elif abs(V0 - 10*mph2fps) <= 10**(-3):

        if no == 0:
            if N == 4:
                mpciterations = 35/sf_T # 35
            elif N == 6:
                mpciterations = 33/sf_T # 33
            elif N == 8:
                mpciterations = 31/sf_T # 31
            elif N == 10:
                mpciterations = 29/sf_T  # 29

        if no == 1:
            if N == 4:
                if ns == 4:
                    mpciterations = 60/sf_T  # 36
            if N == 6:
                if ns == 4:
                    mpciterations = 60/sf_T  # 34
            if N == 8:
                if ns == 4:
                    mpciterations = 45/sf_T  # 32
                elif ns == 6:
                    mpciterations = 32/sf_T


        elif no == 2:
            if N == 4:
                if ns == 4:
                    mpciterations = 18/sf_T  # 18
                elif ns == 6:
                    mpciterations = 14/sf_T  # 14 = wider dy with V terminal constraint, unstable
            if N == 6:
                if ns == 4:
                    mpciterations = 38/sf_T  # 38
                elif ns == 6:
                    mpciterations = 36/sf_T  # 20 = wider dy with V terminal constraint, stops
            if N == 8:
                if ns == 4:
                    mpciterations = 80 / sf_T  # 32
                elif ns == 6:
                    mpciterations = 32 / sf_T

        elif no == 5:
            if N == 8:
                if ns == 4:
                    mpciterations = 80 / sf_T  # 32

        elif no == 7:
            if N == 8:
                if ns == 4:
                    mpciterations = 80 / sf_T  # 32

    elif abs(V0 - 15*mph2fps) <= 10**(-3):

        if no == 2:
            if N == 4:
                if ns == 4:
                    mpciterations = 12/sf_T  # 41
                elif ns == 6:
                    mpciterations = 14/sf_T  # 14 = wider dy with V terminal constraint, unstable
            elif N == 6:
                if ns == 4:
                    mpciterations = 38/sf_T  # 38
                elif ns == 6:
                    mpciterations = 36/sf_T  # 20 = wider dy with V terminal constraint, stops

    elif abs(V0 - 30*mph2fps) <= 10**(-3):

        if no == 1:
            if N == 4:
                if ns == 4:
                    mpciterations = 36/sf_T  # 36


    mpciterations = int(mpciterations)

    return mpciterations