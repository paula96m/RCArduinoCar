




def r_to_c(i):
    C = 80
    MIN = 50
    MAX = 130
    RC = 50
    RMIN = 0
    RMAX = 130
    if i <= 50:
        C -= MIN
        RC -= RMIN
        x = (C / RC) * i + MIN
    else:
        MAX -= C
        RMAX -= RC
        x = (MAX / RMAX) * i + C - RC
    return x

for i in range(101):
    print("{} -> {}".format(i, r_to_c(i)))

