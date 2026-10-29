class Angles:
    def __init__(self, S1=0.0, S2=0.0, S3=0.0, S4=0.0):
        self.Change(S1, S2, S3, S4)

    def Change(self, S1, S2, S3, S4):
        self.S1 = S1
        self.S2 = S2
        self.S3 = S3
        self.S4 = S4


class Coordinates:
    def __init__(self, X=0.0, Y=0.0, Z=0.0, A=0.0):
        self.Change(X, Y, Z, A)

    def Change(self, X, Y, Z, A):
        self.X = X
        self.Y = Y
        self.Z = Z
        self.A = A
