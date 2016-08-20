class myClass():
    def __init__(self):
        self.mydata = []
        self.myvalue = 1.4 
    def print_values(self):
        for item in self.mydata:
            print item
    def multi(self, x):
        new_value = self.myvalue * x
        self.myvalue  = new_value
        return new_value


A = myClass()
B = myClass()

A.multi(10)
print(A.myvalue)
print(B.myvalue)