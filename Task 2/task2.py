from numpy import outer
from regex import D


class Enemy():
    def __init__(self,name,power_points):
        self.name = name
        self.rem_points = power_points

    def attack(self):
        print(f"Attack on {self.name} started")
        while True:
            myobj = TroopSelection(self.rem_points)
            (rem_points,present_state,output) = myobj.take_input()
            print(f"Present State: {present_state}")
            print(f"Output: {output}")
            print("\n")
            if(rem_points<=0):
                print(f"{self.name} destroyed")
                break
            else:
                self.rem_points = rem_points

class TroopSelection():
    def __init__(self,remaining_points):
        self.troop = ''
        self.rem_points = remaining_points
    
    def take_input(self):
        self.troop = input("Which troop you want to send : ")
        (rem_points,present_state,output) = self.execution()
        return (rem_points,present_state,output)

    def execution(self):
        if(self.troop == 'B'):
            print("Attacked by Barbarian")
            self.obj = BState(self.rem_points)
            (rem_points,present_state,output) = self.obj.process()
        elif(self.troop == 'G'):
            print("Attacked by Giant")
            self.obj = GState(self.rem_points)
            (rem_points,present_state,output) = self.obj.process()           
        elif(self.troop == 'P'):
            print("Attacked by P.E.K.K.A")
            self.obj = PState(self.rem_points)
            (rem_points,present_state,output) = self.obj.process()

        return (rem_points,present_state,output)

class BState():
    def __init__(self,remaining_points):
        self.power_points = 5
        self.rem_points = remaining_points
        self.output = 0
        self.present_state = 0

    def process(self):
        self.rem_points = self.rem_points - self.power_points
        if(self.rem_points>0):
            self.output = 0
        else:
            self.output = -self.rem_points
        (rem_points,present_state,output) = self.output_process()
        return (rem_points,present_state,output)

    def output_process(self):
        if(self.rem_points > 0):
            self.present_state = format(int(self.rem_points/5),'b')
            self.output = '000'
            if(len(self.present_state)==1):
                self.present_state = '00' + str(self.present_state)
            elif(len(self.present_state)==2):
                self.present_state = '0' + str(self.present_state)
            else:
                self.present_state = str(self.present_state)
        else:
            self.present_state = '000'
            self.output = format(int(self.output/5),'b')
            if(len(self.output)==1):
                self.output = '10' + str(self.output)
            else:
                self.output = '1' + str(self.output)

        return (self.rem_points,self.present_state,self.output)

class GState():
    def __init__(self,remaining_points):
        self.power_points = 10
        self.rem_points = remaining_points
        self.output = 0
        self.present_state = 0

    def process(self):
        self.rem_points = self.rem_points - self.power_points
        if(self.rem_points>0):
            self.output = 0
        else:
            self.output = -self.rem_points
        (rem_points,present_state,output) = self.output_process()
        return (rem_points,present_state,output)

    def output_process(self):
        if(self.rem_points > 0):
            self.present_state = format(int(self.rem_points/5),'b')
            self.output = '000'
            if(len(self.present_state)==1):
                self.present_state = '00' + str(self.present_state)
            elif(len(self.present_state)==2):
                self.present_state = '0' + str(self.present_state)
            else:
                self.present_state = str(self.present_state)
        else:
            self.present_state = '000'
            self.output = format(int(self.output/5),'b')
            if(len(self.output)==1):
                self.output = '10' + str(self.output)
            else:
                self.output = '1' + str(self.output)
        
        return (self.rem_points,self.present_state,self.output)

class PState():
    def __init__(self,remaining_points):
        self.power_points = 15
        self.rem_points = remaining_points
        self.output = 0
        self.present_state = 0

    def process(self):
        self.rem_points = self.rem_points - self.power_points
        if(self.rem_points>0):
            self.output = 0
        else:
            self.output = -self.rem_points
        (rem_points,present_state,output) = self.output_process()
        return (rem_points,present_state,output)

    def output_process(self):
        if(self.rem_points > 0):
            self.present_state = format(int(self.rem_points/5),'b')
            self.output = '000'
            if(len(self.present_state)==1):
                self.present_state = '00' + str(self.present_state)
            elif(len(self.present_state)==2):
                self.present_state = '0' + str(self.present_state)
            else:
                self.present_state = str(self.present_state)
        else:
            self.present_state = '000'
            self.output = format(int(self.output/5),'b')
            if(len(self.output)==1):
                self.output = '10' + str(self.output)
            else:
                self.output = '1' + str(self.output)
        return (self.rem_points,self.present_state,self.output)


        

wizard_tower = Enemy("Wizard Tower",35)
wizard_tower.attack()



