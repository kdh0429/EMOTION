import serial        

class Arduino:    
    def __init__(self, port='COM3', baudrate=57600):
        self.port = port
        self.baudrate = baudrate
        self.arduino = serial.Serial(self.port, baudrate=self.baudrate)
    
    def change_expression(self, action_num):
        self.arduino.write(action_num)
        print("[Arduino]Completed")      
        

if __name__ == "__main__":
    A = Arduino()
    while True:
        c = input()
        c = c.encode('utf-8')
        A.change_expression(c)
    