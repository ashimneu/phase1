import os
from Tkinter import *
from subprocess import call
from subprocess import Popen
import subprocess



def f1():
    global proc
    #os.system("rosrun phase1 Quad3D.py")
    proc = subprocess.Popen(["rosrun", "phase1", "Quad3D.py"])
def f2():
    #os.system("rosnode kill -a")
    global proc
    proc.kill()

root = Tk()
root.title("root")
root.geometry("500x500")

button1 = Button(root,text="Run", bg="black", fg="white",  width=25, padx = 20, command=f1)
button1.pack()
button2 = Button(root,text="Stop", bg="black", fg="white",  width=25, padx = 20, command=f2)
button2.pack()
proc = None
root.mainloop()
