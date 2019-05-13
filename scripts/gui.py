from tkinter import *

class Main:

    def __init__(self, master):
        # Master Setup
        self.master = master
        self.master.title("Qudrotor Project: Phase 1")
        self.master.bind("<Escape>",quit)
        self.master.bind("1",self.launch_quad2d_window)
        self.master.bind("2",self.launch_quad3d_window)

        # Frame 1 setup
        self.frame1 = Frame(self.master, width=300, height=75)
        self.frame1.grid_propagate(False)
        self.frame1.grid(row=0, column=0, sticky="nsew")
        self.frame1.grid_rowconfigure(0, weight=1)
        self.frame1.grid_columnconfigure(0, weight=1)

        # Frame 2 Setup
        self.frame2 = Frame(self.master, width=300, height=75)
        self.frame2.grid_propagate(False)
        self.frame2.grid(row=1, column=0, sticky="nsew")
        self.frame2.grid_rowconfigure(0, weight=1)
        self.frame2.grid_columnconfigure(0, weight=1)

        # Layout Setup
        self.b1 = Button(self.frame1, text="Launch 2D Quadrotor", command=self.launch_quad2d_window)
        self.b1.grid(sticky="we")

        self.b2 = Button(self.frame2, text="Launch 3D Quadrotor", command=self.launch_quad3d_window)
        self.b2.grid(stick="we")



    def launch_quad2d_window(self,*args):
        self.newWindow = Toplevel(self.master)
        childWindow1 = quad2d_class(self.newWindow)
        self.master.withdraw()

    def launch_quad3d_window(self,*args):
        self.newWindow = Toplevel(self.master)
        childWindow2 = quad3d_class(self.newWindow)
        self.master.withdraw()


class quad2d_class():

    def __init__(self, master):
        # Master Setup
        self.master = master
        self.master.title("2D Quadrotor Simulator")
        #self.master.overrideredirect(1) # Disables title bar
        self.master.geometry("800x500")
        self.master.bind("<Escape>",quit)
        self.master.bind("`", self.return2Main)

       # Frame Setup
        self.frame1 = Frame(self.master,bg="White", width=8020, height=800, pady = 80, padx = 80)
        self.frame1.grid(row=0, column=0)
        self.frame1.grid_propagate(False)

        # Layout Setup
        self.btnBack = Button(self.master, text="Back", command=self.return2Main)
        self.btnBack.place(x=0,y=0)
        self.btnQuit = Button(self.master, text="Quit", command=quit)
        self.btnQuit.place(x=743,y=0)

        self.lbl_suggestion0 = Label(self.frame1, text="Input Format:", bg="white", fg="black", font=("Courier", 18), width=17)
        self.lbl_suggestion0.place(x=500,y=15)

        self.lbl_initialpose = Label(self.frame1, text="Initial Pose: ", fg="black", font=("Courier", 18), width=17)
        self.lbl_initialpose.place(x=30,y=50)

        self.lbl_tkfheight = Label(self.frame1, text="Takeoff Height: ", fg="black", font=("Courier", 18), width=17)
        self.lbl_tkfheight.place(x=30,y=85)

        self.lbl_hoverpose = Label(self.frame1, text="Hover Pose: ", fg="black", font=("Courier", 18), width=17)
        self.lbl_hoverpose.place(x=30,y=120)

        self.lbl_hovertime = Label(self.frame1, text="Hover Time: ", fg="black", font=("Courier", 18), width=17)
        self.lbl_hovertime.place(x =30, y=155)

        self.lbl_suggestion1 = Label(self.frame1, text="y,z,\u03A6", bg="white", fg="black", font=("Courier", 18), width=17)
        self.lbl_suggestion1.place(x=500,y=50)

        self.lbl_suggestion2 = Label(self.frame1, text="z", bg="white", fg="black", font=("Courier", 18), width=17)
        self.lbl_suggestion2.place(x=500,y=85)

        self.lbl_suggestion3 = Label(self.frame1, text="y,z", bg="white", fg="black", font=("Courier", 18), width=17)
        self.lbl_suggestion3.place(x=500,y=120)

        self.lbl_suggestion4 = Label(self.frame1, text="time", bg="white", fg="black", font=("Courier", 18), width=17)
        self.lbl_suggestion4.place(x=500,y=155)


        self.entry_initialpose = Entry(self.frame1, fg="black", font=("Courier", 18))
        self.entry_initialpose.place(x =250, y=50)

        self.entry_tkfheight = Entry(self.frame1, fg="black", font=("Courier", 18))
        self.entry_tkfheight.place(x =250, y=85)

        self.entry_hoverpose = Entry(self.frame1, fg="black", font=("Courier", 18))
        self.entry_hoverpose.place(x =250, y=120)

        self.entry_hovertime = Entry(self.frame1, fg="black", font=("Courier", 18))
        self.entry_hovertime.place(x =250, y=155)

        self.btnOk = Button(self.frame1, text="Simulate", command=self.extractInputs, height=5, width=8)
        self.btnOk.place(x=600,y=300)

        # User Inputs for Quadrotor Waypoints
        self.initialpose = 0
        self.tkfheight = 0
        self.hoverpose = 0
        self.hovertime = 0

    def extractInputs(self,*args):
        tmp_initialpose = self.entry_initialpose.get().strip()
        tmp_tkfheight = self.entry_tkfheight.get().strip()
        tmp_hoverpose = self.entry_hoverpose.get().strip()
        tmp_hovertime = self.entry_hovertime.get().strip()

        #print('len initial pose: ', len(tmp_initialpose))
        #print('len tkfheight: ', len(tmp_tkfheight))
        #print('len hoverpose: ', len(tmp_hoverpose))
        #print('len hovertime: ', len(tmp_hovertime))

        #error_in_input = check4error(tmp_initialpose,tmp_tkfheight,tmp_hoverpose,tmp_hovertime,3)
        #error_in_input = False
        try:
            self.initialpose = [float(x) for x in tmp_initialpose.split(',')]
            self.tkfheight = float(tmp_tkfheight)
            self.hoverpose = [float(x) for x in tmp_hoverpose.split(',')]
            self.hovertime = float(tmp_hovertime)

        except:
            print('One or more inputs are incorrect. Please follow the specified input formats.')

    def return2Main(self,*args):
        print('Exiting 2D Quadrotor Window.')
        root.deiconify()
        self.master.withdraw()


class quad3d_class():

    def __init__(self, master):
        # Master Setup
        self.master = master
        self.master.title("3D Quadrotor Simulator")
        #self.master.overrideredirect(1) # Disables title bar
        self.master.geometry("800x500")
        self.master.bind("<Escape>",quit)
        self.master.bind("`", self.return2Main)

        # Frame Setup
        self.frame1 = Frame(self.master,bg="White", width=8020, height=800, pady = 80, padx = 80)
        self.frame1.grid(row=0, column=0)
        self.frame1.grid_propagate(False)

        # Layout Setup
        self.btnBack = Button(self.master, text="Back", command=self.return2Main)
        self.btnBack.place(x=0,y=0)
        self.btnQuit = Button(self.master, text="Quit", command=quit)
        self.btnQuit.place(x=743,y=0)

        self.lbl_suggestion0 = Label(self.frame1, text="Input Format:", bg="white", fg="black", font=("Courier", 18), width=17)
        self.lbl_suggestion0.place(x=500,y=15)

        self.lbl_initialpose = Label(self.frame1, text="Initial Pose: ", fg="black", font=("Courier", 18), width=17)
        self.lbl_initialpose.place(x=30,y=50)

        self.lbl_tkfheight = Label(self.frame1, text="Takeoff Height: ", fg="black", font=("Courier", 18), width=17)
        self.lbl_tkfheight.place(x=30,y=85)

        self.lbl_hoverpose = Label(self.frame1, text="Hover Pose: ", fg="black", font=("Courier", 18), width=17)
        self.lbl_hoverpose.place(x=30,y=120)

        self.lbl_hovertime = Label(self.frame1, text="Hover Time: ", fg="black", font=("Courier", 18), width=17)
        self.lbl_hovertime.place(x =30, y=155)

        self.lbl_suggestion1 = Label(self.frame1, text="x,y,z,\u03A6,\u03F4,\u03A8", bg="white", fg="black", font=("Courier", 18), width=17)
        self.lbl_suggestion1.place(x=500,y=50)

        self.lbl_suggestion2 = Label(self.frame1, text="z", bg="white", fg="black", font=("Courier", 18), width=17)
        self.lbl_suggestion2.place(x=500,y=85)

        self.lbl_suggestion3 = Label(self.frame1, text="x,y,z", bg="white", fg="black", font=("Courier", 18), width=17)
        self.lbl_suggestion3.place(x=500,y=120)

        self.lbl_suggestion4 = Label(self.frame1, text="time", bg="white", fg="black", font=("Courier", 18), width=17)
        self.lbl_suggestion4.place(x=500,y=155)


        self.entry_initialpose = Entry(self.frame1, fg="black", font=("Courier", 18))
        self.entry_initialpose.place(x =250, y=50)

        self.entry_tkfheight = Entry(self.frame1, fg="black", font=("Courier", 18))
        self.entry_tkfheight.place(x =250, y=85)

        self.entry_hoverpose = Entry(self.frame1, fg="black", font=("Courier", 18))
        self.entry_hoverpose.place(x =250, y=120)

        self.entry_hovertime = Entry(self.frame1, fg="black", font=("Courier", 18))
        self.entry_hovertime.place(x =250, y=155)

        self.btnOk = Button(self.frame1, text="Simulate", command=self.extractInputs, height=5, width=8)
        self.btnOk.place(x=600,y=300)

        # User Inputs for Quadrotor Waypoints
        self.initialpose = 0
        self.tkfheight = 0
        self.hoverpose = 0
        self.hovertime = 0

    def extractInputs(self,*args):
        tmp_initialpose = self.entry_initialpose.get().strip()
        tmp_tkfheight = self.entry_tkfheight.get().strip()
        tmp_hoverpose = self.entry_hoverpose.get().strip()
        tmp_hovertime = self.entry_hovertime.get().strip()

        #print('len initial pose: ', len(tmp_initialpose))
        #print('len tkfheight: ', len(tmp_tkfheight))
        #print('len hoverpose: ', len(tmp_hoverpose))
        #print('len hovertime: ', len(tmp_hovertime))

        #error_in_input = check4error(tmp_initialpose,tmp_tkfheight,tmp_hoverpose,tmp_hovertime,3)
        #error_in_input = False
        try:
            self.initialpose = [float(x) for x in tmp_initialpose.split(',')]
            self.tkfheight = float(tmp_tkfheight)
            self.hoverpose = [float(x) for x in tmp_hoverpose.split(',')]
            self.hovertime = float(tmp_hovertime)

        except:
            print('One or more inputs are incorrect. Please follow the specified input formats.')


    def return2Main(self,*args):
        print('Exiting 3D Quadrotor Window.')
        root.deiconify()
        self.master.withdraw()
'''
def check4error(tmp_initialpose, tmp_tkfheight, tmp_hoverpose, tmp_hovertime, tmp_quadrotor_type):
    there_is_error = True


    if tmp_quadrotor_type == 2:
        if len(initialpose)==3 and len(tkfheight)==1 and len(hoverpose)==2 and len(hovertime)==1 :
            there_is_error = False
        else:
            there_is_error = False
    elif tmp_quadrotor_type == 3:
        if len(initialpose)==6 and len(tkfheight)==1 and len(hoverpose)==3 and len(hovertime)==1 :
            there_is_error = False
        else:
            there_is_error = True
    return there_is_error'''



if __name__ == '__main__':
    root = Tk()
    Window1 = Main(root)
    Window1.master.resizable(width=False, height=False)

    root.mainloop()