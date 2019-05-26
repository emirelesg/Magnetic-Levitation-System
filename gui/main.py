"""
    GUI interface for controlling the Magnetic Levitation System.
    09.05.2019
"""

from controller import Controller
from time import time, sleep
from plot import Plot
import tkinter as tk
import matplotlib

class App:
    """
        Main Tkinter GUI.
    """

    # Window constants.
    WINDOW_TITLE = 'Magnetic Levitation Control'            # Title for the window.
    LOOP_DT = 20                                            # Update rate of the GUI in miliseconds.
    PADY = 10                                               # Y padding for all elements added to the gui.
    PADX = 10                                               # X padding for all elements added to the gui.
    PADY_BOTTOM = 10                                        # Bottom padding for slider labels.

    # Serial communications constants.
    SERIAL_PORT = 'COM12'                                   # Port where the controller is connected.
    BAUD_RATE = 1000000

    # PWM constants. PID_MAX, TIVA_CLOCK, and PWM_PRESCALER must match the values programmed on the Tiva.
    PID_MAX = 255                                           # Maximum value that the PID can reach. Must agree with the value programmed.
    TIVA_CLOCK = 80000000                                   # Clock of the Tiva in Hz. Used to calculate the PWM clock.
    PWM_PRESCALER = 32                                      # Prescaler for the PWM clock. 
    PWM_CLOCK = TIVA_CLOCK / PWM_PRESCALER                  # Actual clock of the PWM module.

    # Slider range constants.
    SET_POINT_RANGE = (0, 10)                               # Available range for the set point in milimeters.
    PWM_RANGE = (PWM_CLOCK / 60000, PWM_CLOCK / 300)        # Available range for the coil PWM in Hz. 
    KP_RANGE = (5, 15)                                     
    KI_RANGE = (0, 10)
    KD_RANGE = (-0.1, 0.1)

    # Define the minimum step that the slider changes. If the range is large enough and the step small, then the step becomes invalid.
    KP_STEP = 0.1
    KI_STEP = 0.001
    KD_STEP = 0.001
    SET_POINT_STEP = 0.05

    # Set the initial values for the controller.
    INITIAL_KP = 11.7
    INITIAL_KI = 1
    INITIAL_KD = -0.021
    INITIAL_SET_POINT = 6.5
    INITIAL_PWM_FREQUENCY = 7150

    def __init__(self, root):
        """
            Create controls, set their position, and initial values.
        """

        # Variables.
        self.closing = False
        self.controller = Controller(self.SERIAL_PORT, self.BAUD_RATE)

        # Create tKinter window, set title and resize properties.
        self.root = root
        self.root.wm_title(self.WINDOW_TITLE)
        self.root.resizable(False, False)

        # Watch for close and key press events.
        self.root.protocol('WM_DELETE_WINDOW', self.onClosing)
        self.root.bind("<Key>", self.keyPressed)

        # Create a frame on row 0 col 0. This frame will contain all of the controls.
        self.controls = tk.Frame()
        self.controls.grid(in_=self.root, row=0, column=0, sticky=tk.NSEW, pady=self.PADY, padx=self.PADX)

        # Create a frame on row 0 col 1. This frame conatins all of the plots.
        self.plots = tk.Frame()
        self.plots.grid(in_=self.root, row=0, column=1, sticky=tk.NSEW, pady=self.PADY, padx=self.PADX)

        # Create all controls and labels. Here are the slider properties managed.
        self.ctrlSetPoint = tk.Scale(self.root, from_=self.SET_POINT_RANGE[0], to=self.SET_POINT_RANGE[1], resolution=self.SET_POINT_STEP, digits=4, length=200, orient=tk.HORIZONTAL, command=self.update)
        self.ctrlProportional = tk.Scale(self.root, from_=self.KP_RANGE[0], to=self.KP_RANGE[1], resolution=self.KP_STEP, digits=3, length=200, orient=tk.HORIZONTAL, command=self.update)
        self.ctrlIntegral = tk.Scale(self.root, from_=self.KI_RANGE[0], to=self.KI_RANGE[1], resolution=self.KI_STEP, digits=4, length=200, orient=tk.HORIZONTAL, command=self.update)
        self.ctrlDerivative = tk.Scale(self.root, from_=self.KD_RANGE[0], to=self.KD_RANGE[1], resolution=self.KD_STEP, digits=4, length=200, orient=tk.HORIZONTAL, command=self.update)
        self.ctrlPWMFrequency = tk.Scale(self.root, from_=self.PWM_RANGE[0], to=self.PWM_RANGE[1], resolution=0, digits=3, length=200, orient=tk.HORIZONTAL, command=self.update)
        self.labelSetPoint = tk.Label(self.root, text='Set Point [mm]')
        self.labelProportional = tk.Label(self.root, text='Proportional Control')
        self.labelIntegral = tk.Label(self.root, text='Integral Control')
        self.labelDerivative = tk.Label(self.root, text='Derivative Control')
        self.labelPWMFrequency = tk.Label(self.root, text='PWM Frequency (Hz)')
        self.labelReset = tk.Label(self.root, text='Press (r) to reset the controller.')
        self.labelIncreaseSetPoint = tk.Label(self.root, text='Press (w) to increase the set point.')
        self.labelDecreaseSetPoint = tk.Label(self.root, text='Press (s) to decrease the set point.')
        self.labelQuit = tk.Label(self.root, text='Press (q) to quit.')

        # Place controls and labels inside of the control frame.
        self.labelSetPoint.grid(in_=self.controls, row=0, column=0, sticky=tk.W)
        self.ctrlSetPoint.grid(in_=self.controls, row=1, column=0, pady=self.PADY_BOTTOM)
        self.labelProportional.grid(in_=self.controls, row=2, column=0, sticky=tk.W)
        self.ctrlProportional.grid(in_=self.controls, row=3, column=0, pady=self.PADY_BOTTOM)
        self.labelIntegral.grid(in_=self.controls, row=4, column=0, sticky=tk.W)
        self.ctrlIntegral.grid(in_=self.controls, row=5, column=0, pady=self.PADY_BOTTOM)
        self.labelDerivative.grid(in_=self.controls, row=6, column=0, sticky=tk.W)
        self.ctrlDerivative.grid(in_=self.controls, row=7, column=0, pady=self.PADY_BOTTOM)
        self.labelPWMFrequency.grid(in_=self.controls, row=8, column=0, sticky=tk.W)
        self.ctrlPWMFrequency.grid(in_=self.controls, row=9, column=0, pady=self.PADY_BOTTOM)
        self.labelReset.grid(in_=self.controls, row=10, column=0, pady=self.PADY_BOTTOM)
        self.labelIncreaseSetPoint.grid(in_=self.controls, row=11, column=0, pady=self.PADY_BOTTOM)
        self.labelDecreaseSetPoint.grid(in_=self.controls, row=12, column=0, pady=self.PADY_BOTTOM)
        self.labelQuit.grid(in_=self.controls, row=13, column=0, pady=self.PADY_BOTTOM)

        # Create plots by using the Plot class. Time range is 1 second for all plots.
        self.distancePlot = Plot(self.root, frame=self.plots, row=0, column=0, title='Sensor Reading', xlabel='Time [s]', ylabel='Distance [mm]', xlim=(0, 1), ylim=self.SET_POINT_RANGE, scroll_gap=0)
        self.errorPlot = Plot(self.root, frame=self.plots, row=0, column=1, title='Error', xlabel='Time [s]', ylabel='Error', xlim=(0, 1), ylim=(-self.SET_POINT_RANGE[1], self.SET_POINT_RANGE[1]), scroll_gap=0.1)
        self.dutyCyclePlot = Plot(self.root, frame=self.plots, row=0, column=2, title='PID Output / DutyCycle', xlabel='Time [s]', ylabel='PID', xlim=(0, 1), ylim=(0, self.PID_MAX * 1.2), scroll_gap=0.1)
        self.kpPlot = Plot(self.root, frame=self.plots, row=1, column=0, title='Proportional', xlabel='Time [s]', ylabel='Kp', xlim=(0, 1), ylim=(-self.PID_MAX, self.PID_MAX), scroll_gap=0.1)
        self.kiPlot = Plot(self.root, frame=self.plots, row=1, column=1, title='Integral', xlabel='Time [s]', ylabel='Kd', xlim=(0, 1), ylim=(-self.PID_MAX, self.PID_MAX), scroll_gap=0.1)
        self.kdPlot = Plot(self.root, frame=self.plots, row=1, column=2, title='Derivative', xlabel='Time [s]', ylabel='Kd', xlim=(0, 1), ylim=(-self.PID_MAX, self.PID_MAX), scroll_gap=0.1)
        
        # Set the initial values for the controls.
        self.ctrlSetPoint.set(self.INITIAL_SET_POINT)
        self.ctrlProportional.set(self.INITIAL_KP)
        self.ctrlIntegral.set(self.INITIAL_KI)
        self.ctrlDerivative.set(self.INITIAL_KD)
        self.ctrlPWMFrequency.set(self.INITIAL_PWM_FREQUENCY)
        self.update()

        # Start the controller thread and start the main application loop.
        self.controller.start()
        self.loop()

    def resetController(self):
        """
            Request the controller to reset all pid values. Also clears all of the plots.
        """ 

        self.controller.commandQ.put({
            'type': 'reset'
        })
        self.distancePlot.clear()
        self.errorPlot.clear()
        self.dutyCyclePlot.clear()
        self.kpPlot.clear()
        self.kiPlot.clear()
        self.kdPlot.clear()

    def keyPressed(self, event):
        """
            Called when a key gets pressed.
        """

        # Get key from event.
        key = event.char

        if key == 'r':
            self.resetController()
        elif key == 'q':
            self.onClosing()
        elif key == 'w':
            sp = self.ctrlSetPoint.get()
            if sp < self.SET_POINT_RANGE[1]:
                self.ctrlSetPoint.set(sp + self.SET_POINT_STEP)
        elif key == 's':
            sp = self.ctrlSetPoint.get()
            if sp > self.SET_POINT_RANGE[0]:
                self.ctrlSetPoint.set(sp - self.SET_POINT_STEP)
        elif key == '1':
            self.ctrlSetPoint.set(6.5)
        elif key == '2':
            self.ctrlSetPoint.set(4.3)

    def update(self, val=None):
        """
            Called everytime a slider changes. Sends the controller the current value of the slider.
            By doing so, the controller sends it to the Tiva.
        """

        self.controller.commandQ.put({
            'type': 'constants',
            'setPoint': self.ctrlSetPoint.get(),
            'proportional': self.ctrlProportional.get(),
            'integral': self.ctrlIntegral.get(),
            'derivative': self.ctrlDerivative.get(),
            'frequency': int(self.PWM_CLOCK / self.ctrlPWMFrequency.get())
        })

    def loop(self):
        """
            Updates the plots by getting the data received by the controller.
        """

        # Counter for messages received from the controller.
        messagesReceived = 0
        while not self.controller.messageQ.empty() and not self.closing:

            # Get new message and increase counter.
            messagesReceived += 1
            message = self.controller.messageQ.get()

            # If the message is of type 'sendParameters', this means the Tiva has requested parameters.
            if (message['type'] is 'sendParameters'):
                
                # Send parameters to Tiva.
                print('parameters requested')
                self.update()

            # Otherwise, the message must contain new data values.
            else:

                # Add the latest point received to all of the plots.
                self.distancePlot.addPoint(message['time'], message['distance'])
                self.errorPlot.addPoint(message['time'], message['error'])
                self.dutyCyclePlot.addPoint(message['time'], message['dutyCycle'])
                self.kpPlot.addPoint(message['time'], message['kp'])
                self.kiPlot.addPoint(message['time'], message['ki'])
                self.kdPlot.addPoint(message['time'], message['kd'])
        
        # After adding all points to the plots, redraw the plot. This is the most expensive instruction
        # of the draw cycle. Therefore it is only done when messages are received.
        if messagesReceived > 0:
            self.distancePlot.update()
            self.errorPlot.update()
            self.dutyCyclePlot.update()
            self.kpPlot.update()
            self.kiPlot.update()
            self.kdPlot.update()

        # Avoid calling the loop again if the close button has already been pressed.
        if not self.closing:
            self.root.after(int(self.LOOP_DT), self.loop)

    def onClosing(self):
        """
            Called when the user closes the main gui.
        """

        # Set flag and send signal to stop controller.
        self.closing = True
        self.controller.stop.set()

        # Wait for the controller to end.
        self.controller.join()

        # Close window.
        self.root.destroy()

if __name__ == '__main__':
    
    # Start main application.
    root = tk.Tk()
    app = App(root)
    root.mainloop()
