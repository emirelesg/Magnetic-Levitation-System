from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.patches import Ellipse, Circle, Polygon, Rectangle
from matplotlib.figure import Figure
from matplotlib import transforms
import matplotlib.image as image
import numpy as np
import tkinter as tk
import matplotlib

# Use the tkinter backend for matplotlib.
matplotlib.use('TkAgg')

class Plot():
    """
        Wrapper for a pyplot figure embbedded in a Tkinter window.
    """

    def __init__(self, root, frame, row=0, column=0, columnspan=1, title="", xlabel="", ylabel="", xlim=(0,100), ylim=(0,100), scroll_gap=0):
        self.figure = Figure(figsize=(4, 3), dpi=100)
        self.ax = self.figure.add_subplot(1, 1, 1)
        self.ax.grid(True)
        self.ax.set_title(title)
        self.ax.set_xlabel(xlabel)
        self.ax.set_ylabel(ylabel)
        self.scroll_gap = scroll_gap
        self.default_xlim = xlim
        self.default_ylim = ylim
        self.ax.set_xlim(self.default_xlim)
        self.ax.set_ylim(self.default_ylim)
        self.x = [0]
        self.y = [0]
        self.line, = self.ax.plot(self.x, self.y, 'b-')
        self.canvas = FigureCanvasTkAgg(self.figure, root)
        self.canvas.get_tk_widget().grid(in_=frame, row=row, column=column, pady=10, padx=10, columnspan=columnspan)
        self.bg = self.figure.canvas.copy_from_bbox(self.ax.bbox)

    def addPoint(self, x, y):
        """
            Given a pair x, y, adds the point to the plot.
            The plot has a limit of 300 points.
        """

        if (x is not None and y is not None):
            if (len(self.x) > 500):
                self.y.pop(0)
                self.x.pop(0)
            self.x.append(x)
            self.y.append(y)
            #self.update()
    
    def clear(self):
        """
            Resets scale and clears plot.
        """

        self.ax.set_ylim(self.default_ylim)
        self.ax.set_xlim(self.default_xlim)
        self.x = [0]
        self.y = [0] 
        self.update()

    def changeScale(self, k):
        """
            Given a factor k updates the scale of the plot. 
        """

        r = np.ceil(20 / k)
        self.default_xlim = (-r, r)
        self.default_ylim = (-r, r)

    def update(self, scrollX=True, scrollY=False):
        """
            Updates the x and y data of the plot. If a scroll gap is
            specified, then the plot starts to scroll before it reaches this gap.
        """

        # Update the data of the plot to match that of the arrays.
        self.line.set_xdata(self.x)
        self.line.set_ydata(self.y)
        
        # Scroll plot in increasing X and Y values. 
        # Todo: Negative scroll values.
        if scrollX:
            lastx = self.x[-1]
            if (lastx + self.scroll_gap > self.default_xlim[1]):
                self.ax.set_xlim((lastx - (self.default_xlim[1] - self.default_xlim[0] - self.scroll_gap), lastx+self.scroll_gap))
        if scrollY:
            lasty = self.y[-1]
            if (lasty + self.scroll_gap > self.default_ylim[1]):
                self.ax.set_ylim((lasty - (self.default_ylim[1] - self.default_ylim[0] - self.scroll_gap), lasty+self.scroll_gap))
            
        # Redraw plot (slow)
        try:
            self.figure.canvas.draw()
            self.figure.canvas.flush_events()
        except tk._tkinter.TclError:
            pass


