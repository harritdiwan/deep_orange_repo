import numpy as np
import sys
from matplotlib.lines import Line2D


class PointPicker(object):
    def __init__(self,fig,ax,mc=1,mx=float("inf"),sc=None,uc=None,rc=None):
        self.fig = fig
        self.ax = ax
        self.selected = []
        self.sc = sc # selected callback
        self.uc = uc # unselected callback
        self.rc = rc  # reset callback
        self.max_clicks = mc # max clicks
        self.max_points = mx # max selected points
        self.gr = None # plot
        self.colors = ["yellow", "red", "green"]

    def reset(self):
        for dt,pl in self.selected:
            pl.remove()
        self.selected = []

    @property
    def all(self):
        if self.gr is not None:
            xdata = self.gr.get_xdata().tolist()
            ydata = self.gr.get_ydata().tolist()
            return [(pt, 1) for pt in zip(xdata,ydata)]
        else:
            return None

    @property
    def data(self):
        return [(pt[0], pt[2]) for pt in self.selected]

    def onpick(self,event):
        if isinstance(event.artist, Line2D):
            self.gr = event.artist
            xdata = self.gr.get_xdata()
            ydata = self.gr.get_ydata()
            ind = event.ind

            pt_data = (np.take(xdata, ind)[0], np.take(ydata, ind)[0])
            self.update(pt_data)

    def update(self,pt_data):
        pts = [pt[0] for pt in self.selected] # pt data

        try:
            ind = pts.index(pt_data)
        except ValueError:
            if len(self.selected) >= self.max_points:
                self.reset()

            pt_plot, = self.ax.plot(pt_data[0], pt_data[1], 'o', ms=12, alpha=1,
                                    color=self.getColor(1), visible=True, linewidth=1)
            self.selected.append([pt_data, pt_plot, 1])
            if self.sc is not None:
                self.sc(pt_data)
        else:
            if self.selected[ind][2] < self.max_clicks:
                self.selected[ind][2] += 1
                self.selected[ind][1].remove()
                self.selected[ind][1], = self.ax.plot(pt_data[0], pt_data[1], 'o', ms=12, alpha=1,
                                                      color=self.getColor(self.selected[ind][2]), visible=True, linewidth=1)
            else:
                self.selected[ind][1].remove()
                rem = self.selected.pop(ind)
                if self.uc is not None:
                    self.uc(rem[0])

        if len(self.selected) >= self.max_points:
            if self.rc is not None:
                self.rc(self.data)

        self.fig.canvas.draw()

    def getColor(self,clicks):
        return self.colors[clicks-1 % len(self.colors)]
