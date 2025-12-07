from csbenchlab.helpers.metric_helpers import *
import numpy as np

# Reference file for metric f8fbe916-b8b0-4d71-bf50-8c8ccd47154f

# Implement this to generate reference values for the metric
class LiveMetricPoleCart(LiveMetricBase):
    def __init__(self):
        # init matplotlib figure. I want to plot cart in 1d space with pole on top
        self.fig, self.ax = plt.subplots(figsize=(10, 4))
        self.ax.set_xlim(-10, 10)
        self.ax.set_ylim(-0.5, 2.5)
        self.ax.set_aspect('equal')
        self.ax.axhline(y=0, color='k', linewidth=0.5)
        self.ax.grid(True, alpha=0.3)
        self.ax.set_xlabel('Cart Position (m)')
        self.ax.set_ylabel('Height (m)')
        self.ax.set_title('Cart Inverted Pendulum Live Visualization')

        # Initialize cart and pole line objects
        self.cart, = self.ax.plot([], [], 'bs', markersize=10, label='Cart')
        self.pole, = self.ax.plot([], [], 'r-', linewidth=2, label='Pole')
        self.ax.legend()

        plt.ion()  # Turn on interactive mode

    def __call__(self, time, value):

        if not hasattr(self, 'last_t'):
            self.last_t = time
            return
        # this is called at each simulation step
        if time - self.last_t == 0:
            return
        dt = (time - self.last_t)
        self.last_t = time
        x = value[2]
        theta = np.pi + value[3]
        self.cart.set_data([x], [0])
        pole_x = x + np.sin(theta)
        pole_y = np.cos(theta)
        self.pole.set_data([x, pole_x], [0, pole_y])
        self.fig.canvas.draw_idle()
        plt.pause(dt)


metric = LiveMetricPoleCart