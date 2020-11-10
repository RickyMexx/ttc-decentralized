"""animation_renderer.py
"""

from experiment_plotter import load_experiments, EXP_LST
from matplotlib import pyplot as plt
import numpy as np
import matplotlib.animation as animation

SIM_TOTAL_TIME = 2000
SIM_DT = 1
T = np.arange(0.0, SIM_TOTAL_TIME, SIM_DT)


def step_model(t, exp, desired=False) -> ([], []):
    # Compute direct kinematics for the 2R arm (assuming l=1 for both joints)
    if desired is False:
        x1 = np.cos(exp['q1'][t])
        x2 = x1 + np.cos(exp['q1'][t] + exp['q2'][t])
        y1 = np.sin(exp['q1'][t])
        y2 = y1 + np.sin(exp['q1'][t] + exp['q2'][t])
    else:
        x1 = np.cos(exp['qd1'][t])
        x2 = x1 + np.cos(exp['qd1'][t] + exp['qd2'][t])
        y1 = np.sin(exp['qd1'][t])
        y2 = y1 + np.sin(exp['qd1'][t] + exp['qd2'][t])
    return [0.0, x1, x2], [0.0, y1, y2]


def animate_experiment(exp, model, file: str) -> None:
    fig = plt.figure()
    ax = fig.add_subplot(111, autoscale_on=False, xlim=(-2, 2), ylim=(-2, 2))
    ax.set_aspect('equal')
    ax.grid()

    line, = ax.plot([], [], 'o-', lw=3, label='Pose')
    line_des, = ax.plot([], [], 'o--', lw=2, label='Desired')
    time_template = 'time = %d ms'
    time_text = ax.text(0.05, 0.9, '', transform=ax.transAxes)
    ax.legend()

    def init():
        line.set_data([], [])
        line_des.set_data([], [])
        time_text.set_text('')
        return line, time_text

    def animate(i):
        thisx, thisy = step_model(i, exp[model])
        this_desx, this_desy = step_model(i, exp[model], desired=True)
        line.set_data(thisx, thisy)
        line_des.set_data(this_desx, this_desy)
        time_text.set_text(time_template % (i))
        return line, line_des, time_text

    ani = animation.FuncAnimation(fig, animate, np.arange(50, 2000),
                                  interval=1, blit=True, init_func=init)
    plt.show()


if __name__ == '__main__':
    exp_dict = load_experiments(EXP_LST)
    animate_experiment(exp_dict, 'track_circle_nom_fbl', None)
    #animate_experiment(exp_dict, 'track_nom_fbl', None)
    exit(0)
