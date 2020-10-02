"""experiment_plotter.py
"""

import os
import numpy as np
import pandas as pd
from matplotlib import pyplot as plt

EXP_PATH = './data/'
IMG_PATH = './images/'
EXP_LST = ['reg_nom_fbl', 'reg_nom_pd', 'reg_nom_pp',
           'reg_real_fbl', 'reg_real_pd', 'reg_real_pp',
           'track_nom_fbl', 'track_nom_pd', 'track_nom_pp',]


def read_table(fname: str) -> pd.DataFrame:
    path = os.path.join(EXP_PATH, 'exp_' + fname + '.csv')
    df = pd.read_csv(path)
    return df


def load_experiments(exp_lst: ['str']) -> {'str': pd.DataFrame}:
    exp_dict = {}
    for exp in exp_lst:
        df = read_table(exp)
        exp_dict[exp] = df
    return exp_dict


def plot_curve(ax, x: np.array, c: str, label: str = None, lw=1.0, alpha=1.0):
    ax.plot(np.arange(0, x.shape[0], 1), x, c, label=label, alpha=alpha, lw=lw)


def setup_ax(ax, min_x, max_x, step_major_x, step_minor_x, min_y, max_y, step_major_y, step_minor_y):
    ax.set_xticks(np.arange(min_x, max_x+1, step_major_x))
    ax.set_xticks(np.arange(min_x, max_x+1, step_minor_x), minor=True)
    ax.set_yticks(np.arange(min_y, max_y, step_major_y))
    ax.set_yticks(np.arange(min_y, max_y, step_minor_y), minor=True)
    ax.grid(which='minor', alpha=0.2)
    ax.grid(which='major', alpha=0.5)
    return

def reg_q_nom_comparison(exp_dict):
    """Regulation q comparison."""
    fig, ax = plt.subplots(1, 2, figsize=(12, 5))
    ax[0].set_title('q1')
    setup_ax(ax[0], 0, 2000, 500, 100, -0.5, np.pi + 0.01, 0.5, 0.1)
    plot_curve(ax[0], exp_dict['reg_nom_pd']['q1'], 'C2', 'pd', lw=2)
    plot_curve(ax[0], exp_dict['reg_nom_fbl']['q1'], 'C0', 'fbl', lw=2)
    plot_curve(ax[0], exp_dict['reg_nom_pp']['q1'], 'C1', 'pp', lw=2)
    plot_curve(ax[0], exp_dict['reg_nom_fbl']['qd1'], 'r--', 'target', lw=4, alpha=0.5)
    ax[0].legend()

    ax[1].set_title('q2')
    setup_ax(ax[1], 0, 2000, 500, 100, -3, 5, 0.5, 0.1)
    plot_curve(ax[1], exp_dict['reg_nom_pd']['q2'], 'C2', 'pd', lw=2)
    plot_curve(ax[1], exp_dict['reg_nom_fbl']['q2'], 'C0', 'fbl', lw=2)
    plot_curve(ax[1], exp_dict['reg_nom_pp']['q2'], 'C1', 'pp', lw=2)
    plot_curve(ax[1], exp_dict['reg_nom_fbl']['qd2'], 'r--', 'target', lw=4, alpha=0.5)
    ax[1].legend()
    plt.show()

def reg_err_nom_real_comparison(exp_dict):
    """Regulation nominal/real error comparison
       after transition phase"""
    fig, ax = plt.subplots(1, 2, figsize=(12, 5))
    ax[0].set_title('e1 w/no mismatch')
    setup_ax(ax[0], 0, 1100, 200, 100, -0.1, 0.1, 0.01, 0.001)
    plot_curve(ax[0],  exp_dict['reg_nom_pd']['e1'][900:], 'C2', 'pd', lw=2)
    plot_curve(ax[0], exp_dict['reg_nom_fbl']['e1'][900:], 'C0', 'fbl', lw=2)
    plot_curve(ax[0],  exp_dict['reg_nom_pp']['e1'][900:], 'C1', 'pp', lw=2)
    plot_curve(ax[0], np.zeros((1100,)), 'k--', 'target', lw=3, alpha=0.5)
    ax[0].legend()

    ax[1].set_title('e1 w/ mismatch')
    setup_ax(ax[1], 0, 1100, 200, 100, -0.1, 0.1, 0.01, 0.001)
    plot_curve(ax[1],  exp_dict['reg_real_pd']['e1'][900:], 'C2', 'pd', lw=2)
    plot_curve(ax[1], exp_dict['reg_real_fbl']['e1'][900:], 'C0', 'fbl', lw=2)
    plot_curve(ax[1],  exp_dict['reg_real_pp']['e1'][900:], 'C1', 'pp', lw=2)
    plot_curve(ax[1], np.zeros((1100,)), 'k--', 'target', lw=3, alpha=0.5)
    plt.show()



if __name__ == '__main__':
    exp_dict = load_experiments(EXP_LST)
    #reg_q_nom_comparison(exp_dict)
    #reg_err_nom_real_comparison(exp_dict)
    exit(0)
