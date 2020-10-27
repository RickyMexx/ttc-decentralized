"""experiment_plotter.py
"""

import os
import numpy as np
import pandas as pd
from matplotlib import pyplot as plt

EXP_PATH = './data/'
IMG_PATH = './images/'
EXP_LST = ['reg_nom_fbl', 'reg_nom_pd', 'reg_nom_pp', 'reg_nom_pp_er',
           'reg_real_fbl', 'reg_real_pd', 'reg_real_pp',
           'track_nom_fbl', 'track_nom_pd', 'track_nom_pp',
           'track_real_fbl', 'track_real_pd', 'track_real_pp',
           'track_real_pp_sm', 'free', 'gravity_nom', 'gravity_real']
           #'reg_nom_lqr_01', 'reg_nom_lqr_02', 'reg_nom_lqr_03']


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
    """
    ax.set_xticks(np.arange(min_x, max_x + 1, step_major_x))
    ax.set_xticks(np.arange(min_x, max_x + 1, step_minor_x), minor=True)
    ax.set_yticks(np.arange(min_y, max_y, step_major_y))
    ax.set_yticks(np.arange(min_y, max_y, step_minor_y), minor=True)
    """
    ax.grid(which='minor', alpha=0.2)
    ax.grid(which='major', alpha=0.5)

    ax.set_xlabel('ms')
    ax.set_ylabel('rad')
    return


def reg_q_nom_comparison(exp_dict):
    """Regulation q comparison."""
    fig, ax = plt.subplots(1, 2, figsize=(8, 3))
    ax[0].set_title('q1')
    setup_ax(ax[0], 0, 600, 100, 20, -0.5, np.pi + 0.01, 0.5, 0.1)
    plot_curve(ax[0], exp_dict['reg_nom_fbl']['qd1'][:600], 'r--', 'target', lw=2, alpha=0.8)
    plot_curve(ax[0], exp_dict['reg_nom_fbl']['q1'][:600], 'C0', 'fbl', lw=2)
    plot_curve(ax[0], exp_dict['reg_nom_pp']['q1'][:600], 'C1', 'pp', lw=2)
    plot_curve(ax[0], exp_dict['reg_nom_pd']['q1'][:600], 'C2', 'pd', lw=2)
    ax[0].legend()

    ax[1].set_title('q2')
    setup_ax(ax[1], 0, 600, 100, 20, -3, 5, 0.5, 0.1)
    plot_curve(ax[1], exp_dict['reg_nom_fbl']['qd2'][:600], 'r--', 'target', lw=2, alpha=0.8)
    plot_curve(ax[1], exp_dict['reg_nom_fbl']['q2'][:600], 'C0', 'fbl', lw=2)
    plot_curve(ax[1], exp_dict['reg_nom_pp']['q2'][:600], 'C1', 'pp', lw=2)
    plot_curve(ax[1], exp_dict['reg_nom_pd']['q2'][:600], 'C2', 'pd', lw=2)
    ax[1].legend()
    plt.tight_layout()
    plt.savefig('images/reg_q_nom_comparison.png')
    # plt.show()


def reg_u_nom_comparison(exp_dict):
    """Regulation u comparison."""
    fig, ax = plt.subplots(1, 2, figsize=(8, 3))
    ax[0].set_title('u1')
    setup_ax(ax[0], 0, 600, 100, 20, -5.0, 5.1, 1.0, 0.5)
    plot_curve(ax[0], exp_dict['reg_nom_pd']['u1'][:600], 'C2', 'pd', lw=1)
    plot_curve(ax[0], exp_dict['reg_nom_fbl']['u1'][:600], 'C0', 'fbl', lw=2)
    plot_curve(ax[0], exp_dict['reg_nom_pp']['u1'][:600], 'C1', 'pp', lw=2)
    ax[0].legend()
    ax[0].set_ylabel('Nm')

    ax[1].set_title('u2')
    setup_ax(ax[1], 0, 600, 100, 20, -5.0, 5.1, 1.0, 0.5)
    plot_curve(ax[1], exp_dict['reg_nom_pd']['u2'][:600], 'C2', 'pd', lw=1)
    plot_curve(ax[1], exp_dict['reg_nom_fbl']['u2'][:600], 'C0', 'fbl', lw=2)
    plot_curve(ax[1], exp_dict['reg_nom_pp']['u2'][:600], 'C1', 'pp', lw=2)

    ax[1].legend()
    ax[1].set_ylabel('Nm')
    plt.tight_layout()
    plt.savefig('images/reg_u_nom_comparison.png')


def reg_q_nom_pp_comparison(exp_dict):
    """Regulation nominal q PP comparison
    between PP and PP with Error Rejection"""
    fig, ax = plt.subplots(1, 2, figsize=(8, 3))
    ax[0].set_title('q1')
    setup_ax(ax[0], 0, 550, 100, 20, 1.2, 2.2, 0.1, 0.05)
    plot_curve(ax[0], exp_dict['reg_nom_fbl']['qd1'][50:600], 'r--', 'target', lw=2, alpha=0.8)
    plot_curve(ax[0], exp_dict['reg_nom_pp']['q1'][50:600], 'C0', 'pp', lw=2)
    plot_curve(ax[0], exp_dict['reg_nom_pp_er']['q1'][50:600], 'C1', 'pp with error rejection', lw=2)
    ax[0].legend()

    ax[1].set_title('q2')
    setup_ax(ax[1], 0, 600, 100, 20, -3, 5, 0.5, 0.1)
    plot_curve(ax[1], exp_dict['reg_nom_fbl']['qd2'][:600], 'r--', 'target', lw=2, alpha=0.8)
    plot_curve(ax[1], exp_dict['reg_nom_pp']['q2'][:600], 'C0', 'pp', lw=2)
    plot_curve(ax[1], exp_dict['reg_nom_pp_er']['q2'][:600], 'C1', 'pp with error rejection', lw=2)
    ax[1].legend()
    plt.tight_layout()
    plt.savefig('images/reg_q_nom_pp_comparison.png')

def reg_u_nom_lqr_comparison(exp_dict):
    """Regulation nominal u LQR comparison
    01 -> Q = 1, R = 1
    02 -> Q = 1, R = 10
    03 -> Q = 1, R = 50
    """
    fig, ax = plt.subplots(1, 2, figsize=(8, 3))
    ax[0].set_title('u1')
    setup_ax(ax[0], 0, 200, 100, 20, -5, 5.1, 1, 0.5)
    plot_curve(ax[0], exp_dict['reg_nom_lqr_01']['u1'][:200], 'C0', 'R=1', lw=2)
    plot_curve(ax[0], exp_dict['reg_nom_lqr_02']['u1'][:200], 'C1', 'R=10', lw=2)
    plot_curve(ax[0], exp_dict['reg_nom_lqr_03']['u1'][:200], 'C2', 'R=50', lw=2)
    ax[0].set_ylabel('Nm')
    ax[0].legend()

    ax[1].set_title('u2')
    setup_ax(ax[1], 0, 100, 50, 20, -5, 5.1, 1, 0.5)
    plot_curve(ax[1], exp_dict['reg_nom_lqr_01']['u2'][:100], 'C0', 'R=1', lw=2)
    plot_curve(ax[1], exp_dict['reg_nom_lqr_02']['u2'][:100], 'C1', 'R=10', lw=2)
    plot_curve(ax[1], exp_dict['reg_nom_lqr_03']['u2'][:100], 'C2', 'R=50', lw=2)
    ax[1].set_ylabel('Nm')
    ax[1].legend()
    plt.tight_layout()
    plt.savefig('images/reg_u_nom_lqr_comparison.png')

def reg_q_nom_lqr_comparison(exp_dict):
    """Regulation nominal u LQR comparison
    01 -> Q = 1, R = 1
    02 -> Q = 1, R = 10
    03 -> Q = 1, R = 50
    """
    fig, ax = plt.subplots(1, 2, figsize=(8, 3))
    ax[0].set_title('q1')
    setup_ax(ax[0], 0, 200, 100, 20, -5, 5.1, 1, 0.5)
    plot_curve(ax[0], exp_dict['reg_nom_lqr_03']['qd1'][:200], 'r--', 'target', lw=2, alpha=0.8)
    plot_curve(ax[0], exp_dict['reg_nom_lqr_01']['q1'][:200], 'C0', 'R=1', lw=2)
    plot_curve(ax[0], exp_dict['reg_nom_lqr_02']['q1'][:200], 'C1', 'R=10', lw=2)
    plot_curve(ax[0], exp_dict['reg_nom_lqr_03']['q1'][:200], 'C2', 'R=50', lw=2)

    ax[1].set_title('q2')
    setup_ax(ax[1], 0, 100, 50, 20, -5, 5.1, 1, 0.5)
    plot_curve(ax[1], exp_dict['reg_nom_lqr_03']['qd2'][:100], 'r--', 'target', lw=2, alpha=0.8)
    plot_curve(ax[1], exp_dict['reg_nom_lqr_01']['q2'][:100], 'C0', 'R=1', lw=2)
    plot_curve(ax[1], exp_dict['reg_nom_lqr_02']['q2'][:100], 'C1', 'R=10', lw=2)
    plot_curve(ax[1], exp_dict['reg_nom_lqr_03']['q2'][:100], 'C2', 'R=50', lw=2)
    ax[1].legend()
    plt.tight_layout()
    plt.savefig('images/reg_q_nom_lqr_comparison.png')



def reg_err_nom_real_comparison(exp_dict):
    """Regulation nominal/real error comparison
       after transition phase"""
    fig, ax = plt.subplots(1, 2, figsize=(8, 3))
    ax[0].set_title('e1 w/no mismatch')
    setup_ax(ax[0], 0, 500, 200, 100, -0.1, 0.1, 0.01, 0.001)
    plot_curve(ax[0], exp_dict['reg_nom_pd']['e1'][50:550], 'C2', 'pd', lw=2)
    plot_curve(ax[0], exp_dict['reg_nom_fbl']['e1'][50:550], 'C0', 'fbl', lw=2)
    plot_curve(ax[0], exp_dict['reg_nom_pp']['e1'][50:550], 'C1', 'pp', lw=2)
    plot_curve(ax[0], np.zeros((500,)), 'k--', 'target', lw=3, alpha=0.5)
    ax[0].legend()

    ax[1].set_title('e1 w/ mismatch')
    setup_ax(ax[1], 0, 500, 200, 100, -0.1, 0.2, 0.02, 0.01)
    plot_curve(ax[1], exp_dict['reg_real_pd']['e1'][50:550], 'C2', 'pd', lw=2)
    plot_curve(ax[1], exp_dict['reg_real_fbl']['e1'][50:550], 'C0', 'fbl', lw=2)
    plot_curve(ax[1], exp_dict['reg_real_pp']['e1'][50:550], 'C1', 'pp', lw=2)
    plot_curve(ax[1], np.zeros((500,)), 'k--', 'target', lw=3, alpha=0.5)
    plt.tight_layout()
    # plt.show()
    plt.savefig('images/reg_err_nom_real_comparison.png')


def free_evolution_plot(exp_dict):
    """Free evolution of the manipulator"""
    fig, ax = plt.subplots(1, 2, figsize=(8, 3))
    ax[0].set_title('q1')
    setup_ax(ax[0], 0, 2000, 500, 100, -np.pi, np.pi / 2, np.pi / 12, np.pi / 24)
    plot_curve(ax[0], exp_dict['free']['q1'], 'C0', lw=2)

    ax[1].set_title('q2')
    setup_ax(ax[1], 0, 2000, 500, 100, -np.pi, np.pi / 2, np.pi / 12, np.pi / 24)
    plot_curve(ax[1], exp_dict['free']['q2'], 'C0', lw=2)
    plt.tight_layout()
    plt.savefig('images/free_evol.png')


def reg_q_real_comparison(exp_dict):
    """Regulation q comparison with model's mismatch."""
    fig, ax = plt.subplots(1, 2, figsize=(8, 3))
    ax[0].set_title('q1')
    setup_ax(ax[0], 0, 600, 100, 20, -0.5, np.pi + 0.01, 0.5, 0.1)
    plot_curve(ax[0], exp_dict['reg_real_fbl']['qd1'][:600], 'r--', 'target', lw=2, alpha=0.8)
    plot_curve(ax[0], exp_dict['reg_real_fbl']['q1'][:600], 'C0', 'fbl', lw=2)
    plot_curve(ax[0], exp_dict['reg_real_pp']['q1'][:600], 'C1', 'pp', lw=2)
    plot_curve(ax[0], exp_dict['reg_real_pd']['q1'][:600], 'C2', 'pd', lw=2)
    ax[0].legend()

    ax[1].set_title('q2')
    setup_ax(ax[1], 0, 600, 100, 20, -3, 5, 0.5, 0.1)
    plot_curve(ax[1], exp_dict['reg_real_fbl']['qd2'][:600], 'r--', 'target', lw=2, alpha=0.8)
    plot_curve(ax[1], exp_dict['reg_real_fbl']['q2'][:600], 'C0', 'fbl', lw=2)
    plot_curve(ax[1], exp_dict['reg_real_pp']['q2'][:600], 'C1', 'pp', lw=2)
    plot_curve(ax[1], exp_dict['reg_real_pd']['q2'][:600], 'C2', 'pd', lw=2)
    ax[1].legend()
    plt.tight_layout()
    plt.savefig('images/reg_q_real_comparison.png')
    # plt.show()


def track_q_nom_comparison(exp_dict):
    """Tracking task Nominal Conditions
    Comparison on q values"""
    fig, ax = plt.subplots(1, 2, figsize=(8, 3))
    ax[0].set_title('q1')
    setup_ax(ax[0], 0, 600, 100, 50, -1.1, 1.2, 0.5, 0.2)
    plot_curve(ax[0], exp_dict['track_nom_fbl']['q1'][:500], 'C0', 'fbl', lw=2)
    plot_curve(ax[0], exp_dict['track_nom_pp']['q1'][:500], 'C1', 'pp', lw=2)
    plot_curve(ax[0], exp_dict['track_nom_pd']['q1'][:500], 'C2', 'pd', lw=2)
    plot_curve(ax[0], exp_dict['track_nom_fbl']['qd1'][:500], 'r--', 'target', lw=2, alpha=0.8)

    ax[1].set_title('q2')
    setup_ax(ax[1], 0, 1201, 300, 150, -1.1, 1.6, 0.5, 0.2)
    plot_curve(ax[1], exp_dict['track_nom_fbl']['q2'][:500], 'C0', 'fbl', lw=2)
    plot_curve(ax[1], exp_dict['track_nom_pp']['q2'][:500], 'C1', 'pp', lw=2)
    plot_curve(ax[1], exp_dict['track_nom_pd']['q2'][:500], 'C2', 'pd', lw=2)
    plot_curve(ax[1], exp_dict['track_nom_fbl']['qd2'][:500], 'r--', 'target', lw=2, alpha=0.8)
    ax[1].legend(loc='upper right')
    plt.tight_layout()
    plt.savefig('images/track_q_nom_comparison.png')


def track_q_real_comparison(exp_dict):
    """Tracking task Real Conditions
    Comparison on q values"""
    fig, ax = plt.subplots(1, 2, figsize=(8, 3))
    ax[0].set_title('q1')
    setup_ax(ax[0], 0, 600, 100, 50, -1.1, 1.2, 0.5, 0.2)
    plot_curve(ax[0], exp_dict['track_real_fbl']['q1'][:500], 'C0', 'fbl', lw=2)
    plot_curve(ax[0], exp_dict['track_real_pp']['q1'][:500], 'C1', 'pp', lw=2)
    plot_curve(ax[0], exp_dict['track_real_pd']['q1'][:500], 'C2', 'pd', lw=2)
    plot_curve(ax[0], exp_dict['track_real_fbl']['qd1'][:500], 'r--', 'target', lw=2, alpha=0.8)

    ax[1].set_title('q2')
    setup_ax(ax[1], 0, 1201, 300, 150, -1.1, 1.6, 0.5, 0.2)
    plot_curve(ax[1], exp_dict['track_real_fbl']['q2'][:500], 'C0', 'fbl', lw=2)
    plot_curve(ax[1], exp_dict['track_real_pp']['q2'][:500], 'C1', 'pp', lw=2)
    plot_curve(ax[1], exp_dict['track_real_pd']['q2'][:500], 'C2', 'pd', lw=2)
    plot_curve(ax[1], exp_dict['track_real_fbl']['qd2'][:500], 'r--', 'target', lw=2, alpha=0.8)
    ax[1].legend(loc='upper right')
    plt.tight_layout()
    plt.savefig('images/track_q_real_comparison.png')


def track_e_real_pp_sm_comparison(exp_dict):
    """Tracking task Real Conditions
        Comparison on e1 values for pp and pp_sm"""
    fig, ax = plt.subplots(1, 2, figsize=(8, 3))
    ax[0].set_title('e1')
    setup_ax(ax[0], 0, 1101, 250, 100, -0.05, 0.05, 0.01, 0.005)
    plot_curve(ax[0], exp_dict['track_real_pp']['e1'][900:], 'C1', 'pp', lw=2)
    plot_curve(ax[0], exp_dict['track_real_pp_sm']['e1'][900:], 'C4', 'pp w/ sliding mode', lw=2)

    ax[1].set_title('e2')
    setup_ax(ax[1], 0, 1201, 300, 150, -0.02, 0.26, 0.02, 0.01)
    plot_curve(ax[1], exp_dict['track_real_pp']['e2'][900:], 'C1', 'pp', lw=2)
    plot_curve(ax[1], exp_dict['track_real_pp_sm']['e2'][900:], 'C4', 'pp w/ sliding mode', lw=2)
    ax[1].legend(loc='upper right')
    plt.tight_layout()
    plt.savefig('images/track_e_real_pp_sm_comparison.png')

def gravity_comparison(exp_dict):
    """Gravity and Coulomb Friction controller comparison 
    with nominal and mismatched models [nr=0.001]"""
    fig, ax = plt.subplots(1, 2, figsize=(8, 3))
    ax[0].set_title('q1')
    setup_ax(ax[0], 0, 2001, 500, 100, -3.0, 1.0, 0.5, 0.2)
    plot_curve(ax[0], exp_dict['gravity_nom']['q1'], 'C0', 'nominal', lw=2)
    plot_curve(ax[0], exp_dict['gravity_real']['q1'], 'C1', 'mismatch[0.01]', lw=2)
    plot_curve(ax[0], exp_dict['gravity_nom']['qd1'], 'r--', 'target', lw=2, alpha=0.8)

    ax[1].set_title('q2')
    setup_ax(ax[1], 0, 2001, 500, 100, -1.0, 8.0, 1.0, 0.5)
    plot_curve(ax[1], exp_dict['gravity_nom']['q2'], 'C0', 'nominal', lw=2)
    plot_curve(ax[1], exp_dict['gravity_real']['q2'], 'C1', 'mismatch[0.01]', lw=2)
    plot_curve(ax[1], exp_dict['gravity_nom']['qd2'], 'r--', 'target', lw=2, alpha=0.8)
    ax[1].legend(loc='upper right')
    plt.tight_layout()
    plt.savefig('images/gravity_comparison.png')


if __name__ == '__main__':
    exp_dict = load_experiments(EXP_LST)
    free_evolution_plot(exp_dict)
    reg_q_nom_comparison(exp_dict)
    reg_q_nom_pp_comparison(exp_dict)
    reg_u_nom_comparison(exp_dict)
    reg_err_nom_real_comparison(exp_dict)
    track_q_nom_comparison(exp_dict)
    reg_q_real_comparison(exp_dict)
    track_e_real_pp_sm_comparison(exp_dict)
    track_q_real_comparison(exp_dict)
    gravity_comparison(exp_dict)
    #reg_u_nom_lqr_comparison(exp_dict)
    exit(0)
