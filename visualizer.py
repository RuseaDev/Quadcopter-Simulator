import numpy as np
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from scipy import stats
from scipy.linalg import expm
import sys

plt.rcParams.update({
    'figure.dpi': 110,
    'axes.grid': True,
    'axes.grid.alpha': 0.3,
    'axes.spines.top': False,
    'axes.spines.right': False,
})
RNG = np.random.default_rng(42)



def sim01_central_limit_theorem():
    """
    The CLT in action: averages of ANY distribution converge to Gaussian.
    Watch uniform, exponential, and Laplace means become normal as N grows.
    This is WHY Kalman filters assume Gaussian noise — real sensor noise is
    the sum of many small independent contributions.
    """
    print("SIM 01: Central Limit Theorem")
    distributions = {
        'Uniform [0,1]':     lambda n, s: RNG.uniform(0, 1, (s, n)),
        'Exponential(λ=1)':  lambda n, s: RNG.exponential(1, (s, n)),
        'Laplace(0,1)':      lambda n, s: RNG.laplace(0, 1, (s, n)),
    }
    sample_sizes = [1, 5, 30, 100]
    n_samples = 5000
 
    fig, axes = plt.subplots(3, 4, figsize=(14, 9))
    fig.suptitle('Simulation 01: Central Limit Theorem\nSample means converge to Gaussian as N grows', fontsize=13)
 
    for row, (name, dist_fn) in enumerate(distributions.items()):
        for col, n in enumerate(sample_sizes):
            data   = dist_fn(n, n_samples)
            means  = data.mean(axis=1)
            ax     = axes[row, col]
 
            ax.hist(means, bins=60, density=True, color='steelblue',
                    alpha=0.7, edgecolor='none')
            mu, sigma = means.mean(), means.std()
            xx = np.linspace(mu - 4*sigma, mu + 4*sigma, 200)
            ax.plot(xx, stats.norm.pdf(xx, mu, sigma), 'r-', lw=2)
 
            ax.set_title(f'N={n}', fontsize=10)
            if col == 0:
                ax.set_ylabel(name, fontsize=9)
            ax.set_xlabel('Sample mean', fontsize=8)
            ax.tick_params(labelsize=7)
 
    plt.tight_layout()
    plt.savefig('sim01_clt.png', dpi=110)
    plt.show()
    print("  → Saved sim01_clt.png\n")
 