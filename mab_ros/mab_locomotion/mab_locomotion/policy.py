import numpy as np
import jax
import jax.numpy as jnp
import flax.linen as nn
from flax.linen.initializers import constant, orthogonal


def get_policy(algorithm_config):
    return (
        Policy(
            algorithm_config["std_dev"],
            algorithm_config["policy_mean_abs_clip"],
            algorithm_config["policy_std_min_clip"], algorithm_config["policy_std_max_clip"]
        )
    )


class Policy(nn.Module):
    std_dev: float
    policy_mean_abs_clip: float
    policy_std_min_clip: float
    policy_std_max_clip: float

    @nn.compact
    def __call__(self, x):
        policy_mean = nn.Dense(512, kernel_init=orthogonal(np.sqrt(2)), bias_init=constant(0.0))(x)
        policy_mean = nn.LayerNorm()(policy_mean)
        policy_mean = nn.elu(policy_mean)
        policy_mean = nn.Dense(256, kernel_init=orthogonal(np.sqrt(2)), bias_init=constant(0.0))(policy_mean)
        policy_mean = nn.elu(policy_mean)
        policy_mean = nn.Dense(128, kernel_init=orthogonal(np.sqrt(2)), bias_init=constant(0.0))(policy_mean)
        policy_mean = nn.elu(policy_mean)
        policy_mean = nn.Dense(13, kernel_init=orthogonal(0.01), bias_init=constant(0.0))(policy_mean)
        policy_mean = jnp.clip(policy_mean, -self.policy_mean_abs_clip, self.policy_mean_abs_clip)

        return policy_mean
