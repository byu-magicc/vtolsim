#this file implements the jax method to autotune the pid controllers
import jax.numpy as jnp
from jax import grad, jit, vmap
from jax import random


key = random.key(0)



grad_tanh = grad(jnp.tanh)
print(grad_tanh(2.0))



#lets compute some gradients
def sigmoid(x):
    return 0.5*(jnp.tanh(x/2) + 1)


#outputs probability of a label being true
def predict(W, b, inputs):
    return sigmoid(jnp.dot(inputs, W) + b)


# Build a toy dataset.
inputs = jnp.array([[0.52, 1.12,  0.77],
                   [0.88, -1.08, 0.15],
                   [0.52, 0.06, -1.30],
                   [0.74, -2.49, 1.39]])
targets = jnp.array([True, True, False, True])

#training loss is the negative log-likelihood of the training examples
def loss(W, b):
    preds = predict(W, b, inputs)
    label_probs = preds * targets + (1 - preds)*(1 - targets)
    return -jnp.sum(jnp.log(label_probs))


key, W_key, b_key = random.split(key, 3)
W = random.normal(W_key, (3,))
b = random.normal(b_key, ())



W_grad = grad(loss, argnums=0)(W, b)
print('W_grad', W_grad)