# -*- coding: utf-8 -*-
"""Untitled6.ipynb

Automatically generated by Colaboratory.

Original file is located at
    https://colab.research.google.com/drive/1nqvLvoUq4EvZvxxcC9-98OI0TfagJwM_
"""

import numpy as np
import tensorflow as tf
from tensorflow import keras
from tensorflow.keras.layers import Dense, Input, Activation, Dropout, LayerNormalization
from tensorflow.keras import Sequential, activations
from tensorflow.keras import regularizers
import matplotlib.pyplot as plt

# STEP 1
# First, Generate Dataset.
L_1 = 1
L_2 = 1
L_3 = 1

dataset_size = 1000


joint_dataset = []
ee_pos_dataset = []

def forward_kinematics(joints):
    """ Takes in joint angles & outputs end effector position """
    theta_1 = joints[0]
    theta_2 = joints[1]
    theta_3 = joints[2]

    position = [
        np.cos(theta_1) * (L_1 + np.cos(theta_2) * L_2 + np.cos(theta_2 + theta_3) * L_3),
        np.sin(theta_1) * (L_1 + np.cos(theta_2) * L_2 + np.cos(theta_2 + theta_3) * L_3),
        np.sin(theta_2) * L_2 + np.sin(theta_2 + theta_3) * L_3,
    ]

    return np.array(position)

for _ in range(dataset_size):
    joints = np.random.uniform(-np.pi, np.pi, size=(3,))
    ee_pos = forward_kinematics(joints)

    joint_dataset.append(joints)
    ee_pos_dataset.append(ee_pos)

# STEP 2
# Next, split the dataset into Training & Test and train the network.

train_split = 0.8
train_cutoff = int(0.8 * dataset_size)

train_joint_dataset = np.array(joint_dataset[:train_cutoff])
test_joint_dataset = np.array(joint_dataset[train_cutoff:])
train_ee_pos_dataset = np.array(ee_pos_dataset[:train_cutoff])
test_ee_pos_dataset = np.array(ee_pos_dataset[train_cutoff:])

n_inputs = 3
n_outputs = 3

# hyperparameters
num_epochs = 1000
batch_size = 32
lr = 1e-5
dropout = 0.1
l2_reg = 1e-4

model = Sequential([
    Dense(50, kernel_initializer=keras.initializers.GlorotNormal(), kernel_regularizer=regularizers.l2(l2_reg),bias_regularizer=tf.keras.regularizers.l2(1e-4),activity_regularizer=tf.keras.regularizers.l2(1e-5)),
    Activation(activations.tanh),
    Dropout(dropout),
    #Dense(50, kernel_initializer=keras.initializers.GlorotNormal(), kernel_regularizer=regularizers.l2(l2_reg),bias_regularizer=tf.keras.regularizers.l2(1e-4),activity_regularizer=tf.keras.regularizers.l2(1e-5)),
    #Activation(activations.tanh),
    #Dropout(dropout),
    Dense(n_outputs, kernel_initializer=keras.initializers.GlorotNormal()),
])

optimizer = keras.optimizers.Adam(learning_rate=lr)

model.compile(optimizer=optimizer, loss="mse")

history = model.fit(
    x=train_ee_pos_dataset,
    y=train_joint_dataset,
    validation_data=(test_ee_pos_dataset, test_joint_dataset),
    batch_size = batch_size,
    epochs = num_epochs,
)

loss_history = history.history["loss"]
val_loss_history = history.history["val_loss"]

plt.figure(figsize=(12,8))
plt.plot(range(num_epochs), loss_history, label="MSE (Train)")
plt.plot(range(num_epochs), val_loss_history, label="MSE (Validation)")
plt.xlabel("Epoch")
plt.ylabel("Mean Squared Error (MSE)")
plt.title("Learned IK Training Curves")
plt.legend()
plt.show()


# STEP 3

K = 500
traj = np.zeros((K,3))
traj[:,0] = 2*np.cos(np.linspace(0,2*np.pi,num=K))
traj[:,1] = 2*np.sin(np.linspace(0,2*np.pi,num=K))
traj[:,2] = np.sin(np.linspace(0,8*np.pi,num=K))


fig = plt.figure(figsize=(12,8))
ax = plt.axes(projection="3d")

ax.plot3D(traj[:,0], traj[:,1], traj[:,2])
plt.title("Target Trajectory")
ax.set_xlabel("X")
ax.set_ylabel("Y")
ax.set_zlabel("Z")
plt.show()

joint_traj = model.predict(traj)


fig = plt.figure(figsize=(12,8))
ax = plt.axes(projection="3d")

ax.plot3D(joint_traj[:,0], joint_traj[:,1], joint_traj[:,2])
plt.title("Predicted Joint Trajectory")
ax.set_xlabel("Theta_1")
ax.set_ylabel("Theta_2")
ax.set_zlabel("Theta_3")
plt.show()


# use these joint trajectories to recover the resultant EE trajectory.

resulting_ee_traj = np.array([forward_kinematics(joint_traj[i,:]) for i in range(K)])

fig = plt.figure(figsize=(12,8))
ax = plt.axes(projection="3d")

ax.plot3D(resulting_ee_traj[:,0], resulting_ee_traj[:,1], resulting_ee_traj[:,2], label="Predicted")
ax.plot3D(traj[:,0], traj[:,1], traj[:,2], label="Target")
plt.title("Resultant End Effector Trajectory vs. Target")
ax.set_xlabel("X")
ax.set_ylabel("Y")
ax.set_zlabel("Z")
plt.show()



plt.plot(joint_traj[:,0])

plt.plot(joint_traj[:,1])
plt.plot(joint_traj[:,2])

