import tensorflow as tf

print("tf version is:", tf.__version__)

sess = tf.Session()
sess.close()
print("End tf test, leaving python")